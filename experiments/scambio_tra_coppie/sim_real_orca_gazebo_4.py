"""
Four CrazySim/Gazebo SITL Crazyflies.

CF1 <-> CF2 and CF3 <-> CF4 exchange positions.

Strategies:
- orca
- deadlock-right-bias

Features:
- Multi-agent ORCA
- PD goal controller with velocity damping
- Optional deadlock detection and right-bias
- Goal arrival verification
- CSV logging

Requires orca_lib.py in the same directory.
"""

import argparse
import contextlib
import csv
import math
import sys
import time
from collections import deque
from itertools import combinations
from pathlib import Path
from threading import Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from orca_lib import compute_orca_constraint

import warnings

warnings.filterwarnings(
    "ignore",
    message=r"Using legacy TYPE_.*",
    category=DeprecationWarning
)

# --------------------------------------------------
# CONSTANTS
# --------------------------------------------------

DT = 0.05
TIME_HORIZON = 3.0
FLIGHT_HEIGHT = 0.50
GOAL_TOLERANCE = 0.12
MAX_TELEMETRY_AGE = 0.40
MAX_VERTICAL_CHANGE = 0.20
N_DRONES = 4
PAIRS = [(0, 1), (2, 3)]


# --------------------------------------------------
# UTILITY FUNCTIONS
# --------------------------------------------------

def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def norm(v):
    return math.hypot(*v)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


# --------------------------------------------------
# PD GOAL CONTROLLER
# --------------------------------------------------

def toward_goal(position, goal, speed, measured_velocity, kp, kd):
    """
    Preferred velocity with position feedback and velocity damping.

    v_pref = Kp * (goal - position) - Kd * measured_velocity

    The resulting velocity is limited to the preferred speed.
    ORCA subsequently computes the collision-avoiding velocity.
    """
    ex = goal[0] - position[0]
    ey = goal[1] - position[1]

    vx = kp * ex - kd * measured_velocity[0]
    vy = kp * ey - kd * measured_velocity[1]

    magnitude = math.hypot(vx, vy)

    if magnitude > speed:
        scale = speed / magnitude
        vx *= scale
        vy *= scale

    return (vx, vy)


# --------------------------------------------------
# RIGHT BIAS
# --------------------------------------------------

def turn_right(v, degrees):
    """Rotate preferred velocity clockwise."""
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1])


# --------------------------------------------------
# MULTI-AGENT ORCA SOLVER
# --------------------------------------------------

def solve_orca(preferred, constraints, max_speed):
    """
    Find the velocity closest to preferred subject to:
    - all ORCA half-plane constraints
    - maximum speed

    Simplified 2D multi-constraint solver.
    """
    tol = 1e-8

    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1]

    lines = []

    for point, direction in constraints:
        length = norm(direction)

        if length < 1e-12:
            raise ValueError('Invalid ORCA line')

        d = (direction[0] / length, direction[1] / length)
        normal = (-d[1], d[0])
        lines.append((point, d, normal, dot(normal, point)))

    def feasible(v):
        return norm(v) <= max_speed + tol and all(
            dot(n, v) >= c - tol for _, _, n, c in lines
        )

    pref_length = norm(preferred)

    if pref_length <= max_speed:
        initial = preferred
    else:
        initial = (
            preferred[0] * max_speed / pref_length,
            preferred[1] * max_speed / pref_length
        )

    candidates = [initial, (0.0, 0.0)]

    for point, direction, _, _ in lines:
        b = dot(point, direction)
        discriminant = b * b + max_speed ** 2 - dot(point, point)

        if discriminant < -tol:
            continue

        root = math.sqrt(max(0.0, discriminant))
        low, high = -b - root, -b + root

        target_t = dot(
            (preferred[0] - point[0], preferred[1] - point[1]),
            direction
        )

        for t in (clamp(target_t, low, high), low, high):
            candidates.append((
                point[0] + t * direction[0],
                point[1] + t * direction[1]
            ))

    for (_, _, n1, c1), (_, _, n2, c2) in combinations(lines, 2):
        determinant = n1[0] * n2[1] - n1[1] * n2[0]

        if abs(determinant) < 1e-12:
            continue

        candidates.append((
            (c1 * n2[1] - n1[1] * c2) / determinant,
            (n1[0] * c2 - c1 * n2[0]) / determinant
        ))

    valid = [v for v in candidates if feasible(v)]

    if not valid:
        raise ValueError('ORCA_INFEASIBLE: no feasible velocity')

    return min(
        valid,
        key=lambda v: (
            (v[0] - preferred[0]) ** 2 +
            (v[1] - preferred[1]) ** 2
        )
    )


# --------------------------------------------------
# VELOCITY ESTIMATOR FOR ORCA
# --------------------------------------------------

class VelocityEstimator:
    def __init__(self, alpha=0.25):
        self.alpha = alpha
        self.previous = None
        self.previous_time = None
        self.velocity = (0.0, 0.0)

    def reset(self, position):
        self.previous = position
        self.previous_time = time.monotonic()
        self.velocity = (0.0, 0.0)

    def update(self, position):
        now = time.monotonic()

        if self.previous is None:
            self.reset(position)
            return self.velocity

        dt = now - self.previous_time

        if dt > 0:
            raw = (
                (position[0] - self.previous[0]) / dt,
                (position[1] - self.previous[1]) / dt
            )

            if norm(raw) <= 2.0:
                a = self.alpha
                self.velocity = (
                    a * raw[0] + (1 - a) * self.velocity[0],
                    a * raw[1] + (1 - a) * self.velocity[1]
                )

            self.previous = position
            self.previous_time = now

        return self.velocity


# --------------------------------------------------
# TELEMETRY
# --------------------------------------------------

class Telemetry:
    def __init__(self, cf, name):
        self.name = name
        self.lock = Lock()
        self.latest = None
        self.received_at = 0.0

        self.config = LogConfig(name=f'Gazebo4{name}', period_in_ms=50)

        for variable in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
            self.config.add_variable('stateEstimate.' + variable, 'float')

        cf.log.add_config(self.config)

        if not self.config.valid:
            raise RuntimeError(f'{name}: stateEstimate logging unavailable')

        self.config.data_received_cb.add_callback(self.on_data)
        self.config.error_cb.add_callback(self.on_error)
        self.config.start()

    def on_data(self, timestamp, data, logconf):
        entry = {
            k: float(data['stateEstimate.' + k])
            for k in ('x', 'y', 'z', 'vx', 'vy', 'vz')
        }

        with self.lock:
            self.latest = entry
            self.received_at = time.monotonic()

    def on_error(self, logconf, message):
        print(f'Telemetry {self.name}: {message}', file=sys.stderr)

    def read(self, required=True):
        with self.lock:
            state = self.latest.copy() if self.latest is not None else None
            age = time.monotonic() - self.received_at

        if required and (state is None or age > MAX_TELEMETRY_AGE):
            raise RuntimeError(
                f'{self.name}: stale/missing telemetry ({age:.2f}s)'
            )

        return state

    def close(self):
        self.config.stop()


# --------------------------------------------------
# DEADLOCK DETECTION
# --------------------------------------------------

class PairDeadlockDetector:
    def __init__(self, pair, args):
        self.pair = pair
        self.args = args
        self.history = deque()
        self.active = False
        self.activated_at = None
        self.activations = 0

    def update(self, t, positions, goals, starts):
        i, j = self.pair

        errors = [distance(positions[k], goals[k]) for k in (i, j)]
        separation = distance(positions[i], positions[j])
        total_error = sum(errors)

        self.history.append((t, total_error))

        while len(self.history) > 1 and t - self.history[0][0] > self.args.stall_window:
            self.history.popleft()

        if self.active:
            axis = (
                starts[j][0] - starts[i][0],
                starts[j][1] - starts[i][1]
            )

            relative = (
                positions[j][0] - positions[i][0],
                positions[j][1] - positions[i][1]
            )

            passed = axis[0] * relative[0] + axis[1] * relative[1] < 0

            if (
                (passed and separation > 2 * self.args.radius + 0.06)
                or t - self.activated_at >= self.args.bias_timeout
            ):
                self.active = False
                self.history.clear()
                print(f't={t:.1f}s: pair {i+1}-{j+1} RIGHT-BIAS OFF')

        elif (
            len(self.history) >= 2
            and t - self.history[0][0] >= self.args.stall_window - 2 * DT
            and separation <= self.args.stall_distance
            and all(
                error > self.args.stall_min_goal_distance
                for error in errors
            )
        ):
            progress = self.history[0][1] - total_error

            if progress < self.args.stall_progress:
                self.active = True
                self.activated_at = t
                self.activations += 1
                print(f't={t:.1f}s: pair {i+1}-{j+1} STALL -> RIGHT-BIAS ON')

        return self.active


# --------------------------------------------------
# LANDING
# --------------------------------------------------

def land(scfs, telemetry):
    print('Landing...')

    heights = []

    for tel in telemetry:
        state = tel.read(required=False)
        heights.append(max(0.05, state['z'] if state else FLIGHT_HEIGHT))

    try:
        for step in range(80):
            fraction = min(1.0, step / 70)

            for scf, h0 in zip(scfs, heights):
                z = max(0.05, h0 * (1 - fraction))
                scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)

            time.sleep(DT)

    finally:
        for scf in scfs:
            scf.cf.commander.send_stop_setpoint()
            scf.cf.supervisor.send_arming_request(False)


# --------------------------------------------------
# SAVE RESULTS
# --------------------------------------------------

def save_results(output, rows, status):
    output.mkdir(parents=True, exist_ok=True)
    (output / 'status.txt').write_text(status + '\n')

    if not rows:
        return

    path = output / 'gazebo_orca_4_swap.csv'

    with path.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print('CSV:', path.resolve())
    print(f'Minimum separation: {min(row["min_distance"] for row in rows):.3f} m')


# --------------------------------------------------
# MAIN EXPERIMENT
# --------------------------------------------------

def run(args):
    cflib.crtp.init_drivers(enable_debug_driver=False)

    rows = []
    telemetry = []
    status = 'NOT_STARTED'
    airborne = False
    takeoff_done = False
    output = Path(args.output)

    uris = [f'udp://127.0.0.1:{19850 + i}' for i in range(N_DRONES)]

    with contextlib.ExitStack() as stack:
        scfs = [
            stack.enter_context(
                SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
            )
            for uri in uris
        ]

        for scf in scfs:
            scf.cf.param.set_value('stabilizer.controller', '1')

        time.sleep(0.5)

        try:
            # TELEMETRY INITIALIZATION

            telemetry = [
                Telemetry(scf.cf, str(i + 1))
                for i, scf in enumerate(scfs)
            ]

            deadline = time.monotonic() + 8.0

            while time.monotonic() < deadline:
                if all(tel.read(required=False) is not None for tel in telemetry):
                    break
                time.sleep(DT)

            states = [tel.read() for tel in telemetry]
            starts = [(s['x'], s['y']) for s in states]

            if any(not all(math.isfinite(v) for v in s.values()) for s in states):
                raise RuntimeError('Invalid telemetry')

            if any(abs(s['z'] - 0.02) > 0.20 for s in states):
                raise RuntimeError('Reset Gazebo: drones must start on the ground')

            min_initial = min(
                distance(starts[i], starts[j])
                for i, j in combinations(range(N_DRONES), 2)
            )

            if min_initial < args.initial_min_separation:
                raise RuntimeError('Initial drone separation too small')

            print('Starting XY coordinates:', starts)

            # TAKEOFF

            for scf in scfs:
                scf.cf.supervisor.send_arming_request(True)

            airborne = True
            print('Takeoff to', FLIGHT_HEIGHT, 'm')

            began = time.monotonic()
            stable_since = None

            while time.monotonic() - began < 10.0:
                states = [tel.read() for tel in telemetry]

                for scf in scfs:
                    scf.cf.commander.send_hover_setpoint(
                        0.0, 0.0, 0.0, FLIGHT_HEIGHT
                    )

                if all(abs(s['z'] - FLIGHT_HEIGHT) < 0.08 for s in states):
                    if stable_since is None:
                        stable_since = time.monotonic()

                    if time.monotonic() - stable_since >= 1.0:
                        break
                else:
                    stable_since = None

                time.sleep(DT)

            else:
                raise RuntimeError('TAKEOFF_FAILED')

            takeoff_done = True
            print('TAKEOFF_OK')

            if args.takeoff_only:
                status = 'TAKEOFF_ONLY'
                return

            # GOALS: CF1 <-> CF2, CF3 <-> CF4

            states = [tel.read() for tel in telemetry]
            starts = [(s['x'], s['y']) for s in states]
            goals = [starts[1], starts[0], starts[3], starts[2]]
            heights = [s['z'] for s in states]

            estimators = [VelocityEstimator() for _ in range(N_DRONES)]

            for estimator, start in zip(estimators, starts):
                estimator.reset(start)

            detectors = [
                PairDeadlockDetector(pair, args)
                for pair in PAIRS
            ]

            print('Goals:', goals)
            print('Strategy:', args.strategy)
            print('Preferred speed limits:', args.speeds)
            print(f'PD gains: Kp={args.kp}, Kd={args.kd}')
            print(
                f'Arrival: distance <= {args.goal_tolerance} m, '
                f'speed <= {args.arrival_speed} m/s'
            )

            status = 'TIMEOUT'
            t0 = time.monotonic()
            last_print = -1.0
            arrival_since = None

            # CONTROL LOOP

            while True:
                tick = time.monotonic()
                t = tick - t0

                if t >= args.duration:
                    break

                states = [tel.read() for tel in telemetry]

                if any(not all(math.isfinite(v) for v in s.values()) for s in states):
                    status = 'BAD_ESTIMATE'
                    break

                positions = [(s['x'], s['y']) for s in states]
                measured_velocities = [(s['vx'], s['vy']) for s in states]
                measured_speeds = [norm(v) for v in measured_velocities]
                errors = [distance(p, g) for p, g in zip(positions, goals)]

                pair_distances = {
                    (i, j): distance(positions[i], positions[j])
                    for i, j in combinations(range(N_DRONES), 2)
                }

                min_distance = min(pair_distances.values())

                # SAFETY CHECKS

                if min_distance < args.emergency_distance:
                    status = 'SAFETY_STOP'
                    break

                if any(
                    distance(p, start) > args.geofence
                    for p, start in zip(positions, starts)
                ):
                    status = 'GEOFENCE'
                    break

                if any(
                    abs(s['z'] - z) > MAX_VERTICAL_CHANGE
                    for s, z in zip(states, heights)
                ):
                    status = 'VERTICAL_CHANGE'
                    break

                # GOAL COMPLETION: DISTANCE + LOW VELOCITY

                arrived = [
                    errors[i] <= args.goal_tolerance
                    and measured_speeds[i] <= args.arrival_speed
                    for i in range(N_DRONES)
                ]

                if all(arrived):
                    if arrival_since is None:
                        arrival_since = t

                    if t - arrival_since >= args.arrival_hold:
                        status = 'GOALS_REACHED'
                        break
                else:
                    arrival_since = None

                # VELOCITY ESTIMATION FOR ORCA

                velocities = [
                    estimator.update(position)
                    for estimator, position in zip(estimators, positions)
                ]

                # NEW PD PREFERRED VELOCITIES

                preferred = [
                    toward_goal(
                        positions[i],
                        goals[i],
                        args.speeds[i],
                        measured_velocities[i],
                        args.kp,
                        args.kd
                    )
                    for i in range(N_DRONES)
                ]
                
                # EARLY RIGHT-BIAS
                # The drones start avoiding head-on symmetry immediately.
                # Right-bias is disabled after the two drones have passed.

                if args.strategy == 'deadlock-right-bias':
                    bias = []

                    for i, j in PAIRS:
                        start_axis = (
                            starts[j][0] - starts[i][0],
                            starts[j][1] - starts[i][1]
                        )

                        current_axis = (
                            positions[j][0] - positions[i][0],
                            positions[j][1] - positions[i][1]
                        )

                        passed = (
                            start_axis[0] * current_axis[0]
                            + start_axis[1] * current_axis[1]
                        ) < 0

                        bias.append(not passed)

                else:
                    bias = [False, False]

                for pair_id, (i, j) in enumerate(PAIRS):
                    if bias[pair_id]:
                        preferred[i] = turn_right(preferred[i], args.bias_angle)
                        preferred[j] = turn_right(preferred[j], args.bias_angle)

                # MULTI-AGENT ORCA

                safe = []

                try:
                    for i in range(N_DRONES):
                        constraints = []

                        for j in range(N_DRONES):
                            if i == j:
                                continue

                            point, direction, _ = compute_orca_constraint(
                                positions[i], velocities[i],
                                positions[j], velocities[j],
                                args.radius, TIME_HORIZON, DT
                            )

                            constraints.append((point, direction))
                        try:
                            velocity = solve_orca(
                                preferred[i],
                                constraints,
                                args.max_speed
                            )

                            safe.append(velocity)

                        except ValueError:
                            print(f"\nORCA INFEASIBLE FOR CF{i + 1}")
                            print(f"Time: {t:.3f} s")
                            print(f"Position: {positions[i]}")
                            print(f"Velocity: {velocities[i]}")
                            print(f"Preferred velocity: {preferred[i]}")
                            print(f"Max speed: {args.max_speed}")

                            for j in range(N_DRONES):
                                if j != i:
                                    print(
                                        f"Distance CF{i + 1}-CF{j + 1}: "
                                        f"{distance(positions[i], positions[j]):.4f} m"
                                    )

                            for k, (point, direction) in enumerate(constraints):
                                print(
                                    f"Constraint {k + 1}: "
                                    f"point={point}, direction={direction}"
                                )

                            raise

                except ValueError as exc:
                    status = 'ORCA_INFEASIBLE'
                    print(exc)
                    break

                # SEND COMMANDS

                for scf, v, state, height in zip(scfs, safe, states, heights):
                    vz = clamp(1.5 * (height - state['z']), -0.12, 0.12)

                    scf.cf.commander.send_velocity_world_setpoint(
                        v[0], v[1], vz, 0.0
                    )

                # SAVE DATA

                row = {
                    't': round(t, 4),
                    'strategy': args.strategy,
                    'bias_pair12': int(bias[0]),
                    'bias_pair34': int(bias[1]),
                    'min_distance': min_distance,
                    'bias_activations12': detectors[0].activations,
                    'bias_activations34': detectors[1].activations,
                    'all_arrived': int(all(arrived))
                }

                for (i, j), d in pair_distances.items():
                    row[f'distance_{i+1}{j+1}'] = d

                for i in range(N_DRONES):
                    k = i + 1

                    row.update({
                        f'x{k}': positions[i][0],
                        f'y{k}': positions[i][1],
                        f'z{k}': states[i]['z'],
                        f'goal_x{k}': goals[i][0],
                        f'goal_y{k}': goals[i][1],
                        f'goal_distance{k}': errors[i],
                        f'arrived{k}': int(arrived[i]),
                        f'pref_speed{k}': norm(preferred[i]),
                        f'pref_vx{k}': preferred[i][0],
                        f'pref_vy{k}': preferred[i][1],
                        f'orca_vx{k}': safe[i][0],
                        f'orca_vy{k}': safe[i][1],
                        f'orca_speed{k}': norm(safe[i]),
                        f'measured_vx{k}': measured_velocities[i][0],
                        f'measured_vy{k}': measured_velocities[i][1],
                        f'measured_speed{k}': measured_speeds[i],
                        f'est_vx{k}': velocities[i][0],
                        f'est_vy{k}': velocities[i][1]
                    })

                rows.append(row)

                # TERMINAL LOG

                if t - last_print >= 1.0:
                    print(
                        f't={t:5.1f}s | min_dist={min_distance:.3f}m | '
                        f'errors={[round(e, 2) for e in errors]} | '
                        f'speeds={[round(v, 2) for v in measured_speeds]} | '
                        f'arrived={arrived} | bias={bias}'
                    )
                    last_print = t

                wait = DT - (time.monotonic() - tick)

                if wait > 0:
                    time.sleep(wait)

            print('Final status:', status)

        except KeyboardInterrupt:
            status = 'INTERRUPTED'
            print('Interrupted')

        finally:
            save_results(output, rows, status)

            if airborne:
                if takeoff_done:
                    land(scfs, telemetry)
                else:
                    for scf in scfs:
                        scf.cf.commander.send_stop_setpoint()
                        scf.cf.supervisor.send_arming_request(False)

            for tel in telemetry:
                tel.close()


# --------------------------------------------------
# ARGUMENTS
# --------------------------------------------------

def positive(value):
    x = float(value)

    if not math.isfinite(x) or x <= 0:
        raise argparse.ArgumentTypeError('Must be finite and positive')

    return x


def main():
    parser = argparse.ArgumentParser(
        description='Four-drone swap with ORCA and PD goal controller'
    )

    parser.add_argument(
        '--strategy',
        choices=('orca', 'deadlock-right-bias'),
        default='orca'
    )

    for i in range(1, 5):
        parser.add_argument(f'--speed{i}', type=positive, default=0.10)

    parser.add_argument('--max-speed', type=positive, default=0.15)

    # PD goal controller
    parser.add_argument('--kp', type=positive, default=0.8)
    parser.add_argument('--kd', type=positive, default=1.2)

    # Goal arrival verification
    parser.add_argument('--goal-tolerance', type=positive, default=0.04)
    parser.add_argument('--arrival-speed', type=positive, default=0.02)
    parser.add_argument('--arrival-hold', type=positive, default=1.0)

    parser.add_argument('--radius', type=positive, default=0.25)
    parser.add_argument('--initial-min-separation', type=positive, default=0.8)
    parser.add_argument('--emergency-distance', type=positive, default=0.55)
    parser.add_argument('--geofence', type=positive, default=3.0)
    parser.add_argument('--duration', type=positive, default=120.0)
    parser.add_argument('--output', default='risultati_4_droni_pd')
    parser.add_argument('--takeoff-only', action='store_true')

    # Deadlock detection
    parser.add_argument('--stall-window', type=positive, default=4.0)
    parser.add_argument('--stall-progress', type=positive, default=0.02)
    parser.add_argument('--stall-distance', type=positive, default=0.90)
    parser.add_argument('--stall-min-goal-distance', type=positive, default=0.25)

    # Right-bias
    parser.add_argument('--bias-angle', type=positive, default=45.0)
    parser.add_argument('--bias-timeout', type=positive, default=15.0)

    args = parser.parse_args()
    args.speeds = [getattr(args, f'speed{i}') for i in range(1, 5)]
    
    print(f"args.emergency_distance: {args.emergency_distance}")
    print(f"args.radius: {args.radius}")

    if max(args.speeds) > args.max_speed:
        parser.error('Preferred speeds must be <= --max-speed')

    if args.initial_min_separation <= 2 * args.radius:
        parser.error('Initial separation must exceed combined ORCA radius')

    if args.emergency_distance >= 2 * args.radius:
        parser.error('Emergency distance must be smaller than combined ORCA radius')

    if args.bias_angle > 90:
        parser.error('Right-bias angle must be <= 90 degrees')

    run(args)


if __name__ == '__main__':
    main()
