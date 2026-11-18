"""
Three CrazySim/Gazebo SITL Crazyflies.

Three nominal straight trajectories intersect at one crossing point.

Expected launch layout (three drones only):
CF1 = (0, 0), CF2 = (1, 0), CF3 = (0, 1).
Their goal positions are (1, 1), (0, 1), (1, 0).
The measured starting positions, not hard-coded coordinates, define the goals.

Strategies:
- orca
- deadlock-right-bias

Features:
- Multi-agent ORCA
- PD goal controller with velocity damping
- Right-bias activated only after a measured pairwise deadlock
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
N_DRONES = 3


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

        self.config = LogConfig(name=f'Gazebo3{name}', period_in_ms=50)

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
# PAIRWISE DEADLOCK DETECTION (NO PROACTIVE BIAS)
# --------------------------------------------------

class PairDeadlockDetector:
    """Detects sustained lack of goal progress for two nearby, slow drones.

    The detector is deliberately based on observed telemetry, NOT on where
    the drones started or whether their nominal routes cross. Merely moving
    toward the common crossing point never triggers a right-bias.
    """

    def __init__(self, pair, args):
        self.pair = pair
        self.args = args
        self.history = deque()
        self.active = False
        self.activated_at = None
        self.last_deactivated_at = float('-inf')
        self.activations = 0

    def update(self, t, positions, goals, measured_speeds, starts):
        i, j = self.pair
        separation = distance(positions[i], positions[j])
        errors = (distance(positions[i], goals[i]),
                  distance(positions[j], goals[j]))
        self.history.append((t, errors[0] + errors[1]))

        while len(self.history) > 1 and t - self.history[0][0] > self.args.stall_window:
            self.history.popleft()

        if self.active:
            # Stop bias after the pair has crossed its original relative axis
            # and gained clearance, or after a bounded intervention interval.
            axis = (starts[j][0] - starts[i][0], starts[j][1] - starts[i][1])
            relative = (positions[j][0] - positions[i][0],
                        positions[j][1] - positions[i][1])
            passed = axis[0] * relative[0] + axis[1] * relative[1] < 0.0
            cleared = passed and separation > 2 * self.args.radius + 0.10
            timed_out = t - self.activated_at >= self.args.bias_timeout
            if cleared or timed_out:
                self.active = False
                self.last_deactivated_at = t
                self.history.clear()
                print(f't={t:.2f}s: pair {i+1}-{j+1} RIGHT-BIAS OFF '
                      f'({"passed" if cleared else "timeout"})')

        else:
            enough_history = (
                len(self.history) >= 2
                and t - self.history[0][0] >= self.args.stall_window - 2 * DT
            )
            if enough_history:
                progress = self.history[0][1] - sum(errors)
                slow = all(measured_speeds[k] <= self.args.stall_speed for k in (i, j))
                far_from_goal = all(e > self.args.stall_min_goal_distance for e in errors)
                near = separation <= self.args.stall_distance
                cooled_down = t - self.last_deactivated_at >= self.args.bias_cooldown

                if (progress < self.args.stall_progress and slow
                        and far_from_goal and near and cooled_down):
                    self.active = True
                    self.activated_at = t
                    self.activations += 1
                    print(f't={t:.2f}s: pair {i+1}-{j+1} DEADLOCK -> '
                          f'RIGHT-BIAS ON (progress={progress:.3f}m, '
                          f'distance={separation:.3f}m)')

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

    path = output / 'gazebo_orca_3_crossing.csv'

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

            # THREE STRAIGHT PATHS THROUGH THE SAME CROSSING POINT.
            # For the expected right-angle triangle layout, this is the
            # circumcenter: (starts[1] + starts[2]) / 2.
            # Reflect each start across the crossing point to define its goal.
            states = [tel.read() for tel in telemetry]
            starts = [(s['x'], s['y']) for s in states]
            center = (
                (starts[1][0] + starts[2][0]) / 2.0,
                (starts[1][1] + starts[2][1]) / 2.0,
            )
            goals = [
                (2.0 * center[0] - p[0], 2.0 * center[1] - p[1])
                for p in starts
            ]
            heights = [s['z'] for s in states]
            # Require the expected L-shaped square-corner layout so every
            # drone is equally far from the crossing center. This makes the
            # nominal collision simultaneous for equal preferred speeds.
            lengths = [distance(p, center) for p in starts]
            if min(lengths) < 0.05 or max(lengths) - min(lengths) > args.layout_tolerance:
                raise RuntimeError(
                    'Three-drone layout is not the expected square-corner '
                    f'configuration (center={center}, distances={lengths}). '
                    'Use start positions (0,0), (1,0), (0,1).'
                )

            detectors = [
                PairDeadlockDetector(pair, args)
                for pair in combinations(range(N_DRONES), 2)
            ]

            estimators = [VelocityEstimator() for _ in range(N_DRONES)]

            for estimator, start in zip(estimators, starts):
                estimator.reset(start)

            print('Crossing point:', center)
            print('Nominal routes:', list(zip(starts, goals)))
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

                completed = False
                if all(arrived):
                    if arrival_since is None:
                        arrival_since = t

                    if t - arrival_since >= args.arrival_hold:
                        status = 'GOALS_REACHED'
                        completed = True
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
                
                # Activate right-bias ONLY when sustained pairwise deadlock
                # is detected from measured progress and measured speeds.
                bias = [False] * N_DRONES
                if args.strategy == 'deadlock-right-bias' and not completed:
                    for detector in detectors:
                        if detector.update(t, positions, goals, measured_speeds, starts):
                            i, j = detector.pair
                            bias[i] = True
                            bias[j] = True
                    for i in range(N_DRONES):
                        if bias[i]:
                            preferred[i] = turn_right(preferred[i], args.bias_angle)

                # After the arrival condition has been held long enough,
                # log the final measured state but stop issuing new velocities.
                if completed:
                    safe = [(0.0, 0.0)] * N_DRONES
                else:
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

                if not completed:
                    for scf, v, state, height in zip(scfs, safe, states, heights):
                        vz = clamp(1.5 * (height - state['z']), -0.12, 0.12)
                        scf.cf.commander.send_velocity_world_setpoint(
                            v[0], v[1], vz, 0.0
                        )

                # SAVE DATA

                row = {
                    't': round(t, 4),
                    'strategy': args.strategy,
                    'crossing_x': center[0],
                    'crossing_y': center[1],
                    'bias1': int(bias[0]),
                    'bias2': int(bias[1]),
                    'bias3': int(bias[2]),
                    'deadlock_activations12': detectors[0].activations,
                    'deadlock_activations13': detectors[1].activations,
                    'deadlock_activations23': detectors[2].activations,
                    'min_distance': min_distance,
                    'all_arrived': int(all(arrived)),
                    'goals_reached': int(completed)
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
                        f'crossing_distance{k}': distance(positions[i], center),
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

                if completed:
                    print(f'GOALS_REACHED at t={t:.3f}s')
                    for i in range(N_DRONES):
                        print(f'CF{i+1}: position={positions[i]}, '
                              f'goal={goals[i]}, error={errors[i]*100:.2f}cm, '
                              f'speed={measured_speeds[i]:.4f}m/s')
                    break

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
        description='Three-drone common-crossing test with ORCA and PD goal controller'
    )

    parser.add_argument(
        '--strategy',
        choices=('orca', 'deadlock-right-bias'),
        default='orca'
    )

    for i in range(1, 4):
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
    parser.add_argument('--output', default='risultati_3_droni_crossing')
    parser.add_argument('--layout-tolerance', type=positive, default=0.12)
    parser.add_argument('--takeoff-only', action='store_true')

    # Deadlock: requires sustained low progress AND low measured speeds
    # in a pair whose separation is below --stall-distance.
    parser.add_argument('--stall-window', type=positive, default=3.0)
    parser.add_argument('--stall-progress', type=positive, default=0.04)
    parser.add_argument('--stall-distance', type=positive, default=0.90)
    parser.add_argument('--stall-speed', type=positive, default=0.05)
    parser.add_argument('--stall-min-goal-distance', type=positive, default=0.15)

    # Right-bias only after deadlock, with bounded duration and cooldown
    parser.add_argument('--bias-angle', type=positive, default=45.0)
    parser.add_argument('--bias-timeout', type=positive, default=8.0)
    parser.add_argument('--bias-cooldown', type=positive, default=3.0)

    args = parser.parse_args()
    args.speeds = [getattr(args, f'speed{i}') for i in range(1, N_DRONES + 1)]
    
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
