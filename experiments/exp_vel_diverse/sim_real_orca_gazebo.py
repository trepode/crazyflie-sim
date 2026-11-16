#!/usr/bin/env python3

"""
Two CrazySim/Gazebo SITL drones exchanging positions.

Strategies:
- orca: standard ORCA
- deadlock-right-bias: standard ORCA with right-bias activated
  only when insufficient progress suggests a deadlock.

Requires orca_lib.py in the same directory.
"""

import argparse
import contextlib
import csv
import math
import sys
import time
from collections import deque
from pathlib import Path
from threading import Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from orca_lib import compute_orca_constraint, choose_orca_velocity


# --------------------------------------------------
# CONSTANTS
# --------------------------------------------------

DT = 0.05
TIME_HORIZON = 3.0
FLIGHT_HEIGHT = 0.50
GOAL_TOLERANCE = 0.12
MAX_TELEMETRY_AGE = 0.40
MAX_VERTICAL_CHANGE = 0.20


# --------------------------------------------------
# UTILITY FUNCTIONS
# --------------------------------------------------

def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def norm(v):
    return math.hypot(*v)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def toward_goal(position, goal, speed):
    dx = goal[0] - position[0]
    dy = goal[1] - position[1]
    d = math.hypot(dx, dy)

    if d <= GOAL_TOLERANCE:
        return (0.0, 0.0)

    return (speed * dx / d, speed * dy / d)


# --------------------------------------------------
# RIGHT BIAS
# --------------------------------------------------

def turn_right(v, degrees):
    """Rotate preferred velocity clockwise."""
    theta = math.radians(degrees)
    c = math.cos(theta)
    s = math.sin(theta)
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1])


# --------------------------------------------------
# VELOCITY ESTIMATOR
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

        self.config = LogConfig(name=f'GazeboOrca{name}', period_in_ms=50)

        for var in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
            self.config.add_variable('stateEstimate.' + var, 'float')

        cf.log.add_config(self.config)

        if not self.config.valid:
            raise RuntimeError(f'{name}: stateEstimate.* unavailable')

        self.config.data_received_cb.add_callback(self.on_data)
        self.config.error_cb.add_callback(self.on_error)
        self.config.start()

    def on_data(self, timestamp, data, logconf):
        values = {
            k: float(data['stateEstimate.' + k])
            for k in ('x', 'y', 'z', 'vx', 'vy', 'vz')
        }

        with self.lock:
            self.latest = values
            self.received_at = time.monotonic()

    def on_error(self, logconf, message):
        print(f'Telemetry {self.name}: {message}', file=sys.stderr)

    def read(self, required=True):
        with self.lock:
            state = self.latest.copy() if self.latest is not None else None
            age = time.monotonic() - self.received_at

        if required and (state is None or age > MAX_TELEMETRY_AGE):
            raise RuntimeError(f'Telemetry {self.name} missing/stale: {age:.2f}s')

        return state

    def close(self):
        self.config.stop()


# --------------------------------------------------
# DEADLOCK DETECTION + RIGHT BIAS
# --------------------------------------------------

class DeadlockRightBias:
    """
    Detect insufficient progress toward the goals.

    Activate right-bias when a possible deadlock is detected.
    ORCA remains responsible for computing safe velocities.
    """

    def __init__(self, args):
        self.args = args
        self.history = deque()
        self.active = False
        self.activated_at = None
        self.activations = 0

    def update(self, t, positions, goals, starts):
        errors = [distance(p, g) for p, g in zip(positions, goals)]
        total_error = sum(errors)
        separation = distance(*positions)

        self.history.append((t, total_error))

        while len(self.history) > 1 and t - self.history[0][0] > self.args.stall_window:
            self.history.popleft()

        # Right-bias currently active
        if self.active:
            start_axis = (
                starts[1][0] - starts[0][0],
                starts[1][1] - starts[0][1]
            )
            relative_now = (
                positions[1][0] - positions[0][0],
                positions[1][1] - positions[0][1]
            )

            passed = (
                relative_now[0] * start_axis[0]
                + relative_now[1] * start_axis[1]
            ) < 0.0

            safe_separation = 2.0 * self.args.radius + 0.06

            if passed and separation > safe_separation:
                self.active = False
                self.history.clear()
                print(f't={t:.1f}s: RIGHT BIAS OFF (drones passed)')

            elif t - self.activated_at >= self.args.bias_timeout:
                self.active = False
                self.history.clear()
                print(f't={t:.1f}s: RIGHT BIAS OFF (timeout)')

        # Detect a possible deadlock
        elif (
            len(self.history) >= 2
            and t - self.history[0][0] >= self.args.stall_window - 2 * DT
            and separation <= self.args.stall_distance
            and all(error > GOAL_TOLERANCE for error in errors)
        ):
            previous_error = self.history[0][1]
            progress = previous_error - total_error

            if progress < self.args.stall_progress:
                self.active = True
                self.activated_at = t
                self.activations += 1

                print(
                    f't={t:.1f}s: DEADLOCK SUSPECTED - RIGHT BIAS ON '
                    f'(progress={progress:.3f}m)'
                )

        return self.active


# --------------------------------------------------
# LANDING
# --------------------------------------------------

def land(scfs, telemetries):
    print('Landing...')

    heights = []
    for tel in telemetries:
        state = tel.read(required=False)
        heights.append(max(0.05, state['z'] if state else FLIGHT_HEIGHT))

    try:
        for k in range(80):
            fraction = min(1.0, k / 70.0)

            for scf, h0 in zip(scfs, heights):
                z = max(0.05, h0 * (1.0 - fraction))
                scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)

            time.sleep(DT)

    finally:
        for scf in scfs:
            scf.cf.commander.send_stop_setpoint()
            scf.cf.supervisor.send_arming_request(False)


# --------------------------------------------------
# SAVE RESULTS
# --------------------------------------------------

def save_csv(output, rows, status):
    output.mkdir(parents=True, exist_ok=True)
    (output / 'status.txt').write_text(status + '\n')

    if not rows:
        return

    path = output / 'gazebo_orca_swap.csv'

    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print('CSV:', path.resolve())
    min_distance = min(row['distance'] for row in rows)
    print('Minimum XY separation:', round(min_distance, 4), 'm')


# --------------------------------------------------
# MAIN EXPERIMENT
# --------------------------------------------------

def run(args):
    cflib.crtp.init_drivers(enable_debug_driver=False)

    rows = []
    telemetries = []
    output = Path(args.output)
    status = 'NOT_STARTED'
    airborne = False
    takeoff_completed = False

    with contextlib.ExitStack() as stack:
        scfs = [
            stack.enter_context(
                SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
            )
            for uri in (args.uri1, args.uri2)
        ]

        for scf in scfs:
            scf.cf.param.set_value('stabilizer.controller', '1')

        time.sleep(0.5)

        try:
            # ----------------------------------
            # TELEMETRY INITIALIZATION
            # ----------------------------------

            telemetries = [
                Telemetry(scf.cf, str(i + 1))
                for i, scf in enumerate(scfs)
            ]

            deadline = time.monotonic() + 8.0

            while time.monotonic() < deadline:
                if all(tel.read(required=False) is not None for tel in telemetries):
                    break
                time.sleep(DT)

            states = [tel.read() for tel in telemetries]
            starts = [(s['x'], s['y']) for s in states]

            if any(not all(math.isfinite(v) for v in s.values()) for s in states):
                raise RuntimeError('Invalid initial telemetry')

            if distance(*starts) < args.initial_min_separation:
                raise RuntimeError('Initial separation too small')

            if any(abs(s['z'] - 0.02) > 0.20 for s in states):
                raise RuntimeError('Reset Gazebo: drones must begin on ground')

            print('Initial positions:', starts)

            # ----------------------------------
            # TAKEOFF
            # ----------------------------------

            for scf in scfs:
                scf.cf.supervisor.send_arming_request(True)

            airborne = True
            print('Takeoff to', FLIGHT_HEIGHT, 'm')

            start_takeoff = time.monotonic()
            stable_since = None

            while time.monotonic() - start_takeoff < 10.0:
                states = [tel.read() for tel in telemetries]

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
                raise RuntimeError('Takeoff failed')

            takeoff_completed = True
            print('TAKEOFF_OK')

            if args.takeoff_only:
                status = 'TAKEOFF_ONLY'
                return

            # ----------------------------------
            # INITIAL POSITIONS AND GOALS
            # ----------------------------------

            states = [tel.read() for tel in telemetries]
            starts = [(s['x'], s['y']) for s in states]

            # Swap initial positions
            goals = [starts[1], starts[0]]
            z_refs = [s['z'] for s in states]

            if distance(*starts) < 2.0 * args.radius:
                raise RuntimeError('Combined ORCA radius exceeds initial distance')

            if distance(*starts) > args.geofence:
                raise RuntimeError('Increase --geofence')

            estimators = [VelocityEstimator(), VelocityEstimator()]

            for estimator, position in zip(estimators, starts):
                estimator.reset(position)

            detector = DeadlockRightBias(args)

            print('Strategy:', args.strategy)
            print(f'CF1: {args.speed1:.3f} m/s; CF2: {args.speed2:.3f} m/s')
            print('Goals:', goals)

            status = 'TIMEOUT'
            t0 = time.monotonic()
            last_print = -1.0

            # ----------------------------------
            # CONTROL LOOP
            # ----------------------------------

            while True:
                tic = time.monotonic()
                t = tic - t0

                if t >= args.duration:
                    break

                states = [tel.read() for tel in telemetries]

                if any(not all(math.isfinite(v) for v in s.values()) for s in states):
                    status = 'BAD_ESTIMATE'
                    break

                positions = [(s['x'], s['y']) for s in states]
                separation = distance(*positions)
                errors = [distance(p, g) for p, g in zip(positions, goals)]

                # --------------------------------
                # SAFETY CHECKS
                # --------------------------------

                if separation < args.emergency_distance:
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
                    for s, z in zip(states, z_refs)
                ):
                    status = 'VERTICAL_CHANGE'
                    break

                if all(error <= GOAL_TOLERANCE for error in errors):
                    status = 'GOALS_REACHED'
                    break

                # --------------------------------
                # VELOCITY ESTIMATION
                # --------------------------------

                velocities = [
                    estimator.update(position)
                    for estimator, position in zip(estimators, positions)
                ]

                # --------------------------------
                # DEADLOCK DETECTION
                # --------------------------------

                if args.strategy == 'deadlock-right-bias':
                    bias_on = detector.update(t, positions, goals, starts)
                else:
                    bias_on = False

                # --------------------------------
                # PREFERRED VELOCITIES
                # --------------------------------

                preferred = [
                    toward_goal(positions[0], goals[0], args.speed1),
                    toward_goal(positions[1], goals[1], args.speed2)
                ]

                if bias_on:
                    preferred = [
                        turn_right(v, args.bias_angle)
                        for v in preferred
                    ]

                # --------------------------------
                # ORCA CONSTRAINTS
                # --------------------------------

                lp1, ld1, u1 = compute_orca_constraint(
                    positions[0], velocities[0],
                    positions[1], velocities[1],
                    args.radius, TIME_HORIZON, DT
                )

                lp2, ld2, u2 = compute_orca_constraint(
                    positions[1], velocities[1],
                    positions[0], velocities[0],
                    args.radius, TIME_HORIZON, DT
                )

                # --------------------------------
                # SAFE VELOCITIES
                # --------------------------------

                try:
                    safe1 = choose_orca_velocity(
                        preferred[0], lp1, ld1, args.max_speed
                    )
                    safe2 = choose_orca_velocity(
                        preferred[1], lp2, ld2, args.max_speed
                    )
                except ValueError as exc:
                    status = 'ORCA_INFEASIBLE'
                    print('ORCA error:', exc)
                    break

                # --------------------------------
                # SEND COMMANDS
                # --------------------------------

                for scf, v, s, z in zip(scfs, (safe1, safe2), states, z_refs):
                    vz = clamp(1.5 * (z - s['z']), -0.12, 0.12)
                    scf.cf.commander.send_velocity_world_setpoint(
                        v[0], v[1], vz, 0.0
                    )

                # --------------------------------
                # SAVE DATA
                # --------------------------------

                rows.append({
                    't': round(t, 4),
                    'strategy': args.strategy,
                    'right_bias_active': int(bias_on),
                    'bias_activations': detector.activations,
                    'distance': separation,
                    'goal_distance1': errors[0],
                    'goal_distance2': errors[1],
                    'x1': positions[0][0],
                    'y1': positions[0][1],
                    'z1': states[0]['z'],
                    'x2': positions[1][0],
                    'y2': positions[1][1],
                    'z2': states[1]['z'],
                    'goal1x': goals[0][0],
                    'goal1y': goals[0][1],
                    'goal2x': goals[1][0],
                    'goal2y': goals[1][1],
                    'vx1': states[0]['vx'],
                    'vy1': states[0]['vy'],
                    'vx2': states[1]['vx'],
                    'vy2': states[1]['vy'],
                    'vel_est_1x': velocities[0][0],
                    'vel_est_1y': velocities[0][1],
                    'vel_est_2x': velocities[1][0],
                    'vel_est_2y': velocities[1][1],
                    'pref1x': preferred[0][0],
                    'pref1y': preferred[0][1],
                    'pref2x': preferred[1][0],
                    'pref2y': preferred[1][1],
                    'orca1x': safe1[0],
                    'orca1y': safe1[1],
                    'orca2x': safe2[0],
                    'orca2y': safe2[1],
                    'orca_u1': norm(u1),
                    'orca_u2': norm(u2),
                    'correction1': distance(preferred[0], safe1),
                    'correction2': distance(preferred[1], safe2)
                })

                # --------------------------------
                # TERMINAL LOG
                # --------------------------------

                if t - last_print >= 1.0:
                    mode = 'RIGHT-BIAS' if bias_on else 'ORCA'
                    print(
                        f't={t:5.1f}s | {mode} | dist={separation:.3f}m | '
                        f'goals={errors[0]:.2f},{errors[1]:.2f}m'
                    )
                    last_print = t

                wait = DT - (time.monotonic() - tic)

                if wait > 0:
                    time.sleep(wait)

            print('Final status:', status)

        except KeyboardInterrupt:
            status = 'INTERRUPTED'
            print('Interrupted')

        finally:
            save_csv(output, rows, status)

            if airborne:
                if takeoff_completed:
                    land(scfs, telemetries)
                else:
                    for scf in scfs:
                        scf.cf.commander.send_stop_setpoint()
                        scf.cf.supervisor.send_arming_request(False)

            for tel in telemetries:
                tel.close()


# --------------------------------------------------
# ARGUMENTS
# --------------------------------------------------

def positive(value):
    x = float(value)

    if not math.isfinite(x) or x <= 0:
        raise argparse.ArgumentTypeError('Must be a finite positive number')

    return x


def main():
    p = argparse.ArgumentParser(
        description='CrazySim Gazebo: ORCA with optional deadlock-only right-bias'
    )

    p.add_argument('--uri1', default='udp://127.0.0.1:19850')
    p.add_argument('--uri2', default='udp://127.0.0.1:19851')
    p.add_argument(
        '--strategy',
        choices=('orca', 'deadlock-right-bias'),
        default='deadlock-right-bias'
    )

    p.add_argument('--speed1', type=positive, default=0.05)
    p.add_argument('--speed2', type=positive, default=0.05)
    p.add_argument('--max-speed', type=positive, default=0.07)
    p.add_argument('--radius', type=positive, default=0.25)
    p.add_argument('--emergency-distance', type=positive, default=0.40)
    p.add_argument('--initial-min-separation', type=positive, default=0.60)
    p.add_argument('--geofence', type=positive, default=1.20)
    p.add_argument('--duration', type=positive, default=90.0)
    p.add_argument('--output', default='risultati_gazebo')
    p.add_argument('--takeoff-only', action='store_true')

    # Deadlock detection
    p.add_argument('--stall-window', type=positive, default=4.0)
    p.add_argument('--stall-progress', type=positive, default=0.02)
    p.add_argument('--stall-distance', type=positive, default=0.90)

    # Right-bias
    p.add_argument('--bias-angle', type=positive, default=45.0)
    p.add_argument('--bias-timeout', type=positive, default=15.0)

    args = p.parse_args()

    if (
        args.uri1 == args.uri2
        or any(
            not uri.startswith('udp://127.0.0.1:')
            for uri in (args.uri1, args.uri2)
        )
    ):
        p.error('Use two distinct local CrazySim SITL UDP URIs')

    if max(args.speed1, args.speed2) > args.max_speed:
        p.error('Preferred speeds must be <= max-speed')

    if args.emergency_distance >= 2.0 * args.radius:
        p.error('Emergency distance must be smaller than combined ORCA radius')

    if args.initial_min_separation <= 2.0 * args.radius:
        p.error('Initial minimum separation must exceed combined ORCA radius')

    if args.bias_angle > 90.0:
        p.error('Bias angle must be <= 90 degrees')

    run(args)


if __name__ == '__main__':
    main()
