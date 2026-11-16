#!/usr/bin/env python3
"""CrazySim/Gazebo Garden SITL: two Crazyflies exchange their initial XY positions.

ORCA calculation is unchanged: import compute_orca_constraint and
choose_orca_velocity from the supplied orca_lib.py.

IMPORTANT: This is for CrazySim's Gazebo backend (Gazebo <-> Crazyflie SITL
<-> CFLib UDP), NOT an arbitrary Gazebo vehicle with ROS topics.

Example:
  python sim_real_orca_gazebo.py --speed1 0.05 --speed2 0.03 \
      --max-speed 0.07 --duration 90 --output risultati_gazebo

Start two Crazyflies in Gazebo before running. Each must be simulated by
CrazySim SITL and available at udp://127.0.0.1:19850 and :19851.
"""

import argparse
import contextlib
import csv
import math
import sys
import time
from pathlib import Path
from threading import Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from orca_lib import compute_orca_constraint, choose_orca_velocity

DT = 0.05
TIME_HORIZON = 3.0
GOAL_TOLERANCE = 0.12
FLIGHT_HEIGHT = 0.50
MAX_VERTICAL_CHANGE = 0.20
MAX_TELEMETRY_AGE = 0.40
MAX_VALID_ESTIMATED_SPEED = 2.0
VELOCITY_ALPHA = 0.25


def norm(v):
    return math.hypot(*v)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def velocity_toward_goal(position, goal, speed):
    dx, dy = goal[0] - position[0], goal[1] - position[1]
    length = math.hypot(dx, dy)
    if length <= GOAL_TOLERANCE:
        return (0.0, 0.0)
    return speed * dx / length, speed * dy / length


class VelocityEstimator:
    """Position-difference velocity with exponential smoothing (same as source)."""

    def __init__(self, alpha=VELOCITY_ALPHA):
        self.alpha = alpha
        self.previous_position = None
        self.previous_time = None
        self.velocity = (0.0, 0.0)

    def reset(self, position):
        self.previous_position = position
        self.previous_time = time.monotonic()
        self.velocity = (0.0, 0.0)

    def update(self, position):
        now = time.monotonic()
        if self.previous_position is None:
            self.reset(position)
            return self.velocity
        dt = now - self.previous_time
        if dt > 0.0:
            raw = ((position[0] - self.previous_position[0]) / dt,
                   (position[1] - self.previous_position[1]) / dt)
            if norm(raw) <= MAX_VALID_ESTIMATED_SPEED:
                a = self.alpha
                self.velocity = (a * raw[0] + (1.0 - a) * self.velocity[0],
                                 a * raw[1] + (1.0 - a) * self.velocity[1])
            self.previous_position = position
            self.previous_time = now
        return self.velocity


class Telemetry:
    """CrazySim SITL firmware state estimate, valid for its Gazebo backend."""

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
            raise RuntimeError(f'{name}: stateEstimate.* missing from firmware log TOC')
        self.config.data_received_cb.add_callback(self.on_data)
        self.config.error_cb.add_callback(self.on_error)
        self.config.start()

    def on_data(self, timestamp, data, logconf):
        entry = {k: float(data['stateEstimate.' + k]) for k in
                 ('x', 'y', 'z', 'vx', 'vy', 'vz')}
        with self.lock:
            self.latest = entry
            self.received_at = time.monotonic()

    def on_error(self, logconf, message):
        print(f'Telemetry {self.name}: {message}', file=sys.stderr)

    def read(self, required=True):
        with self.lock:
            state = self.latest.copy() if self.latest else None
            age = time.monotonic() - self.received_at
        if required and (state is None or age > MAX_TELEMETRY_AGE):
            raise RuntimeError(f'{self.name}: missing/stale telemetry (age {age:.2f}s)')
        return state

    def close(self):
        self.config.stop()


def send_planar_commands(scfs, commands, states, z_targets):
    """ORCA returns world-frame XY velocities; altitude is controlled separately."""
    for scf, (vx, vy), state, z_ref in zip(scfs, commands, states, z_targets):
        vz = clamp(1.5 * (z_ref - state['z']), -0.12, 0.12)
        scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0.0)


def land_virtual_drones(scfs, telemetries):
    print('Landing virtual Crazyflies...')
    heights = []
    for tel in telemetries:
        state = tel.read(required=False)
        heights.append(max(0.05, state['z'] if state else FLIGHT_HEIGHT))
    try:
        for k in range(80):
            frac = min(1.0, k / 70.0)
            for scf, start_height in zip(scfs, heights):
                target_height = max(0.05, start_height * (1.0 - frac))
                scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, target_height)
            time.sleep(DT)
    finally:
        for scf in scfs:
            scf.cf.commander.send_stop_setpoint()
            scf.cf.supervisor.send_arming_request(False)


def save_csv(output, rows):
    if not rows:
        print('No navigation samples recorded.')
        return
    output.mkdir(parents=True, exist_ok=True)
    path = output / 'gazebo_orca_swap.csv'
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print('Saved:', path.resolve())
    print(f'Min recorded XY separation: {min(r["distance"] for r in rows):.3f} m')


def run(args):
    cflib.crtp.init_drivers(enable_debug_driver=False)
    output = Path(args.output)
    rows = []
    status = 'NOT_STARTED'
    takeoff_completed = False
    airborne = False
    telemetries = []

    with contextlib.ExitStack() as stack:
        scfs = [stack.enter_context(SyncCrazyflie(
            uri, cf=Crazyflie(rw_cache='./cache')))
            for uri in (args.uri1, args.uri2)]
        for scf in scfs:
            scf.cf.param.set_value('stabilizer.controller', '1')
        time.sleep(0.5)

        try:
            telemetries = [Telemetry(scf.cf, str(i + 1))
                           for i, scf in enumerate(scfs)]
            print('Waiting for both Gazebo/SITL position estimates...')
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                if all(tel.read(required=False) is not None for tel in telemetries):
                    break
                time.sleep(DT)
            states = [tel.read() for tel in telemetries]
            starts = [(st['x'], st['y']) for st in states]
            if any(not all(math.isfinite(value) for value in st.values()) for st in states):
                raise RuntimeError('Invalid initial telemetry')
            if dist(starts[0], starts[1]) < args.initial_min_separation:
                raise RuntimeError('Initial drone separation is too small')
            if any(abs(st['z'] - 0.02) > 0.20 for st in states):
                raise RuntimeError('Drones do not appear to be on the ground; reset Gazebo')
            print(f'Initial positions: CF1={starts[0]} CF2={starts[1]}')

            for scf in scfs:
                scf.cf.supervisor.send_arming_request(True)
            airborne = True
            print(f'Takeoff -> {FLIGHT_HEIGHT:.2f} m')
            begin = time.monotonic()
            stable_since = None
            while time.monotonic() - begin < 10.0:
                states = [tel.read() for tel in telemetries]
                for scf in scfs:
                    scf.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, FLIGHT_HEIGHT)
                if all(abs(st['z'] - FLIGHT_HEIGHT) < 0.08 for st in states):
                    if stable_since is None:
                        stable_since = time.monotonic()
                    if time.monotonic() - stable_since >= 1.0:
                        break
                else:
                    stable_since = None
                time.sleep(DT)
            else:
                raise RuntimeError('TAKEOFF_FAILED: check Gazebo SITL estimator/controller')
            takeoff_completed = True
            print('TAKEOFF_OK')
            if args.takeoff_only:
                status = 'TAKEOFF_ONLY'
                return

            states = [tel.read() for tel in telemetries]
            starts = [(st['x'], st['y']) for st in states]
            goals = [starts[1], starts[0]]  # EXACT swap of start positions.
            z_refs = [st['z'] for st in states]
            if dist(*starts) < 2.0 * args.radius:
                raise RuntimeError('Initial separation smaller than ORCA combined radius')
            if dist(*starts) > args.geofence:
                raise RuntimeError('Swap goal outside geofence: increase --geofence')
            estimators = [VelocityEstimator(), VelocityEstimator()]
            for estimator, start in zip(estimators, starts):
                estimator.reset(start)

            print(f'SWAP: CF1 -> {goals[0]} at {args.speed1:.3f} m/s preferred')
            print(f'      CF2 -> {goals[1]} at {args.speed2:.3f} m/s preferred')
            print(f'ORCA: radius={args.radius:.3f} m/agent, tau={TIME_HORIZON}s, '
                  f'max_speed={args.max_speed:.3f}m/s')
            status = 'TIMEOUT'
            t0 = time.monotonic()
            last_print = -1.0
            while True:
                cycle_start = time.monotonic()
                t = cycle_start - t0
                if t >= args.duration:
                    break
                states = [tel.read() for tel in telemetries]
                if any(not all(math.isfinite(value) for value in st.values()) for st in states):
                    status = 'BAD_ESTIMATE'
                    break
                positions = [(st['x'], st['y']) for st in states]
                distance = dist(*positions)
                if distance < args.emergency_distance:
                    status = f'SAFETY_STOP_{distance:.3f}'
                    break
                if any(dist(position, start) > args.geofence
                       for position, start in zip(positions, starts)):
                    status = 'GEOFENCE'
                    break
                if any(abs(st['z'] - z_ref) > MAX_VERTICAL_CHANGE
                       for st, z_ref in zip(states, z_refs)):
                    status = 'VERTICAL_CHANGE'
                    break
                goal_distances = [dist(p, g) for p, g in zip(positions, goals)]
                if all(d <= GOAL_TOLERANCE for d in goal_distances):
                    status = 'GOALS_REACHED'
                    break

                estimated_velocities = [e.update(p) for e, p in zip(estimators, positions)]
                preferred = [velocity_toward_goal(positions[0], goals[0], args.speed1),
                             velocity_toward_goal(positions[1], goals[1], args.speed2)]

                # Same reciprocal ORCA calculations as the input script.
                lp1, ld1, u1 = compute_orca_constraint(
                    positions[0], estimated_velocities[0],
                    positions[1], estimated_velocities[1],
                    args.radius, TIME_HORIZON, DT)
                lp2, ld2, u2 = compute_orca_constraint(
                    positions[1], estimated_velocities[1],
                    positions[0], estimated_velocities[0],
                    args.radius, TIME_HORIZON, DT)
                try:
                    v_orca1 = choose_orca_velocity(preferred[0], lp1, ld1, args.max_speed)
                    v_orca2 = choose_orca_velocity(preferred[1], lp2, ld2, args.max_speed)
                except ValueError as error:
                    status = 'ORCA_INFEASIBLE'
                    print(f'ORCA cannot find feasible velocities: {error}')
                    break
                send_planar_commands(scfs, [v_orca1, v_orca2], states, z_refs)
                rows.append({
                    't': round(t, 4), 'distance': distance,
                    'goal_distance1': goal_distances[0], 'goal_distance2': goal_distances[1],
                    'x1': positions[0][0], 'y1': positions[0][1], 'z1': states[0]['z'],
                    'x2': positions[1][0], 'y2': positions[1][1], 'z2': states[1]['z'],
                    'goal1x': goals[0][0], 'goal1y': goals[0][1],
                    'goal2x': goals[1][0], 'goal2y': goals[1][1],
                    'vx1': states[0]['vx'], 'vy1': states[0]['vy'],
                    'vx2': states[1]['vx'], 'vy2': states[1]['vy'],
                    'vel_est_1x': estimated_velocities[0][0],
                    'vel_est_1y': estimated_velocities[0][1],
                    'vel_est_2x': estimated_velocities[1][0],
                    'vel_est_2y': estimated_velocities[1][1],
                    'pref1x': preferred[0][0], 'pref1y': preferred[0][1],
                    'pref2x': preferred[1][0], 'pref2y': preferred[1][1],
                    'orca1x': v_orca1[0], 'orca1y': v_orca1[1],
                    'orca2x': v_orca2[0], 'orca2y': v_orca2[1],
                    'orca_u1': norm(u1), 'orca_u2': norm(u2),
                    'correction1': dist(preferred[0], v_orca1),
                    'correction2': dist(preferred[1], v_orca2),
                })
                if t - last_print >= 1.0:
                    print(f't={t:6.1f}s distance={distance:.3f}m '
                          f'g1={goal_distances[0]:.2f} g2={goal_distances[1]:.2f} '
                          f'ORCA1={tuple(round(x, 3) for x in v_orca1)} '
                          f'ORCA2={tuple(round(x, 3) for x in v_orca2)}')
                    last_print = t
                delay = DT - (time.monotonic() - cycle_start)
                if delay > 0.0:
                    time.sleep(delay)
            print('Experiment status:', status)
        except KeyboardInterrupt:
            status = 'INTERRUPTED'
            print('Interrupted; landing in simulation.')
        finally:
            # This finally block also saves partial logs after safety stops/errors.
            save_csv(output, rows)
            if airborne:
                if takeoff_completed:
                    land_virtual_drones(scfs, telemetries)
                else:
                    print('Failed takeoff: simulated motor stop only')
                    for scf in scfs:
                        scf.cf.commander.send_stop_setpoint()
                        scf.cf.supervisor.send_arming_request(False)
            for tel in telemetries:
                tel.close()
            output.mkdir(parents=True, exist_ok=True)
            (output / 'status.txt').write_text(status + '\n')


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def main():
    parser = argparse.ArgumentParser(description='CrazySim Gazebo: two-Crazyflie position swap with asymmetric preferred speeds')
    parser.add_argument('--uri1', default='udp://127.0.0.1:19850')
    parser.add_argument('--uri2', default='udp://127.0.0.1:19851')
    parser.add_argument('--speed1', type=positive, default=0.2, help='CF1 preferred speed, m/s')
    parser.add_argument('--speed2', type=positive, default=0.1, help='CF2 preferred speed, m/s')
    parser.add_argument('--max-speed', type=positive, default=0.07, help='Common ORCA maximum speed, m/s')
    parser.add_argument('--radius', type=positive, default=0.25, help='ORCA safety radius per drone, m')
    parser.add_argument('--emergency-distance', type=positive, default=0.40)
    parser.add_argument('--initial-min-separation', type=positive, default=0.60)
    parser.add_argument('--geofence', type=positive, default=1.20, help='Max XY distance from each initial position, m')
    parser.add_argument('--duration', type=positive, default=90.0, help='Maximum navigation time, seconds')
    parser.add_argument('--output', default='risultati_gazebo')
    parser.add_argument('--takeoff-only', action='store_true')
    args = parser.parse_args()
    if args.uri1 == args.uri2 or any(not uri.startswith('udp://127.0.0.1:') for uri in (args.uri1, args.uri2)):
        parser.error('Use TWO DISTINCT local CrazySim UDP URIs only; no hardware radio connections.')
    if max(args.speed1, args.speed2) > args.max_speed:
        parser.error('Both preferred speeds must be <= --max-speed')
    if args.emergency_distance >= 2.0 * args.radius:
        parser.error('--emergency-distance should be less than the ORCA combined radius (2*radius)')
    if args.initial_min_separation <= 2.0 * args.radius:
        parser.error('--initial-min-separation must exceed the combined ORCA radius')
    run(args)


if __name__ == '__main__':
    main()
