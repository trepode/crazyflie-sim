"""Two Crazyflies, Motive X-Z tracking, ORCA swap goals.
Default DRY_RUN=True: validate tracking/ORCA before enabling motors.
Requires cflib, natnet_tracker.py, orca_lib.py in PYTHONPATH.
"""
import os
import warnings
import logging
warnings.filterwarnings('ignore')
logging.disable(logging.CRITICAL)
for name in ('cflib', 'natnet'):
    log = logging.getLogger(name)
    log.disabled = True
    log.propagate = False

import csv
import math
import time
from contextlib import ExitStack
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from natnet_tracker import NatNetTracker
from orca_lib import compute_orca_constraint, choose_orca_velocity

URI_1 = 'radio://0/80/2M/E7E7E7E7E9'
URI_2 = 'radio://0/80/2M/E7E7E7E7E7'  # verify actual second radio URI
MOTIVE_ID_1, MOTIVE_ID_2 = 2, 3
DRY_RUN = True
FLIGHT_HEIGHT = 0.5
PREFERRED_SPEED = 0.05
MAX_SPEED = 0.07
RADIUS = 0.20
TIME_HORIZON = 3.0
DT = 0.05
MAX_TEST_TIME = 20.0
GOAL_TOLERANCE = 0.12
EMERGENCY_DISTANCE = 0.42
MAX_VERTICAL_CHANGE = 0.18
MAX_DISTANCE_FROM_START = 1.2
MAX_MEASURED_SPEED = 0.7
TRACKING_TIMEOUT = 0.25
INITIAL_TRACKING_SECONDS = 1.0
MIN_START_SEPARATION = 1.0
MAX_START_SEPARATION = 1.8
LOG_FILE = 'real_orca_log.csv'


def norm(v):
    return math.hypot(*v)


def sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def horizontal(p):
    return p[0], p[2]


def cf_velocity(v_motive):
    # Calibration from isolated tests; verify sign before real flight.
    return -v_motive[1], -v_motive[0]


def preferred_velocity(p, goal):
    d = sub(goal, p)
    length = norm(d)
    if length <= GOAL_TOLERANCE:
        return (0.0, 0.0)
    return (PREFERRED_SPEED * d[0] / length,
            PREFERRED_SPEED * d[1] / length)


def orca_velocity(p, v, other_p, other_v, preferred):
    point, direction, u = compute_orca_constraint(
        p, v, other_p, other_v, RADIUS, TIME_HORIZON, DT)
    safe = choose_orca_velocity(preferred, point, direction, MAX_SPEED)
    if not all(math.isfinite(x) for x in (*safe, *u)):
        raise RuntimeError('ORCA returned non-finite velocity')
    if norm(safe) > MAX_SPEED + 1e-5:
        raise RuntimeError('ORCA output exceeds speed limit')
    return safe, u


def stop(cf):
    cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)


def send(cf, velocity):
    vx, vy = cf_velocity(velocity)
    cf.commander.send_velocity_world_setpoint(vx, vy, 0.0, 0.0)
    return vx, vy


def send_pair(cf1, cf2, v1, v2):
    # Sequential sends at 20 Hz, not simultaneous; keep test speed low.
    return send(cf1, v1), send(cf2, v2)


def takeoff(cf):
    cf.high_level_commander.takeoff(FLIGHT_HEIGHT, 3.0)


def land(cf):
    cf.high_level_commander.land(0.0, 3.0)


class FreshMotive:
    """Read only rigid bodies included in the newly received NatNet frame.

    natnet_tracker caches bodies indefinitely. The callback is augmented here
    to capture the IDs actually present in the most recent frame.
    """
    def __init__(self):
        self.tracker = NatNetTracker(server_ip='127.0.0.1',
                                    local_ip='127.0.0.1',
                                    use_multicast=False)
        self.frame_number = 0
        self.frame_time = None
        self.current_ids = set()
        self._original_callback = self.tracker._on_frame_received
        # Replace the callback registered in NatNetClient with our wrapper.
        handlers = self.tracker.client.on_data_frame_received_event.handlers
        handlers.remove(self._original_callback)
        handlers.append(self._on_frame)

    def _on_frame(self, frame):
        self._original_callback(frame)
        self.frame_number += 1
        self.frame_time = time.monotonic()
        self.current_ids = {rb.id_num for rb in frame.rigid_bodies}

    def connect(self):
        self.tracker.connect()

    def read(self):
        before = self.frame_number
        self.tracker.update()
        if self.frame_number == before:
            raise RuntimeError('No new Motive frame')
        if self.frame_time is None or time.monotonic() - self.frame_time > TRACKING_TIMEOUT:
            raise RuntimeError('Motive frame is stale')
        if not {MOTIVE_ID_1, MOTIVE_ID_2}.issubset(self.current_ids):
            raise RuntimeError('One or both rigid bodies absent in latest frame')
        a = self.tracker.get_info_by_id(MOTIVE_ID_1)
        b = self.tracker.get_info_by_id(MOTIVE_ID_2)
        if a is None or b is None:
            raise RuntimeError('Motive rigid body unavailable')
        p1, p2 = tuple(a['position']), tuple(b['position'])
        if not all(math.isfinite(x) for x in (*p1, *p2)):
            raise RuntimeError('Invalid Motive coordinates')
        return self.frame_time, p1, p2

    def close(self):
        self.tracker.close()


def stable_initial_positions(motive, cf1=None, cf2=None):
    start = time.monotonic()
    samples = []
    last_frame = None
    while time.monotonic() - start < 10.0:
        if cf1 is not None:
            stop(cf1)
            stop(cf2)
        try:
            stamp, p1, p2 = motive.read()
        except RuntimeError:
            samples.clear()
            time.sleep(DT)
            continue
        if last_frame is not None and stamp <= last_frame:
            samples.clear()
            continue
        last_frame = stamp
        samples.append((p1, p2))
        if len(samples) >= max(10, int(INITIAL_TRACKING_SECONDS / DT)):
            def mean(idx):
                return tuple(sum(s[idx][axis] for s in samples) / len(samples)
                             for axis in range(3))
            a, b = mean(0), mean(1)
            if max(norm(sub(horizontal(s[0]), horizontal(a))) for s in samples) > 0.08:
                samples.clear()
                continue
            if max(norm(sub(horizontal(s[1]), horizontal(b))) for s in samples) > 0.08:
                samples.clear()
                continue
            return a, b
        time.sleep(DT)
    raise RuntimeError('Could not acquire stable fresh Motive tracking')


def run_loop(motive, cf1, cf2, writer, start1, start2):
    goal1, goal2 = horizontal(start2), horizontal(start1)
    initial_distance = norm(sub(horizontal(start1), horizontal(start2)))
    if not MIN_START_SEPARATION <= initial_distance <= MAX_START_SEPARATION:
        raise RuntimeError(f'Unsafe start separation {initial_distance:.3f} m')
    previous = None
    filtered1, filtered2 = (0.0, 0.0), (0.0, 0.0)
    start_time = time.monotonic()
    last_print = 0.0
    while time.monotonic() - start_time < MAX_TEST_TIME:
        loop_start = time.monotonic()
        stamp, pos1, pos2 = motive.read()
        p1, p2 = horizontal(pos1), horizontal(pos2)
        separation = norm(sub(p1, p2))
        if separation < EMERGENCY_DISTANCE:
            raise RuntimeError(f'EMERGENCY: separation {separation:.3f} m')
        if abs(pos1[1] - start1[1]) > MAX_VERTICAL_CHANGE or abs(pos2[1] - start2[1]) > MAX_VERTICAL_CHANGE:
            raise RuntimeError('Unexpected vertical displacement')
        if norm(sub(p1, horizontal(start1))) > MAX_DISTANCE_FROM_START or norm(sub(p2, horizontal(start2))) > MAX_DISTANCE_FROM_START:
            raise RuntimeError('Geofence exceeded')
        if previous is not None:
            prev_t, prev_p1, prev_p2 = previous
            dt = stamp - prev_t
            if dt <= 0 or dt > TRACKING_TIMEOUT:
                raise RuntimeError('Bad Motive sample interval')
            raw1 = tuple(x / dt for x in sub(p1, prev_p1))
            raw2 = tuple(x / dt for x in sub(p2, prev_p2))
            if max(norm(raw1), norm(raw2)) > MAX_MEASURED_SPEED:
                raise RuntimeError('Tracking jump or excessive measured velocity')
            alpha = 0.25
            filtered1 = tuple(alpha * raw1[i] + (1 - alpha) * filtered1[i] for i in (0, 1))
            filtered2 = tuple(alpha * raw2[i] + (1 - alpha) * filtered2[i] for i in (0, 1))
        previous = stamp, p1, p2
        pref1, pref2 = preferred_velocity(p1, goal1), preferred_velocity(p2, goal2)
        if norm(pref1) == 0 and norm(pref2) == 0:
            print('\nBoth goals reached')
            break
        safe1, u1 = orca_velocity(p1, filtered1, p2, filtered2, pref1)
        safe2, u2 = orca_velocity(p2, filtered2, p1, filtered1, pref2)
        if cf1 is not None:
            cmd1, cmd2 = send_pair(cf1, cf2, safe1, safe2)
        else:
            cmd1, cmd2 = cf_velocity(safe1), cf_velocity(safe2)
        elapsed = loop_start - start_time
        writer.writerow([elapsed, *pos1, *pos2, *filtered1, *filtered2,
                         *pref1, *pref2, *safe1, *safe2, *u1, *u2,
                         *cmd1, *cmd2, separation])
        if elapsed - last_print >= 0.5:
            print(f't={elapsed:5.1f}s d={separation:.3f}m '
                  f'CF1 pref={tuple(round(x,3) for x in pref1)} safe={tuple(round(x,3) for x in safe1)} '
                  f'CF2 pref={tuple(round(x,3) for x in pref2)} safe={tuple(round(x,3) for x in safe2)}')
            last_print = elapsed
        remaining = DT - (time.monotonic() - loop_start)
        if remaining > 0:
            time.sleep(remaining)


def main():
    motive = FreshMotive()
    try:
        motive.connect()
        with open(LOG_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'cf1_X', 'cf1_Y', 'cf1_Z', 'cf2_X', 'cf2_Y', 'cf2_Z',
                             'v1_X', 'v1_Z', 'v2_X', 'v2_Z', 'pref1_X', 'pref1_Z',
                             'pref2_X', 'pref2_Z', 'safe1_X', 'safe1_Z',
                             'safe2_X', 'safe2_Z', 'u1_X', 'u1_Z', 'u2_X', 'u2_Z',
                             'cmd1_vx', 'cmd1_vy', 'cmd2_vx', 'cmd2_vy', 'separation'])
            if DRY_RUN:
                print('DRY RUN: NO motors / radio commands')
                start1, start2 = stable_initial_positions(motive)
                run_loop(motive, None, None, writer, start1, start2)
                return
            cflib.crtp.init_drivers(enable_debug_driver=False)
            with ExitStack() as stack:
                scf1 = stack.enter_context(SyncCrazyflie(URI_1, cf=Crazyflie(rw_cache='./cache')))
                scf2 = stack.enter_context(SyncCrazyflie(URI_2, cf=Crazyflie(rw_cache='./cache')))
                cf1, cf2 = scf1.cf, scf2.cf
                airborne = False
                try:
                    # This assumes a working position estimator / high-level commander
                    # for both drones. Validate takeoff separately first.
                    cf1.param.set_value('commander.enHighLevel', '1')
                    cf2.param.set_value('commander.enHighLevel', '1')
                    takeoff(cf1)
                    takeoff(cf2)
                    airborne = True
                    time.sleep(3.5)
                    start1, start2 = stable_initial_positions(motive, cf1, cf2)
                    print('Starting two-drone ORCA swap')
                    run_loop(motive, cf1, cf2, writer, start1, start2)
                finally:
                    if airborne:
                        try:
                            for _ in range(5):
                                stop(cf1)
                                stop(cf2)
                                time.sleep(DT)
                            land(cf1)
                            land(cf2)
                            time.sleep(3.5)
                            cf1.high_level_commander.stop()
                            cf2.high_level_commander.stop()
                        except Exception as e:
                            print('Landing error:', e)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    except Exception as e:
        print(f'ABORT: {type(e).__name__}: {e}')
    finally:
        try:
            motive.close()
        finally:
            print('NatNet closed; CSV:', LOG_FILE)


if __name__ == '__main__':
    main()
