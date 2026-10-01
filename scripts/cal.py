import time
import math
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from natnet_tracker import NatNetTracker


# ============================================================
# CONFIGURATION
# ============================================================

URI = "radio://0/80/2M/E7E7E7E7E9"
MOTIVE_ID = 2

FLIGHT_HEIGHT = 0.5

# Very conservative calibration speed
SPEED = 0.10
MOVE_TIME = 2.0
STOP_TIME = 2.0

DT = 0.02


# ============================================================
# MOTIVE
# ============================================================

def create_tracker():

    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False,
    )

    tracker.connect()

    print("Connected to Motive.")

    return tracker


def get_motive_position(tracker):

    tracker.update()

    info = tracker.get_info_by_id(MOTIVE_ID)

    if info is None:
        return None

    x, y, z = info["position"]

    return x, y, z


# ============================================================
# AVERAGE MOTIVE POSITION
# ============================================================

def average_position(tracker, duration=0.5):

    xs = []
    ys = []
    zs = []

    start = time.monotonic()

    while time.monotonic() - start < duration:

        position = get_motive_position(tracker)

        if position is not None:

            x, y, z = position

            xs.append(x)
            ys.append(y)
            zs.append(z)

        time.sleep(DT)

    if not xs:
        return None

    return (
        sum(xs) / len(xs),
        sum(ys) / len(ys),
        sum(zs) / len(zs),
    )


# ============================================================
# STOP
# ============================================================

def stop_drone(cf, tracker, duration=STOP_TIME):

    start = time.monotonic()

    while time.monotonic() - start < duration:

        cf.commander.send_velocity_world_setpoint(
            0.0,
            0.0,
            0.0,
            0.0
        )

        tracker.update()

        time.sleep(DT)


# ============================================================
# MOVEMENT TEST
# ============================================================

def movement_test(cf, tracker, name, vx, vy):

    print()
    print("=" * 60)
    print(f"TEST {name}")
    print(f"Crazyflie command: vx={vx:+.3f}, vy={vy:+.3f}")
    print("=" * 60)

    # Let the drone stabilize
    stop_drone(cf, tracker)

    start_pos = average_position(
        tracker,
        duration=0.5
    )

    if start_pos is None:
        print("ERROR: Motive tracking lost.")
        return None

    sx, sy, sz = start_pos

    print(
        f"Motive START: "
        f"X={sx:+.3f} "
        f"Y={sy:+.3f} "
        f"Z={sz:+.3f}"
    )

    # --------------------------------------------------------
    # Move
    # --------------------------------------------------------

    start_time = time.monotonic()

    while time.monotonic() - start_time < MOVE_TIME:

        cf.commander.send_velocity_world_setpoint(
            vx,
            vy,
            0.0,
            0.0
        )

        tracker.update()

        time.sleep(DT)

    # Stop again
    stop_drone(cf, tracker)

    end_pos = average_position(
        tracker,
        duration=0.5
    )

    if end_pos is None:
        print("ERROR: Motive tracking lost.")
        return None

    ex, ey, ez = end_pos

    dx = ex - sx
    dy = ey - sy
    dz = ez - sz

    # Horizontal displacement in Motive
    horizontal_distance = math.sqrt(
        dx * dx + dz * dz
    )

    print(
        f"Motive END:   "
        f"X={ex:+.3f} "
        f"Y={ey:+.3f} "
        f"Z={ez:+.3f}"
    )

    print()
    print(
        f"Delta Motive: "
        f"dX={dx:+.3f} "
        f"dY={dy:+.3f} "
        f"dZ={dz:+.3f}"
    )

    print(
        f"Horizontal displacement X-Z: "
        f"{horizontal_distance:.3f} m"
    )

    # Unit direction in Motive X-Z
    if horizontal_distance > 0.01:

        ux = dx / horizontal_distance
        uz = dz / horizontal_distance

        print(
            f"Direction Motive X-Z: "
            f"({ux:+.3f}, {uz:+.3f})"
        )

    else:

        ux = None
        uz = None

        print(
            "WARNING: horizontal movement too small "
            "to determine direction."
        )

    return {
        "name": name,
        "dx": dx,
        "dy": dy,
        "dz": dz,
        "distance": horizontal_distance,
        "ux": ux,
        "uz": uz,
    }


# ============================================================
# MAIN
# ============================================================

def main():

    tracker = create_tracker()

    cflib.crtp.init_drivers()

    results = []

    try:

        print(f"Connecting to {URI}")

        with SyncCrazyflie(
            URI,
            cf=Crazyflie(rw_cache="./cache")
        ) as scf:

            print("Crazyflie connected.")

            # Make sure Motive sees the drone
            position = get_motive_position(tracker)

            if position is None:

                print(
                    f"ERROR: Motive rigid body "
                    f"{MOTIVE_ID} not found."
                )

                return

            print(
                f"Motive rigid body "
                f"{MOTIVE_ID} found."
            )

            print()
            print("Starting takeoff...")

            with MotionCommander(
                scf,
                default_height=FLIGHT_HEIGHT
            ):

                print("Takeoff completed.")

                # Stabilization
                stop_drone(
                    scf.cf,
                    tracker,
                    duration=2.0
                )

                # --------------------------------------------
                # CF +X
                # --------------------------------------------

                result = movement_test(
                    scf.cf,
                    tracker,
                    "+X",
                    +SPEED,
                    0.0
                )

                if result:
                    results.append(result)

                # --------------------------------------------
                # CF -X
                # --------------------------------------------

                result = movement_test(
                    scf.cf,
                    tracker,
                    "-X",
                    -SPEED,
                    0.0
                )

                if result:
                    results.append(result)

                # --------------------------------------------
                # CF +Y
                # --------------------------------------------

                result = movement_test(
                    scf.cf,
                    tracker,
                    "+Y",
                    0.0,
                    +SPEED
                )

                if result:
                    results.append(result)

                # --------------------------------------------
                # CF -Y
                # --------------------------------------------

                result = movement_test(
                    scf.cf,
                    tracker,
                    "-Y",
                    0.0,
                    -SPEED
                )

                if result:
                    results.append(result)

                stop_drone(
                    scf.cf,
                    tracker
                )

                print()
                print("Calibration completed.")
                print("Landing...")

        # ====================================================
        # SUMMARY
        # ====================================================

        print()
        print("=" * 60)
        print("CALIBRATION SUMMARY")
        print("=" * 60)

        for r in results:

            print(
                f"{r['name']:>3} CF  ->  "
                f"Motive dX={r['dx']:+.3f}  "
                f"dZ={r['dz']:+.3f}"
            )

            if r["ux"] is not None:

                print(
                    f"          unit direction X-Z = "
                    f"({r['ux']:+.3f}, "
                    f"{r['uz']:+.3f})"
                )

    except KeyboardInterrupt:

        print()
        print("Test interrupted.")

    finally:

        tracker.close()

        print("NatNet closed.")


if __name__ == "__main__":
    main()