import time
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from natnet_tracker import NatNetTracker


# --------------------------------------------------
# PARAMETERS
# --------------------------------------------------

URI = "radio://0/80/2M/E7E7E7E7E9"
MOTIVE_ID = 3

FLIGHT_HEIGHT = 0.5

SPEED = 0.05       # 5 cm/s
MOVE_TIME = 1.5    # seconds
STOP_TIME = 1.0    # pause between tests
DT = 0.05


# --------------------------------------------------
# NATNET
# --------------------------------------------------

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

    return info["position"]


# --------------------------------------------------
# STOP DRONE
# --------------------------------------------------

def stop_drone(cf, tracker, duration=STOP_TIME):

    start_time = time.monotonic()

    while time.monotonic() - start_time < duration:

        cf.commander.send_velocity_world_setpoint(
            0.0,
            0.0,
            0.0,
            0.0
        )

        tracker.update()

        time.sleep(DT)


# --------------------------------------------------
# MOVEMENT TEST
# --------------------------------------------------

def run_movement_test(cf, tracker, name, vx, vy):

    print()
    print("=" * 50)
    print(f"TEST {name}")
    print(f"Command: vx={vx:.2f}, vy={vy:.2f}")
    print("=" * 50)

    # Make sure drone is stationary
    stop_drone(cf, tracker)

    # Position before movement
    start_position = get_motive_position(tracker)

    if start_position is None:
        print("ERROR: Motive rigid body not found.")
        return

    x_start, y_start, z_start = start_position

    print(
        f"Motive start: "
        f"x={x_start:.3f}, "
        f"y={y_start:.3f}, "
        f"z={z_start:.3f}"
    )

    # --------------------------------------------------
    # MOVE
    # --------------------------------------------------

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

    # --------------------------------------------------
    # STOP
    # --------------------------------------------------

    stop_drone(cf, tracker)

    # Position after movement
    end_position = get_motive_position(tracker)

    if end_position is None:
        print("ERROR: Motive rigid body not found.")
        return

    x_end, y_end, z_end = end_position

    dx = x_end - x_start
    dy = y_end - y_start
    dz = z_end - z_start

    print(
        f"Motive end:   "
        f"x={x_end:.3f}, "
        f"y={y_end:.3f}, "
        f"z={z_end:.3f}"
    )

    print(
        f"Delta Motive: "
        f"dx={dx:+.3f} m, "
        f"dy={dy:+.3f} m, "
        f"dz={dz:+.3f} m"
    )


# --------------------------------------------------
# MAIN
# --------------------------------------------------

def main():

    tracker = create_tracker()

    cflib.crtp.init_drivers()

    try:

        print(f"Connecting to {URI}")

        with SyncCrazyflie(
            URI,
            cf=Crazyflie(rw_cache="./cache")
        ) as scf:

            print("Crazyflie connected.")

            with MotionCommander(
                scf,
                default_height=FLIGHT_HEIGHT
            ):

                print("Takeoff completed.")

                # Give the drone time to stabilize
                time.sleep(3.0)

                # ------------------------------------------
                # TEST +X
                # ------------------------------------------

                run_movement_test(
                    scf.cf,
                    tracker,
                    "+X",
                    SPEED,
                    0.0
                )

                # ------------------------------------------
                # TEST -X
                # ------------------------------------------

                run_movement_test(
                    scf.cf,
                    tracker,
                    "-X",
                    -SPEED,
                    0.0
                )

                # ------------------------------------------
                # TEST +Y
                # ------------------------------------------

                run_movement_test(
                    scf.cf,
                    tracker,
                    "+Y",
                    0.0,
                    SPEED
                )

                # ------------------------------------------
                # TEST -Y
                # ------------------------------------------

                run_movement_test(
                    scf.cf,
                    tracker,
                    "-Y",
                    0.0,
                    -SPEED
                )

                # Final stop
                stop_drone(
                    scf.cf,
                    tracker
                )

                print()
                print("All movement tests completed.")
                print("Landing...")

    except KeyboardInterrupt:

        print("\nTest interrupted.")

    finally:

        tracker.close()
        print("NatNet closed.")


if __name__ == "__main__":
    main()