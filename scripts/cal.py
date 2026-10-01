import time
import math
import warnings
import logging

# ============================================================
# SUPPRESS WARNINGS
# ============================================================

warnings.filterwarnings("ignore")
logging.disable(logging.WARNING)
warnings.filterwarnings("ignore", category=DeprecationWarning)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from natnet_tracker import NatNetTracker


# ============================================================
# CONFIGURATION
# ============================================================

URI = "radio://0/80/2M/E7E7E7E7E7"
MOTIVE_ID = 2

FLIGHT_HEIGHT = 0.5

# Small calibration velocity
SPEED = 0.06

# Each movement lasts 1 second
MOVE_TIME = 1.0

# Wait between movements
STABILIZE_TIME = 2.0

# Loop period
DT = 0.02

# Motive acquisition
REQUIRED_VALID_SAMPLES = 20
MOTIVE_TIMEOUT = 10.0


# ============================================================
# MOTIVE
# ============================================================

def create_tracker():

    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False
    )

    tracker.connect()

    print("Connected to Motive.")

    return tracker


def get_motive_position(tracker):
    """
    Read current rigid-body position from Motive.

    Returns:
        (X_motive, Y_motive, Z_motive)

    In our setup:
        Motive Y = vertical
        Motive X-Z = horizontal plane
    """

    tracker.update()

    info = tracker.get_info_by_id(MOTIVE_ID)

    if info is None:
        return None

    x, y, z = info["position"]

    return x, y, z


# ============================================================
# STOP CRAZYFLIE
# ============================================================

def send_stop(cf):

    cf.commander.send_velocity_world_setpoint(
        0.0,
        0.0,
        0.0,
        0.0
    )


# ============================================================
# WAIT FOR MOTIVE AFTER TAKEOFF
# ============================================================

def wait_for_motive(tracker, cf):

    print()
    print("Waiting for stable Motive tracking...")

    positions = []

    start_time = time.monotonic()

    while (
        time.monotonic() - start_time
        < MOTIVE_TIMEOUT
    ):

        send_stop(cf)

        position = get_motive_position(tracker)

        if position is None:

            positions = []

        else:

            positions.append(position)

            if len(positions) >= REQUIRED_VALID_SAMPLES:

                # Average the valid samples
                x = sum(
                    p[0] for p in positions
                ) / len(positions)

                y = sum(
                    p[1] for p in positions
                ) / len(positions)

                z = sum(
                    p[2] for p in positions
                ) / len(positions)

                print(
                    f"Motive stable: "
                    f"{REQUIRED_VALID_SAMPLES} samples."
                )

                return x, y, z

        time.sleep(DT)

    return None


# ============================================================
# STABILIZE DRONE
# ============================================================

def stabilize(cf, tracker, duration):

    start_time = time.monotonic()

    while (
        time.monotonic() - start_time
        < duration
    ):

        send_stop(cf)

        # Keep NatNet updated
        tracker.update()

        time.sleep(DT)


# ============================================================
# AVERAGE MOTIVE POSITION
# ============================================================

def average_position(
    tracker,
    duration=0.3
):
    """
    Average Motive measurements for a short period.

    This gives a cleaner starting position than using
    only one measurement.
    """

    positions = []

    start_time = time.monotonic()

    while (
        time.monotonic() - start_time
        < duration
    ):

        position = get_motive_position(
            tracker
        )

        if position is not None:

            positions.append(
                position
            )

        time.sleep(DT)

    if len(positions) == 0:
        return None

    x = sum(
        p[0] for p in positions
    ) / len(positions)

    y = sum(
        p[1] for p in positions
    ) / len(positions)

    z = sum(
        p[2] for p in positions
    ) / len(positions)

    return x, y, z


# ============================================================
# SINGLE CALIBRATION TEST
# ============================================================

def calibration_movement(
    cf,
    tracker,
    name,
    vx_cf,
    vy_cf
):

    print()
    print("=" * 65)
    print(f"TEST {name}")
    print("=" * 65)

    print(
        f"Crazyflie command: "
        f"vx={vx_cf:+.3f} m/s   "
        f"vy={vy_cf:+.3f} m/s"
    )

    # --------------------------------------------------------
    # 1. STABILIZE
    # --------------------------------------------------------

    print("Stabilizing...")

    stabilize(
        cf,
        tracker,
        STABILIZE_TIME
    )

    # --------------------------------------------------------
    # 2. MEASURE START POSITION
    # --------------------------------------------------------

    start_position = average_position(
        tracker
    )

    if start_position is None:

        print(
            "ERROR: Motive position unavailable."
        )

        send_stop(cf)

        return None

    sx, sy, sz = start_position

    print(
        f"Motive START: "
        f"X={sx:+.3f} "
        f"Y={sy:+.3f} "
        f"Z={sz:+.3f}"
    )

    # --------------------------------------------------------
    # 3. MOVE
    # --------------------------------------------------------

    movement_start = time.monotonic()

    last_position = start_position

    while (
        time.monotonic() - movement_start
        < MOVE_TIME
    ):

        # Send velocity in CRAZYFLIE world frame
        cf.commander.send_velocity_world_setpoint(
            vx_cf,
            vy_cf,
            0.0,
            0.0
        )

        # Measure position with MOTIVE
        position = get_motive_position(
            tracker
        )

        if position is not None:

            last_position = position

        time.sleep(DT)

    # --------------------------------------------------------
    # 4. SAVE END POSITION BEFORE STOPPING
    # --------------------------------------------------------

    ex, ey, ez = last_position

    # --------------------------------------------------------
    # 5. STOP
    # --------------------------------------------------------

    send_stop(cf)

    # --------------------------------------------------------
    # 6. CALCULATE DISPLACEMENT
    # --------------------------------------------------------

    dx = ex - sx
    dy = ey - sy
    dz = ez - sz

    # Motive horizontal plane is X-Z
    horizontal_distance = math.sqrt(
        dx ** 2 +
        dz ** 2
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

    # --------------------------------------------------------
    # 7. NORMALIZE HORIZONTAL DIRECTION
    # --------------------------------------------------------

    if horizontal_distance > 0.01:

        dir_x = (
            dx / horizontal_distance
        )

        dir_z = (
            dz / horizontal_distance
        )

        print(
            f"Direction Motive X-Z: "
            f"({dir_x:+.3f}, "
            f"{dir_z:+.3f})"
        )

    else:

        dir_x = 0.0
        dir_z = 0.0

        print(
            "WARNING: horizontal movement "
            "too small for calibration."
        )

    # --------------------------------------------------------
    # RETURN RESULT
    # --------------------------------------------------------

    return {

        "name": name,

        "vx_cf": vx_cf,
        "vy_cf": vy_cf,

        "dx": dx,
        "dy": dy,
        "dz": dz,

        "distance":
            horizontal_distance,

        "dir_x": dir_x,
        "dir_z": dir_z
    }


# ============================================================
# PRINT FINAL RESULTS
# ============================================================

def print_results(results):

    print()
    print()
    print("=" * 76)
    print("CALIBRATION RESULTS")
    print("=" * 76)

    print(
        f"{'CF command':<12}"
        f"{'dX Motive':>14}"
        f"{'dZ Motive':>14}"
        f"{'dir X':>14}"
        f"{'dir Z':>14}"
    )

    print("-" * 76)

    for r in results:

        print(
            f"{r['name']:<12}"
            f"{r['dx']:>+14.3f}"
            f"{r['dz']:>+14.3f}"
            f"{r['dir_x']:>+14.3f}"
            f"{r['dir_z']:>+14.3f}"
        )

    print("=" * 76)


# ============================================================
# MAIN
# ============================================================

def main():

    tracker = None

    try:

        # ====================================================
        # CONNECT MOTIVE
        # ====================================================

        tracker = create_tracker()

        # ====================================================
        # INITIALIZE CRAZYFLIE
        # ====================================================

        cflib.crtp.init_drivers()

        print(
            f"Connecting to {URI}"
        )

        # ====================================================
        # CONNECT RADIO
        # ====================================================

        with SyncCrazyflie(
            URI,
            cf=Crazyflie(
                rw_cache="./cache"
            )
        ) as scf:

            print(
                "Crazyflie connected."
            )

            # =================================================
            # TAKEOFF
            # =================================================

            print("Taking off...")

            with MotionCommander(
                scf,
                default_height=FLIGHT_HEIGHT
            ):

                print(
                    "Takeoff complete."
                )

                # =============================================
                # WAIT FOR MOTIVE AFTER TAKEOFF
                # =============================================

                initial_position = (
                    wait_for_motive(
                        tracker,
                        scf.cf
                    )
                )

                if initial_position is None:

                    print(
                        "Motive did not acquire "
                        "the drone."
                    )

                    print(
                        "Landing."
                    )

                    return

                print()

                print(
                    "Initial Motive position:"
                )

                print(
                    f"X={initial_position[0]:+.3f} "
                    f"Y={initial_position[1]:+.3f} "
                    f"Z={initial_position[2]:+.3f}"
                )

                print()
                print(
                    "Starting calibration."
                )

                print(
                    "CTRL+C to stop and land."
                )

                # =============================================
                # CALIBRATION TESTS
                # =============================================

                tests = [

                    (
                        "+X",
                        +SPEED,
                        0.0
                    ),

                    (
                        "-X",
                        -SPEED,
                        0.0
                    ),

                    (
                        "+Y",
                        0.0,
                        +SPEED
                    ),

                    (
                        "-Y",
                        0.0,
                        -SPEED
                    )
                ]

                results = []

                for (
                    name,
                    vx,
                    vy
                ) in tests:

                    result = (
                        calibration_movement(
                            scf.cf,
                            tracker,
                            name,
                            vx,
                            vy
                        )
                    )

                    if result is not None:

                        results.append(
                            result
                        )

                # =============================================
                # STOP
                # =============================================

                send_stop(
                    scf.cf
                )

                # =============================================
                # RESULTS
                # =============================================

                print_results(
                    results
                )

                print()
                print(
                    "Calibration complete."
                )

                print(
                    "Landing..."
                )

            # MotionCommander exits -> landing

            print(
                "MotionCommander closed."
            )

        # SyncCrazyflie exits -> radio closes

        print(
            "Crazyflie radio connection closed."
        )


    # ========================================================
    # CTRL+C
    # ========================================================

    except KeyboardInterrupt:

        print()
        print(
            "CTRL+C received -> "
            "stopping and landing."
        )


    # ========================================================
    # ALWAYS CLOSE NATNET
    # ========================================================

    finally:

        if tracker is not None:

            try:

                tracker.close()

                print(
                    "NatNet connection closed."
                )

            except Exception:
                pass

        print(
            "Program terminated."
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()