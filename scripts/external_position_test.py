import time
import threading
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)

import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

from natnet_tracker import NatNetTracker


# --------------------------------------------------
# PARAMETERS
# --------------------------------------------------

URI = "radio://0/80/2M/E7E7E7E7E9"
MOTIVE_ID = 3

FLIGHT_HEIGHT = 0.5

MOCAP_DT = 0.02       # 50 Hz
PRINT_DT = 0.25


# --------------------------------------------------
# CRAZYFLIE STATE
# --------------------------------------------------

cf_state = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

state_lock = threading.Lock()


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


# --------------------------------------------------
# CRAZYFLIE LOGGING
# --------------------------------------------------

def log_callback(timestamp, data, logconf):

    with state_lock:

        cf_state["x"] = data["stateEstimate.x"]
        cf_state["y"] = data["stateEstimate.y"]
        cf_state["z"] = data["stateEstimate.z"]


def start_logging(scf):

    log_conf = LogConfig(
        name="State",
        period_in_ms=100
    )

    log_conf.add_variable(
        "stateEstimate.x",
        "float"
    )

    log_conf.add_variable(
        "stateEstimate.y",
        "float"
    )

    log_conf.add_variable(
        "stateEstimate.z",
        "float"
    )

    scf.cf.log.add_config(log_conf)

    log_conf.data_received_cb.add_callback(
        log_callback
    )

    log_conf.start()

    return log_conf


# --------------------------------------------------
# SEND MOTIVE POSITION TO CRAZYFLIE
# --------------------------------------------------

def send_motive_position(cf, tracker):

    info = tracker.get_info_by_id(MOTIVE_ID)

    if info is None:
        return None

    x, y, z = info["position"]

    # Send external position measurement to Crazyflie estimator
    cf.extpos.send_extpos(
        x,
        y,
        z
    )

    return x, y, z


# --------------------------------------------------
# ESTIMATOR RESET
# --------------------------------------------------

def reset_estimator(cf):

    print("Resetting estimator...")

    cf.param.set_value(
        "kalman.resetEstimation",
        "1"
    )

    time.sleep(0.1)

    cf.param.set_value(
        "kalman.resetEstimation",
        "0"
    )

    print("Estimator reset.")

    # Give estimator time to converge
    time.sleep(2.0)


# --------------------------------------------------
# MAIN
# --------------------------------------------------

def main():

    tracker = create_tracker()

    cflib.crtp.init_drivers()

    log_conf = None

    try:

        print(f"Connecting to {URI}")

        with SyncCrazyflie(
            URI,
            cf=Crazyflie(rw_cache="./cache")
        ) as scf:

            print("Crazyflie connected.")

            # ------------------------------------------
            # First Motive measurement
            # ------------------------------------------

            tracker.update()

            info = tracker.get_info_by_id(MOTIVE_ID)

            if info is None:
                print(
                    f"ERROR: Motive rigid body "
                    f"{MOTIVE_ID} not found."
                )
                return

            print(
                f"Motive rigid body {MOTIVE_ID} found."
            )

            # ------------------------------------------
            # Start CF state logging
            # ------------------------------------------

            log_conf = start_logging(scf)

            # ------------------------------------------
            # Feed some Motive measurements BEFORE reset
            # ------------------------------------------

            print(
                "Sending Motive position "
                "to Crazyflie estimator..."
            )

            for _ in range(50):

                tracker.update()

                send_motive_position(
                    scf.cf,
                    tracker
                )

                time.sleep(MOCAP_DT)

            # ------------------------------------------
            # Reset estimator
            # ------------------------------------------

            reset_estimator(scf.cf)

            # ------------------------------------------
            # Continue sending external position
            # while estimator converges
            # ------------------------------------------

            print(
                "Waiting for estimator convergence..."
            )

            start = time.monotonic()

            while time.monotonic() - start < 3.0:

                tracker.update()

                send_motive_position(
                    scf.cf,
                    tracker
                )

                time.sleep(MOCAP_DT)

            # ------------------------------------------
            # TAKEOFF
            # ------------------------------------------

            print("Starting takeoff...")

            with MotionCommander(
                scf,
                default_height=FLIGHT_HEIGHT
            ):

                print("Takeoff completed.")

                print()
                print(
                    "Comparing Motive and "
                    "Crazyflie stateEstimate..."
                )
                print()

                last_print = 0.0
                start = time.monotonic()

                # Hover for 5 seconds
                while time.monotonic() - start < 5.0:

                    tracker.update()

                    motive_position = send_motive_position(
                        scf.cf,
                        tracker
                    )

                    now = time.monotonic()

                    if (
                        motive_position is not None
                        and now - last_print >= PRINT_DT
                    ):

                        mx, my, mz = motive_position

                        with state_lock:
                            cx = cf_state["x"]
                            cy = cf_state["y"]
                            cz = cf_state["z"]

                        print(
                            f"Motive: "
                            f"({mx:+.3f}, "
                            f"{my:+.3f}, "
                            f"{mz:+.3f}) | "
                            f"CF estimate: "
                            f"({cx:+.3f}, "
                            f"{cy:+.3f}, "
                            f"{cz:+.3f})"
                        )

                        last_print = now

                    time.sleep(MOCAP_DT)

                print()
                print("Hover test completed.")
                print("Landing...")

    except KeyboardInterrupt:

        print("\nTest interrupted.")

    finally:

        if log_conf is not None:

            try:
                log_conf.stop()
            except Exception:
                pass

        tracker.close()

        print("NatNet closed.")


if __name__ == "__main__":
    main()