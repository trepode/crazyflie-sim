
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from natnet_tracker import NatNetTracker


URI = "radio://0/80/2M/E7E7E7E7E9"
MOTIVE_ID = 3

DT = 0.05  # 20 Hz


def motive_to_cf_position(position):
    x, y, z = position

    # Same horizontal axis mapping as real_orca.py
    # Motive Y is vertical, Crazyflie Z is vertical.
    return -z, -x, y


def main():
    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False,
    )

    tracker.connect()
    print("Motive connected")

    try:
        cflib.crtp.init_drivers()

        with SyncCrazyflie(
            URI,
            cf=Crazyflie(rw_cache="./cache")
        ) as scf:

            cf = scf.cf
            print("CF1 connected")
            print("MOTORS OFF - move the drone by hand")

            # Use the Kalman estimator
            cf.param.set_value(
                "stabilizer.estimator", "2"
            )

            log_config = LogConfig(
                name="Position",
                period_in_ms=100
            )

            for variable in [
                "stateEstimate.x",
                "stateEstimate.y",
                "stateEstimate.z",
            ]:
                log_config.add_variable(
                    variable, "float"
                )

            with SyncLogger(scf, log_config) as logger:
                log_iterator = iter(logger)

                start = time.monotonic()
                last_print = start

                while time.monotonic() - start < 30:
                    tracker.update()

                    info = tracker.get_info_by_id(
                        MOTIVE_ID
                    )
                    if info is not None:
                       print("Motive:", info["position"])

                    if info is None:
                        print("Motive tracking missing")
                        time.sleep(DT)
                        continue

                    p_motive = info["position"]
                    print("RAW MOTIVE:", p_motive)
                    x_cf, y_cf, z_cf = (
                        motive_to_cf_position(p_motive)
                    )

                    # Send external position to estimator
                    #cf.extpos.send_extpos(
                    #    x_cf, y_cf, z_cf
                    #)

                    # Read estimated position
                    # (SyncLogger returns the next log packet)
                    try:
                        _, data, _ = next(log_iterator)
                    except StopIteration:
                        break

                    if time.monotonic() - last_print >= 0.5:
                        print(
                            f"Motive/CF: "
                            f"({x_cf:+.3f}, "
                            f"{y_cf:+.3f}, "
                            f"{z_cf:+.3f}) | "
                            f"Estimate: "
                            f"({data['stateEstimate.x']:+.3f}, "
                            f"{data['stateEstimate.y']:+.3f}, "
                            f"{data['stateEstimate.z']:+.3f})"
                        )
                        last_print = time.monotonic()

                    time.sleep(DT)

    finally:
        tracker.close()
        print("Test finished")


if __name__ == "__main__":
    main()
