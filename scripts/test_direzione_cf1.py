import argparse
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from natnet_tracker import NatNetTracker


URI = "radio://0/80/2M/E7E7E7E7E9"
MOTIVE_ID = 3

FLIGHT_HEIGHT = 1
STEP = 0.10
TRACKING_TIMEOUT = 0.40


def read_fresh_pose(tracker, previous_frame, timeout=3.0):
    deadline = time.monotonic() + timeout
    last_diagnostic = "Nessuna lettura effettuata"

    while time.monotonic() < deadline:
        snap = tracker.get_tracking_snapshot((MOTIVE_ID,))

        rb = snap["bodies"].get(MOTIVE_ID)
        received = snap["received_at"]

        age = (
            time.monotonic() - received
            if received is not None
            else float("inf")
        )

        new_frame = snap["frame"] > previous_frame
        tracked = rb["tracked"] if rb is not None else None
        position = rb["position"] if rb is not None else None

        last_diagnostic = (
            f"frame={snap['frame']} | "
            f"previous_frame={previous_frame} | "
            f"new_frame={new_frame} | "
            f"age={age:.3f}s | "
            f"tracked={tracked} | "
            f"position={position}"
        )

        if (
            new_frame
            and age < TRACKING_TIMEOUT
            and tracked is True
            and position is not None
        ):
            return position, snap["frame"]

        time.sleep(0.05)

    raise RuntimeError(
        "Tracking Motive non valido. "
        + last_diagnostic
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fly", action="store_true")
    args = parser.parse_args()

    if not args.fly:
        print("Test disarmato. Usa --fly solo dopo i controlli.")
        return

    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False
    )

    tracker.connect()
    tracker.start_async()

    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)

        with SyncCrazyflie(
            URI,
            cf=Crazyflie(rw_cache="./cache")
        ) as scf:

            cf = scf.cf
            takeoff_attempted = False

            try:
                cf.supervisor.send_arming_request(True)
                time.sleep(1.0)

                takeoff_attempted = True
                cf.high_level_commander.takeoff(
                    absolute_height_m=FLIGHT_HEIGHT,
                    duration_s=3.0,
                    yaw=None
                )

                time.sleep(3.5)

                # Motive deve vedere CF1 dopo il decollo.
                baseline = tracker.num_frames
                p0, frame = read_fresh_pose(tracker, baseline)

                print("POSIZIONE INIZIALE:", p0)

                # Un solo piccolo movimento relativo +X_CF.
                cf.high_level_commander.go_to(
                    x=0.0,
                    y=STEP,
                    z=0.0,
                    yaw=0.0,
                    duration_s=2.0,
                    relative=True
                )

                # Sorveglia il tracking durante il movimento.
                end = time.monotonic() + 2.5

                while time.monotonic() < end:
                    _, frame = read_fresh_pose(
                        tracker, frame, timeout=0.40
                    )
                    time.sleep(0.05)

                p1, _ = read_fresh_pose(tracker, frame)

                dx = p1[0] - p0[0]
                dz = p1[2] - p0[2]

                print("POSIZIONE FINALE:", p1)
                print(f"Delta X Motive: {dx:+.3f} m")
                print(f"Delta Z Motive: {dz:+.3f} m")
                print("Atteso dalla calibrazione: Delta Z negativo")

            finally:
                if takeoff_attempted:
                    print("LAND CF1")
                    try:
                        cf.high_level_commander.land(
                            absolute_height_m=0.0,
                            duration_s=3.0,
                            yaw=None
                        )
                        time.sleep(3.5)
                    finally:
                        cf.high_level_commander.stop()

    finally:
        tracker.close()


if __name__ == "__main__":
    main()