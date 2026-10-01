import time
from natnet_tracker import NatNetTracker

MOTIVE_ID = 3

tracker = NatNetTracker(
    server_ip="127.0.0.1",
    local_ip="127.0.0.1",
    use_multicast=False,
)

try:
    tracker.connect()
    print("Connected to Motive.")
    print("Premi CTRL+C per terminare.\n")

    while True:
        tracker.update()

        info = tracker.get_info_by_id(MOTIVE_ID)

        if info is not None:
            x, y, z = info["position"]

            print(
                f"x = {x:+.3f}   "
                f"y = {y:+.3f}   "
                f"z = {z:+.3f}"
            )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nTest terminato.")

finally:
    tracker.close()
    print("NatNet closed.")