
import time
import math
from natnet_tracker import NatNetTracker

MOTIVE_ID_1 = 2
MOTIVE_ID_2 = 3

ALPHA = 0.25
MAX_MEASURED_SPEED = 2.0


class VelocityEstimator:
    def __init__(self, alpha=ALPHA):
        self.alpha = alpha
        self.previous_position = None
        self.previous_time = None
        self.velocity = (0.0, 0.0)

    def update(self, position, timestamp):
        if self.previous_position is None:
            self.previous_position = position
            self.previous_time = timestamp
            return self.velocity

        dt = timestamp - self.previous_time

        if dt <= 0:
            return self.velocity

        vx = (
            position[0] - self.previous_position[0]
        ) / dt

        vz = (
            position[1] - self.previous_position[1]
        ) / dt

        self.previous_position = position
        self.previous_time = timestamp

        if math.hypot(vx, vz) > MAX_MEASURED_SPEED:
            return self.velocity

        old_vx, old_vz = self.velocity

        self.velocity = (
            self.alpha * vx + (1 - self.alpha) * old_vx,
            self.alpha * vz + (1 - self.alpha) * old_vz
        )

        return self.velocity


def main():
    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False
    )

    estimator1 = VelocityEstimator()
    estimator2 = VelocityEstimator()

    try:
        tracker.connect()
        print("Motive connected")

        last_frame = tracker.num_frames

        while True:
            tracker.update()

            frame = tracker.num_frames

            # Do not estimate velocity from repeated frames.
            if frame == last_frame:
                time.sleep(0.01)
                continue

            last_frame = frame

            info1 = tracker.get_info_by_id(MOTIVE_ID_1)
            info2 = tracker.get_info_by_id(MOTIVE_ID_2)

            if info1 is None or info2 is None:
                print("Tracking unavailable")
                continue

            p1_3d = info1["position"]
            p2_3d = info2["position"]

            # Motive horizontal plane: X-Z
            p1 = (p1_3d[0], p1_3d[2])
            p2 = (p2_3d[0], p2_3d[2])

            now = time.monotonic()

            v1 = estimator1.update(p1, now)
            v2 = estimator2.update(p2, now)

            print(
                f"CF1 p={p1} v={v1} | "
                f"CF2 p={p2} v={v2}"
            )

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopped")

    finally:
        tracker.close()


if __name__ == "__main__":
    main()
