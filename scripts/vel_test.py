import time
import math

from natnet_tracker import NatNetTracker


CF2 = 2
CF3 = 3

COLLISION_DISTANCE = 0.20  # 20 cm


def create_tracker():
    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False,
    )

    tracker.connect()

    print("Connected to NatNet.")

    return tracker


def compute_velocities(rigid_bodies, previous_positions, previous_times):

    velocities = {}

    now = time.monotonic()

    for rigid_body_id, info in rigid_bodies.items():

        x, y, z = info["position"]

        if rigid_body_id in previous_positions:

            old_x, old_y, old_z = previous_positions[rigid_body_id]

            dt = now - previous_times[rigid_body_id]

            if dt > 0:

                vx = (x - old_x) / dt
                vy = (y - old_y) / dt
                vz = (z - old_z) / dt

                velocities[rigid_body_id] = (vx, vy, vz)

        previous_positions[rigid_body_id] = (x, y, z)
        previous_times[rigid_body_id] = now

    return velocities


def compute_distance(rigid_bodies, id_a, id_b):
    """
    Compute planar distance between two rigid bodies.
    """

    if id_a not in rigid_bodies or id_b not in rigid_bodies:
        return None

    x_a, y_a, z_a = rigid_bodies[id_a]["position"]
    x_b, y_b, z_b = rigid_bodies[id_b]["position"]

    dx = x_b - x_a
    dy = y_b - y_a

    return math.sqrt(dx**2 + dy**2)


def check_collision_risk(rigid_bodies, id_a, id_b):

    distance = compute_distance(
        rigid_bodies,
        id_a,
        id_b
    )

    if distance is None:
        print("Waiting for both rigid bodies...")
        return

    print(
        f"Distance CF{id_a}-CF{id_b}: "
        f"{distance:.3f} m"
    )

    if distance < COLLISION_DISTANCE:

        print(
            f"!!! PERICOLO SCONTRO !!! "
            f"Distanza = {distance * 100:.1f} cm"
        )


def main():

    tracker = create_tracker()

    previous_positions = {}
    previous_times = {}

    try:

        while True:

            # Receive new Motive data
            tracker.update()

            # Get rigid body states
            rigid_bodies = tracker.get_all_info()

            # Compute velocities
            velocities = compute_velocities(
                rigid_bodies,
                previous_positions,
                previous_times
            )

            # Check distance between CF2 and CF3
            check_collision_risk(
                rigid_bodies,
                CF2,
                CF3
            )

            time.sleep(0.05)

    except KeyboardInterrupt:

        print("\nStopping NatNet...")

    finally:

        tracker.close()
        print("NatNet closed.")


if __name__ == "__main__":
    main()