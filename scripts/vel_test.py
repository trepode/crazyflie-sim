import time

from natnet_tracker import NatNetTracker

CF2 = 2
CF3 = 3

states = {
    CF2: {
        "x": 0.0,
        "y": 0.0,
        "vx": 0.0,
        "vy": 0.0,
    },
    CF3: {
        "x": 0.0,
        "y": 0.0,
        "vx": 0.0,
        "vy": 0.0,
    }
}


def create_tracker():
    """
    Create and connect the NatNet tracker.
    """
    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False,
    )

    tracker.connect()

    print("Connected to NatNet.")

    return tracker


def compute_velocities(rigid_bodies, previous_positions, previous_times):
    """
    Compute the velocity of every rigid body from consecutive
    position measurements.

    Returns:
        velocities = {
            rigid_body_id: (vx, vy, vz)
        }
    """

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

        # Update previous state
        previous_positions[rigid_body_id] = (x, y, z)
        previous_times[rigid_body_id] = now

    return velocities


def print_states(rigid_bodies, velocities):
    """
    Print position and velocity of every rigid body.
    """

    for rigid_body_id, info in rigid_bodies.items():

        x, y, z = info["position"]

        if rigid_body_id in velocities:

            vx, vy, vz = velocities[rigid_body_id]

            print(
                f"Rigid Body {rigid_body_id} | "
                f"Position: ({x:.3f}, {y:.3f}, {z:.3f}) | "
                f"Velocity: ({vx:.3f}, {vy:.3f}, {vz:.3f}) m/s"
            )


def main():

    tracker = create_tracker()

    previous_positions = {}
    previous_times = {}

    try:

        while True:

            # Receive new NatNet frame
            tracker.update()

            # Get all rigid bodies from Motive
            rigid_bodies = tracker.get_all_info()

            # Compute velocities
            velocities = compute_velocities(
                rigid_bodies,
                previous_positions,
                previous_times,
            )

            # Print current state
            # print_states(
            #     rigid_bodies,
            #     velocities,
            # )

            time.sleep(0.05)

    except KeyboardInterrupt:

        print("\nStopping NatNet...")

    finally:

        tracker.close()
        print("NatNet closed.")


if __name__ == "__main__":
    main()