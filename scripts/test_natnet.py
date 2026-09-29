import time

from natnet_tracker import NatNetTracker


tracker = NatNetTracker(
    server_ip="127.0.0.1",
    local_ip="127.0.0.1",
    use_multicast=False,
)

tracker.connect()
tracker.start_async()

print("Connected to NatNet.")

# Aspettiamo il primo frame
if tracker.wait_for_first_frame(timeout=5):
    print("Receiving data!")
else:
    print("No data received.")
    tracker.close()
    exit()


while True:

    position = tracker.get_position(3)
    orientation = tracker.get_orientation(3)
    
    print(f"Position:    {position}")
    print(f"Orientation: {orientation}")

    time.sleep(0.1)
