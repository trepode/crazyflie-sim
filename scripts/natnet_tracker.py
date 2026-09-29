import threading
from typing import Optional

from natnet import DataDescriptions, DataFrame, NatNetClient


class NatNetTracker:
    """
    Wrapper around the NatNet client.

    Stores the latest state received for each rigid body.
    """

    def __init__(
        self,
        server_ip: str = "127.0.0.1",
        local_ip: str = "127.0.0.1",
        use_multicast: bool = False,
    ):
        self.server_ip = server_ip
        self.local_ip = local_ip
        self.use_multicast = use_multicast

        self.client = NatNetClient(
            server_ip_address=server_ip,
            local_ip_address=local_ip,
            use_multicast=use_multicast,
        )

        # Store the latest data received for each rigid body.
        # The key is the rigid body ID.
        self._rigid_bodies = {}

        # Store the latest data descriptions received from the server.
        self._data_descriptions = None

        # Total number of frames received.
        self._num_frames = 0

        # Protect shared data when using asynchronous mode.
        self._lock = threading.Lock()

        # Event used to detect when the first frame is received.
        self._first_frame_event = threading.Event()

        # Register NatNet callbacks.
        self.client.on_data_frame_received_event.handlers.append(
            self._on_frame_received
        )

        self.client.on_data_description_received_event.handlers.append(
            self._on_description_received
        )

    # ------------------------------------------------------------------
    # NATNET CALLBACKS
    # ------------------------------------------------------------------

    def _on_frame_received(self, data_frame: DataFrame):
        """
        Called automatically whenever a new NatNet frame is received.
        """

        with self._lock:
            self._num_frames += 1

            # Update the latest state of every rigid body in this frame.
            for rigid_body in data_frame.rigid_bodies:
                self._rigid_bodies[rigid_body.id_num] = rigid_body

            # Signal that at least one frame has been received.
            self._first_frame_event.set()

    def _on_description_received(self, desc: DataDescriptions):
        """
        Called when the server sends the model/data descriptions.
        """

        with self._lock:
            self._data_descriptions = desc

    # ------------------------------------------------------------------
    # CONNECTION
    # ------------------------------------------------------------------

    def connect(self):
        """
        Connect to the NatNet server.

        Also requests the model definitions from the server.
        """

        self.client.connect()

        # Request information about the available models/rigid bodies.
        self.client.request_modeldef()

    def start_async(self):
        """
        Start receiving NatNet data asynchronously.

        The NatNet client will handle incoming frames in the background.
        """

        self.client.run_async()

    def update(self):
        """
        Process incoming NatNet data in synchronous mode.

        This method should be called periodically from the main loop.
        """

        self.client.update_sync()

    def close(self):
        """
        Close the NatNet connection.
        """

        self.client.shutdown()

    # ------------------------------------------------------------------
    # GENERAL INFORMATION
    # ------------------------------------------------------------------

    @property
    def num_frames(self) -> int:
        """
        Return the total number of frames received.
        """

        with self._lock:
            return self._num_frames

    def wait_for_first_frame(self, timeout: Optional[float] = None) -> bool:
        """
        Wait until the first frame is received.

        Args:
            timeout: Maximum time to wait in seconds.
                     None means wait indefinitely.

        Returns:
            True if a frame was received.
            False if the timeout expired.
        """

        return self._first_frame_event.wait(timeout)

    # ------------------------------------------------------------------
    # RIGID BODY
    # ------------------------------------------------------------------

    def get_rigid_body(self, id_num: int):
        """
        Return the latest rigid body data for the given ID.

        Args:
            id_num: Rigid body ID.

        Returns:
            The latest RigidBody object, or None if the rigid body
            has never been received.
        """

        with self._lock:
            return self._rigid_bodies.get(id_num)

    def get_info_by_id(self, id_num: int):
        """
        Return the main information for a rigid body.

        Args:
            id_num: Rigid body ID.

        Returns:
            A dictionary containing the ID, position and orientation,
            or None if the rigid body has never been received.
        """

        rigid_body = self.get_rigid_body(id_num)

        if rigid_body is None:
            return None

        return {
            "id": rigid_body.id_num,
            "position": rigid_body.pos,
            "orientation": rigid_body.rot,
        }

    def get_position(self, id_num: int):
        """
        Return the latest position of a rigid body.

        Args:
            id_num: Rigid body ID.

        Returns:
            Position as (x, y, z), or None if the rigid body
            has never been received.
        """

        rigid_body = self.get_rigid_body(id_num)

        if rigid_body is None:
            return None

        return rigid_body.pos

    def get_orientation(self, id_num: int):
        """
        Return the latest orientation of a rigid body.

        The orientation is represented as a quaternion.

        Args:
            id_num: Rigid body ID.

        Returns:
            Quaternion as (x, y, z, w), or None if the rigid body
            has never been received.
        """

        rigid_body = self.get_rigid_body(id_num)

        if rigid_body is None:
            return None

        return rigid_body.rot

    def rigid_body_exists(self, id_num: int) -> bool:
        """
        Check whether a rigid body with the given ID has been received.
        """

        with self._lock:
            return id_num in self._rigid_bodies

    def get_all_rigid_bodies(self) -> dict:
        """
        Return all rigid bodies currently stored.

        Returns:
            A dictionary in the form:
            {id: rigid_body}
        """

        with self._lock:
            return self._rigid_bodies.copy()

    def get_all_info(self) -> dict:
        """
        Return the main information for all currently tracked rigid bodies.

        Returns:
            A dictionary in the form:

            {
                id: {
                    "id": ...,
                    "position": ...,
                    "orientation": ...
                }
            }
        """

        with self._lock:
            return {
                id_num: {
                    "id": rigid_body.id_num,
                    "position": rigid_body.pos,
                    "orientation": rigid_body.rot,
                }
                for id_num, rigid_body in self._rigid_bodies.items()
            }


    def __enter__(self):
        """
        Connect to NatNet when entering a context manager.
        """

        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Close the NatNet connection when leaving a context manager.
        """

        self.close()
