import math
import threading
import time

from typing import Optional
from natnet import DataDescriptions, DataFrame, NatNetClient


class NatNetTracker:
    """
    Riceve i frame NatNet e conserva uno snapshot
    atomico dei Rigid Bodies.

    Permette di controllare:
    - arrivo di nuovi frame;
    - momento dell'ultimo frame ricevuto;
    - tracking valido per ogni Rigid Body;
    - ultima ricezione valida per ogni Rigid Body.
    """

    def __init__(
        self,
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False,
    ):
        self.server_ip = server_ip
        self.local_ip = local_ip
        self.use_multicast = use_multicast

        self.client = NatNetClient(
            server_ip_address=server_ip,
            local_ip_address=local_ip,
            use_multicast=use_multicast,
        )

        self._rigid_bodies = {}
        self._data_descriptions = None

        self._num_frames = 0
        self._last_frame_time = None

        self._last_valid_at = {}
        self._last_tracking_flags = {}

        self._lock = threading.Lock()
        self._first_frame_event = threading.Event()

        self.client.on_data_frame_received_event.handlers.append(
            self._on_frame_received
        )

        self.client.on_data_description_received_event.handlers.append(
            self._on_description_received
        )

    # --------------------------------------------------
    # VALIDITA' TRACKING
    # --------------------------------------------------

    @staticmethod
    def _tracking_valid(rb):
        """
        True  = Rigid Body tracciato
        False = tracking non valido
        None  = libreria non espone il flag

        Non consideriamo mai None equivalente a True.
        """

        for attr in ("tracking_valid", "is_tracking_valid"):
            flag = getattr(rb, attr, None)

            if callable(flag):
                flag = flag()

            if isinstance(flag, bool):
                return flag

        # Alcune implementazioni conservano il flag
        # nel bit 0 del campo params.
        params = getattr(rb, "params", None)

        if isinstance(params, int):
            return bool(params & 1)

        return None

    # --------------------------------------------------
    # CALLBACK NATNET
    # --------------------------------------------------

    def _on_frame_received(self, data_frame: DataFrame):
        """
        Viene eseguito quando arriva un nuovo frame.

        Il timestamp viene aggiornato qui, NON quando
        lo script richiede una posizione memorizzata.
        """

        now = time.monotonic()

        bodies = {
            rb.id_num: rb
            for rb in data_frame.rigid_bodies
        }

        flags = {
            rb_id: self._tracking_valid(rb)
            for rb_id, rb in bodies.items()
        }

        with self._lock:
            self._num_frames += 1
            self._last_frame_time = now

            # Sostituiamo l'intero contenuto.
            # Un RB scomparso non conserva il vecchio dato.
            self._rigid_bodies = bodies
            self._last_tracking_flags = flags

            for rb_id, rb in bodies.items():
                if flags[rb_id] is not True:
                    continue

                try:
                    pos = tuple(float(v) for v in rb.pos)
                except (ValueError, TypeError):
                    continue

                if (
                    len(pos) == 3
                    and all(math.isfinite(v) for v in pos)
                ):
                    self._last_valid_at[rb_id] = now

            self._first_frame_event.set()

    def _on_description_received(
        self,
        desc: DataDescriptions,
    ):
        with self._lock:
            self._data_descriptions = desc

    # --------------------------------------------------
    # CONNECTION
    # --------------------------------------------------

    def connect(self):
        self.client.connect()
        self.client.request_modeldef()

    def start_async(self):
        """
        Avvia la ricezione NatNet indipendentemente
        dal ciclo di controllo principale.

        Dopo start_async() non chiamare update().
        """
        self.client.run_async()

    def update(self):
        self.client.update_sync()

    def close(self):
        self.client.shutdown()

    # --------------------------------------------------
    # FRAME INFORMATION
    # --------------------------------------------------

    @property
    def num_frames(self):
        with self._lock:
            return self._num_frames

    def wait_for_first_frame(
        self,
        timeout: Optional[float] = None,
    ):
        return self._first_frame_event.wait(timeout)

    def get_tracking_snapshot(self, ids):
        """
        Restituisce dati coerenti dello stesso frame.

        Per ogni RB:
        - position
        - tracked
        - rb_type

        Restituisce anche:
        - frame
        - received_at
        - last_valid_at
        """

        with self._lock:
            result = {}

            for rb_id in ids:
                rb = self._rigid_bodies.get(rb_id)

                if rb is None:
                    result[rb_id] = None
                    continue

                try:
                    position = tuple(
                        float(v) for v in rb.pos
                    )

                    if (
                        len(position) != 3
                        or not all(
                            math.isfinite(v)
                            for v in position
                        )
                    ):
                        position = None

                except (ValueError, TypeError):
                    position = None

                result[rb_id] = {
                    "position": position,
                    "tracked": self._last_tracking_flags.get(
                        rb_id
                    ),
                    "rb_type": type(rb).__name__,
                }

            return {
                "frame": self._num_frames,
                "received_at": self._last_frame_time,
                "bodies": result,
                "last_valid_at": {
                    rb_id: self._last_valid_at.get(rb_id)
                    for rb_id in ids
                },
            }

    # --------------------------------------------------
    # COMPATIBILITA' CON IL VECCHIO TRACKER
    # --------------------------------------------------

    def get_rigid_body(self, id_num):
        with self._lock:
            return self._rigid_bodies.get(id_num)

    def get_info_by_id(self, id_num):
        rb = self.get_rigid_body(id_num)

        if rb is None:
            return None

        return {
            "id": rb.id_num,
            "position": rb.pos,
            "orientation": rb.rot,
        }

    def get_position(self, id_num):
        rb = self.get_rigid_body(id_num)
        return None if rb is None else rb.pos

    def get_orientation(self, id_num):
        rb = self.get_rigid_body(id_num)
        return None if rb is None else rb.rot

    def rigid_body_exists(self, id_num):
        with self._lock:
            return id_num in self._rigid_bodies

    def get_all_rigid_bodies(self):
        with self._lock:
            return self._rigid_bodies.copy()

    def get_all_info(self):
        with self._lock:
            return {
                rb_id: {
                    "id": rb.id_num,
                    "position": rb.pos,
                    "orientation": rb.rot,
                }
                for rb_id, rb in self._rigid_bodies.items()
            }

    def __enter__(self):
        self.connect()
        return self

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback,
    ):
        self.close()