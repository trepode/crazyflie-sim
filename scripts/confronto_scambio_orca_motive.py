#!/usr/bin/env python3

"""
Scambio simmetrico fra due Crazyflie reali.

Modalita':
    pure
    alan-lite
    right-bias

Tracking:
    Motive / NatNet

Controllo di volo:
    Crazyflie High Level Commander

I goal vengono assegnati UNA SOLA VOLTA,
dal primo nuovo frame valido dopo il decollo.

Se Motive perde il tracking:
    stop navigazione
    richiesta LAND a entrambi

Per default NON decolla.
Il volo richiede esplicitamente --fly.
"""

import argparse
import csv
import math
import sys
import time

from pathlib import Path
from collections import deque
from contextlib import contextmanager

import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from natnet_tracker import NatNetTracker


# ============================================================
# CONFIGURAZIONE
# ============================================================

URI1 = "radio://0/80/2M/E7E7E7E7E9"
URI2 = "radio://0/80/2M/E7E7E7E7E7"

MOTIVE_ID1 = 2
MOTIVE_ID2 = 3

PREFERRED_SPEED = 0.2
MAX_SPEED = 0.25

DT = 0.30
COMMAND_DURATION = 0.25

TAKEOFF_TIME = 3.0
LANDING_TIME = 3.0

FLIGHT_HEIGHT = 1

TIME_HORIZON = 3.0
GOAL_TOLERANCE = 0.12

EMERGENCY_DISTANCE = 0.50
MAX_DISTANCE_FROM_START = 2.5
MAX_VERTICAL_CHANGE = 0.20

VELOCITY_ALPHA = 0.25
MAX_VALID_MOTIVE_SPEED = 2.0

MIN_START_SEPARATION = 0.85

# Da calibrare rispetto al pavimento reale.
MOTIVE_FLOOR_Y = 0.0

# Tempo massimo per acquisire entrambi i droni
# dopo il decollo.
INITIAL_ACQUISITION_TIMEOUT = 2.0

# Controllo tracking ogni 50 millisecondi.
TRACKING_CHECK_PERIOD = 0.05

# Perdita di tracking oltre 400 millisecondi.
TRACKING_TIMEOUT = 0.40


# ============================================================
# OPERAZIONI VETTORIALI
# ============================================================

def norm(v):
    return math.hypot(v[0], v[1])


def dist(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1]
    )


def velocity_toward_goal(position, goal, speed):

    dx = goal[0] - position[0]
    dy = goal[1] - position[1]

    d = math.hypot(dx, dy)

    if d < GOAL_TOLERANCE:
        return (0.0, 0.0)

    return (
        speed * dx / d,
        speed * dy / d
    )


# ============================================================
# NAVIGAZIONE CON GOAL FISSO
# ============================================================

class FixedSwapNavigator:

    def __init__(self, name, start, goal):

        self.name = name
        self.start = start
        self.goal = goal

        self.waypoints = 1
        self.reached = 0
        self.reach_time = None

        self.events = [
            (0.0, "SWAP_GOAL", goal)
        ]

    def update_goal(self, t, position, neighbour=None):

        if (
            not self.reached
            and dist(position, self.goal) < GOAL_TOLERANCE
        ):
            self.reached = 1
            self.reach_time = t

            self.events.append(
                (t, "GOAL_REACHED", self.goal)
            )

            print(
                f"{self.name}: GOAL REACHED "
                f"a t={t:.2f}s"
            )

    def preferred_velocity(self, position):

        if self.reached:
            return (0.0, 0.0)

        return velocity_toward_goal(
            position,
            self.goal,
            PREFERRED_SPEED
        )


# ============================================================
# ALAN-LITE
# ============================================================

class ALANLiteSelector:

    STUCK_WINDOW = 3.0
    MIN_PROGRESS = 0.025

    NEIGHBOR_LIMIT = 1.0
    MAX_BASE_PROGRESS = 0.012

    ESCAPE_TIME = 2.5
    COOLDOWN = 0.5

    LOOKAHEAD = 1.5

    def __init__(self, start):

        self.start = start

        self.history = deque()
        self.goal = None

        self.active_until = -1.0
        self.cooldown_until = 0.0

        self.action = "GOAL"
        self.angle = 0.0

        self.activations = 0
        self.last_progress = float("nan")

    @staticmethod
    def rotate(v, degrees):

        theta = math.radians(degrees)

        c = math.cos(theta)
        s = math.sin(theta)

        return (
            c * v[0] - s * v[1],
            s * v[0] + c * v[1]
        )

    def select(
        self,
        t,
        position,
        goal,
        neighbor,
        v_goal,
        lp,
        ld,
        orca_solver
    ):

        if self.goal != goal:

            self.goal = goal
            self.history.clear()

            self.active_until = -1.0
            self.action = "GOAL"

        distance_goal = dist(position, goal)

        self.history.append(
            (t, distance_goal)
        )

        while (
            self.history
            and t - self.history[0][0] > self.STUCK_WINDOW
        ):
            self.history.popleft()

        self.last_progress = (
            self.history[0][1] - distance_goal
        )

        if t < self.active_until:

            return (
                self.rotate(v_goal, self.angle),
                self.action
            )

        if self.action != "GOAL":

            self.action = "GOAL"
            self.cooldown_until = t + self.COOLDOWN

        if t < self.cooldown_until:
            return v_goal, "GOAL"

        if (
            not self.history
            or t - self.history[0][0]
            < self.STUCK_WINDOW - 0.10
        ):
            return v_goal, "GOAL"

        if self.last_progress >= self.MIN_PROGRESS:
            return v_goal, "GOAL"

        if dist(position, neighbor) > self.NEIGHBOR_LIMIT:
            return v_goal, "GOAL"

        if norm(v_goal) < 1e-6:
            return v_goal, "GOAL"

        try:

            base_cmd = orca_solver(
                v_goal,
                lp,
                ld,
                MAX_SPEED
            )

        except ValueError:
            return v_goal, "GOAL"

        goal_unit = (
            v_goal[0] / norm(v_goal),
            v_goal[1] / norm(v_goal)
        )

        predicted_goal_speed = (
            base_cmd[0] * goal_unit[0]
            + base_cmd[1] * goal_unit[1]
        )

        if predicted_goal_speed > self.MAX_BASE_PROGRESS:
            return v_goal, "GOAL"

        candidates = (
            (45, "LEFT_45"),
            (-45, "RIGHT_45"),
            (90, "LEFT_90"),
            (-90, "RIGHT_90")
        )

        best = None

        for angle, name in candidates:

            pref = self.rotate(
                v_goal,
                angle
            )

            try:

                command = orca_solver(
                    pref,
                    lp,
                    ld,
                    MAX_SPEED
                )

            except ValueError:
                continue

            future = (
                position[0]
                + command[0] * self.LOOKAHEAD,

                position[1]
                + command[1] * self.LOOKAHEAD
            )

            if (
                dist(future, self.start)
                > MAX_DISTANCE_FROM_START - 0.05
            ):
                continue

            toward = (
                command[0] * goal_unit[0]
                + command[1] * goal_unit[1]
            )

            score = (
                0.25 * norm(command)
                + 0.20 * toward
            )

            if (
                best is None
                or score > best[0] + 1e-8
            ):
                best = (
                    score,
                    angle,
                    name,
                    pref,
                    command
                )

        if (
            best is None
            or norm(best[4]) < 0.01
        ):
            return v_goal, "GOAL"

        self.angle = best[1]
        self.action = best[2]

        self.active_until = t + self.ESCAPE_TIME
        self.activations += 1

        return best[3], self.action


# ============================================================
# RIGHT-BIAS
# ============================================================

class RightPassingSelector:

    TRIGGER_DISTANCE = 0.90
    RELEASE_MARGIN = 0.10

    BIAS_DEGREES = -60.0

    def __init__(self, start, goal):

        self.start = start
        self.goal = goal

        self.committed = False
        self.completed = False

        self.activations = 0
        self.action = "GOAL"

    def select(
        self,
        position,
        neighbor,
        v_goal,
        combined_radius
    ):

        if norm(v_goal) < 1e-8:
            return v_goal, "GOAL"

        if self.completed:
            return v_goal, "GOAL"

        sep = dist(
            position,
            neighbor
        )

        if (
            not self.committed
            and sep <= self.TRIGGER_DISTANCE
        ):
            self.committed = True
            self.activations += 1

        axis = (
            self.goal[0] - self.start[0],
            self.goal[1] - self.start[1]
        )

        axis_len = norm(axis)

        passed = False

        if axis_len > 1e-8:

            projection = (
                (neighbor[0] - position[0]) * axis[0]
                + (neighbor[1] - position[1]) * axis[1]
            ) / axis_len

            passed = projection <= 0

        if (
            self.committed
            and passed
            and sep > combined_radius + self.RELEASE_MARGIN
        ):
            self.committed = False
            self.completed = True

        if self.committed:

            return (
                ALANLiteSelector.rotate(
                    v_goal,
                    self.BIAS_DEGREES
                ),
                "RIGHT_60"
            )

        return v_goal, "GOAL"


# ============================================================
# STIMA VELOCITA'
# ============================================================

class VelocityEstimator:

    def __init__(self, alpha=0.25):

        self.alpha = alpha

        self.previous_position = None
        self.previous_time = None

        self.velocity = (0.0, 0.0)

    def reset(self, position):

        self.previous_position = position
        self.previous_time = time.monotonic()

        self.velocity = (0.0, 0.0)

    def update(self, position):

        now = time.monotonic()

        if self.previous_position is None:

            self.reset(position)
            return self.velocity

        dt = now - self.previous_time

        if dt > 0:

            raw = (
                (position[0] - self.previous_position[0]) / dt,
                (position[1] - self.previous_position[1]) / dt
            )

            if norm(raw) <= MAX_VALID_MOTIVE_SPEED:

                a = self.alpha

                self.velocity = (
                    a * raw[0]
                    + (1 - a) * self.velocity[0],

                    a * raw[1]
                    + (1 - a) * self.velocity[1]
                )

            self.previous_position = position
            self.previous_time = now

        return self.velocity


# ============================================================
# TRASFORMAZIONE MOTIVE -> CRAZYFLIE
# ============================================================

def motive_to_cf_velocity(v_motive):

    vx_m, vz_m = v_motive

    return (
        -vz_m,
        -vx_m
    )


def mot_xy(p):

    return (
        p[0],
        p[2]
    )


# ============================================================
# ESTRAZIONE POSIZIONI VALIDE
# ============================================================

def pair_from_snapshot(snapshot, ids):
    """
    Estrae le due pose dallo stesso frame NatNet.

    Entrambi i Rigid Bodies devono avere:
    - tracking valido;
    - posizione numerica valida.
    """

    pair = []

    for rb_id in ids:

        rb = snapshot["bodies"][rb_id]

        if rb is None:
            return None

        if rb["tracked"] is not True:
            return None

        if rb["position"] is None:
            return None

        pair.append(rb["position"])

    return tuple(pair)


def unknown_tracking_ids(snapshot, ids):

    return [
        rb_id
        for rb_id in ids
        if (
            snapshot["bodies"][rb_id] is not None
            and snapshot["bodies"][rb_id]["tracked"] is None
        )
    ]


# ============================================================
# ASSEGNAZIONE INIZIALE DEI GOAL
# ============================================================

def first_valid_pair_after_takeoff(
    tracker,
    ids,
    timeout=INITIAL_ACQUISITION_TIMEOUT
):
    """
    Chiamata SOLO dopo il decollo.

    Ignora i frame ricevuti prima di questa funzione.

    Attende il primo nuovo frame che contiene
    posizioni valide per entrambi i Crazyflie.
    """

    baseline = tracker.get_tracking_snapshot(ids)["frame"]

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:

        snap = tracker.get_tracking_snapshot(ids)

        if (
            snap["frame"] <= baseline
            or snap["received_at"] is None
        ):
            time.sleep(TRACKING_CHECK_PERIOD)
            continue

        baseline = snap["frame"]

        unknown = unknown_tracking_ids(snap, ids)

        if unknown:

            raise RuntimeError(
                "NatNet non espone tracking_valid "
                f"per i Rigid Bodies {unknown}. "
                "Impossibile verificare il tracking."
            )

        age = (
            time.monotonic()
            - snap["received_at"]
        )

        if age > TRACKING_TIMEOUT:

            time.sleep(TRACKING_CHECK_PERIOD)
            continue

        pair = pair_from_snapshot(
            snap,
            ids
        )

        if pair is not None:

            print(
                f"PRIMO FRAME VALIDO: {snap['frame']}"
            )

            return pair, snap["frame"]

        time.sleep(TRACKING_CHECK_PERIOD)

    raise RuntimeError(
        "Dopo il decollo Motive non ha ricevuto "
        "un nuovo frame valido con entrambi i droni."
    )


# ============================================================
# WATCHDOG MOTIVE
# ============================================================

def tracking_health(
    snapshot,
    ids,
    now,
    timeout=TRACKING_TIMEOUT
):
    """
    Restituisce:

        OK:
            entrambi tracciati

        WAIT:
            tracking temporaneamente non valido;
            non inviare comandi ORCA

        LOST:
            tracking perso oltre il timeout;
            richiedere LAND a entrambi
    """

    last_frame = snapshot["received_at"]

    # Nessun nuovo frame NatNet
    if (
        last_frame is None
        or now - last_frame > timeout
    ):

        return (
            "LOST",
            "Nessun nuovo frame NatNet"
        )

    unknown = unknown_tracking_ids(
        snapshot,
        ids
    )

    if unknown:

        return (
            "LOST",
            f"Flag tracking assente per {unknown}"
        )

    # Controlliamo CF1 e CF2 separatamente.
    for rb_id in ids:

        last_valid = snapshot["last_valid_at"][rb_id]

        if (
            last_valid is None
            or now - last_valid > timeout
        ):

            return (
                "LOST",
                f"Rigid Body {rb_id} "
                "non tracciato validamente "
                f"da oltre {timeout:.2f}s"
            )

    # Anche se non e' ancora scaduto il timeout,
    # non usiamo pose non valide per navigare.
    pair = pair_from_snapshot(
        snapshot,
        ids
    )

    if pair is None:

        return (
            "WAIT",
            "Tracking temporaneamente non valido"
        )

    return (
        "OK",
        "Entrambi i Rigid Bodies tracciati"
    )


# ============================================================
# HIGH LEVEL COMMANDER
# ============================================================

def send_cmd(cf, v):
    """
    Converte lo spostamento nel riferimento CF.

    Manteniamo high_level_commander.go_to(),
    come nello script funzionante in laboratorio.
    """

    vx_cf, vy_cf = motive_to_cf_velocity(v)

    cf.high_level_commander.go_to(
        x=vx_cf * COMMAND_DURATION,
        y=vy_cf * COMMAND_DURATION,
        z=0.0,
        yaw=0.0,
        duration_s=COMMAND_DURATION,
        relative=True
    )


@contextmanager
def high_level_flight_pair(scf1, scf2):
    """
    Gestisce decollo e atterraggio di entrambi.

    In caso di errore o interruzione richiede
    LAND a entrambi prima di attendere.

    E' una procedura best-effort, non una
    garanzia di atterraggio sicuro.
    """

    cf1 = scf1.cf
    cf2 = scf2.cf

    takeoff_attempted = False

    try:

        cf1.supervisor.send_arming_request(True)
        cf2.supervisor.send_arming_request(True)

        time.sleep(1.0)

        takeoff_attempted = True

        cf1.high_level_commander.takeoff(
            absolute_height_m=FLIGHT_HEIGHT,
            duration_s=TAKEOFF_TIME,
            yaw=None
        )

        cf2.high_level_commander.takeoff(
            absolute_height_m=FLIGHT_HEIGHT,
            duration_s=TAKEOFF_TIME,
            yaw=None
        )

        time.sleep(TAKEOFF_TIME + 0.5)

        yield cf1, cf2

    finally:

        if takeoff_attempted:

            print(
                "LANDING: invio comando "
                "a ENTRAMBI i Crazyflie"
            )

            # Prima inviamo LAND a entrambi.
            for i, cf in enumerate(
                (cf1, cf2),
                start=1
            ):

                try:

                    cf.high_level_commander.land(
                        absolute_height_m=0.0,
                        duration_s=LANDING_TIME,
                        yaw=None
                    )

                except Exception as exc:

                    print(
                        f"ERRORE LAND CF{i}: {exc}",
                        file=sys.stderr
                    )

            # Poi attendiamo la discesa.
            time.sleep(LANDING_TIME + 0.5)

            for i, cf in enumerate(
                (cf1, cf2),
                start=1
            ):

                try:
                    cf.high_level_commander.stop()

                except Exception as exc:

                    print(
                        f"ERRORE STOP CF{i}: {exc}",
                        file=sys.stderr
                    )


# ============================================================
# LOG
# ============================================================

def write_rows(output, rows, events):

    if rows:

        with (
            output / "crazy_sim_orca.csv"
        ).open("w", newline="") as fp:

            writer = csv.DictWriter(
                fp,
                fieldnames=list(rows[0])
            )

            writer.writeheader()
            writer.writerows(rows)

    if events:

        with (
            output / "events.csv"
        ).open("w", newline="") as fp:

            writer = csv.DictWriter(
                fp,
                fieldnames=[
                    "t",
                    "agent",
                    "event",
                    "point"
                ]
            )

            writer.writeheader()
            writer.writerows(events)


# ============================================================
# MAIN
# ============================================================

def run(args):

    from orca_lib import (
        compute_orca_constraint,
        choose_orca_velocity
    )

    if args.id1 == args.id2:
        raise ValueError("Rigid Body ID identici")

    if args.uri1 == args.uri2:
        raise ValueError("URI Crazyflie identici")

    if (
        not args.uri1.startswith("radio://")
        or not args.uri2.startswith("radio://")
    ):
        raise ValueError("Sono richiesti URI radio://")

    if (
        2 * args.orca_radius
        < EMERGENCY_DISTANCE + 0.08
    ):
        raise ValueError(
            "Diametro ORCA troppo piccolo "
            "rispetto alla distanza di emergenza"
        )

    output = Path(args.output)
    output.mkdir(
        parents=True,
        exist_ok=True
    )

    rows = []
    exp1 = None
    exp2 = None

    ids = (
        args.id1,
        args.id2
    )

    tracker = NatNetTracker(
        server_ip=args.motive_server,
        local_ip=args.motive_local,
        use_multicast=False
    )

    try:

        tracker.connect()

        # Lo streaming NatNet gira in un thread
        # indipendente dal calcolo ORCA.
        tracker.start_async()

        print("Motive connesso.")

        # --------------------------------------------------
        # DIAGNOSTICA SENZA DECOLLO
        # --------------------------------------------------

        if not args.fly:

            print(
                "Modalita' diagnostica: "
                "nessuna connessione radio, nessun decollo."
            )

            if args.check_tracking:

                print(
                    "Solleva manualmente i due Rigid Bodies."
                )

                end = (
                    time.monotonic()
                    + args.check_seconds
                )

                while time.monotonic() < end:

                    snap = tracker.get_tracking_snapshot(ids)

                    if snap["received_at"] is None:
                        age = float("inf")
                    else:
                        age = (
                            time.monotonic()
                            - snap["received_at"]
                        )

                    print(
                        f"frame={snap['frame']} "
                        f"age={age:.3f}s | "
                        f"RB{args.id1}="
                        f"{snap['bodies'][args.id1]} | "
                        f"RB{args.id2}="
                        f"{snap['bodies'][args.id2]}"
                    )

                    time.sleep(0.2)

            return

        # --------------------------------------------------
        # CONNESSIONI RADIO
        # --------------------------------------------------

        print(
            "ATTENZIONE: sessione reale "
            "autorizzata con --fly"
        )

        cflib.crtp.init_drivers(
            enable_debug_driver=False
        )

        with SyncCrazyflie(
            args.uri1,
            cf=Crazyflie(rw_cache="./cache")
        ) as scf1:

            with SyncCrazyflie(
                args.uri2,
                cf=Crazyflie(rw_cache="./cache")
            ) as scf2:

                print(
                    "Radio collegata: entrambi i Crazyflie"
                )

                with high_level_flight_pair(
                    scf1,
                    scf2
                ) as (cf1, cf2):

                    # ======================================
                    # ACQUISIZIONE GOAL DOPO IL DECOLLO
                    # ======================================

                    (
                        start1_3d,
                        start2_3d
                    ), first_frame = first_valid_pair_after_takeoff(
                        tracker,
                        ids
                    )

                    print(
                        "Posizioni iniziali:",
                        start1_3d,
                        start2_3d
                    )

                    print(
                        "Frame iniziale:",
                        first_frame
                    )

                    # ======================================
                    # CONTROLLO QUOTE
                    # ======================================

                    if MOTIVE_FLOOR_Y is None:

                        raise RuntimeError(
                            "Impostare MOTIVE_FLOOR_Y"
                        )

                    h1 = (
                        start1_3d[1]
                        - MOTIVE_FLOOR_Y
                    )

                    h2 = (
                        start2_3d[1]
                        - MOTIVE_FLOOR_Y
                    )

                    # ======================================
                    # ASSEGNAZIONE GOAL INVERTITI
                    # ======================================

                    start1 = mot_xy(start1_3d)
                    start2 = mot_xy(start2_3d)

                    sep0 = dist(
                        start1,
                        start2
                    )

                    if sep0 < max(
                        MIN_START_SEPARATION,
                        2 * args.orca_radius + 0.15
                    ):

                        raise RuntimeError(
                            "Droni troppo vicini "
                            "dopo il decollo"
                        )

                    # Il goal di CF1 e' start2.
                    # Il goal di CF2 e' start1.

                    exp1 = FixedSwapNavigator(
                        "CF1",
                        start1,
                        start2
                    )

                    exp2 = FixedSwapNavigator(
                        "CF2",
                        start2,
                        start1
                    )

                    print(
                        "GOAL CF1:",
                        exp1.goal
                    )

                    print(
                        "GOAL CF2:",
                        exp2.goal
                    )

                    print(
                        "I goal sono ora fissi."
                    )

                    # ======================================
                    # INIZIALIZZAZIONE ALGORITMI
                    # ======================================

                    est1 = VelocityEstimator(
                        VELOCITY_ALPHA
                    )

                    est2 = VelocityEstimator(
                        VELOCITY_ALPHA
                    )

                    est1.reset(start1)
                    est2.reset(start2)

                    al1 = ALANLiteSelector(start1)
                    al2 = ALANLiteSelector(start2)

                    rb1 = RightPassingSelector(
                        start1,
                        start2
                    )

                    rb2 = RightPassingSelector(
                        start2,
                        start1
                    )

                    print(
                        "Mode:",
                        args.mode
                    )

                    print(
                        "ORCA radius:",
                        args.orca_radius
                    )

                    print(
                        "Watchdog NatNet attivo"
                    )

                    print(
                        "Controllo ogni",
                        TRACKING_CHECK_PERIOD,
                        "secondi"
                    )

                    # ======================================
                    # CICLO PRINCIPALE
                    # ======================================

                    t0 = time.monotonic()

                    status = "TIMEOUT"

                    last_print = -1.0

                    last_used_frame = first_frame
                    next_command_at = t0

                    try:

                        while True:

                            tick = time.monotonic()
                            t = tick - t0

                            if t >= args.duration:
                                break

                            # --------------------------------
                            # WATCHDOG NATNET
                            # --------------------------------

                            snap = tracker.get_tracking_snapshot(
                                ids
                            )

                            health, reason = tracking_health(
                                snap,
                                ids,
                                tick
                            )

                            if health == "LOST":

                                status = "TRACKING_LOST"

                                print(
                                    "WATCHDOG NATNET:",
                                    reason
                                )

                                print(
                                    "LAND per entrambi"
                                )

                                break

                            # Nessun nuovo comando se:
                            # - tracking temporaneamente perso;
                            # - frame gia' utilizzato;
                            # - non e' ancora passato DT.

                            if (
                                health == "WAIT"
                                or snap["frame"] <= last_used_frame
                                or tick < next_command_at
                            ):

                                time.sleep(
                                    TRACKING_CHECK_PERIOD
                                )

                                continue

                            # --------------------------------
                            # POSIZIONI MOTIVE
                            # --------------------------------

                            pair = pair_from_snapshot(
                                snap,
                                ids
                            )

                            if pair is None:

                                time.sleep(
                                    TRACKING_CHECK_PERIOD
                                )

                                continue

                            p1_3d, p2_3d = pair

                            p1 = mot_xy(p1_3d)
                            p2 = mot_xy(p2_3d)

                            sep = dist(
                                p1,
                                p2
                            )

                            # --------------------------------
                            # CONTROLLI SICUREZZA
                            # --------------------------------

                            if sep < EMERGENCY_DISTANCE:

                                status = (
                                    f"SEPARATION_{sep:.3f}"
                                )

                                break

                            if (
                                dist(start1, p1)
                                > MAX_DISTANCE_FROM_START
                                or dist(start2, p2)
                                > MAX_DISTANCE_FROM_START
                            ):

                                status = "GEOFENCE"
                                break

                            if (
                                abs(
                                    p1_3d[1]
                                    - start1_3d[1]
                                ) > MAX_VERTICAL_CHANGE
                                or abs(
                                    p2_3d[1]
                                    - start2_3d[1]
                                ) > MAX_VERTICAL_CHANGE
                            ):

                                status = "VERTICAL_CHANGE"
                                break

                            # --------------------------------
                            # GOAL
                            # --------------------------------

                            exp1.update_goal(
                                t,
                                p1,
                                p2
                            )

                            exp2.update_goal(
                                t,
                                p2,
                                p1
                            )

                            if (
                                exp1.reached
                                and exp2.reached
                            ):

                                status = "BOTH_REACHED"
                                break

                            # --------------------------------
                            # VELOCITA' MISURATE
                            # --------------------------------

                            v1 = est1.update(p1)
                            v2 = est2.update(p2)

                            # --------------------------------
                            # VELOCITA' PREFERITE
                            # --------------------------------

                            goal1 = exp1.preferred_velocity(
                                p1
                            )

                            goal2 = exp2.preferred_velocity(
                                p2
                            )

                            # --------------------------------
                            # VINCOLI ORCA
                            # --------------------------------

                            lp1, ld1, u1 = compute_orca_constraint(
                                p1,
                                v1,
                                p2,
                                v2,
                                args.orca_radius,
                                TIME_HORIZON,
                                DT
                            )

                            lp2, ld2, u2 = compute_orca_constraint(
                                p2,
                                v2,
                                p1,
                                v1,
                                args.orca_radius,
                                TIME_HORIZON,
                                DT
                            )

                            # --------------------------------
                            # SELEZIONE ALGORITMO
                            # --------------------------------

                            if args.mode == "alan-lite":

                                pref1, action1 = al1.select(
                                    t,
                                    p1,
                                    exp1.goal,
                                    p2,
                                    goal1,
                                    lp1,
                                    ld1,
                                    choose_orca_velocity
                                )

                                pref2, action2 = al2.select(
                                    t,
                                    p2,
                                    exp2.goal,
                                    p1,
                                    goal2,
                                    lp2,
                                    ld2,
                                    choose_orca_velocity
                                )

                            elif args.mode == "right-bias":

                                pref1, action1 = rb1.select(
                                    p1,
                                    p2,
                                    goal1,
                                    2 * args.orca_radius
                                )

                                pref2, action2 = rb2.select(
                                    p2,
                                    p1,
                                    goal2,
                                    2 * args.orca_radius
                                )

                            else:

                                pref1 = goal1
                                pref2 = goal2

                                action1 = "GOAL"
                                action2 = "GOAL"

                            # --------------------------------
                            # SOLUZIONE ORCA
                            # --------------------------------

                            try:

                                c1 = choose_orca_velocity(
                                    pref1,
                                    lp1,
                                    ld1,
                                    MAX_SPEED
                                )

                                c2 = choose_orca_velocity(
                                    pref2,
                                    lp2,
                                    ld2,
                                    MAX_SPEED
                                )

                            except ValueError:

                                status = "ORCA_INFEASIBLE"
                                break

                            # --------------------------------
                            # ULTIMO CONTROLLO PRIMA DEL COMANDO
                            # --------------------------------

                            check_now = time.monotonic()

                            latest = tracker.get_tracking_snapshot(
                                ids
                            )

                            check_health, reason = tracking_health(
                                latest,
                                ids,
                                check_now
                            )

                            if check_health != "OK":

                                if check_health == "LOST":

                                    status = "TRACKING_LOST"

                                    print(
                                        "WATCHDOG NATNET:",
                                        reason
                                    )

                                    break

                                time.sleep(
                                    TRACKING_CHECK_PERIOD
                                )

                                continue

                            # --------------------------------
                            # COMANDI HIGH LEVEL COMMANDER
                            # --------------------------------

                            send_cmd(
                                cf1,
                                c1
                            )

                            send_cmd(
                                cf2,
                                c2
                            )

                            last_used_frame = snap["frame"]

                            next_command_at = (
                                tick + DT
                            )

                            # --------------------------------
                            # LOG
                            # --------------------------------

                            rows.append({
                                "t": round(t, 4),
                                "mode": args.mode,
                                "distance": sep,

                                "reached1": exp1.reached,
                                "reached2": exp2.reached,

                                "goal1x": exp1.goal[0],
                                "goal1y": exp1.goal[1],

                                "goal2x": exp2.goal[0],
                                "goal2y": exp2.goal[1],

                                "goal_distance1": dist(
                                    p1,
                                    exp1.goal
                                ),

                                "goal_distance2": dist(
                                    p2,
                                    exp2.goal
                                ),

                                "x1": p1[0],
                                "y1": p1[1],
                                "z1": p1_3d[1],

                                "x2": p2[0],
                                "y2": p2[1],
                                "z2": p2_3d[1],

                                "vx1": v1[0],
                                "vy1": v1[1],

                                "vx2": v2[0],
                                "vy2": v2[1],

                                "pref1x": pref1[0],
                                "pref1y": pref1[1],

                                "pref2x": pref2[0],
                                "pref2y": pref2[1],

                                "orca1x": c1[0],
                                "orca1y": c1[1],

                                "orca2x": c2[0],
                                "orca2y": c2[1],

                                "orca_u1": norm(u1),
                                "orca_u2": norm(u2),

                                "alan_action1": action1,
                                "alan_action2": action2,

                                "alan_activations1": al1.activations,
                                "alan_activations2": al2.activations,

                                "right_activations1": rb1.activations,
                                "right_activations2": rb2.activations,

                                "progress_window1": al1.last_progress,
                                "progress_window2": al2.last_progress,
                            })

                            if t - last_print > 1.0:

                                print(
                                    f"t={t:5.1f}s "
                                    f"distance={sep:.3f}m | "
                                    f"actions=({action1},{action2}) | "
                                    f"commands={c1},{c2} | "
                                    f"frame={snap['frame']}"
                                )

                                last_print = t

                            time.sleep(
                                TRACKING_CHECK_PERIOD
                            )

                    except KeyboardInterrupt:

                        status = "INTERRUPTED"

                        print(
                            "Interruzione manuale: "
                            "richiesta LAND"
                        )

                    finally:

                        print(
                            "TEST TERMINATO:",
                            status
                        )

                        # Uscendo dal blocco with,
                        # high_level_flight_pair()
                        # invia LAND a entrambi.

                print(
                    "Gestione HLC terminata."
                )

    finally:

        events = []

        for name, obj in (
            ("CF1", exp1),
            ("CF2", exp2)
        ):

            if obj is not None:

                for t, kind, pos in obj.events:

                    events.append({
                        "t": t,
                        "agent": name,
                        "event": kind,
                        "point": repr(pos)
                    })

        events.sort(
            key=lambda x: x["t"]
        )

        try:
            write_rows(
                output,
                rows,
                events
            )

        finally:
            tracker.close()

        if rows:
            print(
                "Log salvato:",
                (output / "crazy_sim_orca.csv").resolve()
            )


# ============================================================
# ARGOMENTI TERMINALE
# ============================================================

if __name__ == "__main__":

    ap = argparse.ArgumentParser(
        description=(
            "Motive REAL - "
            "ORCA / ALAN-lite / Right-bias"
        )
    )

    ap.add_argument(
        "--mode",
        choices=[
            "pure",
            "alan-lite",
            "right-bias"
        ],
        required=True
    )

    ap.add_argument(
        "--duration",
        type=float,
        default=50.0
    )

    ap.add_argument(
        "--orca-radius",
        type=float,
        default=0.30
    )

    ap.add_argument(
        "--output",
        default=None
    )

    ap.add_argument(
        "--fly",
        action="store_true"
    )

    ap.add_argument(
        "--check-tracking",
        action="store_true"
    )

    ap.add_argument(
        "--check-seconds",
        type=float,
        default=15.0
    )

    ap.add_argument(
        "--motive-server",
        default="127.0.0.1"
    )

    ap.add_argument(
        "--motive-local",
        default="127.0.0.1"
    )

    ap.add_argument(
        "--id1",
        type=int,
        default=MOTIVE_ID1
    )

    ap.add_argument(
        "--id2",
        type=int,
        default=MOTIVE_ID2
    )

    ap.add_argument(
        "--uri1",
        default=URI1
    )

    ap.add_argument(
        "--uri2",
        default=URI2
    )

    args = ap.parse_args()

    if args.output is None:

        args.output = (
            "motive_scambio_"
            + args.mode.replace("-", "_")
        )

    if args.duration <= 0:

        ap.error(
            "La durata deve essere positiva"
        )

    if args.check_seconds <= 0:

        ap.error(
            "check-seconds deve essere positivo"
        )

    if args.check_tracking and args.fly:

        ap.error(
            "--check-tracking e --fly "
            "non possono essere usati insieme"
        )

    run(args)