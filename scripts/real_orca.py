# ============================================================
# WARNING / LOGGING SUPPRESSION
# IMPORTANT: THIS MUST BE BEFORE CFLIB IMPORTS
# ============================================================

import os
import sys
import warnings
import logging
import contextlib


# ------------------------------------------------------------
# PYTHON WARNINGS
# ------------------------------------------------------------

warnings.filterwarnings("ignore")
warnings.simplefilter("ignore")

# Also suppress warnings inherited by imported modules
os.environ["PYTHONWARNINGS"] = "ignore"


# ------------------------------------------------------------
# PYTHON LOGGING
# ------------------------------------------------------------

logging.disable(logging.CRITICAL)

for logger_name in [
    "cflib",
    "cflib.crtp",
    "cflib.crazyflie",
    "cflib.crazyflie.log",
    "cflib.crazyflie.syncCrazyflie",
    "cflib.positioning.motion_commander",
]:

    logger = logging.getLogger(logger_name)

    logger.handlers.clear()
    logger.propagate = False
    logger.disabled = True
    logger.setLevel(logging.CRITICAL + 1)


# ============================================================
# IMPORTS
# ============================================================

import time
import math
import csv

import cflib.crtp

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from natnet_tracker import NatNetTracker

from orca_lib import (
    compute_orca_constraint,
    choose_orca_velocity,
)


# ============================================================
# CONFIGURATION
# ============================================================

# ------------------------------------------------------------
# CRAZYFLIE
# ------------------------------------------------------------

URI_1 = "radio://0/80/2M/E7E7E7E7E9"
URI_2 = "radio://0/80/2M/E7E7E7E7E7"


# ------------------------------------------------------------
# MOTIVE
# ------------------------------------------------------------

MOTIVE_ID_1 = 2
MOTIVE_ID_2 = 3


# ------------------------------------------------------------
# FLIGHT
# ------------------------------------------------------------

FLIGHT_HEIGHT = 0.5

# Velocity at which each drone tries to reach the
# initial position of the other drone.
PREFERRED_SPEED = 0.05

# Maximum velocity ORCA is allowed to return.
MAX_SPEED = 0.07

# Control period: 20 Hz
DT = 0.05


# ------------------------------------------------------------
# ORCA
# ------------------------------------------------------------

# Radius assigned to EACH Crazyflie.
#
# orca_lib uses:
#
#       combined_radius = 2 * RADIUS
#
# therefore RADIUS=0.20 means a combined ORCA radius of 0.40 m.
RADIUS = 0.20

# Prediction horizon
TIME_HORIZON = 3.0


# ------------------------------------------------------------
# GOAL
# ------------------------------------------------------------

GOAL_TOLERANCE = 0.12


# ------------------------------------------------------------
# SAFETY
# ------------------------------------------------------------

# Hard emergency stop.
# This is independent of ORCA.
EMERGENCY_DISTANCE = 0.25

# Maximum horizontal displacement from the starting point.
MAX_DISTANCE_FROM_START = 1.20

# Motive Y is vertical.
MAX_VERTICAL_CHANGE = 0.20

# Maximum experiment duration.
MAX_TEST_TIME = 20.0

# Maximum time waiting for Motive.
MOTIVE_TIMEOUT = 10.0

# Number of samples required before accepting the
# initial Motive position.
REQUIRED_VALID_SAMPLES = 20


# ------------------------------------------------------------
# VELOCITY ESTIMATION
# ------------------------------------------------------------

VELOCITY_ALPHA = 0.25

# Reject obviously invalid finite-difference velocities.
MAX_VALID_MOTIVE_SPEED = 2.0


# ------------------------------------------------------------
# LOG
# ------------------------------------------------------------

CSV_FILE = "real_orca_log.csv"


# ============================================================
# MOTIVE -> CRAZYFLIE TRANSFORMATION
# ============================================================

def motive_to_cf_velocity(v_motive):
    """
    Convert a horizontal velocity expressed in Motive X-Z
    coordinates into Crazyflie world X-Y coordinates.

    Experimental calibration:

        +X_CF  ->  -Z_Motive
        +Y_CF  ->  -X_Motive

    Therefore:

        vx_CF = -vz_Motive
        vy_CF = -vx_Motive

    ORCA always works in Motive X-Z.
    This transformation is applied ONLY immediately before
    sending the velocity to the Crazyflie.
    """

    vx_motive, vz_motive = v_motive

    vx_cf = -vz_motive
    vy_cf = -vx_motive

    return vx_cf, vy_cf


# ============================================================
# VECTOR OPERATIONS
# ============================================================

def distance_2d(a, b):
    """
    Euclidean distance in Motive X-Z plane.
    """

    dx = b[0] - a[0]
    dz = b[1] - a[1]

    return math.sqrt(
        dx * dx +
        dz * dz
    )


def vector_norm(v):

    return math.sqrt(
        v[0] * v[0] +
        v[1] * v[1]
    )


# ============================================================
# NAVIGATION
# ============================================================

def velocity_toward_goal(position, goal, speed):
    """
    Compute preferred velocity toward the goal.

    Everything is expressed in Motive X-Z.

                 goal - position
    v_pref = V -------------------
                ||goal-position||
    """

    dx = goal[0] - position[0]
    dz = goal[1] - position[1]

    distance = math.sqrt(
        dx * dx +
        dz * dz
    )

    if distance < GOAL_TOLERANCE:

        return (
            0.0,
            0.0
        )

    return (
        speed * dx / distance,
        speed * dz / distance
    )


# ============================================================
# VELOCITY ESTIMATOR
# ============================================================

class VelocityEstimator:

    def __init__(self, alpha=0.25):

        self.alpha = alpha

        self.previous_position = None
        self.previous_time = None

        self.velocity = (
            0.0,
            0.0,
            0.0
        )


    def reset(self, position):

        self.previous_position = position
        self.previous_time = time.monotonic()

        self.velocity = (
            0.0,
            0.0,
            0.0
        )


    def update(self, position):

        now = time.monotonic()

        if self.previous_position is None:

            self.reset(position)

            return self.velocity


        dt = now - self.previous_time

        if dt <= 0.0:

            return self.velocity


        px, py, pz = self.previous_position
        x, y, z = position


        # ----------------------------------------------------
        # FINITE DIFFERENCE
        # ----------------------------------------------------

        raw_vx = (x - px) / dt
        raw_vy = (y - py) / dt
        raw_vz = (z - pz) / dt


        raw_speed = math.sqrt(
            raw_vx ** 2 +
            raw_vy ** 2 +
            raw_vz ** 2
        )


        # ----------------------------------------------------
        # REJECT INVALID SPIKES
        # ----------------------------------------------------

        if raw_speed <= MAX_VALID_MOTIVE_SPEED:

            a = self.alpha

            old_vx, old_vy, old_vz = self.velocity


            # ------------------------------------------------
            # EMA FILTER
            # ------------------------------------------------

            self.velocity = (

                a * raw_vx +
                (1.0 - a) * old_vx,

                a * raw_vy +
                (1.0 - a) * old_vy,

                a * raw_vz +
                (1.0 - a) * old_vz,
            )


        self.previous_position = position
        self.previous_time = now

        return self.velocity


# ============================================================
# MOTIVE
# ============================================================

def create_tracker():

    tracker = NatNetTracker(
        server_ip="127.0.0.1",
        local_ip="127.0.0.1",
        use_multicast=False,
    )

    tracker.connect()

    print("Motive connected.")

    return tracker


def read_two_drones(tracker):
    """
    One NatNet update, then read both rigid bodies.

    Motive gives:

        X, Y, Z

    with Y vertical.

    ORCA will later use only X-Z.
    """

    tracker.update()

    info1 = tracker.get_info_by_id(
        MOTIVE_ID_1
    )

    info2 = tracker.get_info_by_id(
        MOTIVE_ID_2
    )

    if (
        info1 is None
        or info2 is None
    ):

        return None

    return (
        info1["position"],
        info2["position"],
    )


# ============================================================
# CRAZYFLIE COMMANDS
# ============================================================

def send_zero_velocity(cf):

    cf.commander.send_velocity_world_setpoint(
        0.0,
        0.0,
        0.0,
        0.0
    )


def send_motive_velocity(cf, v_motive):
    """
    v_motive is an ORCA/navigation velocity expressed
    in Motive X-Z.

    Convert to Crazyflie world X-Y and send.
    """

    vx_cf, vy_cf = motive_to_cf_velocity(
        v_motive
    )

    cf.commander.send_velocity_world_setpoint(
        vx_cf,
        vy_cf,
        0.0,
        0.0
    )

    return (
        vx_cf,
        vy_cf
    )


# ============================================================
# WAIT FOR BOTH DRONES IN MOTIVE
# ============================================================

def wait_for_two_drones(
    tracker,
    cf1,
    cf2
):

    print(
        "Waiting for both Motive rigid bodies..."
    )

    valid_samples = []

    start_time = time.monotonic()


    while (
        time.monotonic() - start_time
        < MOTIVE_TIMEOUT
    ):

        # Keep both drones hovering while Motive acquisition
        # is performed.

        send_zero_velocity(cf1)
        send_zero_velocity(cf2)


        positions = read_two_drones(
            tracker
        )


        if positions is None:

            valid_samples = []

            time.sleep(DT)

            continue


        p1, p2 = positions

        valid_samples.append(
            (p1, p2)
        )


        if (
            len(valid_samples)
            >= REQUIRED_VALID_SAMPLES
        ):

            # ------------------------------------------------
            # AVERAGE INITIAL POSITION CF1
            # ------------------------------------------------

            p1_avg = (

                sum(
                    sample[0][0]
                    for sample in valid_samples
                ) / len(valid_samples),

                sum(
                    sample[0][1]
                    for sample in valid_samples
                ) / len(valid_samples),

                sum(
                    sample[0][2]
                    for sample in valid_samples
                ) / len(valid_samples),
            )


            # ------------------------------------------------
            # AVERAGE INITIAL POSITION CF2
            # ------------------------------------------------

            p2_avg = (

                sum(
                    sample[1][0]
                    for sample in valid_samples
                ) / len(valid_samples),

                sum(
                    sample[1][1]
                    for sample in valid_samples
                ) / len(valid_samples),

                sum(
                    sample[1][2]
                    for sample in valid_samples
                ) / len(valid_samples),
            )


            print(
                "Both drones acquired by Motive."
            )

            return (
                p1_avg,
                p2_avg
            )


        time.sleep(DT)


    return None


# ============================================================
# ORCA
# ============================================================

def calculate_orca_velocity(
    position,
    velocity,
    other_position,
    other_velocity,
    preferred_velocity
):
    """
    Calculate one ORCA constraint and select the safe velocity.

    ALL quantities in this function are expressed in
    Motive X-Z coordinates.
    """

    # --------------------------------------------------------
    # ORCA HALF-PLANE
    # --------------------------------------------------------

    line_point, line_direction, u = (
        compute_orca_constraint(

            position,
            velocity,

            other_position,
            other_velocity,

            RADIUS,
            TIME_HORIZON,
            DT
        )
    )


    # --------------------------------------------------------
    # SELECT VELOCITY CLOSEST TO v_pref
    # --------------------------------------------------------

    safe_velocity = (
        choose_orca_velocity(

            preferred_velocity,

            line_point,
            line_direction,

            MAX_SPEED
        )
    )


    return (
        safe_velocity,
        u
    )


# ============================================================
# SAFETY
# ============================================================

def check_safety(
    p1_3d,
    p2_3d,
    start1_3d,
    start2_3d
):

    x1, y1, z1 = p1_3d
    x2, y2, z2 = p2_3d

    sx1, sy1, sz1 = start1_3d
    sx2, sy2, sz2 = start2_3d


    # --------------------------------------------------------
    # 1. INTER-DRONE DISTANCE
    # --------------------------------------------------------

    separation = distance_2d(
        (x1, z1),
        (x2, z2)
    )


    if separation < EMERGENCY_DISTANCE:

        return (
            False,
            f"EMERGENCY DISTANCE: "
            f"{separation:.3f} m"
        )


    # --------------------------------------------------------
    # 2. VERTICAL MOTION
    #
    # Motive Y is vertical.
    # --------------------------------------------------------

    vertical_change_1 = abs(
        y1 - sy1
    )

    vertical_change_2 = abs(
        y2 - sy2
    )


    if (
        vertical_change_1
        > MAX_VERTICAL_CHANGE
    ):

        return (
            False,
            f"CF1 abnormal vertical change: "
            f"{vertical_change_1:.3f} m"
        )


    if (
        vertical_change_2
        > MAX_VERTICAL_CHANGE
    ):

        return (
            False,
            f"CF2 abnormal vertical change: "
            f"{vertical_change_2:.3f} m"
        )


    # --------------------------------------------------------
    # 3. GEOFENCE
    # --------------------------------------------------------

    dist_start_1 = distance_2d(
        (x1, z1),
        (sx1, sz1)
    )

    dist_start_2 = distance_2d(
        (x2, z2),
        (sx2, sz2)
    )


    if (
        dist_start_1
        > MAX_DISTANCE_FROM_START
    ):

        return (
            False,
            "CF1 left allowed area"
        )


    if (
        dist_start_2
        > MAX_DISTANCE_FROM_START
    ):

        return (
            False,
            "CF2 left allowed area"
        )


    return (
        True,
        "OK"
    )


# ============================================================
# CSV
# ============================================================

def create_csv():

    # "w" means every run starts with a NEW file.
    # Old experiment data are deleted.

    file = open(
        CSV_FILE,
        "w",
        newline=""
    )

    writer = csv.writer(
        file
    )


    writer.writerow([

        "time",

        # Positions
        "cf1_x_motive",
        "cf1_y_motive",
        "cf1_z_motive",

        "cf2_x_motive",
        "cf2_y_motive",
        "cf2_z_motive",

        # Measured velocities
        "cf1_vx_motive",
        "cf1_vz_motive",

        "cf2_vx_motive",
        "cf2_vz_motive",

        # Preferred velocities
        "cf1_pref_x",
        "cf1_pref_z",

        "cf2_pref_x",
        "cf2_pref_z",

        # ORCA safe velocities
        "cf1_safe_x",
        "cf1_safe_z",

        "cf2_safe_x",
        "cf2_safe_z",

        # ORCA correction u
        "cf1_u_x",
        "cf1_u_z",

        "cf2_u_x",
        "cf2_u_z",

        # Actual Crazyflie commands
        "cf1_cmd_vx",
        "cf1_cmd_vy",

        "cf2_cmd_vx",
        "cf2_cmd_vy",

        # Distances
        "distance",

        "cf1_goal_distance",
        "cf2_goal_distance",
    ])


    return (
        file,
        writer
    )


# ============================================================
# MAIN
# ============================================================

def main():

    tracker = None
    csv_file = None


    try:

        # ====================================================
        # MOTIVE
        # ====================================================

        tracker = create_tracker()


        # ====================================================
        # CSV
        # ====================================================

        csv_file, csv_writer = (
            create_csv()
        )


        # ====================================================
        # CFLIB DRIVERS
        #
        # Redirect stderr ONLY during driver initialization.
        # This suppresses low-level invasive driver messages.
        # ====================================================

        with open(
            os.devnull,
            "w"
        ) as devnull:

            with contextlib.redirect_stderr(
                devnull
            ):

                cflib.crtp.init_drivers(
                    enable_debug_driver=False
                )


        # ====================================================
        # CF1
        # ====================================================

        print("Connecting CF1...")


        with SyncCrazyflie(
            URI_1,
            cf=Crazyflie(
                rw_cache="./cache"
            )
        ) as scf1:


            print("CF1 connected.")


            # =================================================
            # CF2
            # =================================================

            print("Connecting CF2...")


            with SyncCrazyflie(
                URI_2,
                cf=Crazyflie(
                    rw_cache="./cache"
                )
            ) as scf2:


                print("CF2 connected.")


                # =============================================
                # TAKEOFF
                # =============================================

                print("Taking off...")


                with MotionCommander(
                    scf1,
                    default_height=FLIGHT_HEIGHT
                ) as mc1:


                    with MotionCommander(
                        scf2,
                        default_height=FLIGHT_HEIGHT
                    ) as mc2:


                        print(
                            "Both drones airborne."
                        )


                        # =====================================
                        # MOTIVE ACQUISITION
                        # =====================================

                        acquired = (
                            wait_for_two_drones(
                                tracker,
                                scf1.cf,
                                scf2.cf
                            )
                        )


                        if acquired is None:

                            print(
                                "ERROR: Motive acquisition failed."
                            )

                            return


                        start1_3d, start2_3d = (
                            acquired
                        )


                        # =====================================
                        # INITIAL POSITIONS
                        # =====================================

                        print()

                        print(
                            "=" * 70
                        )

                        print(
                            "INITIAL MOTIVE POSITIONS"
                        )

                        print(
                            "=" * 70
                        )


                        print(
                            f"CF1: "
                            f"X={start1_3d[0]:+.3f} "
                            f"Y={start1_3d[1]:+.3f} "
                            f"Z={start1_3d[2]:+.3f}"
                        )


                        print(
                            f"CF2: "
                            f"X={start2_3d[0]:+.3f} "
                            f"Y={start2_3d[1]:+.3f} "
                            f"Z={start2_3d[2]:+.3f}"
                        )


                        # =====================================
                        # ORCA PLANE = MOTIVE X-Z
                        # =====================================

                        start1 = (
                            start1_3d[0],
                            start1_3d[2]
                        )

                        start2 = (
                            start2_3d[0],
                            start2_3d[2]
                        )


                        initial_separation = (
                            distance_2d(
                                start1,
                                start2
                            )
                        )


                        print(
                            f"Initial horizontal separation: "
                            f"{initial_separation:.3f} m"
                        )


                        # =====================================
                        # SWAP GOALS
                        #
                        # CF1 wants CF2 initial position.
                        # CF2 wants CF1 initial position.
                        #
                        # Therefore WITHOUT ORCA their desired
                        # trajectories go directly toward each
                        # other.
                        # =====================================

                        goal1 = start2
                        goal2 = start1


                        print()

                        print(
                            f"CF1 goal: "
                            f"({goal1[0]:+.3f}, "
                            f"{goal1[1]:+.3f})"
                        )

                        print(
                            f"CF2 goal: "
                            f"({goal2[0]:+.3f}, "
                            f"{goal2[1]:+.3f})"
                        )

                        print(
                            "=" * 70
                        )


                        # =====================================
                        # VELOCITY ESTIMATORS
                        # =====================================

                        estimator1 = (
                            VelocityEstimator(
                                VELOCITY_ALPHA
                            )
                        )

                        estimator2 = (
                            VelocityEstimator(
                                VELOCITY_ALPHA
                            )
                        )


                        estimator1.reset(
                            start1_3d
                        )

                        estimator2.reset(
                            start2_3d
                        )


                        # =====================================
                        # STABILIZATION
                        # =====================================

                        print()
                        print(
                            "Stabilizing for 2 seconds..."
                        )


                        hover_start = (
                            time.monotonic()
                        )


                        while (
                            time.monotonic()
                            - hover_start
                            < 2.0
                        ):

                            send_zero_velocity(
                                scf1.cf
                            )

                            send_zero_velocity(
                                scf2.cf
                            )

                            tracker.update()

                            time.sleep(DT)


                        # =====================================
                        # LAST POSITION BEFORE START
                        # =====================================

                        positions = (
                            read_two_drones(
                                tracker
                            )
                        )


                        if positions is None:

                            print(
                                "ERROR: Tracking lost "
                                "before ORCA."
                            )

                            return


                        p1_3d, p2_3d = positions


                        estimator1.reset(
                            p1_3d
                        )

                        estimator2.reset(
                            p2_3d
                        )


                        # =====================================
                        # ORCA CONTROL LOOP
                        # =====================================

                        print()
                        print(
                            "Starting movement + ORCA."
                        )

                        print(
                            "Both drones now try to reach "
                            "the other's initial position."
                        )

                        print(
                            "ORCA modifies v_pref when a "
                            "collision is predicted."
                        )

                        print(
                            "CTRL+C -> stop and land."
                        )

                        print()


                        test_start = (
                            time.monotonic()
                        )


                        while True:

                            loop_start = (
                                time.monotonic()
                            )


                            # =================================
                            # TIME
                            # =================================

                            elapsed = (
                                loop_start -
                                test_start
                            )


                            if (
                                elapsed
                                > MAX_TEST_TIME
                            ):

                                print()
                                print(
                                    "Test timeout."
                                )

                                break


                            # =================================
                            # MOTIVE
                            # =================================

                            positions = (
                                read_two_drones(
                                    tracker
                                )
                            )


                            if positions is None:

                                print()
                                print(
                                    "SAFETY STOP: "
                                    "Motive tracking lost."
                                )

                                break


                            p1_3d, p2_3d = (
                                positions
                            )


                            # =================================
                            # SAFETY
                            # =================================

                            safe, reason = (
                                check_safety(
                                    p1_3d,
                                    p2_3d,
                                    start1_3d,
                                    start2_3d
                                )
                            )


                            if not safe:

                                print()
                                print(
                                    f"SAFETY STOP: "
                                    f"{reason}"
                                )

                                break


                            # =================================
                            # POSITIONS IN ORCA FRAME
                            #
                            # Motive X-Z
                            # =================================

                            p1 = (
                                p1_3d[0],
                                p1_3d[2]
                            )

                            p2 = (
                                p2_3d[0],
                                p2_3d[2]
                            )


                            # =================================
                            # REAL MEASURED VELOCITIES
                            # =================================

                            vel1_3d = (
                                estimator1.update(
                                    p1_3d
                                )
                            )

                            vel2_3d = (
                                estimator2.update(
                                    p2_3d
                                )
                            )


                            v1 = (
                                vel1_3d[0],
                                vel1_3d[2]
                            )

                            v2 = (
                                vel2_3d[0],
                                vel2_3d[2]
                            )


                            # =================================
                            # PREFERRED VELOCITIES
                            #
                            # THESE POINT TOWARD EACH OTHER.
                            # =================================

                            v_pref1 = (
                                velocity_toward_goal(
                                    p1,
                                    goal1,
                                    PREFERRED_SPEED
                                )
                            )


                            v_pref2 = (
                                velocity_toward_goal(
                                    p2,
                                    goal2,
                                    PREFERRED_SPEED
                                )
                            )


                            # =================================
                            # GOAL DISTANCES
                            # =================================

                            d_goal1 = (
                                distance_2d(
                                    p1,
                                    goal1
                                )
                            )

                            d_goal2 = (
                                distance_2d(
                                    p2,
                                    goal2
                                )
                            )


                            # =================================
                            # GOAL CHECK
                            # =================================

                            if (
                                d_goal1
                                < GOAL_TOLERANCE
                                and
                                d_goal2
                                < GOAL_TOLERANCE
                            ):

                                print()
                                print(
                                    "Both goals reached."
                                )

                                break


                            # =================================
                            # ORCA - CF1
                            # =================================

                            v_safe1, u1 = (
                                calculate_orca_velocity(

                                    p1,
                                    v1,

                                    p2,
                                    v2,

                                    v_pref1
                                )
                            )


                            # =================================
                            # ORCA - CF2
                            # =================================

                            v_safe2, u2 = (
                                calculate_orca_velocity(

                                    p2,
                                    v2,

                                    p1,
                                    v1,

                                    v_pref2
                                )
                            )


                            # =================================
                            # MOTIVE -> CRAZYFLIE
                            #
                            # IMPORTANT:
                            #
                            # ORCA output is still Motive X-Z.
                            #
                            # Here and ONLY here we convert it
                            # to Crazyflie world X-Y.
                            # =================================

                            cmd1_vx, cmd1_vy = (
                                send_motive_velocity(
                                    scf1.cf,
                                    v_safe1
                                )
                            )


                            cmd2_vx, cmd2_vy = (
                                send_motive_velocity(
                                    scf2.cf,
                                    v_safe2
                                )
                            )


                            # =================================
                            # CURRENT SEPARATION
                            # =================================

                            separation = (
                                distance_2d(
                                    p1,
                                    p2
                                )
                            )


                            # =================================
                            # ORCA CORRECTION MAGNITUDES
                            # =================================

                            u1_norm = vector_norm(u1)
                            u2_norm = vector_norm(u2)


                            # =================================
                            # CSV LOG
                            # =================================

                            csv_writer.writerow([

                                elapsed,

                                # Positions
                                p1_3d[0],
                                p1_3d[1],
                                p1_3d[2],

                                p2_3d[0],
                                p2_3d[1],
                                p2_3d[2],

                                # Measured velocities
                                v1[0],
                                v1[1],

                                v2[0],
                                v2[1],

                                # Preferred velocities
                                v_pref1[0],
                                v_pref1[1],

                                v_pref2[0],
                                v_pref2[1],

                                # Safe velocities
                                v_safe1[0],
                                v_safe1[1],

                                v_safe2[0],
                                v_safe2[1],

                                # ORCA correction
                                u1[0],
                                u1[1],

                                u2[0],
                                u2[1],

                                # Crazyflie commands
                                cmd1_vx,
                                cmd1_vy,

                                cmd2_vx,
                                cmd2_vy,

                                # Separation
                                separation,

                                # Goal distances
                                d_goal1,
                                d_goal2,
                            ])


                            csv_file.flush()


                            # =================================
                            # TERMINAL STATUS
                            #
                            # One line only.
                            #
                            # pref = desired velocity
                            # safe = ORCA velocity
                            # |u|  = ORCA correction
                            # =================================

                            print(

                                f"\r"

                                f"t={elapsed:5.1f}s | "

                                f"d={separation:.3f}m | "

                                f"CF1 "
                                f"pref=({v_pref1[0]:+.3f},"
                                f"{v_pref1[1]:+.3f}) "
                                f"safe=({v_safe1[0]:+.3f},"
                                f"{v_safe1[1]:+.3f}) "
                                f"|u|={u1_norm:.3f} | "

                                f"CF2 "
                                f"pref=({v_pref2[0]:+.3f},"
                                f"{v_pref2[1]:+.3f}) "
                                f"safe=({v_safe2[0]:+.3f},"
                                f"{v_safe2[1]:+.3f}) "
                                f"|u|={u2_norm:.3f}",

                                end="",
                                flush=True
                            )


                            # =================================
                            # CONTROL PERIOD
                            # =================================

                            loop_time = (
                                time.monotonic()
                                - loop_start
                            )


                            sleep_time = (
                                DT -
                                loop_time
                            )


                            if sleep_time > 0:

                                time.sleep(
                                    sleep_time
                                )


                        # =====================================
                        # STOP BEFORE LANDING
                        # =====================================

                        print()
                        print(
                            "Stopping both drones..."
                        )


                        for _ in range(20):

                            send_zero_velocity(
                                scf1.cf
                            )

                            send_zero_velocity(
                                scf2.cf
                            )

                            time.sleep(
                                0.05
                            )


                        print(
                            "Landing..."
                        )


                print(
                    "Both drones landed."
                )


        print(
            "Radio connections closed."
        )


    # ========================================================
    # CTRL+C
    # ========================================================

    except KeyboardInterrupt:

        print()
        print()
        print(
            "CTRL+C received."
        )

        print(
            "Stopping experiment and closing connections."
        )


    # ========================================================
    # REAL ERRORS ARE NOT HIDDEN
    # ========================================================

    except Exception as e:

        print()
        print(
            f"ERROR: {type(e).__name__}: {e}"
        )


    # ========================================================
    # CLEANUP
    # ========================================================

    finally:

        # ----------------------------------------------------
        # CSV
        # ----------------------------------------------------

        if csv_file is not None:

            try:

                csv_file.close()

            except Exception:

                pass


        # ----------------------------------------------------
        # NATNET
        # ----------------------------------------------------

        if tracker is not None:

            try:

                tracker.close()

                print(
                    "NatNet closed."
                )

            except Exception:

                pass


        print(
            "Program terminated."
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()