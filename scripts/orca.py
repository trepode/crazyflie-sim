import time
import threading
import warnings

warnings.filterwarnings("ignore", category=DeprecationWarning)

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

from orca import compute_orca_constraint, choose_orca_velocity


URI_CF1 = "udp://127.0.0.1:19850"
URI_CF2 = "udp://127.0.0.1:19851"

URIS = {
    URI_CF1,
    URI_CF2,
}

FLIGHT_HEIGHT = 0.5
SPEED = 0.2
MOVE_TIME = 4.0

RADIUS = 0.15
TIME_HORIZON = 2.0
DT = 0.05
MAX_SPEED = 0.2

ORCA_LOG_FILE = "orca_log.csv"
LOG_FILE = "flight_log.txt"

# --------------------------------------------------
# SHARED STATE
# --------------------------------------------------

states = {
    URI_CF1: {
        "x": 0.0,
        "y": 0.0,
        "vx": 0.0,
        "vy": 0.0,
    },
    URI_CF2: {
        "x": 0.0,
        "y": 0.0,
        "vx": 0.0,
        "vy": 0.0,
    }
}

state_lock = threading.Lock()


# --------------------------------------------------
# LOG CALLBACK
# --------------------------------------------------

def log_callback(uri):

    def callback(timestamp, data, logconf):

        with state_lock:

            states[uri]["x"] = data["stateEstimate.x"]
            states[uri]["y"] = data["stateEstimate.y"]

            states[uri]["vx"] = data["stateEstimate.vx"]
            states[uri]["vy"] = data["stateEstimate.vy"]

    return callback


# --------------------------------------------------
# CONFIGURE LOGGING
# --------------------------------------------------

def start_logging(scf):

    uri = scf.cf.link_uri

    log_conf = LogConfig(
        name="State",
        period_in_ms=50
    )

    log_conf.add_variable(
        "stateEstimate.x",
        "float"
    )

    log_conf.add_variable(
        "stateEstimate.y",
        "float"
    )

    log_conf.add_variable(
        "stateEstimate.vx",
        "float"
    )

    log_conf.add_variable(
        "stateEstimate.vy",
        "float"
    )

    scf.cf.log.add_config(log_conf)

    log_conf.data_received_cb.add_callback(
        log_callback(uri)
    )

    log_conf.start()

    return log_conf


# --------------------------------------------------
# FLIGHT
# --------------------------------------------------

def fly_toward_each_other(scf):

    uri = scf.cf.link_uri
    cf = scf.cf

    log_conf = start_logging(scf)

    with MotionCommander(
        scf,
        default_height=FLIGHT_HEIGHT
    ):

        print(f"[{uri}] Takeoff")

        time.sleep(2.0)

        if uri == URI_CF1:

            vx = SPEED
            vy = 0.0

        else:

            vx = -SPEED
            vy = 0.0

        print(
            f"[{uri}] "
            f"v_pref = ({vx:.2f}, {vy:.2f})"
        )

        start_time = time.time()
        last_print = 0

        while time.time() - start_time < MOVE_TIME:

            elapsed = time.time() - start_time

            # --------------------------------------------------
            # ORCA COMPUTATION - ONLY FOR LOGGING
            # --------------------------------------------------

            other_uri = URI_CF2 if uri == URI_CF1 else URI_CF1

            with state_lock:
                my_state = states[uri].copy()
                other_state = states[other_uri].copy()

            p_a = (
                my_state["x"],
                my_state["y"]
            )

            v_a = (
                my_state["vx"],
                my_state["vy"]
            )

            p_b = (
                other_state["x"],
                other_state["y"]
            )

            v_b = (
                other_state["vx"],
                other_state["vy"]
            )

            dx = p_b[0] - p_a[0]
            dy = p_b[1] - p_a[1]

            distance = (dx**2 + dy**2)**0.5

            line_point, line_direction, u = compute_orca_constraint(
                p_a=p_a,
                v_a=v_a,
                p_b=p_b,
                v_b=v_b,
                radius=RADIUS,
                time_horizon=TIME_HORIZON,
                dt=DT
            )

            # Preferred velocity
            v_pref = (vx, vy)

            # ORCA safe velocity
            v_new = choose_orca_velocity(
                v_pref=v_pref,
                line_point=line_point,
                line_direction=line_direction,
                max_speed=MAX_SPEED
            )

            with open(ORCA_LOG_FILE, "a") as f:
                f.write(
                    f"{elapsed:.3f},"
                    f"{uri},"
                    f"{distance:.4f},"
                    f"{u[0]:.4f},"
                    f"{u[1]:.4f},"
                    f"{line_point[0]:.4f},"
                    f"{line_point[1]:.4f},"
                    f"{v_new[0]:.4f},"
                    f"{v_new[1]:.4f}\n"
                )

            # Send velocity command
            cf.commander.send_velocity_world_setpoint(
                v_new[0],
                v_new[1],
                0.0,
                0.0
            )

            # Print approximately every 0.25 s
            if elapsed - last_print >= 0.25:

                with state_lock:

                    s = states[uri].copy()

                with open(LOG_FILE, "a") as f:
                    f.write(
                        f"{elapsed:.2f},"
                        f"{uri},"
                        f"{s['x']:.4f},"
                        f"{s['y']:.4f},"
                        f"{s['vx']:.4f},"
                        f"{s['vy']:.4f}\n"
                    )

                last_print = elapsed

            time.sleep(0.05)

        print(f"[{uri}] Stop")

        # Stop horizontal movement
        for _ in range(20):

            cf.commander.send_velocity_world_setpoint(
                0.0,
                0.0,
                0.0,
                0.0
            )

            time.sleep(0.05)

    log_conf.stop()

    print(f"[{uri}] Landed")


# --------------------------------------------------
# MAIN
# --------------------------------------------------

def main():

    # Clear log file at the beginning of every test
    with open(LOG_FILE, "w") as f:
        f.write("time,drone,x,y,vx,vy\n")

    with open(ORCA_LOG_FILE, "w") as f:
        f.write(
            "time,drone,distance,"
            "ux,uy,"
            "line_x,line_y,"
            "vnew_x,vnew_y\n"
        )

    cflib.crtp.init_drivers()

    factory = CachedCfFactory(
        rw_cache="./cache"
    )

    with Swarm(
        URIS,
        factory=factory
    ) as swarm:

        swarm.parallel_safe(
            fly_toward_each_other
        )

    print("Test completed")
    print(f"Flight data saved to {LOG_FILE}")


if __name__ == "__main__":
    main()
