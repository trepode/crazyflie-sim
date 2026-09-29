import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander


URI = "radio://0/80/2M/E7E7E7E7E3"


def main():
    cflib.crtp.init_drivers()

    print("Connecting to Crazyflie...")

    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache"),
    ) as scf:

        print("Connected!")

        # Abilita i motori
        scf.cf.supervisor.send_arming_request(True)
        time.sleep(1.0)

        # Non usiamo "with MotionCommander"
        mc = MotionCommander(
            scf,
            default_height=0.5,
        )

        try:
            print("Takeoff")

            # Decollo più lento del valore predefinito
            mc.take_off(
                height=1,  # 0.5
                velocity=0.10,
            )

            time.sleep(2.0)

            # print("Forward")
            # mc.forward(
            #     0.5,
            #     velocity=0.15,
            # )
            # time.sleep(1.0)

            # print("Back")
            # mc.back(
            #     0.5,
            #     velocity=0.15,
            # )
            # time.sleep(1.0)

            # print("Left")
            # mc.left(
            #     0.5,
            #     velocity=0.15,
            # )
            # time.sleep(1.0)

            # print("Right")
            # mc.right(
            #     0.5,
            #     velocity=0.15,
            # )
            # time.sleep(1.0)

            # Ferma ogni movimento orizzontale
            print("Hover prima dell'atterraggio")
            mc.stop()
            time.sleep(2.0)

            # Prima parte dell'atterraggio:
            # da 0.5 m a circa 0.2 m
            # print("Discesa fino a 20 cm")
            # mc.down(
            #     0.30,
            #     velocity=0.07,
            # )

            # Pausa stabile a bassa quota
            mc.stop()
            time.sleep(1.5)

            print("Atterraggio finale molto lento")

        finally:
            # Da circa 20 cm fino al suolo
            mc.land(
                velocity=0.03,
            )

            time.sleep(0.5)

            scf.cf.supervisor.send_arming_request(False)

            print("Motori disarmati")


if __name__ == "__main__":
    main()