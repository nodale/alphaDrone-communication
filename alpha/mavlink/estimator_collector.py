import time
import traceback

from mavlink.quick_mavlink import QuickMav


def main():
    mav = QuickMav(
        address="/dev/ttyTHS1",
        baudrate=921600,
        create=True
    )


    mav.sendHeartbeat()

    while True:
        try:
            msg_odo = mav.getOdometry()
            msg_act = mav.getJohnny()

            if msg_odo is not None:
                mav.publish_odometry("estimated_state")

            if msg_act is not None:
                #print(msg_act)
                mav.publish_actuation("actuation")

            #print("states : ", mav.shared_state.copy())

        except Exception as e:
            print("Error in main loop:", e)
            mav.shm.close()
            mav.shm.unlink()
            mav.shm_general_sp.close()
            mav.shm_general_sp.unlink()
            traceback.print_exc()


if __name__ == "__main__":
    main()
