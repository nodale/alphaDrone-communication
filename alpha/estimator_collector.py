import time
import traceback

from include.quick_mavlink import QuickMav


def main():
    mav = QuickMav(
        address="/dev/ttyTHS1",
        baudrate=921600,
        create=True
    )


    mav.sendHeartbeat()

    while True:
        try:
            msg = mav.master.recv_match(type="ODOMETRY", blocking=False)

            if msg is not None:
                mav.est_odo = msg
                mav.publish_odometry("estimated_state")

            #print("states : ", mav.shared_state.copy())
            #time.sleep(0.01)

        except Exception as e:
            print("Error in main loop:", e)
            mav.shm.close()
            mav.shm.unlink()
            mav.shm_general_sp.close()
            mav.shm_general_sp.unlink()
            traceback.print_exc()

            #time.sleep(1)

    #except KeyboardInterrupt:
    #    #print("Stopping MAVLink interface...")
    #    mav.shm.close()
    #    mav.shm.unlink()
    #    time.sleep(1)
    #    
    #finally:
    #    mav.shm.close()
    #    mav.shm.unlink()
    #    time.sleep(1)


if __name__ == "__main__":
    main()
