import time

from quickmav import QuickMav


def main():
    mav = QuickMav(
        address="/dev/ttyTHS1",
        baudrate=921600
    )

    print("Waiting for MAVLink heartbeat...")
    mav.master.wait_heartbeat()
    print("Connected to system:", mav.master.target_system)

    mav.sendHeartbeat()

    try:
        while True:
            msg = mav.master.recv_match(type="ODOMETRY", blocking=False)

            if msg is not None:
                mav.est_odo = msg
                mav.publish_odometry("estimated_state")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopping MAVLink interface...")

    finally:
        mav.shm.close()
        mav.shm.unlink()


if __name__ == "__main__":
    main()
