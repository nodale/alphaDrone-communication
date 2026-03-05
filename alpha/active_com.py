import time
import numpy as np

from multiprocessing import shared_memory
from include.quick_keyboard import QuickKeyboard
from include.quick_mavlink import QuickMav
from include.quick_vicon import QuickVicon
from include.quick_viser import QuickViser


def main():
    LOOP_HZ = 100
    LOOP_PERIOD = 1.0 / LOOP_HZ
    next_time = time.perf_counter()

    keyboard = QuickKeyboard(file="flight_log.h5")
    keyboard.start()

    mav = QuickMav(
        address="/dev/ttyTHS1",   
        baudrate=921600,
        create=False
    )
    mav.sendHeartbeat()

    shm_vic = shared_memory.SharedMemory(name="vicon_state")
    vic_state = np.ndarray((13,), dtype=np.float64, buffer=shm_vic.buf)

    try:
        while not keyboard.quit_flag:
            current_t = int(time.time() * 1e6) & 0xFFFFFFFF

            if keyboard.arm_flag:
                mav.arm()
                keyboard.arm_flag = False

            if keyboard.kill_flag:
                mav.disarm()
                keyboard.kill_flag = False

            if keyboard.force_kill_flag:
                mav.forceDisarm()
                keyboard.force_kill_flag = False

            if keyboard.reboot_flag:
                mav.reboot()
                keyboard.reboot_flag = False

            if keyboard.set_to_lift_flag:
                mav.setTo_lift(current_t)

            if keyboard.set_to_land_flag:
                mav.setTo_land(current_t)

            if keyboard.set_to_active_flag:
                mav.setTo_active()
                keyboard.set_to_active_flag = False

            mav.sendOdometry(
                    current_t,
                    (vic_state[0], vic_state[1], vic_state[2]),
                    (vic_state[6], vic_state[7], vic_state[8], vic_state[9]),
                    (vic_state[3], vic_state[4], vic_state[5]),
                    (vic_state[10], vic_state[11], vic_state[12])
                    )

            next_time += LOOP_PERIOD
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_time = time.perf_counter()

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Shutting down...")
        keyboard.writer.close()
        mav.master.close()


if __name__ == "__main__":
    main()

