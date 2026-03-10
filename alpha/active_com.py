import time
import numpy as np

from multiprocessing import shared_memory, resource_tracker
from include.quick_keyboard import QuickKeyboard
from include.quick_mavlink import QuickMav
from include.quick_vicon import QuickVicon
from include.quick_viser import QuickViser


def main():
    LOOP_HZ = 400
    LOOP_PERIOD = 1.0 / LOOP_HZ
    next_time = time.perf_counter()

    keyboard = QuickKeyboard(file="flight_log.h5")
    keyboard.start()

    mav = QuickMav(
        address="udpout:192.168.0.3:14561",   
        baudrate=921600,
        create=False
    )
    mav.sendHeartbeat()

    shm_vic = shared_memory.SharedMemory(name="vicon_state")
    shm_init_vic = shared_memory.SharedMemory(name="vicon_init_state")
    shm_state_sp = shared_memory.SharedMemory(name="joeystick_state_setpoint")

    vic_state = np.ndarray((13,), dtype=np.float64, buffer=shm_vic.buf)
    vic_init_state = np.ndarray((13,), dtype=np.float64, buffer=shm_init_vic.buf)
    state_sp = np.ndarray((4,), dtype=np.float64, buffer=shm_state_sp.buf)

    resource_tracker.unregister(shm_vic._name, "shared_memory")
    resource_tracker.unregister(shm_init_vic._name, "shared_memory")
    resource_tracker.unregister(shm_state_sp._name, "shared_memory")

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
                vic_init_state[:3] = 0
                time.sleep(0.1)
                vic_init_state[:3] = vic_state[:3]  

            if keyboard.set_to_lift_flag:
                mav.setTo_lift(current_t)

            if keyboard.set_to_land_flag:
                mav.setTo_land(current_t)

            if keyboard.set_to_active_flag:
                mav.setTo_active()
                keyboard.set_to_active_flag = False

            if keyboard.traverse_square_flag:
                mav.actTraverseSquare()
                keyboard.traverse_square_flag = False

            if keyboard.manual_setpoint_flag:
                mav.sendPositionTarget(current_t, state_sp[0], state_sp[1], state_sp[2])
                keyboard.manual_setpoint_flag = False

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
        keyboard.writer.close()
        mav.master.close()
        shm_vic.close()
        time.sleep(2)

    finally:
        print("Shutting down...")
        keyboard.writer.close()
        mav.master.close()
        shm_vic.close()
        time.sleep(2)


if __name__ == "__main__":
    main()

