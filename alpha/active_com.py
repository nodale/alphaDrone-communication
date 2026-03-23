import time
import numpy as np

from multiprocessing import shared_memory, resource_tracker
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
        address="udpout:192.168.0.3:14561",   
        baudrate=921600,
        create=False
    )
    mav.sendHeartbeat()

    shm_vic = shared_memory.SharedMemory(name="vicon_state")
    shm_init_vic = shared_memory.SharedMemory(name="vicon_init_state")
    shm_state_sp = shared_memory.SharedMemory(name="joeystick_state_setpoint")
    shm_actuation = shared_memory.SharedMemory(name="actuation")

    vic_state = np.ndarray((2,6), dtype=np.float64, buffer=shm_vic.buf)
    vic_init_state = np.ndarray((2,6), dtype=np.float64, buffer=shm_init_vic.buf)
    state_sp = np.ndarray((4,), dtype=np.float64, buffer=shm_state_sp.buf)
    actuation = np.ndarray((4,), dtype=np.float64, buffer=shm_actuation.buf)

    resource_tracker.unregister(shm_vic._name, "shared_memory")
    resource_tracker.unregister(shm_init_vic._name, "shared_memory")
    resource_tracker.unregister(shm_state_sp._name, "shared_memory")
    resource_tracker.unregister(shm_actuation._name, "shared_memory")

    try:
        while not keyboard.quit_flag:
            current_t = int(time.time() * 1e6) & 0xFFFFFFFF

            if not keyboard.pause_flag:
                keyboard.log_vicon(vic_state[0])
                keyboard.log_obstacle(vic_state[1])
                keyboard.log_setpoint(state_sp)
                keyboard.log_actuation(actuation)

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
                #mav.reboot() #disabled for now
                keyboard.reboot_flag = False
                vic_init_state[0][:3] = 0
                time.sleep(0.2)
                vic_init_state[0][:3] = vic_state[0][:3]  

            if keyboard.set_to_lift_flag:
                mav.setTo_lift(current_t)

            if keyboard.set_to_land_flag:
                mav.setTo_land(current_t)

            if keyboard.set_to_active_flag:
                mav.setTo_active()
                keyboard.set_to_active_flag = False

            if keyboard.traverse_square_flag:
                mav.act_traverseSquare(current_t, (vic_state[0][0], vic_state[0][1], vic_state[0][2]))

            if keyboard.traverse_eight_flag:
                mav.act_traverseEight(current_t, (vic_state[0][0], vic_state[0][1], vic_state[0][2]))

            if keyboard.manual_setpoint_flag:
                mav.sendPositionYawTarget(current_t, state_sp[0], state_sp[1], state_sp[2], state_sp[3])

            mav.sendOdometry(
                    current_t,
                    (vic_state[0][0], vic_state[0][1], vic_state[0][2]),
                    (vic_state[0][3], vic_state[0][4], vic_state[0][5])
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

