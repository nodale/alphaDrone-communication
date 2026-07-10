import os
os.environ["MAVLINK20"] = "1"

import time
import numpy as np

from multiprocessing import shared_memory, resource_tracker
from joystick.quick_keyboard import QuickKeyboard
from mavlink.quick_mavlink import QuickMav
from vicon.quick_vicon import QuickVicon
from viz.quick_viser import QuickViser
from control.quick_hongi import QuickHongi

def main():
    num_obj = 4

    LOOP_HZ = 200
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
    shm_general_state_sp = shared_memory.SharedMemory(name="general_setpoint")
    shm_actuation = shared_memory.SharedMemory(name="actuation")
    shm_dist = shared_memory.SharedMemory(name="dist")
    shm_corner = shared_memory.SharedMemory(name="obstacle_corners")
    shm_hongi = shared_memory.SharedMemory(name="low_level_control")

    vic_state = np.ndarray((num_obj,6), dtype=np.float64, buffer=shm_vic.buf)
    vic_init_state = np.ndarray((num_obj,6), dtype=np.float64, buffer=shm_init_vic.buf)
    state_sp = np.ndarray((4,), dtype=np.float64, buffer=shm_state_sp.buf)
    general_state_sp = np.ndarray((6,), dtype=np.float64, buffer=shm_general_state_sp.buf)
    actuation = np.ndarray((4,), dtype=np.float64, buffer=shm_actuation.buf)
    dist = np.ndarray((1,), dtype=np.float64, buffer=shm_dist.buf)
    corner = np.ndarray((num_obj, 4, 3), dtype=np.float64, buffer=shm_corner.buf)
    hongi = np.ndarray((4,), dtype=np.float64, buffer=shm_corner.buf)

    resource_tracker.unregister(shm_vic._name, "shared_memory")
    resource_tracker.unregister(shm_init_vic._name, "shared_memory")
    resource_tracker.unregister(shm_state_sp._name, "shared_memory")
    resource_tracker.unregister(shm_general_state_sp._name, "shared_memory")
    resource_tracker.unregister(shm_actuation._name, "shared_memory")
    resource_tracker.unregister(shm_dist._name, "shared_memory")
    resource_tracker.unregister(shm_corner._name, "shared_memory")
    resource_tracker.unregister(shm_hongi._name, "shared_memory")

    #dummies for testing
    #vic_state = np.zeros((num_obj,6), dtype=np.float64)
    #vic_init_state = np.zeros((num_obj,6), dtype=np.float64)
    #state_sp = np.zeros((4,), dtype=np.float64)
    #general_state_sp = np.zeros((6,), dtype=np.float64)
    #actuation = np.zeros((4,), dtype=np.float64)
    #dist = np.zeros((1,), dtype=np.float64)
    #corner = np.zeros((num_obj, 4, 3), dtype=np.float64)
    #hongi = np.zeros((4,), dtype=np.float64)

    try:
        while not keyboard.quit_flag:
            current_t = (time.monotonic_ns() // 1000) & 0xFFFFFFFF

            if not keyboard.pause_flag:
                data = np.hstack(([current_t], vic_state[0].flatten()))
                keyboard.log_vicon(data)
                keyboard.log_obstacle(vic_state[1:])
                keyboard.log_setpoint(general_state_sp)
                keyboard.log_actuation(actuation)
                keyboard.log_corner(corner[1:])
                keyboard.log_distance(dist)

                keyboard.writer.flush()

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

            if keyboard.traverse_bezier_flag:
                mav.act_traverseBezier(current_t, (vic_state[0][0], vic_state[0][1], vic_state[0][2]))

            if keyboard.manual_setpoint_flag:
                mav.sendPositionYawTarget(current_t, state_sp[0], state_sp[1], state_sp[2], state_sp[3])

            if keyboard.activate_hongi_flag:
                mav.sendSwitchControl(hongi)

            mav.sendOdometry(
                    current_t,
                    (vic_state[0][0], vic_state[0][1], vic_state[0][2]),
                    (vic_state[0][3], vic_state[0][4], vic_state[0][5])
                    )

            mav.sendLyapunovScalar(current_t, dist)

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
