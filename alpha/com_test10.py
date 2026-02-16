import time
import numpy as np

from include.quick_keyboard import QuickKeyboard
from include.quick_mavlink import QuickMav
from include.quick_vicon import QuickVicon
from include.quick_viser import QuickViser


def main():
    keyboard = QuickKeyboard(file="flight_log.h5")
    keyboard.start()

    mav = QuickMav(
        address="/dev/ttyTHS1",   
        baudrate=921600
    )
    mav.sendHeartbeat()

    vicon = QuickVicon(
        address="10.183.217.138",
        port=8020,
        block=False
    )

    viser = QuickViser(
        port=8080,
        verbose=True
    )

    print("System initialized")
    print("Waiting for keyboard input...")

    t0 = time.time()
    latest_actuation = None

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


            est = mav.get("ODOMETRY", block=False)
            state_est = None

            if est is not None:
                x, y, z = est.x, est.y, est.z
                vx, vy, vz = est.vx, est.vy, est.vz
                q = est.q

                roll, pitch, yaw = vicon._quat_to_rpy_est(q)

                state_est = np.array([
                    x, y, z,
                    vx, vy, vz,
                    roll, pitch, yaw,
                    est.rollspeed if hasattr(est, "rollspeed") else 0.0,
                    est.pitchspeed if hasattr(est, "pitchspeed") else 0.0,
                    est.yawspeed if hasattr(est, "yawspeed") else 0.0,
                ], dtype=np.float32)

                if not keyboard.pause_flag:
                    t = time.time() - t0
                    row = np.array([
                        t,
                        x, y, z,
                        vx, vy, vz,
                        q[1], q[2], q[3], q[0],
                    ], dtype=np.float32)

                    keyboard.est_ds.resize(keyboard.est_idx + 1, axis=0)
                    keyboard.est_ds[keyboard.est_idx] = row
                    keyboard.est_idx += 1
                    keyboard.writer.flush()

            vic = vicon.get_data()
            state_vic = None
            if vic is not None:
                x, y, z, vx, vy, vz, q, v_roll, v_pitch, v_yaw = vic
                roll, pitch, yaw = vicon._quat_to_rpy_vic(q)

                state_vic = np.array([
                    x, y, z,
                    vx, vy, vz,
                    roll, pitch, yaw,
                    v_roll, v_pitch, v_yaw
                ], dtype=np.float32)

                mav.sendOdometry(
                        current_t,
                        (x, y, z),
                        (q[3], q[0], q[1], q[2]),
                        (vx, vy, vz),
                        (v_roll, v_pitch, v_yaw)
                        )

                if not keyboard.pause_flag:
                    t = time.time() - t0
                    row = np.array([
                        t,
                        x, y, z,
                        vx, vy, vz,
                        roll, pitch, yaw, 0.0
                    ], dtype=np.float32)

                    keyboard.vic_ds.resize(keyboard.vic_idx + 1, axis=0)
                    keyboard.vic_ds[keyboard.vic_idx] = row
                    keyboard.vic_idx += 1
                    keyboard.writer.flush()

            johnny = mav.get("JOHNNY_STATUS", block=False)
            if johnny is not None:
                latest_actuation = np.array(johnny.actuation, dtype=np.float32)

            viser.update_point_clouds(state_est, state_vic)
            viser.update_velocity_lines(state_est, state_vic)
            viser.update_x(state_est, state_vic)

            if latest_actuation is not None:
                viser.update_actuation(state_est, state_vic, latest_actuation)

            if keyboard.traverse_square_flag:
                mav.act_traverseSquare(current_t, (est.x, est.y, est.z))

            if keyboard.traverse_eight_flag:
                mav.act_traverseEight(current_t, (est.x, est.y, est.z))

            status_msg = (
                f"Pause: {keyboard.pause_flag}\n"
                f"Arm: {keyboard.arm_flag}\n"
                f"Kill: {keyboard.kill_flag}\n"
                f"Lift: {keyboard.set_to_lift_flag}\n"
                f"Land: {keyboard.set_to_land_flag}\n"
                f"Active: {keyboard.set_to_active_flag}\n"
                f"Traverse Square: {keyboard.traverse_square_flag}\n"
                f"Traverse Eight: {keyboard.traverse_eight_flag}\n"
                f"Actuation: {latest_actuation}\n"
            )
            viser.update_status(status_msg)

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        print("Shutting down...")
        keyboard.writer.close()
        mav.master.close()


if __name__ == "__main__":
    main()

