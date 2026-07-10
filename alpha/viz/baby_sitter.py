import time
import traceback
import numpy as np

from viz.quick_viser import QuickViser

##################### PARAM #######################

PORT = 8080

##################### ##### #######################

def reform(state):
    new_state = np.zeros_like(state)
    new_state[:3] = state[:3]       # x, y, z
    new_state[3:6] = state[3:6]     # vx, vy, vz
    new_state[10:] = state[10:]     # wx, wy, wz

    qw, qx, qy, qz = state[6:10]

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    new_state[6:9] = [roll, pitch, yaw]

    return new_state

def main():
    vis = QuickViser(
        port=PORT,
        verbose=False
    )
    while True:
        try:
            _state_est = vis.est_state.copy()
            _state_vic = vis.vic_state[0].copy()
            _state_sp = vis.state_sp.copy()
            _state_box = vis.vic_state[1:].copy()

            state_est = reform(_state_est)

            vis.update_point_clouds(state_est, _state_vic, _state_sp)
            vis.update_velocity_lines(state_est)
            vis.update_x(state_est, _state_vic)
            vis.update_heading(state_est, _state_vic)
            vis.update_obstacles(_state_box)

            status_msg = (
                f"vic_position: {_state_vic[0]}, {_state_vic[1]}, {_state_vic[2]}\n"
                f"vic_rotation: {_state_vic[3]}, {_state_vic[4]}, {_state_vic[5]}\n"
                f"est_position: {state_est[0]}, {state_est[1]}, {state_est[2]}\n"
                f"est_rotation: {state_est[6]}, {state_est[7]}, {state_est[8]}\n"
            )
            vis.update_status(status_msg)

            time.sleep(0.005)

        except Exception as e:
            vis.shm_est.close()
            vis.shm_est.unlink()
            vis.shm_vic.close()
            vis.shm_vic.unlink()
            vis.shm_state_sp.close()
            vis.shm_state_sp.unlink()
            traceback.print_exc()
            time.sleep(1)

if __name__ == "__main__":
    main()
