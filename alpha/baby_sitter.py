import time
import traceback
import numpy as np

from include.quick_viser import QuickViser

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
        pitch = np.sign(sinp) * np.pi / 2  # gimbal lock
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

    #print("Viser server running...")
    #print("Open http://localhost:8080 in your browser")

    while True:
        try:
            _state_est = vis.est_state.copy()
            _state_vic = vis.vic_state.copy()
            _state_sp = vis.state_sp.copy()
            print(_state_sp)

            state_est = reform(_state_est)
            state_vic = reform(_state_vic)

            vis.update_point_clouds(state_est, state_vic, _state_sp)
            vis.update_velocity_lines(state_est, state_vic)
            vis.update_x(state_est, state_vic)
            vis.update_heading(state_est, state_vic)

            status_msg = (
                f"vic_position: {state_vic[0]}, {state_vic[1]}, {state_vic[2]}\n"
                f"vic_velocity: {state_vic[3]}, {state_vic[4]}, {state_vic[5]}\n"
                f"vic_rotation: {state_vic[6]}, {state_vic[7]}, {state_vic[8]}\n"
                f"vic_rotation: {state_vic[9]}, {state_vic[10]}, {state_vic[11]}\n"
            )
            vis.update_status(status_msg)

            time.sleep(0.002) 

        except Exception as e:
            vis.shm_est.close()
            vis.shm_vic.close()
            traceback.print_exc()
            time.sleep(1)

if __name__ == "__main__":
    main()
