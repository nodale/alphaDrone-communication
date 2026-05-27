import struct
import torch
import numpy as np

from multiprocessing import shared_memory
from include.quick_hongi import QuickHongi

def _q2e(q):
    q = q / q.norm(dim=-1, keepdim=True)
    w, x, y, z = q.unbind(dim=-1)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    sinp = torch.clamp(sinp, -1.0, 1.0)
    pitch = torch.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = torch.atan2(siny_cosp, cosy_cosp)
    euler = torch.stack((roll, pitch, yaw), dim=-1)

    return euler

def main():
    control_hongi = QuickHongi(device="cpu") 

    low_level_control = np.array([0.0]*4, dtype=np.float64)
    shm_low_level_control = shared_memory.SharedMemory(name="low_level_control", create=True, size=low_level_control.nbytes)
    shared_low_level_control = np.ndarray(low_level_control.shape, dtype=low_level_control.dtype, buffer=shm_low_level_control.buf)


    shm = shared_memory.SharedMemory(name="estimated_state")
    general_shm = shared_memory.SharedMemory(name="general_setpoint")

    state = np.array([0.0]*13, dtype=np.float64, buffer=shm)
    general_sp = np.zeros(6, dtype=np.float64, buffer=general_shm)


    while True:
        try:
            #for testing only
            #state = torch.tensor([0.0, 0.0, 0.0,
            #                      1.0, 0.0, 0.0, 0.0,
            #                      0.0, 0.0, 0.0,
            #                      0.0, 0.0, 0.0])

            #general_sp= torch.tensor([1.0, 0.0, 0.0,
            #                          0.0, 0.0, 0.0,
            #                          0.0, 0.0, 0.0,
            #                          0.0, 0.0, 0.0])

            rot = _q2e(q=state[3:7])
            states = torch.cat((
                    state[:3],
                    rot,
                    state[7:10],
                    state[10:13]
                    ))

            _act = control_hongi.get_action(states=states, setpoints=general_sp)

            shared_low_level_control = _act

        except Exception as e:        
            print("Error in main loop:", e)
            shm.close()
            shm.unlink()
            state_shm.close()
            state_shm.unlink()
            general_shm.close()
            general_shm.unlink()
            shm_low_level_control.close()
            shm_low_level_control.unlink()

if __name__ == "__main__":
    main()

