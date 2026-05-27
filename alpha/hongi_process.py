import time
import traceback

import numpy as np
import torch

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
    return torch.stack((roll, pitch, yaw), dim=-1)


def main():
    control_hongi = QuickHongi(device="cpu")

    low_level_control = np.zeros(4, dtype=np.float64)
    try:
        shm_low_level_control = shared_memory.SharedMemory(
            name="low_level_control",
            create=True,
            size=low_level_control.nbytes,
        )
    except FileExistsError:
        shm_low_level_control = shared_memory.SharedMemory(name="low_level_control", create=False)

    shared_low_level_control = np.ndarray(
        low_level_control.shape,
        dtype=low_level_control.dtype,
        buffer=shm_low_level_control.buf,
    )

    shm = shared_memory.SharedMemory(name="estimated_state")
    general_shm = shared_memory.SharedMemory(name="general_setpoint")

    state = np.ndarray((13,), dtype=np.float64, buffer=shm.buf)
    general_sp = np.ndarray((6,), dtype=np.float64, buffer=general_shm.buf)

    while True:
        try:
            state_t = torch.from_numpy(state.copy()).to(dtype=torch.float64)
            general_sp_t = torch.from_numpy(general_sp.copy()).to(dtype=torch.float64)

            rot = _q2e(q=state_t[3:7])
            states = torch.cat((
                state_t[:3],
                state_t[7:10],
                rot,
                state_t[10:13],
            ))
            setpoints = torch.zeros_like(states)
            setpoints[:3] = general_sp_t[:3]
            setpoints[3:6] = general_sp_t[3:6]

            actuation = control_hongi.get_action(states=states, setpoints=setpoints)
            shared_low_level_control[:] = np.asarray(actuation, dtype=np.float64)

        except Exception:
            print("Error in main loop:")
            shm.unlink()
            general_shm.unlink()
            shm_low_level_control.unlink()

            time.sleep(1)


if __name__ == "__main__":
    main()
