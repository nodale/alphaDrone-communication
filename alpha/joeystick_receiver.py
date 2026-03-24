import socket
import struct

import numpy as np
from multiprocessing import shared_memory

UDP_IP = "192.168.1.10"
UDP_PORT = 8001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

sock.settimeout(0.1)

#print("Listening...")

sp = np.zeros(4, dtype=np.float64)
shm = shared_memory.SharedMemory(name="joeystick_setpoint", create=True, size=sp.nbytes)
shared_sp = np.ndarray(sp.shape, dtype=sp.dtype, buffer=shm.buf)

state_sp = np.zeros(4, dtype=np.float64)
state_shm = shared_memory.SharedMemory(name="joeystick_state_setpoint", create=True, size=state_sp.nbytes)
shared_state_sp = np.ndarray(state_sp.shape, dtype=state_sp.dtype, buffer=state_shm.buf)

general_sp = np.zeros(3, dtype=np.float64)
general_shm = shared_memory.SharedMemory(name="general_setpoint", create=True, size=general_sp.nbytes)
general_shared_sp = np.ndarray(general_sp.shape, dtype=general_sp.dtype, buffer=general_shm.buf)
general_shared_sp[:] = general_sp

while True:
    try:
        data, addr = sock.recvfrom(1024)
        x, y, yaw, z = struct.unpack("ffff", data)

        sp[2], sp[3] = -1.0 * z, 0.2 * yaw

        _x = abs(x)
        _y = abs(y)

        if _x > 0.2:
            state_sp[0] += 0.01 * x
        else:
            state_sp[0] += 0.0

        if _y > 0.2:
            state_sp[1] += 0.01 * y
        else:
            state_sp[1] += 0.0

        shared_sp[:] = sp

        state_sp[2], state_sp[3] = sp[2], sp[3]
        shared_state_sp[:] = state_sp

        #print(f"x:{x:.3f} y:{y:.3f} yaw:{yaw:.3f} z:{z:.3f}")

    except socket.timeout:
        pass

    except Exception as e:        
        print("Error in main loop:", e)
        general_shm.close()
        general_shm.unlink()
        state_shm.close()
        state_shm.unlink()
