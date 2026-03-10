import socket
import struct

import numpy as np
from multiprocessing import shared_memory

UDP_IP = "10.183.217.138"
UDP_PORT = 8001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

sock.settimeout(0.1)

print("Listening...")

sp = np.zeros(4, dtype=np.float64)
shm = shared_memory.SharedMemory(name="joeystick_setpoint", create=True, size=sp.nbytes)
shared_sp = np.ndarray(sp.shape, dtype=sp.dtype, buffer=shm.buf)

state_sp = np.zeros(4, dtype=np.float64)
state_shm = shared_memory.SharedMemory(name="joeystick_state_setpoint", create=True, size=state_sp.nbytes)
shared_state_sp = np.ndarray(state_sp.shape, dtype=state_sp.dtype, buffer=state_shm.buf)

shm_vic = shared_memory.SharedMemory(name="vicon_state")
vic_state = np.ndarray((13,), dtype=np.float64, buffer=shm_vic.buf)

while True:
    try:
        data, addr = sock.recvfrom(1024)
        x, y, yaw, z = struct.unpack("ffff", data)

        sp[0], sp[1], sp[2], sp[3] = 0.1 * x, 0.1 * y, z, 0.2 * yaw
        shared_sp[:] = sp

        state_sp = vic_state[:2] + sp[:2]
        shared_state_sp[:] = state_sp

        print(f"x:{x:.3f} y:{y:.3f} yaw:{yaw:.3f} z:{z:.3f}")

    except socket.timeout:
        pass
