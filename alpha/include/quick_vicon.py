from dataclasses import dataclass
from multiprocessing import shared_memory

import numpy as np
import socket
import msgpack

@dataclass
class QuickVicon:
    def __init__(self, address='10.183.217.138', port='8020', block=False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((address, port))

        self.sock.setblocking(block)

        self.state = np.array([0.0]*7, dtype=np.float64)
        self.shm = shared_memory.SharedMemory(name="vicon_state", create=True, size=self.state.nbytes)
        self.shared_state = np.ndarray(self.state.shape, dtype=self.state.dtype, buffer=self.shm.buf)

    def get_data(self):
        try:
            payload, addr = self.sock.recvfrom(1024)
            msg = msgpack.unpackb(payload, raw=False)

            #this depedns on the vicon's code
            # vicon sends:
            # timestamp, sequence, x, y, z, qw, qx, qy, qz
                
            self.state = [msg[2], msg[3], msg[4], msg[5], msg[6], msg[7], msg[8]]
            return self.state

        except BlockingIOError:
            return None

        except Exception as e:
            print("vicon error: ", e)
            return None

    def _publish_data(self):
        self.shared_state[:] = self.state


