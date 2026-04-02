from dataclasses import dataclass
from multiprocessing import shared_memory
import numpy as np
import socket
import msgpack


@dataclass
class QuickVicon:
    num_objects: int = 4
    dt: float = 0.0125
    alpha_vel: float = 0.2
    alpha_omega: float = 0.2

    def __init__(self, address, port, block):
        self.address, self.port, self.block = address, port, block

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.address, self.port))
        self.sock.setblocking(self.block)

        # [x,y,z,wx,wy,wz]
        self.state = np.zeros((self.num_objects, 6), dtype=np.float64)

        shm_size = self.state.nbytes
        self.shm = shared_memory.SharedMemory(name="vicon_state", create=True, size=shm_size)
        self.shared_state = np.ndarray(self.state.shape, dtype=self.state.dtype, buffer=self.shm.buf)

        self.init_state = np.zeros((self.num_objects, 6), dtype=np.float64)
        self.init_shm = shared_memory.SharedMemory(name="vicon_init_state", create=True, size=self.init_state.nbytes)
        self.init_shared_state = np.ndarray(self.init_state.shape, dtype=self.init_state.dtype, buffer=self.init_shm.buf)

        self.obs_corners = np.zeros((self.num_objects, 4, 3), dtype=np.float64)
        self.obs_corners_shm = shared_memory.SharedMemory(name="obstacle_corners", create=True, size=self.obs_corners.nbytes)
        self.shared_obs_corners = np.ndarray(self.obs_corners.shape, dtype=self.obs_corners.dtype, buffer=self.obs_corners_shm.buf)

    def get_data(self):
        try:
            payload, addr = self.sock.recvfrom(2048)
            msg = msgpack.unpackb(payload, raw=False)

            # (seq, timestamp, [[id,x,y,z,qx,qy,qz], ...])
            seq = msg[0]
            timestamp = msg[1]
            objects = msg[2]
            corners = msg[3]

            return timestamp, objects, corners

        except BlockingIOError:
            return None, None, None

        except Exception as e:
            print("vicon error:", e)
            return None, None, None

    def update(self):
        timestamp, objects, corners = self.get_data()

        if objects is None:
            return False

        else:
            dt = self.dt

            for obj in objects:
                obj_id, x, y, z, qx, qy, qz = obj

                pos = np.array([x, y, z])
                angle = np.array([qx, qy, qz], dtype=np.float64)

                self.state[obj_id, 0:3] = pos - self.init_shared_state[0, 0:3]
                self.state[obj_id, 3:6] = angle

            self.shared_state[:] = self.state

            for obj_id, corner_set in corners:
                self.obs_corners[obj_id] = np.asarray(corner_set, dtype=np.float64)

            self.shared_obs_corners[:] = self.obs_corners

            return True
