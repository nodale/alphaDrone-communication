from dataclasses import dataclass
from multiprocessing import shared_memory
import numpy as np
import socket
import msgpack

@dataclass
class QuickVicon:
    address: str = '10.183.217.138'
    port: int = 8020
    block: bool = False

    def __post_init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.address, self.port))
        self.sock.setblocking(self.block)

        # State: x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz
        self.state = np.zeros(13, dtype=np.float64)
        self.prev_pos = np.zeros(3, dtype=np.float64)
        self.prev_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # w, x, y, z
        self.prev_time = None

        self.shm = shared_memory.SharedMemory(name="vicon_state", create=True, size=self.state.nbytes)
        self.shared_state = np.ndarray(self.state.shape, dtype=self.state.dtype, buffer=self.shm.buf)

    def get_data(self):
        try:
            payload, addr = self.sock.recvfrom(1024)
            msg = msgpack.unpackb(payload, raw=False)

            # Vicon sends: timestamp, sequence, x, y, z, qw, qx, qy, qz
            timestamp = msg[0]
            x, y, z = msg[2], msg[3], msg[4]
            qw, qx, qy, qz = msg[5], msg[6], msg[7], msg[8]

            return timestamp, np.array([x, y, z], dtype=np.float64), np.array([qw, qx, qy, qz], dtype=np.float64)

        except BlockingIOError:
            return None, None, None

        except Exception as e:
            print("vicon error: ", e)
            return None, None, None

    def update_state(self):
        timestamp, pos, quat = self.get_data()
        if pos is None:
            return False 

        if self.prev_time is None:
            dt = 0.0
        else:
            dt = timestamp - self.prev_time
        if dt > 0:
            vel = (pos - self.prev_pos) / dt
        else:
            vel = np.zeros(3)

        wx, wy, wz = np.zeros(3)
        if self.prev_time is not None and dt > 0:
            wx, wy, wz = self.quaternion_to_angular_velocity(self.prev_quat, quat, dt)

        self.state[:3] = pos
        self.state[3:6] = vel
        self.state[6:10] = quat  # qw, qx, qy, qz
        self.state[10:13] = [wx, wy, wz]

        self.shared_state[:] = self.state

        self.prev_pos = pos
        self.prev_quat = quat
        self.prev_time = timestamp

        return True

    @staticmethod
    def quaternion_to_angular_velocity(q1, q2, dt):
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        q1_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])

        w1, x1, y1, z1 = q1_conj
        w2, x2, y2, z2 = q2
        xd = w2*x1 + x2*w1 + y2*z1 - z2*y1
        yd = w2*y1 - x2*z1 + y2*w1 + z2*x1
        zd = w2*z1 + x2*y1 - y2*x1 + z2*w1
        wd = w2*w1 - x2*x1 - y2*y1 - z2*z1

        angle = 2 * np.arccos(np.clip(wd, -1.0, 1.0))
        if angle > np.pi:
            angle -= 2*np.pi

        s = np.sqrt(1 - wd*wd)
        if s < 1e-8:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = np.array([xd, yd, zd]) / s

        omega = axis * (angle / dt)
        return omega
