from dataclasses import dataclass
from multiprocessing import shared_memory
import numpy as np
import socket
import msgpack


@dataclass
class QuickVicon:
    num_objects: int = 2
    dt: float = 0.0125
    alpha_vel: float = 0.2
    alpha_omega: float = 0.2

    def __init__(self, address, port, block):
        self.address, self.port, self.block = address, port, block

    #def __post_init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.address, self.port))
        self.sock.setblocking(self.block)

        # [x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz]
        self.state = np.zeros((self.num_objects, 13), dtype=np.float64)

        self.filtered_quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (self.num_objects, 1))
        self.prev_filtered_quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (self.num_objects, 1))

        self.filtered_pos = np.zeros((self.num_objects, 3))
        self.filtered_quat = np.tile(np.array([1,0,0,0]), (self.num_objects,1))
        #self.filtered_quat = np.zeros((self.num_objects, 4), dtype=np.float64)

        self.prev_time = None

        shm_size = self.state.nbytes
        self.shm = shared_memory.SharedMemory(name="vicon_state", create=True, size=shm_size)
        self.shared_state = np.ndarray(self.state.shape, dtype=self.state.dtype, buffer=self.shm.buf)

        self.init_state = np.zeros((self.num_objects, 13), dtype=np.float64)
        self.init_shm = shared_memory.SharedMemory(name="vicon_init_state", create=True, size=self.init_state.nbytes)
        self.init_shared_state = np.ndarray(self.init_state.shape, dtype=self.init_state.dtype, buffer=self.init_shm.buf)

    def get_data(self):
        try:
           payload, addr = self.sock.recvfrom(4096)
           msg = msgpack.unpackb(payload, raw=False)

           # (seq, timestamp, [[id,x,y,z,qw,qx,qy,qz], ...])
           seq = msg[0]
           timestamp = msg[1]
           objects = msg[2]

           return timestamp, objects

        except BlockingIOError:
            return None, None

        except Exception as e:
            print("vicon error:", e)
            return None, None

    def update_state(self):
        timestamp, objects = self.get_data()

        if objects is None:
            return False

        else:
            dt = self.dt

            for obj in objects:
                obj_id, x, y, z, qw, qx, qy, qz = obj

                pos = np.array([x, y, z])
                quat = np.array([qw, qx, qy, qz], dtype=np.float64)

                if self.prev_time is None:

                    vel = np.zeros(3)
                    wx, wy, wz = np.zeros(3)

                    self.filtered_pos[obj_id] = pos
                    self.filtered_quat[obj_id] = quat

                else:

                    self.filtered_pos[obj_id] = (
                        self.alpha_vel * pos +
                        (1 - self.alpha_vel) * self.filtered_pos[obj_id]
                    )

                    self.filtered_quat[obj_id] = (
                        self.filtered_quat[obj_id]
                        + self.alpha_omega * (quat - self.filtered_quat[obj_id])
                    )

                    self.filtered_quat[obj_id] /= np.linalg.norm(self.filtered_quat[obj_id])

                    vel = (self.filtered_pos[obj_id] - self.prev_filtered_pos[obj_id]) / dt

                    wx, wy, wz = self.quaternion_to_angular_velocity(
                        self.prev_filtered_quat[obj_id],
                        self.filtered_quat[obj_id],
                        dt
                    )

                self.state[obj_id, 0:3] = pos
                self.state[obj_id, 3:6] = vel
                self.state[obj_id, 6:10] = quat
                self.state[obj_id, 10:13] = np.array([wx, wy, wz])

                self.shared_state[:] = self.state - self.init_shared_state

                self.prev_filtered_pos = self.filtered_pos.copy()
                self.prev_filtered_quat = self.filtered_quat.copy()


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

        wd = np.clip(wd, -1.0, 1.0)
        s = np.sqrt(1 - wd*wd)

        if s < 1e-8:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = np.array([xd, yd, zd]) / s

        omega = axis * (angle / dt)

        return omega
