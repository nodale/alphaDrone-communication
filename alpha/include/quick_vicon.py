from dataclasses import dataclass

import numpy as np
import socket
import msgpack

@dataclass
class QuickVicon:
    def __init__(self, address='localhost', port='8020', block=False):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bin((address, port))
        except:
            print("cant connect to vicon")

        self.sock.setblocking(block)

    def get_data(self):
        try:
            payload, addr = self.sock.recvfrom(2048)
            msg = msgpack.unpackb(payload, raw=False)

            #this depedns on the vicon's code
            x, y, z = msg[2:5]
            vx, vy, vz = msg[5:8]
            q = msg[8:12]
            r, p, y = quat_to_rpy(q)
            return(x, y, z, vx, vy, vz, r, p, y)

        except BlockingIOError:
            return None

        except Exception as e:
            print("vicon error: ", e)
            return None

    def quat_to_rpy(q):
        w, x, y, z = q

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

