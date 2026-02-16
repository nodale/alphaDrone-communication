from dataclasses import dataclass

import numpy as np
import socket
import msgpack

@dataclass
class QuickVicon:
    def __init__(self, address='10.183.217.138', port='8020', block=False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((address, port))

        self.sock.setblocking(block)

    def get_data(self):
        try:
            payload, addr = self.sock.recvfrom(1024)
            msg = msgpack.unpackb(payload, raw=False)

            #this depedns on the vicon's code
            #x = msg[2]
            #y = msg[3]
            #z = msg[4]
            #vx = msg[5]
            #vy = msg[6]
            #vz = msg[7]
            q = msg[8:12]
            v_q = msg[12:15]
            r, p, y = self._quat_to_rpy_vic(q)
            #return(x, y, z, vx, vy, vz, r, p, y)
            return(msg[2], msg[3], msg[4], msg[5], msg[6], msg[7], q, v_q[0], v_q[1], v_q[2])

        except BlockingIOError:
            return None

        except Exception as e:
            print("vicon error: ", e)
            return None

    def _quat_to_rpy_est(self, q):
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

    def _quat_to_rpy_vic(self, q):
        x, y, z, w = q

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

