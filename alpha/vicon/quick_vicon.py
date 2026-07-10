import socket
import msgpack


class ViconReceiver:
    """Receives and parses Vicon UDP frames. No shared memory knowledge."""

    def __init__(self, address: str, port: int, timeout: float = 0.005):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((address, port))
        self._sock.settimeout(timeout)

    def receive(self):
        """Returns (objects, corners) or (None, None) on timeout or error.

        objects: list of [obj_id, x, y, z, rx, ry, rz]
        corners: list of (obj_id, [[x,y,z], ...])
        """
        try:
            payload, _ = self._sock.recvfrom(4096)
            _, _, objects, corners = msgpack.unpackb(payload, raw=False)
            return objects, corners
        except socket.timeout:
            return None, None
        except Exception as e:
            print(f"[vicon] recv error: {e}")
            return None, None

    def close(self):
        self._sock.close()
