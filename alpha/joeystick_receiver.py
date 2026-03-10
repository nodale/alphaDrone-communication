import socket
import struct

UDP_IP = "10.183.217.138"
UDP_PORT = 8001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

sock.settimeout(0.01)

print("Listening...")

while True:
    try:
        data, addr = sock.recvfrom(1024)

        x, y, yaw, z = struct.unpack("ffff", data)

        print(f"x:{x:.3f} y:{y:.3f} yaw:{yaw:.3f} z:{z:.3f}")

    except socket.timeout:
        pass
