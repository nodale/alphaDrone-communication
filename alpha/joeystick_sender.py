import pygame
import socket
import time
import struct

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick:", joystick.get_name())

previous = None

def changed(a, b, threshold=0.02):
    if b is None:
        return True
    for i in range(len(a)):
        if abs(a[i] - b[i]) > threshold:
            return True
    return False

while True:
    pygame.event.pump()

    axis0 = joystick.get_axis(0)
    axis1 = joystick.get_axis(1)
    axis2 = joystick.get_axis(2)
    axis3 = joystick.get_axis(3)

    x = -axis1
    y = axis0
    twist = axis2
    throttle = (-axis3 + 1.0) / 2.0

    setpoints = (x, y, twist, throttle)

    if changed(setpoints, previous):

        packet = struct.pack("ffff", x, y, twist, throttle)

        sock.sendto(packet, (UDP_IP, UDP_PORT))
        previous = setpoints

        print(f"x:{x:.3f} y:{y:.3f} yaw:{twist:.3f} z:{throttle:.3f}")

    time.sleep(0.01)
