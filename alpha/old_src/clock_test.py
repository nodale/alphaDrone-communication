# Ubuntu (server)
import socket, time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("10.183.217.138", 9999))

while True:
    data, addr = sock.recvfrom(1024)
    t2 = time.time_ns()  # server receive
    sock.sendto(str(t2).encode(), addr)
