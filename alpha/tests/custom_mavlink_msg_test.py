import pymavlink
from pymavlink import mavutil
import time
import math

mavutil.set_dialect('common')

master = mavutil.mavlink_connection("udpout:127.0.0.1:14580")
try:
    master = mavutil.mavlink_connection("udpout:192.168.0.3:14561", 921600)
except:
    print("not connection :(")

for i in range(2):
    master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,      
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
            0,                                     
            0,                                     
            mavutil.mavlink.MAV_STATE_ACTIVE       
            )
    master.wait_heartbeat(timeout=1)
    print("sending heartbeat")

master.wait_heartbeat(timeout=1)
print("Connected to system:", master.target_system)

timestamp = int(time.time() * 1e6)
seq = 0
while True:
    value = math.sin(time.time())

    master.mav.lyapunov_scalar_send(
        timestamp,
        value
    )

    print("msg num : ", seq)
    seq += 1
    time.sleep(0.4)
