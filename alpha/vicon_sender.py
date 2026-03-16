from __future__ import print_function
import argparse
import time
import socket
import numpy as np
import msgpack
from vicon_dssdk import ViconDataStream

parser = argparse.ArgumentParser()
parser.add_argument('host', nargs='?', default="localhost:801")
args = parser.parse_args()

TARGET_IP   = "10.183.217.138"
TARGET_PORT = 8020
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

packer = msgpack.Packer(use_bin_type=True)

def send_msg(payload):
    sock.sendto(payload, (TARGET_IP, TARGET_PORT))

client = ViconDataStream.Client()

print(f"Connecting to Vicon at {args.host} ...")
while True:
    try:
        client.Connect(args.host)
        break
    except ViconDataStream.DataStreamException:
        time.sleep(1)

client.SetBufferSize(1)
client.EnableSegmentData()
client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)

client.SetAxisMapping(
    ViconDataStream.Client.AxisMapping.EForward,
    ViconDataStream.Client.AxisMapping.ERight,
    ViconDataStream.Client.AxisMapping.EDown
)

client.GetFrame()

object_name = [
    "Mr_Obstacle",
    "Mrs_Obstacle"
    ]

seq = 0
while True:

    client.GetFrame()

    timestamp = int(time.time() * 1e6)
    objects_data = []

    for obj_id, name in enumerate(object_name):

        (x_mm, y_mm, z_mm) = client.GetSegmentGlobalTranslation(name, name)[0]
        (wx, wy, wz), _ = client.GetSegmentGlobalRotationEulerXYZ(name, name)

        x = x_mm * 0.001
        y = y_mm * 0.001
        z = z_mm * 0.001

        objects_data.append([
            obj_id,
            x, y, z,
            wx, wy, wz
        ])

    msg = (
        seq,
        timestamp,
        objects_data
    )

    payload = packer.pack(msg)
    send_msg(payload)

    seq += 1
