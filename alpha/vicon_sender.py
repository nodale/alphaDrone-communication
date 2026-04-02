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

TARGET_IP   = "192.168.1.10"
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
client.EnableMarkerData()
client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)

client.SetAxisMapping(
    ViconDataStream.Client.AxisMapping.EForward,
    ViconDataStream.Client.AxisMapping.ERight,
    ViconDataStream.Client.AxisMapping.EDown
)

client.GetFrame()

object_name = [
    "alphaDrone_v6",
    "Obstacle_1",
    "Obstacle_2",
    "Obstacle_3",
    ]

seq = 0
while True:

    client.GetFrame()

    timestamp = int(time.time() * 1e6)
    objects_data = []
    corners_data = []

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

        marker_names = client.GetMarkerNames(name)

        obj_corners = []

        for marker, _ in marker_names:
            (mx_mm, my_mm, mz_mm), _ = client.GetMarkerGlobalTranslation(name, marker)

            mx = mx_mm * 0.001
            my = my_mm * 0.001
            mz = mz_mm * 0.001

            obj_corners.append([mx, my, mz])

        if len(obj_corners) == 4:
            corners_data.append((
                obj_id,
                obj_corners
            ))

    msg = (
        seq,
        timestamp,
        objects_data,
        corners_data
    )

    payload = packer.pack(msg)
    send_msg(payload)

    seq += 1
