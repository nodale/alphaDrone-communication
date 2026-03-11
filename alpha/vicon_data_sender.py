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

packer = msgpack.Packer(use_bin_type=False)

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
subject = client.GetSubjectNames()[0]
segment = client.GetSegmentNames(subject)[0]

seq = 0
while True:
    client.GetFrame()

    (x_mm, y_mm, z_mm) = client.GetSegmentGlobalTranslation(subject, segment)[0]
    q_raw, _ = client.GetSegmentGlobalRotationQuaternion(subject, segment)
    #quaternion is ararnged as : qx, qy, qz, qw
    #we will send it as : qw, qx, qy, qz to have less processing in the jetoson

    #just truning it from mm to m
    x = x_mm * 0.001
    y = y_mm * 0.001
    z = z_mm * 0.001

    msg = (
        seq,
        int(time.time() * 1e6),
        x, y, z,
        q_raw[3], q_raw[0], q_raw[1], q_raw[2],
    )
    
    payload = packer.pack(msg)
    send_msg(payload)

    seq += 1