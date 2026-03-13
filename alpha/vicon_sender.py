import zenoh, random, time
import numpy as np
from vicon_dssdk import ViconDataStream

TARGET_IP   = "10.183.217.138"
TARGET_PORT = 8020
client = ViconDataStream.Client()
while True:
    try:
        client.Connect("localhost:801")
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
object_name = (
    "Mr_Obstacle",
    "Mrs_Obstacle"
)
seq = 0


session = zenoh.open(zenoh.Config())
key = 'vicon/objects'
pub = session.declare_publisher(key)


while True:
    client.GetFrame()
    timestamp = int(time.time() * 1e6)
    objects_data = []

    for obj_id, name in enumerate(object_name):

        (x_mm, y_mm, z_mm) = client.GetSegmentGlobalTranslation(name, name)[0]
        q_raw, _ = client.GetSegmentGlobalRotationQuaternion(name, name)

        x = x_mm * 0.001
        y = y_mm * 0.001
        z = z_mm * 0.001

        qw = q_raw[3]
        qx = q_raw[0]
        qy = q_raw[1]
        qz = q_raw[2]

        objects_data.append([
            obj_id,
            x, y, z,
            qw, qx, qy, qz
        ])

    msg = (
        seq,
        timestamp,
        objects_data
    )

    buf = f"{msg}"
    pub.put(buf)

    seq += 1
