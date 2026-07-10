import gc
import signal
import numpy as np

from shm.bus import ShmWriter
from shm import channels
from vicon.quick_vicon import ViconReceiver


def main():
    gc.collect()
    gc.disable()  # prevent GC pauses in the hot loop

    state_w   = ShmWriter(channels.VICON_STATE)
    init_w    = ShmWriter(channels.VICON_INIT_STATE)
    corners_w = ShmWriter(channels.OBSTACLE_CORNERS)
    receiver  = ViconReceiver(address="192.168.1.10", port=8020)

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        while running:
            objects, corners = receiver.receive()
            if objects is None:
                continue

            origin = init_w.data[0, :3].copy()

            for obj in objects:
                obj_id, x, y, z, rx, ry, rz = obj
                state_w.data[obj_id, :3] = (x - origin[0], y - origin[1], z - origin[2])
                state_w.data[obj_id, 3:6] = (rx, ry, rz)

            for obj_id, corner_set in corners:
                corners_w.data[obj_id] = np.asarray(corner_set, dtype=np.float64) - origin

    finally:
        receiver.close()
        state_w.close()
        init_w.close()
        corners_w.close()


if __name__ == "__main__":
    main()
