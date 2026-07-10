import gc
import signal

from shm.bus import ShmWriter
from shm import channels
from mavlink.connection import MavlinkConnection

# Message IDs and their requested stream intervals (microseconds)
MSG_INTERVALS = {
    44001: 200,   # JOHNNY_STATUS  (custom — actuation feedback)
    331:   200,   # ODOMETRY       (estimated state from EKF2)
    85:     80,   # POSITION_TARGET_LOCAL_NED
}


def main():
    gc.collect()
    gc.disable()

    state_w = ShmWriter(channels.ESTIMATED_STATE)
    act_w   = ShmWriter(channels.ACTUATION)

    conn = MavlinkConnection(address="/dev/ttyTHS1", baudrate=921600)
    conn.connect(msg_intervals=MSG_INTERVALS)

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        while running:
            msg = conn.recv()
            if msg is None:
                continue

            t = msg.get_type()

            if t == "ODOMETRY":
                state_w.data[:] = [
                    msg.x,  msg.y,  msg.z,
                    msg.vx, msg.vy, msg.vz,
                    *msg.q,
                    msg.rollspeed, msg.pitchspeed, msg.yawspeed,
                ]

            elif t == "JOHNNY_STATUS":
                act_w.data[:] = msg.actuation

    finally:
        conn.close()
        state_w.close()
        act_w.close()


if __name__ == "__main__":
    main()
