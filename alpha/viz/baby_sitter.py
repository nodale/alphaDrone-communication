import gc
import signal
import time

from shm.bus import ShmReader
from shm import channels
from viz.quick_viser import DroneViser

# ── Configuration ─────────────────────────────────────────────────────────────
PORT      = 8080
URDF_PATH = None    # e.g. "assets/alphaDrone.urdf"
RATE      = 0.005   # seconds per frame
# ──────────────────────────────────────────────────────────────────────────────


def main():
    gc.collect()
    gc.disable()

    est_r = ShmReader(channels.ESTIMATED_STATE)
    vic_r = ShmReader(channels.VICON_STATE)
    sp_r  = ShmReader(channels.GENERAL_SETPOINT)

    vis = DroneViser(port=PORT, urdf_path=URDF_PATH)

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        while running:
            vis.update(
                state_est   = est_r.data.copy(),
                vicon_state = vic_r.data.copy(),       # (4, 6)
                setpoint    = sp_r.data.copy(),
                obstacles   = vic_r.data[1:].copy(),   # objects 1..3 are obstacles
            )
            time.sleep(RATE)
    finally:
        est_r.close()
        vic_r.close()
        sp_r.close()


if __name__ == "__main__":
    main()
