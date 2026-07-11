"""
SITL deployment integration test.
PX4 SITL and IsaacLab must already be running. Run from alpha/:
    python tests/test_sitl_deployment.py
"""
import queue
import threading
import time
from pathlib import Path

import yaml

import shm.channels as channels
from shm.bus import ShmWriter
from vicon.quick_vicon import ViconReceiver
from mavlink.connection import MavlinkConnection
from coordinator import active_com

_CFG = Path(__file__).parent / "config" / "sitl.yaml"
MSG_INTERVALS = {44001: 200, 331: 200, 85: 80}


def _load_cfg():
    with open(_CFG) as f:
        return yaml.safe_load(f)


# ── Bridge threads ────────────────────────────────────────────────────────────

def _vicon_bridge(cfg, vic_w, init_w, corner_w, stop):
    recv = ViconReceiver(cfg["isaaclab_host"], cfg["isaaclab_port"])
    initialized = False
    try:
        while not stop.is_set():
            objects, corners = recv.receive()
            if objects is None:
                continue
            for obj in objects:
                idx = obj[0]
                if 0 <= idx < 4:
                    vic_w.data[idx, :] = obj[1:7]
            if corners:
                for cobj in corners:
                    idx = cobj[0]
                    if 0 <= idx < 4:
                        pts = cobj[1]
                        corner_w.data[idx, :len(pts)] = pts
            if not initialized:
                init_w.data[:] = vic_w.data
                initialized = True
    finally:
        recv.close()
    print("[vicon bridge] stopped")


def _estimator_bridge(cfg, state_w, act_w, stop):
    conn = MavlinkConnection(cfg["mavlink_address"], cfg["mavlink_baudrate"])
    conn.connect(msg_intervals=MSG_INTERVALS)
    try:
        while not stop.is_set():
            msg = conn.recv()
            if msg is None:
                continue
            t = msg.get_type()
            if t == "ODOMETRY":
                state_w.data[:] = [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz,
                                   *msg.q, msg.rollspeed, msg.pitchspeed, msg.yawspeed]
            elif t == "JOHNNY_STATUS":
                act_w.data[:] = msg.actuation
    finally:
        conn.close()
    print("[estimator bridge] stopped")


# ── Scripted keyboard ─────────────────────────────────────────────────────────

class ScriptedKeyboard:
    """Implements the KeyboardHandler interface, driven by a time-based schedule."""

    _MODES = {"lift", "land", "square", "eight", "bezier", "manual", "hongi", "idle"}

    def __init__(self, scenario: list):
        self.mode   = "idle"
        self.paused = False
        self.quit   = False
        self._cmds  = queue.SimpleQueue()
        self._t0    = time.monotonic()
        self._sched = sorted(scenario, key=lambda x: x[0])
        threading.Thread(target=self._run, daemon=True).start()

    def pop_command(self):
        try:
            return self._cmds.get_nowait()
        except queue.Empty:
            return None

    def _run(self):
        for delay, action in self._sched:
            remaining = (self._t0 + delay) - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            print(f"[t={delay:.1f}s] → {action}")
            if action == "quit":
                self.quit = True
            elif action in self._MODES:
                self.mode = action
            else:
                self._cmds.put(action)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    cfg  = _load_cfg()
    stop = threading.Event()

    # Allocate all SHM before coordinator opens its ShmReaders
    vic_w    = ShmWriter(channels.VICON_STATE)
    init_w   = ShmWriter(channels.VICON_INIT_STATE)
    corner_w = ShmWriter(channels.OBSTACLE_CORNERS)
    state_w  = ShmWriter(channels.ESTIMATED_STATE)
    act_w    = ShmWriter(channels.ACTUATION)
    dist_w   = ShmWriter(channels.DIST)
    low_w    = ShmWriter(channels.LOW_LEVEL_CTRL)
    joy_w    = ShmWriter(channels.JOYSTICK_SP)
    dist_w.data[0] = cfg["safe_dist"]

    threads = [
        threading.Thread(target=_vicon_bridge,
                         args=(cfg, vic_w, init_w, corner_w, stop), daemon=True),
        threading.Thread(target=_estimator_bridge,
                         args=(cfg, state_w, act_w, stop), daemon=True),
    ]
    for t in threads:
        t.start()

    kb = ScriptedKeyboard(cfg["scenario"])
    try:
        active_com.main(address=cfg["mavlink_address"], baudrate=cfg["mavlink_baudrate"], kb=kb)
    finally:
        stop.set()
        for w in (vic_w, init_w, corner_w, state_w, act_w, dist_w, low_w, joy_w):
            w.close()


if __name__ == "__main__":
    main()
