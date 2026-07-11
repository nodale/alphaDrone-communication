import gc
import signal
import time

import numpy as np

import shm.channels as channels
from shm.bus import ShmReader, ShmWriter
from mavlink.connection import MavlinkConnection
from ops.keyboard import KeyboardHandler
from ops.logger import FlightLogger
from trajectories import REGISTRY

_ADDRESS   = "udpout:192.168.0.3:14561"
_BAUDRATE  = 921600
_LOOP_HZ   = 200
_LOOP_PERIOD = 1.0 / _LOOP_HZ


def main():
    gc.collect()
    gc.disable()

    # ── Shared memory ────────────────────────────────────────────────────────
    vic_r   = ShmReader(channels.VICON_STATE)
    init_r  = ShmReader(channels.VICON_INIT_STATE)   # readable + writable for origin reset
    joy_r   = ShmReader(channels.JOYSTICK_SP)
    act_r   = ShmReader(channels.ACTUATION)
    dist_r  = ShmReader(channels.DIST)
    corner_r = ShmReader(channels.OBSTACLE_CORNERS)
    hongi_r = ShmReader(channels.LOW_LEVEL_CTRL)
    sp_w    = ShmWriter(channels.GENERAL_SETPOINT)   # coordinator owns the setpoint bus
    nn_w    = ShmWriter(channels.NN_CTRL)             # coordinator owns NN enable flags

    # ── Services ─────────────────────────────────────────────────────────────
    conn   = MavlinkConnection(_ADDRESS, _BAUDRATE)
    conn.connect()
    kb     = KeyboardHandler()
    logger = FlightLogger()

    running = True

    def _stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT,  _stop)

    # Reset all stateful trajectories on startup
    for traj in REGISTRY.values():
        traj.reset()

    next_time = time.perf_counter()

    try:
        while running and not kb.quit:
            t_us = (time.monotonic_ns() // 1000) & 0xFFFFFFFF
            pos  = tuple(vic_r.data[0, :3])
            euler = tuple(vic_r.data[0, 3:6])

            # ── One-shot commands ─────────────────────────────────────────
            cmd = kb.pop_command()
            if cmd == "arm":
                conn.arm()
            elif cmd == "disarm":
                conn.disarm()
            elif cmd == "force_disarm":
                conn.force_disarm()
            elif cmd == "reboot":
                init_r.data[0, :3] = 0
                time.sleep(0.2)
                init_r.data[0, :3] = vic_r.data[0, :3]
            elif cmd == "activate":
                conn.set_mode_active()
            elif cmd == "nn_infer_toggle":
                enabled = not bool(nn_w.data[0])
                nn_w.data[:] = (1 if enabled else 0, 0)   # mutually exclusive with learner
                print(f"[nn] inferrer {'ON' if enabled else 'OFF'}")
            elif cmd == "nn_learn_toggle":
                enabled = not bool(nn_w.data[1])
                nn_w.data[:] = (0, 1 if enabled else 0)   # mutually exclusive with inferrer
                print(f"[nn] online learner {'ON' if enabled else 'OFF'}")

            # ── Mode / setpoint dispatch ──────────────────────────────────
            mode = kb.mode
            if mode in REGISTRY:
                sp = REGISTRY[mode].step(t_us, pos)
                sp_w.data[:] = (sp.x, sp.y, sp.z, sp.vx, sp.vy, sp.vz)
                conn.send_position_target(t_us, sp.x, sp.y, sp.z, sp.vx, sp.vy, sp.vz,
                                          yaw=sp.yaw)
            elif mode == "manual":
                x, y, z, yaw = joy_r.data
                sp_w.data[:] = (x, y, z, 0.0, 0.0, 0.0)
                conn.send_position_target(t_us, x, y, z, yaw=yaw)
            elif mode == "hongi":
                conn.send_low_level_control(hongi_r.data)

            # ── Always send odometry + lyapunov ──────────────────────────
            conn.send_mocap_odometry(t_us, pos, euler)
            conn.send_lyapunov_scalar(t_us, float(dist_r.data[0]))

            # ── Logging ───────────────────────────────────────────────────
            if not kb.paused:
                logger.log("vicon",            np.r_[t_us, vic_r.data[0]])
                logger.log("obstacle",         vic_r.data[1:4])
                logger.log("setpoint",         sp_w.data.copy())
                logger.log("actuation",        act_r.data.copy())
                logger.log("corner",           corner_r.data[1:4])
                logger.log("minimum_distance", dist_r.data.copy())
                logger.flush()

            # ── Rate limit ────────────────────────────────────────────────
            next_time += _LOOP_PERIOD
            sleep = next_time - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_time = time.perf_counter()

    finally:
        logger.close()
        conn.close()
        vic_r.close(); init_r.close(); joy_r.close()
        act_r.close(); dist_r.close(); corner_r.close()
        hongi_r.close(); sp_w.close(); nn_w.close()
        print("[coordinator] shutdown")


if __name__ == "__main__":
    main()
