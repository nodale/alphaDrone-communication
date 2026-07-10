import os
os.environ["MAVLINK20"] = "1"

import time

import numpy as np
from pymavlink import mavutil

_COV = [0.001] * 21

# type_mask for SET_POSITION_TARGET_LOCAL_NED
_MASK_POS     = 0b0000111111111000
_MASK_POS_YAW = 0b0000001111111000
_MASK_POS_VEL = 0b0000111111000000


class MavlinkConnection:
    """Pure MAVLink protocol wrapper. No shared memory or trajectory knowledge."""

    def __init__(self, address: str, baudrate: int):
        mavutil.set_dialect("common")
        self.master = mavutil.mavlink_connection(
            address, baudrate,
            robust_parsing=True,
            source_system=255,
            source_component=0,
            autoreconnect=True,
            udp_timeout=0.5,
        )

    def connect(self, msg_intervals: dict = None):
        """Send heartbeat, wait for FC, then request message stream intervals.

        msg_intervals: {mavlink_msg_id: interval_us}
        """
        try:
            for _ in range(2):
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GENERIC,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                self.master.wait_heartbeat(timeout=1)
            print("[mavlink] connected")
        except Exception:
            print("[mavlink] heartbeat failed")

        for msg_id, interval_us in (msg_intervals or {}).items():
            self._command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, interval_us)

    def recv(self, msg_type: str = None, timeout: float = 0.005):
        """Receive next message. Pass msg_type=None to receive any type."""
        return self.master.recv_match(type=msg_type, blocking=True, timeout=timeout)

    def close(self):
        self.master.close()

    # ------------------------------------------------------------------ #
    # Commands                                                             #
    # ------------------------------------------------------------------ #

    def arm(self):
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        print("[mavlink] ARMED")

    def disarm(self):
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        print("[mavlink] DISARMED")

    def force_disarm(self):
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 20190226)
        print("[mavlink] FORCE DISARMED")

    def reboot(self):
        self._command(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
        print("[mavlink] REBOOTING")

    def set_mode(self, mode: str):
        self.master.set_mode(mode)
        print(f"[mavlink] mode → {mode}")

    def set_mode_active(self):
        self.set_mode("OFFBOARD")
        time.sleep(0.1)
        self.set_mode("EXTERNAL1")

    # ------------------------------------------------------------------ #
    # Senders                                                              #
    # ------------------------------------------------------------------ #

    def send_mocap_odometry(self, timestamp_us: int, pos: tuple, euler: tuple):
        """Send Vicon pose as VISION_POSITION_ESTIMATE."""
        self.master.mav.vision_position_estimate_send(
            timestamp_us,
            pos[0], pos[1], pos[2],
            euler[0], euler[1], euler[2],
            _COV,
        )

    def send_position_target(self, timestamp_us: int, x, y, z,
                             vx=0.0, vy=0.0, vz=0.0, yaw=None):
        """Send SET_POSITION_TARGET_LOCAL_NED. Mask is chosen automatically."""
        if yaw is not None:
            mask = _MASK_POS_YAW
        elif vx or vy or vz:
            mask = _MASK_POS_VEL
        else:
            mask = _MASK_POS

        self.master.mav.set_position_target_local_ned_send(
            timestamp_us,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            x, y, z,
            vx, vy, vz,
            0, 0, 0,
            yaw or 0.0, 0,
        )

    def send_lyapunov_scalar(self, timestamp_us: int, value):
        self.master.mav.lyapunov_scalar_send(timestamp_us, value)

    def send_low_level_control(self, control: np.ndarray):
        self.master.mav.low_level_control_send(control)

    # ------------------------------------------------------------------ #
    # Internal                                                             #
    # ------------------------------------------------------------------ #

    def _command(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            cmd, 0, *p,
        )
