from dataclasses import dataclass

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
from multiprocessing import shared_memory, resource_tracker

import numpy as np

import time
import math

@dataclass
class QuickMav:
    freq : float = 50
    timeBoot : float = 0.0

    def __init__(self, address, baudrate, create=False, **kwargs):
        self.timeBoot = time.time()
        try:
            self.master = mavutil.mavlink_connection(address, baudrate, robust_parsing=True, source_system=255, source_component=0, autoreconnect=True, udp_timeout=1)
        except:
            print("error in __init__, MAVlink refuses to connect, maybe wrong address or baudrate")
        super().__init__(**kwargs)

        self.square_points = (
                (0.4, 0.4, -0.3),
                (0.4, -0.4, -0.3),
                (-0.4, -0.4, -0.3),
                (-0.4, 0.4, -0.3),
                (0.0, 0.0, -0.3),
                )
        self.square_progress = 0

        self.eight_points = (0.0, 0.0, -0.3)
        self.eight_progress = 0.0

        if create==True:
            self.state = np.array([0.0]*13, dtype=np.float64)
            self.shm = shared_memory.SharedMemory(name="estimated_state", create=True, size=self.state.nbytes)
            self.shared_state = np.ndarray(self.state.shape, dtype=self.state.dtype, buffer=self.shm.buf)

            self.actuation = np.array([0.0]*4, dtype=np.float64)
            self.shm_actuation = shared_memory.SharedMemory(name="actuation", create=True, size=self.actuation.nbytes)
            self.shared_actuation = np.ndarray(self.actuation.shape, dtype=self.actuation.dtype, buffer=self.shm_actuation.buf)

        try:
            self.shm_general_sp = shared_memory.SharedMemory(name="general_setpoint")
            self.general_sp = np.ndarray((3,), dtype=np.float64, buffer=self.shm_general_sp.buf)
        except FileNotFoundError:
            self.sp = np.array([0.0]*3, dtype=np.float64)
            self.shm_general_sp = shared_memory.SharedMemory(name="general_setpoint", create=True, size=self.sp.nbytes)
            self.general_sp = np.ndarray((3,), dtype=np.float64, buffer=self.shm_general_sp.buf)

    def __del__(self):
        self.master.close()


    def setFreq(self, nfreq):
        self.freq = nfreq

    def sendHeartbeat(self):
        try:
            print("sending heartbeat")
            for i in range(2):
                self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GENERIC,      # or MAV_TYPE_GENERIC
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,   # still fine
                        0,                                       # base_mode
                        0,                                       # custom_mode
                        mavutil.mavlink.MAV_STATE_ACTIVE         # system_status
                        )
                self.master.wait_heartbeat(timeout=1)
                print("HEARTBEAT SENT")
            print("MAVLINK ENGAGED")

        except:
            print("sending heartbeat failed :(")

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                0,                                             
                44001,  #this is for johnny_status                          
                200,                                   
                0, 0, 0, 0, 0                                  
                )

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                0,                                             
                331,    #odometry
                200,    #this is the udpate frwq, apparently putting it to a lowervalue breaks it                               
                0, 0, 0, 0, 0                                  
                )

        #self.master.mav.command_long_send(
        #        self.master.target_system,
        #        self.master.target_component,
        #        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
        #        0,                                             
        #        50,      #sys_status for amp and voltage              
        #        1,                                   
        #        0, 0, 0, 0, 0                                  
        #        )

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                0,                                             
                85,      #setpoints
                80,                                   
                0, 0, 0, 0, 0                                  
                )

    def setFlightmode(self, mode):
        self.master.set_mode(mode)

        print("flight mode is set to ", mode)

    def arm(self):
        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
                )
        print("DRONE ARMED")

    def disarm(self):
        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
                )
        print("DRONE DISARMED")

    def forceDisarm(self):
        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 20190226, 0
                )

        print("DRONE DISARMED")

    def reboot(self):
        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0
                )

        print("PIXHAWK REBOOTING")

    def forceReboot(self):
        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 20190226, 0
                )

        print("PIXHAWK REBOOTING")

    def get(self, TYPE, block=False):
        return self.master.recv_match(type=TYPE, blocking=block, timeout=0.1)

    def getOdometry(self, block=False):
        self.est_odo = self.get("ODOMETRY", block=block)
        return self.est_odo

    def getJohnny(self, block=False):
        self.johnny = self.get("JOHNNY_STATUS", block=block)
        return self.johnny

    def getSetpoint(self, block=False):
        sp = self.get("POSITION_TARGET_LOCAL_NED", block=block)
        return sp

    def publish_odometry(self, name):
        x, y, z = self.est_odo.x, self.est_odo.y, self.est_odo.z
        vx, vy, vz = self.est_odo.vx, self.est_odo.vy, self.est_odo.vz
        qw, qx, qy, qz = self.est_odo.q
        wx, wy, wz = self.est_odo.rollspeed, self.est_odo.pitchspeed, self.est_odo.yawspeed

        #self.state = np.array([x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz], dtype=np.float64)
        #self.shared_state = self.state[:]

        self.shared_state[:] = [
            x, y, z,
            vx, vy, vz,
            qw, qx, qy, qz,
            wx, wy, wz
        ]

    def publish_actuation(self, name):
        actuation = self.johnny.actuation
        
        self.shared_actuation[:] = actuation

    def sendOdometry(self, time, pos, angle, cov1=[0.001]*21):
        self.master.mav.vision_position_estimate_send(
                time,
                pos[0], pos[1], pos[2],
                angle[0], angle[1], angle[2], 
                cov1
                )

    def refeed(self):
        _translation = self.get('LOCAL_POSITION_NED', True)
        _ang = self.get('ATTITUDE', True)
        _q = self.get('ATTITUDE_QUATERNION', True)

        _time = int(time.time() * 1e6) & 0xFFFFFFFF

        self.sendOdometry(
                _time, 
                [_translation.x, _translation.y, _translation.z],
                [_q.q1, _q.q2, _q.q3, _q.q4],
                [_translation.vx, _translation.vy, _translation.vz],
                [_ang.rollspeed, _ang.pitchspeed, _ang.yawspeed]
                )

    def sendVelocityTarget(self, time, vx, vy, vz): 
        self.master.mav.set_position_target_local_ned_send(
                time,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,
                0, 0, 0,  #position
                vx, vy, vz,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
                )

    def sendPositionTarget(self, time, x, y, z): 
        self.master.mav.set_position_target_local_ned_send(
                time,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                x, y, z,  #position
                0, 0, 0,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
                )
        self.general_sp[:] = (x, y, z)

    def sendPositionYawTarget(self, time, x, y, z, yaw): 
        self.master.mav.set_position_target_local_ned_send(
                time,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000001111111000,
                x, y, z,  #position
                0, 0, 0,  #velocity
                0, 0, 0,  #acceleration
                yaw, 0  #yaw yaw_rate
                )
        self.general_sp[:] = (x, y, z)

    def setTo_active(self):
        self.setFlightmode("OFFBOARD")
        time.sleep(0.1)
        self.setFlightmode("EXTERNAL1")

    def setTo_lift(self, time):
        self.sendPositionYawTarget(time, 0.0, 0.0, -0.2, 0.0)

    def setTo_land(self, time):
        self.sendPositionTarget(time, 0.0, 0.0, 0.0)

    def act_traverseSquare(self, time, current_pos):

        start = np.array(self.square_points[self.square_progress])
        end = np.array(self.square_points[(self.square_progress + 1) % 4])
        pos = np.array(current_pos)

        seg = end - start
        seg_len_sq = np.dot(seg, seg)

        if seg_len_sq < 1e-6:
            return

        t = np.dot(pos - start, seg) / seg_len_sq
        t = np.clip(t, 0.0, 1.0)

        lookahead = 0.24
        t_target = min(1.0, t + lookahead)

        target = start + seg * t_target

        if t > 0.9:
            self.square_progress = (self.square_progress + 1) % 4

        self.sendPositionYawTarget(time, target[0], target[1], target[2], 0.0)

    def act_traverseEight(self, time, current_pos):
        x_oscl = 0.7
        y_oscl = 0.7
        omega = 0.04

        dist = math.dist(current_pos, self.eight_points)

        if dist < 0.03:
            self.eight_progress += 1
            self.eight_points = (
                x_oscl * math.sin(omega * self.eight_progress),
                y_oscl * math.sin(2 * omega * self.eight_progress),
                -0.14
            )

        self.sendPositionYawTarget(time, *self.eight_points, 0.0)
