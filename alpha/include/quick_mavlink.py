from dataclasses import dataclass

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

import time

@dataclass
class QuickMav:
    freq : float = 50
    timeBoot : float = 0.0

    def __init__(self, address, baudrate, **kwargs):
        self.timeBoot = time.time()
        try:
            self.master = mavutil.mavlink_connection(address, baudrate, robust_parsing=True, source_system=255, source_component=0, autoreconnect=True, source_port=14552, udp_timeout=1)
        except:
            print("error in __init__, MAVlink refuses to connect, maybe wrong address or baudrate")
        super().__init__(**kwargs)

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

        #self.master.mav.request_data_stream_send(
        #    self.master.target_system,
        #    self.master.target_component,
        #    mavutil.mavlink.MAV_DATA_STREAM_ALL,
        #    100, 
        #    1  
        #)

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                0,                                             
                44001,  #this is for johnny_status                          
                100,                                   
                0, 0, 0, 0, 0                                  
                )

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                0,                                             
                331,    #odometry
                100,                                   
                0, 0, 0, 0, 0                                  
                )

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                0,                                             
                1,      #sys_status for amp and voltage              
                10,                                   
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
        est_odo = self.master.get("ODOMETRY", block=block)
        return est_odo

    def getJohnny(self, block=False):
        johnny = self.master.get("JOHNNY_STATUS", block=block)
        return johnny

    def sendOdometry(self, time, pos, q, vel, rotRates, cov1=[0.002]*21, cov2=[0.002]*21):
        vodom = mavlink2.MAVLink_odometry_message(
                time,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                pos[0], pos[1], pos[2],
                [q[0], q[1], q[2], q[3]],
                vel[0], vel[1], vel[2],
                rotRates[0], rotRates[1], rotRates[2],
                cov1, 
                cov2,
                0,
                0,
                0
                )
        self.master.mav.send(vodom)

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
