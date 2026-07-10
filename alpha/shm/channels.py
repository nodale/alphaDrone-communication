from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Channel:
    name: str
    shape: tuple
    dtype: type


# Vicon / motion capture
VICON_STATE      = Channel("vicon_state",              shape=(4, 6),    dtype=np.float64)
VICON_INIT_STATE = Channel("vicon_init_state",         shape=(4, 6),    dtype=np.float64)
OBSTACLE_CORNERS = Channel("obstacle_corners",          shape=(4, 4, 3), dtype=np.float64)

# Flight controller (MAVLink)
ESTIMATED_STATE  = Channel("estimated_state",          shape=(13,),     dtype=np.float64)
ACTUATION        = Channel("actuation",                shape=(4,),      dtype=np.float64)
GENERAL_SETPOINT = Channel("general_setpoint",         shape=(6,),      dtype=np.float64)

# Controller output
LOW_LEVEL_CTRL   = Channel("low_level_control",        shape=(4,),      dtype=np.float64)

# Obstacle avoidance
DIST             = Channel("dist",                     shape=(1,),      dtype=np.float64)

# Joystick input
JOYSTICK_SP      = Channel("joeystick_state_setpoint", shape=(4,),      dtype=np.float64)
