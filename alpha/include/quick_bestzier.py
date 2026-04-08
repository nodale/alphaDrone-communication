from dataclasses import dataclass

from include.tra_spline import CubicSpline
from include.tra_planner import LinearLocalPlanner

import numpy as np

class QuickBestzier:
    pVel : float = 0.4

    def __init__(self):
        self.splineList = []
        self.llp = LinearLocalPlanner(self.splineList, self.pVel)

        _temp_array = np.array(
                    [[0.0, 0.0],
                    [0.8, -0.6],
                    [1.6, 0.8],
                    [2.4, 0.2]], 
                dtype=np.float64)
        _bezier_curve = CubicSpline(p0=_temp_array[0][:], p1=_temp_array[1][:], p2=_temp_array[2][:], p3=_temp_array[3][:])
        self.splineList.append(_bezier_curve)

    def get_setpoints(self, current_pos):
        self.llp.update_position(current_pos[:2])

        _pos_sp, _vel_sp = self.llp.get_control_target(current_pos[:2])

        return _pos_sp, _vel_sp
