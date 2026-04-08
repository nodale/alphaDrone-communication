from dataclasses import dataclass

from tra_spline import CubicSpline
from tra_planner import LinearLocalPlanner
from quick_state import QuickState

import numpy as np

class QuickBestzier:
    pVel : float = 0.4

    def __init__(self):
        self.splineList = []
        self.llp = linearLocalPlanner(self.splineList, self.pVel)

        _temp_array = np.array(
                [
                    [0.0, 0.0],
                    [0.4, -0.4],
                    [0.8, 0.2],
                    [1.2, 0.0]
                    ], dtype=np.float64)
        self.splineList.append(_temp_array)

    def get_setpoints(self, current_pos):
        self.llp.update_position(current_pos[:2])

        _pos_sp, _vel_sp = self.llp.get_control_target(current_pos[:2])

        return _pos_sp, _vel_sp
