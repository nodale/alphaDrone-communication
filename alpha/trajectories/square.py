import numpy as np

from trajectories.base import Setpoint

_POINTS = (
    ( 0.4,  0.4, -0.3),
    ( 0.4, -0.4, -0.3),
    (-0.4, -0.4, -0.3),
    (-0.4,  0.4, -0.3),
)
_LOOKAHEAD = 0.24


class SquareTrajectory:
    def reset(self):
        self._progress = 0

    def step(self, timestamp_us: int, current_pos: tuple) -> Setpoint:
        start = np.array(_POINTS[self._progress])
        end   = np.array(_POINTS[(self._progress + 1) % 4])
        pos   = np.array(current_pos)
        seg   = end - start

        seg_len_sq = np.dot(seg, seg)
        if seg_len_sq < 1e-6:
            return Setpoint(*start, yaw=0.0)

        t      = np.clip(np.dot(pos - start, seg) / seg_len_sq, 0.0, 1.0)
        target = start + seg * min(1.0, t + _LOOKAHEAD)

        if t > 0.9:
            self._progress = (self._progress + 1) % 4

        return Setpoint(target[0], target[1], target[2], yaw=0.0)
