import math

from trajectories.base import Setpoint

_A     = 1.0
_B     = 1.0
_OMEGA = 0.13
_Z     = -0.80


class EightTrajectory:
    def reset(self):
        self._t0: int = None

    def step(self, timestamp_us: int, current_pos: tuple) -> Setpoint:
        if self._t0 is None:
            self._t0 = timestamp_us
        t = (timestamp_us - self._t0) * 1e-6

        return Setpoint(
            x  = _A * math.sin(_OMEGA * t),
            y  = _B * math.sin(2 * _OMEGA * t),
            z  = _Z,
            vx = _A * _OMEGA       * math.cos(_OMEGA * t),
            vy = 2 * _B * _OMEGA   * math.cos(2 * _OMEGA * t),
        )
