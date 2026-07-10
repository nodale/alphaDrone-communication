from mavlink.quick_bestzier import QuickBestzier

from trajectories.base import Setpoint

_Z = -0.5


class BezierTrajectory:
    def reset(self):
        self._bezier = QuickBestzier()

    def step(self, timestamp_us: int, current_pos: tuple) -> Setpoint:
        pos_sp, vel_sp = self._bezier.get_setpoints(current_pos)
        return Setpoint(
            x=pos_sp[0], y=pos_sp[1], z=_Z,
            vx=vel_sp[0], vy=vel_sp[1],
        )
