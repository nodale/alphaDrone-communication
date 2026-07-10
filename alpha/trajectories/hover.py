from trajectories.base import Setpoint


class LiftTrajectory:
    def reset(self): pass
    def step(self, timestamp_us, current_pos) -> Setpoint:
        return Setpoint(0.0, 0.0, -0.2, yaw=0.0)


class LandTrajectory:
    def reset(self): pass
    def step(self, timestamp_us, current_pos) -> Setpoint:
        return Setpoint(0.0, 0.0, 0.0)
