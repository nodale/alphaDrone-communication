from typing import NamedTuple, Optional, Protocol, runtime_checkable


class Setpoint(NamedTuple):
    x:   float
    y:   float
    z:   float
    vx:  float = 0.0
    vy:  float = 0.0
    vz:  float = 0.0
    yaw: Optional[float] = None


@runtime_checkable
class Trajectory(Protocol):
    def step(self, timestamp_us: int, current_pos: tuple) -> Setpoint: ...
    def reset(self) -> None: ...
