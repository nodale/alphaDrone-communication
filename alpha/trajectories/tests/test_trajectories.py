import pytest

from trajectories.base import Setpoint, Trajectory
from trajectories.hover import LiftTrajectory, LandTrajectory
from trajectories.square import SquareTrajectory
from trajectories.eight import EightTrajectory
from trajectories import REGISTRY

_ALL = [LiftTrajectory, LandTrajectory, SquareTrajectory, EightTrajectory]
_T0  = 0
_T1  = 1_000_000  # 1 s in microseconds
_POS = (0.1, 0.2, -0.3)


@pytest.mark.parametrize("cls", _ALL)
def test_step_returns_setpoint(cls):
    traj = cls()
    traj.reset()
    sp = traj.step(_T0, _POS)
    assert isinstance(sp, Setpoint)


@pytest.mark.parametrize("cls", _ALL)
def test_setpoint_fields_are_finite(cls):
    import math
    traj = cls()
    traj.reset()
    sp = traj.step(_T1, _POS)
    for field in (sp.x, sp.y, sp.z, sp.vx, sp.vy, sp.vz):
        assert math.isfinite(field), f"{cls.__name__} produced non-finite field"


@pytest.mark.parametrize("cls", _ALL)
def test_reset_restarts_state(cls):
    traj = cls()
    traj.reset()
    sp1 = traj.step(_T0, _POS)
    traj.step(_T1, _POS)
    traj.reset()
    sp2 = traj.step(_T0, _POS)
    # After reset, first step should reproduce same result
    assert sp1 == sp2


def test_registry_contains_expected_keys():
    assert {"lift", "land", "square", "eight", "bezier"} <= set(REGISTRY.keys())


@pytest.mark.parametrize("name, traj", list(REGISTRY.items()))
def test_registry_trajectories_satisfy_protocol(name, traj):
    assert isinstance(traj, Trajectory)


@pytest.mark.parametrize("name, traj", list(REGISTRY.items()))
def test_registry_trajectories_step(name, traj):
    traj.reset()
    sp = traj.step(_T0, _POS)
    assert isinstance(sp, Setpoint)
