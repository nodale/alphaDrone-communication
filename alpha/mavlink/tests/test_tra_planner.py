import numpy as np
import pytest

from mavlink.tra_spline import CubicSpline
from mavlink.tra_planner import LinearLocalPlanner


@pytest.fixture
def planner():
    curve = CubicSpline(
        p0=np.array([0.0, 0.0]),
        p1=np.array([0.25, 0.5]),
        p2=np.array([0.75, 0.5]),
        p3=np.array([1.0, 0.0]),
    )
    return LinearLocalPlanner([curve], velocity=1.0)


def test_closest_u_starts_at_zero(planner):
    assert planner.closest_u == 0.0


def test_update_position_advances_u(planner):
    planner.update_position(np.array([0.5, 0.0]))
    assert planner.closest_u > 0.0


def test_get_control_target_returns_two_arrays(planner):
    planner.update_position(np.array([0.2, 0.1]))
    pos_sp, vel_sp = planner.get_control_target(np.array([0.2, 0.1]))
    assert pos_sp.shape == (2,)
    assert vel_sp.shape == (2,)
    assert np.all(np.isfinite(pos_sp))
    assert np.all(np.isfinite(vel_sp))


def test_transition_increments_curve_index(planner):
    # Add a second curve so transition has somewhere to go.
    planner.spline_list.append(planner.spline_list[0])
    planner.transition()
    assert planner.current_curve_i == 1


def test_get_position_dev_shape(planner):
    planner.update_position(np.array([0.1, 0.1]))
    dev = planner.get_position_dev(np.array([0.1, 0.1]))
    assert dev.shape == (2,)
