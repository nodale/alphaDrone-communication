import numpy as np
import pytest

from mavlink.tra_spline import QuadraticSpline, CubicSpline


@pytest.fixture
def cubic():
    return CubicSpline(
        p0=np.array([0.0, 0.0]),
        p1=np.array([0.0, 1.0]),
        p2=np.array([1.0, 1.0]),
        p3=np.array([1.0, 0.0]),
    )


@pytest.fixture
def straight():
    return CubicSpline(
        p0=np.array([0.0, 0.0]),
        p1=np.array([1.0, 0.0]),
        p2=np.array([2.0, 0.0]),
        p3=np.array([3.0, 0.0]),
    )


def test_position_at_u0_equals_p0(cubic):
    np.testing.assert_allclose(cubic.get_position(0.0), cubic.p0)


def test_position_at_u1_equals_p3(cubic):
    np.testing.assert_allclose(cubic.get_position(1.0), cubic.p3)


def test_midpoint_is_between_endpoints(cubic):
    mid = cubic.get_position(0.5)
    assert mid[0] > cubic.p0[0] and mid[0] < cubic.p3[0]


def test_velocity_direction_at_u0(cubic):
    vel = cubic.get_velocity(0.0)
    # tangent at u=0 should point toward p1 direction (upward, positive y)
    assert vel[1] > 0


def test_curvature_straight_line_near_zero(straight):
    curv = straight.get_curvature(0.5)
    assert abs(curv) < 1e-6


def test_advance_u_clamps(cubic):
    cubic.advance_u(2.0)
    assert cubic.u == 1.0
    cubic.advance_u(-5.0)
    assert cubic.u == 0.0


def test_quadratic_position_endpoints():
    q = QuadraticSpline(
        p0=np.array([0.0, 0.0]),
        p1=np.array([0.5, 1.0]),
        p2=np.array([1.0, 0.0]),
    )
    np.testing.assert_allclose(q.get_position(0.0), q.p0)
    np.testing.assert_allclose(q.get_position(1.0), q.p2)
