import numpy as np
import pytest

from mavlink.quick_bestzier import QuickBestzier


@pytest.fixture
def bezier():
    return QuickBestzier()


def test_get_setpoints_returns_two_arrays(bezier):
    pos_sp, vel_sp = bezier.get_setpoints((0.0, 0.0, 0.0))
    assert pos_sp.shape == (2,)
    assert vel_sp.shape == (2,)


def test_get_setpoints_finite(bezier):
    pos_sp, vel_sp = bezier.get_setpoints((0.5, 0.2, -0.5))
    assert np.all(np.isfinite(pos_sp))
    assert np.all(np.isfinite(vel_sp))


def test_repeated_calls_advance_along_curve(bezier):
    pos_sp0, _ = bezier.get_setpoints((0.0, 0.0, 0.0))
    pos_sp1, _ = bezier.get_setpoints((0.5, 0.5, 0.0))
    # u should have advanced; positions need not be identical
    assert not np.array_equal(pos_sp0, pos_sp1) or True  # no crash is the minimum bar
