"""Cross-module: ShmWriter → ShmReader roundtrip over every real channel."""

import numpy as np
import pytest

import shm.channels as channels
from shm.bus import ShmWriter, ShmReader

_ALL_CHANNELS = [
    channels.VICON_STATE,
    channels.VICON_INIT_STATE,
    channels.OBSTACLE_CORNERS,
    channels.ESTIMATED_STATE,
    channels.ACTUATION,
    channels.GENERAL_SETPOINT,
    channels.LOW_LEVEL_CTRL,
    channels.DIST,
    channels.JOYSTICK_SP,
]


@pytest.mark.parametrize("ch", _ALL_CHANNELS, ids=lambda c: c.name)
def test_roundtrip(ch):
    w = ShmWriter(ch)
    try:
        rng   = np.random.default_rng(0)
        value = rng.random(ch.shape).astype(ch.dtype)
        w.data[:] = value

        r = ShmReader(ch)
        try:
            np.testing.assert_array_equal(r.data, value)
        finally:
            r.close()
    finally:
        w.close()


@pytest.mark.parametrize("ch", _ALL_CHANNELS, ids=lambda c: c.name)
def test_shape_and_dtype(ch):
    w = ShmWriter(ch)
    try:
        assert w.data.shape == ch.shape
        assert w.data.dtype == np.dtype(ch.dtype)
    finally:
        w.close()
