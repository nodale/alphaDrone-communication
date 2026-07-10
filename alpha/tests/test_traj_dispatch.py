"""Cross-module: keyboard mode key → REGISTRY trajectory → Setpoint."""

from unittest.mock import patch

import pytest

from ops.keyboard import KEYMAP
from trajectories import REGISTRY
from trajectories.base import Setpoint

_POS = (0.0, 0.0, -0.5)
_T0  = 0

# Only mode keys that map to a registered trajectory
_MODE_PAIRS = [
    (ch, key.action)
    for ch, key in KEYMAP.items()
    if key.kind == "mode" and key.action in REGISTRY
]


@pytest.mark.parametrize("key_char,action", _MODE_PAIRS, ids=[p[1] for p in _MODE_PAIRS])
def test_mode_key_produces_setpoint(key_char, action):
    traj = REGISTRY[action]
    traj.reset()
    sp = traj.step(_T0, _POS)
    assert isinstance(sp, Setpoint)


@pytest.mark.parametrize("key_char,action", _MODE_PAIRS, ids=[p[1] for p in _MODE_PAIRS])
def test_mode_switch_calls_reset(key_char, action):
    with patch("ops.keyboard.threading.Thread"), patch("builtins.print"):
        from ops.keyboard import KeyboardHandler
        kb = KeyboardHandler()

    kb._on_key(key_char)
    assert kb.mode == action

    traj = REGISTRY[action]
    traj.reset()
    sp = traj.step(_T0, _POS)
    assert isinstance(sp, Setpoint)
