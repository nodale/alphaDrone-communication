import queue
import time
from unittest.mock import patch

import pytest

from operator.keyboard import KeyboardHandler, KEYMAP, Key


def test_keymap_valid_kinds():
    for ch, key in KEYMAP.items():
        assert key.kind in ("command", "mode"), f"invalid kind for '{ch}'"


def test_keymap_all_have_description_and_action():
    for ch, key in KEYMAP.items():
        assert key.description
        assert key.action


def _make_handler():
    with patch("operator.keyboard.threading.Thread"):  # don't spawn background thread
        with patch("builtins.print"):
            kb = KeyboardHandler()
    return kb


def test_on_key_mode_sets_mode():
    kb = _make_handler()
    mode_key = next(ch for ch, k in KEYMAP.items() if k.kind == "mode")
    kb._on_key(mode_key)
    assert kb.mode == KEYMAP[mode_key].action


def test_on_key_command_queued():
    kb = _make_handler()
    # pick a command that is neither quit/pause/resume
    cmd_key = next(ch for ch, k in KEYMAP.items()
                   if k.kind == "command" and k.action not in ("quit", "pause", "resume"))
    kb._on_key(cmd_key)
    assert kb.pop_command() == KEYMAP[cmd_key].action


def test_on_key_quit_sets_flag():
    kb = _make_handler()
    kb._on_key('q')
    assert kb.quit is True


def test_on_key_pause_resume():
    kb = _make_handler()
    kb._on_key('p')
    assert kb.paused is True
    kb._on_key('o')
    assert kb.paused is False


def test_on_key_unknown_ignored():
    kb = _make_handler()
    kb._on_key('\x00')  # not in KEYMAP
    assert kb.pop_command() is None
    assert kb.mode == "idle"


def test_pop_command_returns_none_when_empty():
    kb = _make_handler()
    assert kb.pop_command() is None
