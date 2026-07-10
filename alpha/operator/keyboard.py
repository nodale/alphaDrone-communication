"""
Keyboard input handler for SSH sessions.

To add a new mode:    add one line under "Flight modes" in KEYMAP.
To add a new command: add one line under "Commands" in KEYMAP.
The coordinator then handles the mode name or command string.
"""

import queue
import select
import sys
import threading
from dataclasses import dataclass
from typing import Literal, Optional


@dataclass(frozen=True)
class Key:
    description: str
    action: str
    kind: Literal["command", "mode"]


# ── Add / remove bindings here only ─────────────────────────────────────────

KEYMAP: dict[str, Key] = {
    # Commands (one-shot, queued for the coordinator)
    'q': Key("quit",              action="quit",         kind="command"),
    'p': Key("pause logging",     action="pause",        kind="command"),
    'o': Key("resume logging",    action="resume",       kind="command"),
    'l': Key("arm",               action="arm",          kind="command"),
    'j': Key("disarm",            action="disarm",       kind="command"),
    'k': Key("force disarm",      action="force_disarm", kind="command"),
    'r': Key("reboot FC",         action="reboot",       kind="command"),
    'b': Key("activate offboard", action="activate",     kind="command"),
    # Modes (persistent until another mode is selected)
    ' ': Key("idle  (clear mode)",action="idle",         kind="mode"),
    'm': Key("lift",              action="lift",         kind="mode"),
    'n': Key("land",              action="land",         kind="mode"),
    'v': Key("square trajectory", action="square",       kind="mode"),
    'c': Key("figure-eight",      action="eight",        kind="mode"),
    'z': Key("bezier trajectory", action="bezier",       kind="mode"),
    'x': Key("manual setpoint",   action="manual",       kind="mode"),
    'h': Key("hongi control",     action="hongi",        kind="mode"),
}

# ────────────────────────────────────────────────────────────────────────────


class KeyboardHandler:
    """Reads stdin in a background thread. Thread-safe.

    Coordinator interface:
        kb.mode            — current mode string ("idle" when none selected)
        kb.paused          — True while logging is paused
        kb.quit            — True after 'q' is pressed
        kb.pop_command()   — returns next queued command string, or None
    """

    def __init__(self):
        self.mode: str = "idle"
        self.paused: bool = False
        self.quit: bool = False
        self._commands: queue.SimpleQueue[str] = queue.SimpleQueue()
        threading.Thread(target=self._listen, daemon=True).start()
        self._print_help()

    def pop_command(self) -> Optional[str]:
        try:
            return self._commands.get_nowait()
        except queue.Empty:
            return None

    # ------------------------------------------------------------------ #

    def _on_key(self, key: str) -> None:
        binding = KEYMAP.get(key)
        if binding is None:
            return
        print(f"[kb] {binding.description}")
        if binding.kind == "mode":
            self.mode = binding.action
        else:
            if binding.action == "quit":
                self.quit = True
            elif binding.action == "pause":
                self.paused = True
            elif binding.action == "resume":
                self.paused = False
            else:
                self._commands.put(binding.action)

    def _listen(self) -> None:
        while not self.quit:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                self._on_key(sys.stdin.read(1).lower())

    def _print_help(self) -> None:
        commands = [(k, b) for k, b in KEYMAP.items() if b.kind == "command"]
        modes    = [(k, b) for k, b in KEYMAP.items() if b.kind == "mode"]
        print("\n── Commands ────────────────────────────────────────")
        for k, b in commands:
            print(f"  {'space' if k == ' ' else k}  →  {b.description}")
        print("── Modes ───────────────────────────────────────────")
        for k, b in modes:
            print(f"  {'space' if k == ' ' else k}  →  {b.description}")
        print("────────────────────────────────────────────────────\n")
