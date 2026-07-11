"""Shared utilities for the nn/ process family."""

import time
from shm.bus import ShmReader
from shm import channels


def await_ctrl(timeout: float = 60.0) -> ShmReader:
    """Block until the coordinator creates the NN_CTRL SHM segment.

    The coordinator (active_com) owns NN_CTRL and must start first.
    Raises RuntimeError after *timeout* seconds.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return ShmReader(channels.NN_CTRL)
        except FileNotFoundError:
            time.sleep(0.5)
    raise RuntimeError(
        f"NN_CTRL SHM not available after {timeout:.0f}s — is coordinator running?"
    )
