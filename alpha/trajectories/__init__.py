"""
Trajectory registry.

To add a new trajectory:
  1. Create trajectories/my_traj.py with a class implementing step() and reset()
  2. Add one line to REGISTRY below.
"""

from trajectories.hover  import LiftTrajectory, LandTrajectory
from trajectories.square import SquareTrajectory
from trajectories.eight  import EightTrajectory
from trajectories.bezier import BezierTrajectory

# ── Register trajectories here ───────────────────────────────────────────────

REGISTRY: dict = {
    "lift":   LiftTrajectory(),
    "land":   LandTrajectory(),
    "square": SquareTrajectory(),
    "eight":  EightTrajectory(),
    "bezier": BezierTrajectory(),
}

# ─────────────────────────────────────────────────────────────────────────────
