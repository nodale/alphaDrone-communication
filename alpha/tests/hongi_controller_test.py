from pathlib import Path
import torch
import sys

parent_dir = Path(__file__).resolve().parents[1]
sys.path.append(str(parent_dir))

from include.quick_hongi import QuickHongi

controller = QuickHongi()

state = torch.tensor([
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
                      ])

setpoint = torch.tensor([
    0.0, 0.0, -1.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
                      ])

act = controller.get_action()
print(act)


