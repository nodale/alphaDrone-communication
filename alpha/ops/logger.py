"""
HDF5 flight logger.

To add a new channel: add one entry to _DATASETS in __init__.
Then call logger.log("channel_name", data) in the coordinator.
"""

from datetime import datetime

import h5py
import numpy as np


# ── Define logged channels here ──────────────────────────────────────────────

_DATASETS = {
    #  name                row_shape       dtype
    "vicon":            ((7,),          np.float32),   # t, x, y, z, roll, pitch, yaw
    "obstacle":         ((3, 6),        np.float32),   # CoM pose per obstacle
    "setpoint":         ((6,),          np.float32),   # x, y, z, vx, vy, vz
    "actuation":        ((4,),          np.float32),   # motor commands
    "corner":           ((3, 4, 3),     np.float32),   # obstacle corners
    "minimum_distance": ((1,),          np.float32),
}

# ─────────────────────────────────────────────────────────────────────────────


class FlightLogger:
    def __init__(self, log_dir: str = "past_logs"):
        prefix = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._file = h5py.File(f"{log_dir}/{prefix}_LOG.h5", "w")
        self._ds  = {
            name: self._file.create_dataset(
                name,
                shape=(0, *shape),
                maxshape=(None, *shape),
                dtype=dtype,
                chunks=True,
            )
            for name, (shape, dtype) in _DATASETS.items()
        }
        self._idx = {name: 0 for name in _DATASETS}

    def log(self, channel: str, data: np.ndarray) -> None:
        ds  = self._ds[channel]
        idx = self._idx[channel]
        ds.resize((idx + 1, *ds.shape[1:]))
        ds[idx] = data
        self._idx[channel] = idx + 1

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self._file.close()
