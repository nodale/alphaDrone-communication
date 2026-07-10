"""Cross-module: FlightLogger dataset shapes vs shm channel shapes + write/read."""

import numpy as np
import pytest

from ops.logger import FlightLogger, _DATASETS
import shm.channels as channels

# Mapping from logger dataset name → shm channel, for those that share layout.
# Shape must be broadcastable: logger row shape vs channel shape.
_CHANNEL_MAP = {
    "actuation": channels.ACTUATION,   # both (4,)
    "setpoint":  channels.GENERAL_SETPOINT,  # both (6,)
}


@pytest.mark.parametrize("name,ch", _CHANNEL_MAP.items())
def test_logger_row_matches_channel_element_count(name, ch):
    log_shape, _ = _DATASETS[name]
    assert np.prod(log_shape) == np.prod(ch.shape), (
        f"Logger '{name}' row has {np.prod(log_shape)} elements but "
        f"channel '{ch.name}' has {np.prod(ch.shape)}"
    )


def test_all_logger_datasets_write_read(tmp_path):
    lg = FlightLogger(log_dir=str(tmp_path))
    try:
        for name, (shape, dtype) in _DATASETS.items():
            data = np.zeros(shape, dtype=dtype)
            lg.log(name, data)
            np.testing.assert_array_equal(lg._ds[name][0], data)
    finally:
        lg.close()
