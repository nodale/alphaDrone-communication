import tempfile
from pathlib import Path

import h5py
import numpy as np
import pytest

from operator.logger import FlightLogger, _DATASETS


@pytest.fixture
def logger(tmp_path):
    lg = FlightLogger(log_dir=str(tmp_path))
    yield lg
    lg.close()


def test_creates_all_datasets(logger):
    assert set(logger._ds.keys()) == set(_DATASETS.keys())


def test_initial_shapes_are_zero_rows(logger):
    for name, ds in logger._ds.items():
        shape, _ = _DATASETS[name]
        assert ds.shape == (0, *shape)


def test_log_appends_row(logger):
    name, (shape, dtype) = next(iter(_DATASETS.items()))
    data = np.ones(shape, dtype=dtype)
    logger.log(name, data)
    assert logger._ds[name].shape[0] == 1


def test_log_write_read_roundtrip(logger):
    name, (shape, dtype) = next(iter(_DATASETS.items()))
    data = np.arange(int(np.prod(shape)), dtype=dtype).reshape(shape)
    logger.log(name, data)
    np.testing.assert_array_equal(logger._ds[name][0], data)


def test_log_multiple_rows(logger):
    name, (shape, dtype) = next(iter(_DATASETS.items()))
    for i in range(5):
        logger.log(name, np.full(shape, i, dtype=dtype))
    assert logger._ds[name].shape[0] == 5


def test_flush_does_not_raise(logger):
    logger.flush()


def test_file_exists_on_disk(tmp_path):
    lg = FlightLogger(log_dir=str(tmp_path))
    lg.close()
    assert any(tmp_path.iterdir())
