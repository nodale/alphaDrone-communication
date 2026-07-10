import numpy as np
import pytest

from shm.bus import ShmWriter, ShmReader
from shm.channels import Channel

_CH = Channel("_test_bus", shape=(3, 4), dtype=np.float64)


@pytest.fixture
def writer():
    w = ShmWriter(_CH)
    yield w
    try:
        w.close()
    except Exception:
        pass


def test_writer_zeros_on_init(writer):
    assert np.all(writer.data == 0)


def test_writer_correct_shape_dtype(writer):
    assert writer.data.shape == _CH.shape
    assert writer.data.dtype == np.dtype(_CH.dtype)


def test_reader_sees_writer_data(writer):
    writer.data[0, 0] = 42.0
    r = ShmReader(_CH)
    try:
        assert r.data[0, 0] == 42.0
    finally:
        r.close()


def test_reader_live_update(writer):
    r = ShmReader(_CH)
    try:
        writer.data[1, 2] = 99.0
        assert r.data[1, 2] == 99.0
    finally:
        r.close()


def test_reader_close_does_not_unlink(writer):
    r = ShmReader(_CH)
    r.close()
    # writer must still be able to write without error
    writer.data[:] = 1.0


def test_writer_attach_on_existing_segment():
    # Two writers on same channel: second should attach, not crash.
    w1 = ShmWriter(_CH)
    try:
        w2 = ShmWriter(_CH)
        w2.close()
    finally:
        w1.close()
