import socket
import struct
import threading
import time

import msgpack
import pytest

from vicon.quick_vicon import ViconReceiver

_PORT = 18020
_ADDR = "127.0.0.1"

_OBJECTS = [[0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3]]
_CORNERS = [[0, [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]]]
_FRAME   = msgpack.packb([0, 0, _OBJECTS, _CORNERS], use_bin_type=True)


def _send_once(payload, delay=0.0):
    def _go():
        time.sleep(delay)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(payload, (_ADDR, _PORT))
        s.close()
    threading.Thread(target=_go, daemon=True).start()


@pytest.fixture
def receiver():
    r = ViconReceiver(_ADDR, _PORT, timeout=0.5)
    yield r
    r.close()


def test_timeout_returns_none(receiver):
    # No sender → should time out and return (None, None)
    objects, corners = receiver.receive()
    assert objects is None and corners is None


def test_valid_frame_received(receiver):
    _send_once(_FRAME, delay=0.01)
    objects, corners = receiver.receive()
    assert objects == _OBJECTS
    assert corners == _CORNERS


def test_malformed_packet_handled(receiver):
    _send_once(b"garbage\x00\xff", delay=0.01)
    objects, corners = receiver.receive()
    assert objects is None and corners is None


def test_close_does_not_raise(receiver):
    receiver.close()  # double-close must not raise either
