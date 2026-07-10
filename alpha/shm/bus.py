from multiprocessing import shared_memory, resource_tracker

import numpy as np

from shm.channels import Channel


class ShmWriter:
    """Creates and owns a shared memory segment.

    The process that instantiates ShmWriter is the sole owner: it allocates
    the segment on construction and destroys it on close(). Only one writer
    per channel should exist at a time.
    """

    def __init__(self, channel: Channel):
        self._channel = channel
        nbytes = int(np.prod(channel.shape)) * np.dtype(channel.dtype).itemsize
        try:
            self._shm = shared_memory.SharedMemory(name=channel.name, create=True, size=nbytes)
        except FileExistsError:
            self._shm = shared_memory.SharedMemory(name=channel.name, create=False)
        self.data = np.ndarray(channel.shape, dtype=channel.dtype, buffer=self._shm.buf)
        self.data[:] = np.zeros(channel.shape, dtype=channel.dtype)

    def close(self):
        self._shm.close()
        self._shm.unlink()


class ShmReader:
    """Attaches to an existing shared memory segment without taking ownership.

    Never calls unlink() — only the ShmWriter that created the segment should
    destroy it. resource_tracker is unregistered here so Python does not try
    to clean up a segment it does not own when this process exits.
    """

    def __init__(self, channel: Channel):
        self._channel = channel
        self._shm = shared_memory.SharedMemory(name=channel.name, create=False)
        resource_tracker.unregister(self._shm._name, "shared_memory")
        self.data = np.ndarray(channel.shape, dtype=channel.dtype, buffer=self._shm.buf)

    def close(self):
        self._shm.close()
