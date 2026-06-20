import struct
from multiprocessing import resource_tracker

import cv2
import numpy as np


FRAME_HEADER_FORMAT = "QII"
FRAME_HEADER_BYTES = struct.calcsize(FRAME_HEADER_FORMAT)
FRAME_SNAPSHOT_MAX_RETRIES = 3


class FrameSnapshotChanged(RuntimeError):
    """The producer replaced the shared-memory frame while it was copied."""


def remove_shm_from_resource_tracker(name):
    try:
        resource_tracker.unregister("/" + name, "shared_memory")
    except Exception:
        pass


def _read_header(shm, header_size):
    if int(header_size) < FRAME_HEADER_BYTES:
        raise ValueError(f"shared-memory header is too small: {header_size}")
    return struct.unpack(FRAME_HEADER_FORMAT, bytes(shm.buf[:FRAME_HEADER_BYTES]))


def read_frame_from_shm(
    shm,
    header_size=FRAME_HEADER_BYTES,
    max_retries=FRAME_SNAPSHOT_MAX_RETRIES,
):
    """Return a coherent BGR snapshot from the official AR shared memory."""
    for _attempt in range(max(1, int(max_retries))):
        before = _read_header(shm, header_size)
        frame_id, width, height = before
        if width <= 0 or height <= 0:
            raise ValueError(f"invalid shared-memory frame size: {width}x{height}")

        data_size = int(width) * int(height) * 3
        frame_end = int(header_size) + data_size
        if frame_end > len(shm.buf):
            raise BufferError(
                f"shared-memory frame exceeds buffer: need={frame_end} have={len(shm.buf)}"
            )

        image_view = np.ndarray(
            (height, width, 3),
            dtype=np.uint8,
            buffer=shm.buf[int(header_size):frame_end],
        )
        frame = image_view.copy()
        del image_view

        if _read_header(shm, header_size) != before:
            continue

        frame = cv2.flip(frame, 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame_id, frame

    raise FrameSnapshotChanged("shared-memory frame changed during copy")
