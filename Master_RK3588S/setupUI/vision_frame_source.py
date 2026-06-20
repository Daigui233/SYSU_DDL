import struct

import cv2
import numpy as np
from multiprocessing import resource_tracker


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
    header = bytes(shm.buf[:FRAME_HEADER_BYTES])
    return struct.unpack(FRAME_HEADER_FORMAT, header)


def read_frame_from_shm(shm, header_size=FRAME_HEADER_BYTES, max_retries=FRAME_SNAPSHOT_MAX_RETRIES):
    retries = max(1, int(max_retries))
    for _attempt in range(retries):
        before = _read_header(shm, header_size)
        fid, w, h = before
        if w <= 0 or h <= 0:
            raise ValueError(f"invalid shared-memory frame size: {w}x{h}")

        size = int(w) * int(h) * 3
        frame_end = int(header_size) + size
        if frame_end > len(shm.buf):
            raise BufferError(f"shared-memory frame exceeds buffer: need={frame_end} have={len(shm.buf)}")

        img_view = np.ndarray(
            (h, w, 3),
            dtype=np.uint8,
            buffer=shm.buf[int(header_size):frame_end],
        )
        frame = img_view.copy()
        del img_view

        after = _read_header(shm, header_size)
        if after != before:
            continue

        frame = cv2.flip(frame, 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return fid, frame

    raise FrameSnapshotChanged("shared-memory frame changed during copy")
