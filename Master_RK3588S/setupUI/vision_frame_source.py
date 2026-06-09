import struct

import cv2
import numpy as np
from multiprocessing import resource_tracker


def remove_shm_from_resource_tracker(name):
    try:
        resource_tracker.unregister("/" + name, "shared_memory")
    except Exception:
        pass


def read_frame_from_shm(shm, header_size=16):
    header = bytes(shm.buf[:header_size])
    fid, w, h = struct.unpack("QII", header)
    size = w * h * 3
    img_view = np.ndarray((h, w, 3), dtype=np.uint8, buffer=shm.buf[header_size:header_size + size])
    frame = img_view.copy()
    del img_view
    frame = cv2.flip(frame, 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return fid, frame
