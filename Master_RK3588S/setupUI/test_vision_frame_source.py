import struct
import types
import unittest
from unittest import mock

import numpy as np

from vision_frame_source import read_frame_from_shm


def _fake_shm(fid=1, width=4, height=3):
    rgb = np.arange(width * height * 3, dtype=np.uint8).reshape((height, width, 3))
    payload = bytearray(struct.pack("QII", fid, width, height))
    payload.extend(rgb.tobytes())
    return types.SimpleNamespace(buf=memoryview(payload))


class VisionFrameSourceTest(unittest.TestCase):
    def test_reads_stable_frame_snapshot(self):
        fid, frame = read_frame_from_shm(_fake_shm(fid=7), max_retries=1)

        self.assertEqual(fid, 7)
        self.assertEqual(frame.shape, (3, 4, 3))

    def test_retries_when_frame_header_changes_during_copy(self):
        shm = _fake_shm(fid=8)
        headers = [
            (7, 4, 3),
            (8, 4, 3),
            (8, 4, 3),
            (8, 4, 3),
        ]
        with mock.patch("vision_frame_source._read_header", side_effect=headers):
            fid, frame = read_frame_from_shm(shm, max_retries=2)

        self.assertEqual(fid, 8)
        self.assertEqual(frame.shape, (3, 4, 3))


if __name__ == "__main__":
    unittest.main()
