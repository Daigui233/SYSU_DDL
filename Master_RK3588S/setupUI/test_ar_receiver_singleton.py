import os
import queue
import tempfile
import unittest
from unittest import mock

import numpy as np

from .ar_receiver import (
    acquire_instance_lock,
    activate_existing_preview,
    add_camera_fault_overlay,
    camera_frame_is_blank,
    FfplayPreview,
)


class ArReceiverSingletonTest(unittest.TestCase):
    def test_second_instance_cannot_acquire_same_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "ar_receiver.lock")
            previous = os.environ.get("AR_RECEIVER_LOCK_PATH")
            os.environ["AR_RECEIVER_LOCK_PATH"] = path
            first = None
            third = None
            try:
                first = acquire_instance_lock()
                second = acquire_instance_lock()
                self.assertIsNotNone(first)
                self.assertIsNone(second)

                first.close()
                first = None
                third = acquire_instance_lock()
                self.assertIsNotNone(third)
            finally:
                if first is not None:
                    first.close()
                if third is not None:
                    third.close()
                if previous is None:
                    os.environ.pop("AR_RECEIVER_LOCK_PATH", None)
                else:
                    os.environ["AR_RECEIVER_LOCK_PATH"] = previous

    @mock.patch("urllib.request.urlopen")
    def test_existing_instance_preview_is_activated(self, urlopen):
        response = mock.MagicMock()
        response.status = 200
        urlopen.return_value.__enter__.return_value = response

        self.assertTrue(activate_existing_preview())

        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:9105/api/preview")
        self.assertEqual(request.data, b'{"enabled": true}')

    def test_uniform_camera_failure_is_detected_and_labelled(self):
        white = np.full((480, 640, 3), 255, dtype=np.uint8)
        black = np.zeros((480, 640, 3), dtype=np.uint8)
        normal = white.copy()
        normal[:, :320] = 80

        self.assertTrue(camera_frame_is_blank(white))
        self.assertTrue(camera_frame_is_blank(black))
        self.assertFalse(camera_frame_is_blank(normal))
        rendered = add_camera_fault_overlay(white.copy())
        self.assertTrue(np.any(rendered != white))

    def test_preview_tracks_input_and_side_by_side_output_sizes(self):
        state = mock.MagicMock()
        preview = FfplayPreview(state, fps=30.0)
        preview.process = mock.MagicMock()
        preview.process.poll.return_value = None
        preview.frame_size = (1280, 480)
        preview.input_size = (640, 480)
        preview.frames = queue.Queue(maxsize=1)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        preview.show(frame, output_size=(1280, 480))

        queued_frame, queued_renderer, queued_size = (
            preview.frames.get_nowait())
        self.assertIs(queued_frame, frame)
        self.assertIsNone(queued_renderer)
        self.assertEqual(queued_size, (1280, 480))


if __name__ == "__main__":
    unittest.main()
