import os
import tempfile
import unittest
from unittest import mock

from .ar_receiver import (
    acquire_instance_lock,
    activate_existing_preview,
    configure_perception_defaults,
)


class ArReceiverSingletonTest(unittest.TestCase):
    @mock.patch.dict(os.environ, {}, clear=True)
    def test_skeleton_defaults_skip_duplicate_ridge_and_raw_heatmap_render(self):
        configure_perception_defaults()

        self.assertEqual("skeleton", os.environ["VISION_CONTROL_PATH_SOURCE"])
        self.assertEqual("curve", os.environ["MULTITASK_PATH_SOURCE"])
        self.assertEqual("drive", os.environ["MULTITASK_RENDER_MODE"])

    @mock.patch.dict(os.environ, {
        "VISION_CONTROL_PATH_SOURCE": "skeleton",
        "MULTITASK_PATH_SOURCE": "heatmap",
        "MULTITASK_RENDER_MODE": "heatmap",
    }, clear=True)
    def test_explicit_debug_render_overrides_are_preserved(self):
        configure_perception_defaults()

        self.assertEqual("heatmap", os.environ["MULTITASK_PATH_SOURCE"])
        self.assertEqual("heatmap", os.environ["MULTITASK_RENDER_MODE"])

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


if __name__ == "__main__":
    unittest.main()
