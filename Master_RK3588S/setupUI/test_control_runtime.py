import os
import sys
import unittest
from unittest import mock


sys.path.insert(0, os.path.dirname(__file__))

import control_runtime  # noqa: E402


class ControlRuntimeVisionHoldTest(unittest.TestCase):
    @staticmethod
    def _runtime():
        return control_runtime.ControlRuntime(
            log_func=None,
            pose_bridge=object(),
            gamepad_receiver=object(),
            car_link=object(),
        )

    def test_camera_hold_keeps_last_command_for_three_seconds(self):
        runtime = self._runtime()
        with mock.patch(
                "control_runtime.time.monotonic", return_value=10.0):
            runtime.update_vision_command(-42.0, 0.15)
        with mock.patch(
                "control_runtime.time.monotonic", return_value=10.1):
            self.assertTrue(runtime.begin_vision_command_hold(3.0))
        with mock.patch(
                "control_runtime.time.monotonic", return_value=12.9):
            command = runtime._fresh_vision_command()
        self.assertIsNotNone(command)
        self.assertEqual(command["track_error"], -42.0)
        with mock.patch(
                "control_runtime.time.monotonic", return_value=13.11):
            self.assertIsNone(runtime._fresh_vision_command())

    def test_repeated_invalid_frames_do_not_extend_hold(self):
        runtime = self._runtime()
        with mock.patch(
                "control_runtime.time.monotonic", return_value=20.0):
            runtime.update_vision_command(35.0, 0.15)
            runtime.begin_vision_command_hold(3.0)
        with mock.patch(
                "control_runtime.time.monotonic", return_value=22.0):
            self.assertTrue(runtime.begin_vision_command_hold(3.0))
        with mock.patch(
                "control_runtime.time.monotonic", return_value=23.01):
            self.assertIsNone(runtime._fresh_vision_command())

    def test_new_valid_command_cancels_old_hold_window(self):
        runtime = self._runtime()
        with mock.patch(
                "control_runtime.time.monotonic", return_value=30.0):
            runtime.update_vision_command(-20.0, 0.15)
            runtime.begin_vision_command_hold(3.0)
        with mock.patch(
                "control_runtime.time.monotonic", return_value=31.0):
            runtime.update_vision_command(18.0, 0.15)
        with mock.patch(
                "control_runtime.time.monotonic", return_value=31.1):
            command = runtime._fresh_vision_command()
            self.assertEqual(runtime.vision_hold_remaining(), 0.0)
        self.assertIsNotNone(command)
        self.assertEqual(command["track_error"], 18.0)


if __name__ == "__main__":
    unittest.main()
