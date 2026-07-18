import os
import sys
import threading
import unittest
from unittest import mock


sys.path.insert(0, os.path.dirname(__file__))

import control_runtime  # noqa: E402


class _FakePoseBridge:
    def start(self):
        pass

    def stop(self):
        pass

    def snapshot(self):
        return {}


class _FakeGamepadReceiver:
    def start(self):
        pass

    def stop(self):
        pass

    def active_command(self):
        return None, {}

    def snapshot(self):
        return {}


class _RecordingCarLink:
    def __init__(self, sent_event=None):
        self.sent_event = sent_event
        self.sent_commands = []

    def send_cmd(self, **command):
        self.sent_commands.append(dict(command))
        if self.sent_event is not None:
            self.sent_event.set()
        return True

    def safe_stop(self):
        return self.send_cmd(
            track_error=0.0,
            target_speed=0.0,
            state_cmd=control_runtime.STATE_SAFE_STOP,
            flags=0,
        )

    def get_feedback(self):
        return {}

    def get_send_status(self):
        return {"ok": True, "timestamp": None}

    def close(self):
        pass


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

    def test_runtime_forwards_vision_error_to_car_link(self):
        sent_event = threading.Event()
        car_link = _RecordingCarLink(sent_event)
        runtime = control_runtime.ControlRuntime(
            control_interval=0.01,
            log_func=None,
            pose_bridge=_FakePoseBridge(),
            gamepad_receiver=_FakeGamepadReceiver(),
            car_link=car_link,
        )

        runtime.update_vision_command(
            23.5, 0.08,
            state_cmd=control_runtime.STATE_TRACK,
            flags=control_runtime.CONTROL_FLAG_USE_TARGET_SPEED)
        runtime.start()
        try:
            self.assertTrue(sent_event.wait(0.5))
        finally:
            runtime.stop()

        self.assertEqual(car_link.sent_commands[0]["track_error"], 23.5)
        self.assertEqual(car_link.sent_commands[0]["target_speed"], 0.08)
        self.assertEqual(
            car_link.sent_commands[0]["state_cmd"],
            control_runtime.STATE_TRACK)
        self.assertEqual(
            car_link.sent_commands[0]["flags"],
            control_runtime.CONTROL_FLAG_USE_TARGET_SPEED)


if __name__ == "__main__":
    unittest.main()
