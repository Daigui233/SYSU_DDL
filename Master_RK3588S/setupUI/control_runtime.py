import math
import threading
import time

from control_car_link import (
    CONTROL_FLAG_USE_TARGET_SPEED,
    STATE_TRACK,
    CarControlLink,
)
from control_gamepad_receiver import GamepadControlReceiver
from pose_ar_bridge import ARPoseBridge


CONTROL_INTERVAL_SECONDS = 0.05
VISION_COMMAND_TTL_SECONDS = 0.25


def _finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class ControlRuntime:
    """Own pose forwarding, manual override, and the single TC264D link."""

    def __init__(
        self,
        control_interval=CONTROL_INTERVAL_SECONDS,
        vision_ttl=VISION_COMMAND_TTL_SECONDS,
        log_func=print,
        pose_bridge=None,
        gamepad_receiver=None,
        car_link=None,
    ):
        self.control_interval = float(control_interval)
        self.vision_ttl = float(vision_ttl)
        self.log_func = log_func

        self.pose_bridge = pose_bridge if pose_bridge is not None else ARPoseBridge(log_func=log_func)
        self.gamepad_receiver = (
            gamepad_receiver
            if gamepad_receiver is not None
            else GamepadControlReceiver(log_func=log_func)
        )
        self.car_link = car_link if car_link is not None else CarControlLink()

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._vision_command = None
        self._vision_timestamp = 0.0
        self._control_source = "IDLE"
        self._last_command = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self.pose_bridge.start()
        self.gamepad_receiver.start()
        self._thread = threading.Thread(target=self._run, name="control-runtime", daemon=True)
        self._thread.start()
        self._log("Control runtime started")

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.car_link.safe_stop()
        self.gamepad_receiver.stop()
        self.pose_bridge.stop()
        self.car_link.close()
        self._log("Control runtime stopped")

    def update_vision_command(
        self,
        track_error,
        target_speed,
        state_cmd=STATE_TRACK,
        flags=CONTROL_FLAG_USE_TARGET_SPEED,
    ):
        """Publish the latest visual command for automatic fallback."""
        error = _finite_float(track_error)
        speed = _finite_float(target_speed)
        if error is None or speed is None:
            self.clear_vision_command()
            return False
        with self._lock:
            self._vision_command = {
                "track_error": error,
                "target_speed": speed,
                "state_cmd": int(state_cmd),
                "flags": int(flags),
            }
            self._vision_timestamp = time.monotonic()
        return True

    def clear_vision_command(self):
        with self._lock:
            self._vision_command = None
            self._vision_timestamp = 0.0

    def snapshot(self):
        with self._lock:
            vision_age = (
                time.monotonic() - self._vision_timestamp
                if self._vision_timestamp
                else None
            )
            source = self._control_source
            command = dict(self._last_command) if self._last_command else None
        return {
            "source": source,
            "last_command": command,
            "vision_age": vision_age,
            "vision_ttl": self.vision_ttl,
            "pose": self.pose_bridge.snapshot(),
            "gamepad": self.gamepad_receiver.snapshot(),
            "serial": self.car_link.get_feedback(),
        }

    def _run(self):
        had_active_source = False
        stop_sent = False
        while not self._stop_event.is_set():
            gamepad_command, _status = self.gamepad_receiver.active_command()
            vision_command = self._fresh_vision_command()

            if gamepad_command is not None:
                command = gamepad_command
                source = "GAMEPAD"
            elif vision_command is not None:
                command = vision_command
                source = "VISION"
            else:
                command = None
                source = "IDLE"

            if command is not None:
                self.car_link.send_cmd(
                    track_error=command["track_error"],
                    target_speed=command["target_speed"],
                    state_cmd=command["state_cmd"],
                    flags=command["flags"],
                )
                had_active_source = True
                stop_sent = False
            elif had_active_source and not stop_sent:
                self.car_link.safe_stop()
                stop_sent = True

            with self._lock:
                self._control_source = source
                self._last_command = dict(command) if command else None
            self._stop_event.wait(max(0.01, self.control_interval))

    def _fresh_vision_command(self):
        with self._lock:
            if self._vision_command is None or not self._vision_timestamp:
                return None
            if time.monotonic() - self._vision_timestamp > self.vision_ttl:
                return None
            return dict(self._vision_command)

    def _log(self, message):
        if self.log_func is not None:
            self.log_func(message)
