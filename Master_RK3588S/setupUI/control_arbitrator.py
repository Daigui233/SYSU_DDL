import math

from control_car_link import CONTROL_FLAG_USE_TARGET_SPEED, STATE_SAFE_STOP, STATE_TRACK


CONTROL_COMMAND_REPEAT_INTERVAL = 0.2


def _finite_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


class ControlArbitrator:
    """Generate the final TC264D command from vision state and optional gamepad override."""

    def __init__(
        self,
        control_scale=None,
        command_repeat_interval=None,
        state_track=None,
        state_safe_stop=None,
        control_flag_use_target_speed=None,
    ):
        # control_scale is kept only for old call-site compatibility.
        self.control_scale = 1.0
        self.command_repeat_interval = float(
            CONTROL_COMMAND_REPEAT_INTERVAL if command_repeat_interval is None else command_repeat_interval
        )
        self.state_track = int(STATE_TRACK if state_track is None else state_track)
        self.state_safe_stop = int(STATE_SAFE_STOP if state_safe_stop is None else state_safe_stop)
        self.control_flag_use_target_speed = int(
            CONTROL_FLAG_USE_TARGET_SPEED if control_flag_use_target_speed is None else control_flag_use_target_speed
        )

    def should_repeat_command(self, now, last_command_ts):
        return float(now) - float(last_command_ts) >= self.command_repeat_interval

    def decide(
        self,
        now,
        track_error,
        gamepad_cmd=None,
        desired_speed=None,
        state_text=None,
        safe_stop=False,
        state_cmd=None,
        line_loss_age=0.0,
    ):
        command = self._vision_command(
            now,
            track_error,
            desired_speed=desired_speed,
            state_text=state_text,
            safe_stop=safe_stop,
            state_cmd=state_cmd,
            line_loss_age=line_loss_age,
        )
        if gamepad_cmd is not None:
            gamepad_error = _finite_float(gamepad_cmd.get("track_error"))
            gamepad_speed = _finite_float(gamepad_cmd.get("target_speed"))
            try:
                gamepad_state = int(gamepad_cmd["state_cmd"])
                gamepad_flags = int(gamepad_cmd["flags"])
            except (KeyError, TypeError, ValueError):
                gamepad_error = None
                gamepad_speed = None

            if gamepad_error is None or gamepad_speed is None:
                return {
                    "track_error": 0.0,
                    "target_speed": 0.0,
                    "state_cmd": self.state_safe_stop,
                    "flags": 0,
                    "state_text": "GAMEPAD_INVALID_INPUT",
                    "line_loss_age": float(line_loss_age),
                }
            command = {
                "track_error": gamepad_error,
                "target_speed": gamepad_speed,
                "state_cmd": gamepad_state,
                "flags": gamepad_flags,
                "state_text": "GAMEPAD_SAFE_STOP" if gamepad_cmd.get("safe_stop") else "GAMEPAD_TRACK",
                "line_loss_age": float(line_loss_age),
            }
        return command

    def _vision_command(
        self,
        now,
        track_error,
        desired_speed=None,
        state_text=None,
        safe_stop=False,
        state_cmd=None,
        line_loss_age=0.0,
    ):
        if safe_stop:
            return {
                "track_error": 0.0,
                "target_speed": 0.0,
                "state_cmd": int(state_cmd) if state_cmd is not None else self.state_safe_stop,
                "flags": 0,
                "state_text": state_text or "LINE_LOSS_SAFE_STOP",
                "line_loss_age": float(line_loss_age),
            }

        valid_track_error = _finite_float(track_error)
        if valid_track_error is not None:
            speed = _finite_float(desired_speed, 0.0)
            return {
                "track_error": valid_track_error,
                "target_speed": speed,
                "state_cmd": int(state_cmd) if state_cmd is not None else self.state_track,
                "flags": self.control_flag_use_target_speed,
                "state_text": state_text or "VISION",
                "line_loss_age": float(line_loss_age),
            }

        return {
            "track_error": 0.0,
            "target_speed": 0.0,
            "state_cmd": self.state_safe_stop,
            "flags": 0,
            "state_text": "CONTROL_INVALID_INPUT",
            "line_loss_age": float(line_loss_age),
        }
