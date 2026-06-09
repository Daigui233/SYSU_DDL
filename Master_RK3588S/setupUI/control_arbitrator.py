import math


class ControlArbitrator:
    """Generate the final TC264D command from vision state and optional gamepad override."""

    def __init__(
        self,
        track_speed,
        fallback_speed,
        control_scale,
        line_loss_safe_stop_timeout,
        state_track,
        state_safe_stop,
        control_flag_use_target_speed,
    ):
        self.track_speed = float(track_speed)
        self.fallback_speed = float(fallback_speed)
        self.control_scale = float(control_scale)
        self.line_loss_safe_stop_timeout = float(line_loss_safe_stop_timeout)
        self.state_track = int(state_track)
        self.state_safe_stop = int(state_safe_stop)
        self.control_flag_use_target_speed = int(control_flag_use_target_speed)
        self.line_missing_since_ts = None

    def decide(self, now, track_error, gamepad_cmd=None):
        command = self._vision_command(now, track_error)
        if gamepad_cmd is not None:
            command = {
                "track_error": float(gamepad_cmd["track_error"]),
                "target_speed": float(gamepad_cmd["target_speed"]),
                "state_cmd": int(gamepad_cmd["state_cmd"]),
                "flags": int(gamepad_cmd["flags"]),
                "state_text": "GAMEPAD_SAFE_STOP" if gamepad_cmd.get("safe_stop") else "GAMEPAD_TRACK",
                "line_loss_age": self.line_loss_age(now),
            }
        return command

    def line_loss_age(self, now):
        if self.line_missing_since_ts is None:
            return 0.0
        return max(0.0, float(now) - self.line_missing_since_ts)

    def _vision_command(self, now, track_error):
        if track_error is not None and math.isfinite(float(track_error)):
            self.line_missing_since_ts = None
            return {
                "track_error": float(track_error) * self.control_scale,
                "target_speed": self.track_speed,
                "state_cmd": self.state_track,
                "flags": self.control_flag_use_target_speed,
                "state_text": "VISION",
                "line_loss_age": 0.0,
            }

        if self.line_missing_since_ts is None:
            self.line_missing_since_ts = float(now)
        line_loss_age = self.line_loss_age(now)

        if line_loss_age >= self.line_loss_safe_stop_timeout:
            return {
                "track_error": 0.0,
                "target_speed": 0.0,
                "state_cmd": self.state_safe_stop,
                "flags": 0,
                "state_text": "LINE_LOSS_SAFE_STOP",
                "line_loss_age": line_loss_age,
            }

        return {
            "track_error": 0.0,
            "target_speed": self.fallback_speed,
            "state_cmd": self.state_track,
            "flags": self.control_flag_use_target_speed,
            "state_text": "TRACK_FALLBACK",
            "line_loss_age": line_loss_age,
        }
