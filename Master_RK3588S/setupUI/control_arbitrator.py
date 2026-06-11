import json
import math
import os

from control_car_link import CONTROL_FLAG_USE_TARGET_SPEED, STATE_SAFE_STOP, STATE_TRACK
from control_states import TaskState

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_AR_CONFIG_PATH = os.path.join(BASE_DIR, "dist", "main_config.json")

# ===== 控制下发参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# control_scale 用于把上位机图像误差换算成 TC264D 舵机误差输入。
# 这里保留全局默认值，实际视觉控车优先使用下面的 CONTROL_ERROR_PARAMS 分状态参数表。
CONTROL_SCALE = 0.65
CONTROL_ERROR_AUTO_MAX_RATIO = 0.50
CONTROL_ERROR_FALLBACK_MAX_ABS = 160.0

# 分状态误差下发表。修改这里后需要重启 ar_receiver.py 生效。
# scale: 上位机 final_track_error 乘到 TC264D input_track_error 的比例。
# max_abs: 下发给 TC264D 的最大绝对误差；<=0 表示按 dist/main_config.json 的 width 自动计算。
# max_step: 连续两帧下发误差最大变化量，抑制弯道过冲后反向猛修。
# deadband: 小误差死区，减少直道细碎抖动。
# curve_start / curve_gain: 类似 Cardo 的大弯非线性增益，小误差不变，大误差额外增强。
CONTROL_ERROR_PARAMS = {
    TaskState.NORMAL_TRACK.value: {
        "scale": 0.65,
        "max_abs": 0.0,
        "max_step": 22.0,
        "deadband": 3.0,
        "curve_start": 40.0,
        "curve_gain": 0.55,
    },
    TaskState.AVOID_STONE.value: {
        "scale": 0.65,
        "max_abs": 0.0,
        "max_step": 22.0,
        "deadband": 3.0,
        "curve_start": 40.0,
        "curve_gain": 0.55,
    },
    TaskState.RECOVER_LINE.value: {
        "scale": 0.55,
        "max_abs": 85.0,
        "max_step": 8.0,
        "deadband": 3.0,
        "curve_start": 0.0,
        "curve_gain": 0.0,
    },
    TaskState.COLLECT_GOLD.value: {
        "scale": 0.65,
        "max_abs": 0.0,
        "max_step": 14.0,
        "deadband": 2.0,
        "curve_start": 50.0,
        "curve_gain": 0.25,
    },
    TaskState.AVOID_CAR.value: {
        "scale": 0.90,
        "max_abs": 0.0,
        "max_step": 22.0,
        "deadband": 2.0,
        "curve_start": 60.0,
        "curve_gain": 0.20,
    },
    TaskState.AVOID_HUMAN.value: {
        "scale": 0.90,
        "max_abs": 0.0,
        "max_step": 22.0,
        "deadband": 2.0,
        "curve_start": 60.0,
        "curve_gain": 0.20,
    },
    TaskState.TRAFFIC_LIGHT_STOP.value: {
        "scale": 0.0,
        "max_abs": 1.0,
        "max_step": 999.0,
        "deadband": 0.0,
        "curve_start": 0.0,
        "curve_gain": 0.0,
    },
    TaskState.ENDSIGN_STOP.value: {
        "scale": 0.0,
        "max_abs": 1.0,
        "max_step": 999.0,
        "deadband": 0.0,
        "curve_start": 0.0,
        "curve_gain": 0.0,
    },
}

DEFAULT_CONTROL_ERROR_PARAM = {
    "scale": CONTROL_SCALE,
    "max_abs": 0.0,
    "max_step": 10.0,
    "deadband": 3.0,
    "curve_start": 0.0,
    "curve_gain": 0.0,
}

# 没有新视频帧时，重复发送上一次状态机决策的最小间隔，单位 s。
CONTROL_COMMAND_REPEAT_INTERVAL = 0.2


def _load_config_frame_width(config_path=DEFAULT_AR_CONFIG_PATH):
    try:
        with open(config_path, "r", encoding="utf-8") as fp:
            data = json.load(fp)
    except Exception:
        return None

    try:
        width = float(data.get("width"))
    except Exception:
        return None
    if not math.isfinite(width) or width <= 1.0:
        return None
    return width


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
        self.control_scale = float(CONTROL_SCALE if control_scale is None else control_scale)
        self.command_repeat_interval = float(
            CONTROL_COMMAND_REPEAT_INTERVAL if command_repeat_interval is None else command_repeat_interval
        )
        self.state_track = int(STATE_TRACK if state_track is None else state_track)
        self.state_safe_stop = int(STATE_SAFE_STOP if state_safe_stop is None else state_safe_stop)
        self.control_flag_use_target_speed = int(
            CONTROL_FLAG_USE_TARGET_SPEED if control_flag_use_target_speed is None else control_flag_use_target_speed
        )
        self.config_frame_width = _load_config_frame_width()
        self.auto_max_abs = self._auto_max_abs()
        self._last_vision_track_error = None

    def _param_for_state(self, state_text):
        return CONTROL_ERROR_PARAMS.get(str(state_text or ""), DEFAULT_CONTROL_ERROR_PARAM)

    def _auto_max_abs(self):
        if self.config_frame_width is None:
            return CONTROL_ERROR_FALLBACK_MAX_ABS
        return max(1.0, float(self.config_frame_width) * CONTROL_ERROR_AUTO_MAX_RATIO)

    def _max_abs_for_param(self, param):
        max_abs = float(param.get("max_abs", 0.0))
        if max_abs > 0.0:
            return max_abs
        return self.auto_max_abs

    @staticmethod
    def _clamp(value, low, high):
        return max(float(low), min(float(high), float(value)))

    @staticmethod
    def _apply_curve_boost(raw_error, shaped_error, param):
        start = float(param.get("curve_start", 0.0))
        gain = float(param.get("curve_gain", 0.0))
        if start <= 0.0 or gain <= 0.0:
            return shaped_error

        extra = max(0.0, abs(float(raw_error)) - start) * gain
        if extra <= 0.0:
            return shaped_error
        return float(shaped_error) + math.copysign(extra, float(raw_error))

    def _shape_vision_track_error(self, track_error, state_text, safe_stop=False):
        if safe_stop:
            self._last_vision_track_error = 0.0
            return 0.0

        param = self._param_for_state(state_text)
        raw_error = float(track_error)
        shaped = raw_error * float(param["scale"])
        shaped = self._apply_curve_boost(raw_error, shaped, param)
        if abs(shaped) < float(param["deadband"]):
            shaped = 0.0

        max_abs = max(1.0, self._max_abs_for_param(param))
        shaped = self._clamp(shaped, -max_abs, max_abs)

        previous = self._last_vision_track_error
        if previous is not None:
            max_step = max(1.0, float(param["max_step"]))
            shaped = previous + self._clamp(shaped - previous, -max_step, max_step)

        self._last_vision_track_error = shaped
        return shaped

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
            command = {
                "track_error": float(gamepad_cmd["track_error"]),
                "target_speed": float(gamepad_cmd["target_speed"]),
                "state_cmd": int(gamepad_cmd["state_cmd"]),
                "flags": int(gamepad_cmd["flags"]),
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
            shaped_error = self._shape_vision_track_error(0.0, state_text, safe_stop=True)
            return {
                "track_error": shaped_error,
                "target_speed": 0.0,
                "state_cmd": int(state_cmd) if state_cmd is not None else self.state_safe_stop,
                "flags": 0,
                "state_text": state_text or "LINE_LOSS_SAFE_STOP",
                "line_loss_age": float(line_loss_age),
            }

        if track_error is not None and math.isfinite(float(track_error)):
            speed = 0.0
            if desired_speed is not None and math.isfinite(float(desired_speed)):
                speed = float(desired_speed)
            shaped_error = self._shape_vision_track_error(track_error, state_text)
            return {
                "track_error": shaped_error,
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
