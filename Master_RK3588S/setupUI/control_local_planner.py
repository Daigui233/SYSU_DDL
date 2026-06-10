import math

import cv2
import numpy as np

from control_states import TaskState


# ===== 局部规划调车参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# 当前不使用环境变量覆盖，避免源码默认值和实际运行值不一致。
PLANNER_DEFAULTS = {
    # 最终下发给 TC264D 前允许的图像误差上限，单位 px；调大允许更猛转向，调小会更保守。
    "MAX_TRACK_ERROR": 160.0,

    # AVOID_CAR 基础绕行偏置，单位 px；调大绕得更远，但更容易压出赛道中心区域。
    "CAR_AVOID_OFFSET": 55.0,

    # AVOID_HUMAN 基础绕行偏置，单位 px；人是动态障碍，默认比 car 更远离一些。
    "HUMAN_AVOID_OFFSET": 75.0,

    # 障碍框越宽，额外绕行越大；实际偏置取 max(基础偏置, bbox_width * 该系数)。
    "AVOID_BOX_WIDTH_GAIN": 0.35,

    # COLLECT_GOLD 向金币吸引的比例；调大更主动靠金币，调小更偏向继续巡线。
    "GOLD_BIAS_GAIN": 0.45,

    # 单帧金币吸引最大偏置，单位 px；限制金币目标线不会突然偏得太狠。
    "GOLD_MAX_BIAS": 75.0,

    # RECOVER_LINE 保持上一帧 final_track_error 的衰减时间，单位 s；调大丢线后更久保持原方向。
    "RECOVER_DECAY_SECONDS": 3.0,

    # 检测触发状态容易逐帧抖动；该系数决定 final_track_error 朝新目标移动的速度。
    # 调大反应更快，调小目标线更稳。
    "FINAL_ERROR_SMOOTH_ALPHA": 0.45,

    # 每一帧 final_track_error 最大允许变化量，单位 px；用于防止目标线左右瞬移。
    "MAX_ERROR_STEP_PER_FRAME": 28.0,

    # 绘制偏移目标路径时，尽量让路径点留在 road_mask 边界内侧这么多像素。
    "TARGET_PATH_ROAD_MARGIN": 8.0,

    # 最终控制误差从紫色规划线的这个 y 位置读取；数值越小越偏向画面远处，转向会更提前。
    "CONTROL_LOOKAHEAD_Y_RATIO": 0.58,

    # 紫色规划线近处过渡起点；配合 PATH_NEAR_BIAS_GAIN，近处不再完全贴死红线。
    "PATH_NEAR_ANCHOR_Y_RATIO": 0.94,

    # 紫色规划线到较远处逐渐达到完整避障偏移；y 小于该比例时偏移接近完整值。
    "PATH_FULL_BIAS_Y_RATIO": 0.46,

    # Minimum offset gain near the bottom of the image. This avoids a path that
    # visually stays glued to the red midline until it is too late to steer.
    "PATH_NEAR_BIAS_GAIN": 0.28,

    # 紫色规划线最多使用的点数；减少 road_mask 扫描和绘制开销，避免 RK 上帧率被拖低。
    "TARGET_PATH_MAX_POINTS": 28,

    # 避障方向保持时间，单位 s；检测框在中线附近抖动时不立刻左右换边。
    "AVOID_SIDE_HOLD_SECONDS": 0.8,

    # 目标中心必须越过画面中线这么多 px，才允许从上一避障方向切到另一侧。
    "AVOID_SIDE_SWITCH_MARGIN": 36.0,

    # road mask 左右可通行性评分 ROI 的最低 y 位置比例；越大越只看画面下方近处道路。
    "ROAD_SIDE_ROI_MIN_Y_RATIO": 0.38,

    # 左右可通行性评分 ROI 向障碍框上方扩展的比例；用于估计障碍附近哪边 road mask 更多。
    "ROAD_SIDE_ROI_TOP_BOX_GAIN": 0.40,

    # 左右可通行性评分 ROI 向障碍框下方扩展的比例；调大时会看更靠近车身的区域。
    "ROAD_SIDE_ROI_BOTTOM_BOX_GAIN": 0.50,

    # 左右 road mask 像素数差值小于该值时认为两侧差不多，沿用默认侧。
    "ROAD_SIDE_BALANCE_MIN_DELTA": 20.0,

    # 左右 road mask 像素数相对差值门限；调大更不容易因为轻微差异改变绕行侧。
    "ROAD_SIDE_BALANCE_RATIO": 0.05,
}


def _finite_float(value, default=None):
    try:
        result = float(value)
    except Exception:
        return default
    if not math.isfinite(result):
        return default
    return result


def _clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


def _copy_params(defaults, overrides=None):
    params = dict(defaults)
    if overrides:
        params.update(overrides)
    return params


class LocalPlanner:
    """Image-space local planner that converts RK task state into final track error."""

    def __init__(
        self,
        max_track_error=None,
        car_avoid_offset=None,
        human_avoid_offset=None,
        gold_gain=None,
        gold_max_bias=None,
        recover_decay_seconds=None,
        planner_params=None,
    ):
        self.params = _copy_params(PLANNER_DEFAULTS, planner_params)
        self.max_track_error = float(max_track_error if max_track_error is not None else self.params["MAX_TRACK_ERROR"])
        self.car_avoid_offset = float(car_avoid_offset if car_avoid_offset is not None else self.params["CAR_AVOID_OFFSET"])
        self.human_avoid_offset = float(human_avoid_offset if human_avoid_offset is not None else self.params["HUMAN_AVOID_OFFSET"])
        self.gold_gain = float(gold_gain if gold_gain is not None else self.params["GOLD_BIAS_GAIN"])
        self.gold_max_bias = float(gold_max_bias if gold_max_bias is not None else self.params["GOLD_MAX_BIAS"])
        self.recover_decay_seconds = float(
            recover_decay_seconds if recover_decay_seconds is not None else self.params["RECOVER_DECAY_SECONDS"]
        )
        self.avoid_box_width_gain = float(self.params["AVOID_BOX_WIDTH_GAIN"])
        self.final_error_smooth_alpha = float(self.params["FINAL_ERROR_SMOOTH_ALPHA"])
        self.max_error_step_per_frame = float(self.params["MAX_ERROR_STEP_PER_FRAME"])
        self.target_path_road_margin = float(self.params["TARGET_PATH_ROAD_MARGIN"])
        self.control_lookahead_y_ratio = float(self.params["CONTROL_LOOKAHEAD_Y_RATIO"])
        self.path_near_anchor_y_ratio = float(self.params["PATH_NEAR_ANCHOR_Y_RATIO"])
        self.path_full_bias_y_ratio = float(self.params["PATH_FULL_BIAS_Y_RATIO"])
        self.path_near_bias_gain = float(self.params["PATH_NEAR_BIAS_GAIN"])
        self.target_path_max_points = int(self.params["TARGET_PATH_MAX_POINTS"])
        self.avoid_side_hold_seconds = float(self.params["AVOID_SIDE_HOLD_SECONDS"])
        self.avoid_side_switch_margin = float(self.params["AVOID_SIDE_SWITCH_MARGIN"])
        self.road_side_roi_min_y_ratio = float(self.params["ROAD_SIDE_ROI_MIN_Y_RATIO"])
        self.road_side_roi_top_box_gain = float(self.params["ROAD_SIDE_ROI_TOP_BOX_GAIN"])
        self.road_side_roi_bottom_box_gain = float(self.params["ROAD_SIDE_ROI_BOTTOM_BOX_GAIN"])
        self.road_side_balance_min_delta = float(self.params["ROAD_SIDE_BALANCE_MIN_DELTA"])
        self.road_side_balance_ratio = float(self.params["ROAD_SIDE_BALANCE_RATIO"])
        self.last_final_error = 0.0
        self.last_output_error = None
        self.last_avoid_side = None
        self.last_avoid_category = None
        self.last_avoid_side_ts = 0.0
        self.last_valid_ts = 0.0

    def plan(self, perception, task_decision, now=None):
        now = float(now if now is not None else (perception or {}).get("timestamp", 0.0))
        segmentation = (perception or {}).get("segmentation") or {}
        frame_shape = (perception or {}).get("frame_shape") or [0, 0]
        frame_h = int(frame_shape[0]) if len(frame_shape) >= 1 else 0
        frame_w = int(frame_shape[1]) if len(frame_shape) >= 2 else 0
        frame_w = max(frame_w, 1)
        frame_h = max(frame_h, 1)
        center_x = _finite_float(segmentation.get("center_x"), frame_w * 0.5)
        base_error = _finite_float(segmentation.get("track_error"))
        line_valid = bool(segmentation.get("line_valid") and base_error is not None)
        state = str((task_decision or {}).get("task_state") or TaskState.NORMAL_TRACK.value)
        intent = dict((task_decision or {}).get("planner_intent") or {})

        path_offset = 0.0
        raw_final_error = base_error
        final_error = base_error
        reason = "track_center"
        avoid_side = None

        if state == TaskState.LINE_LOSS_SAFE_STOP.value:
            final_error = None
            raw_final_error = None
            reason = "safe_stop"
        elif state == TaskState.RECOVER_LINE.value or not line_valid:
            final_error = self._recover_error(now)
            raw_final_error = final_error
            reason = "hold_last_line"
        elif state == TaskState.AVOID_HUMAN.value:
            target = intent.get("target") or {}
            avoid_side = self._choose_avoid_side(
                segmentation,
                target,
                frame_w,
                frame_h,
                prefer_away=True,
                category="human",
                now=now,
            )
            path_offset = self._side_sign(avoid_side) * self._avoid_offset(target, self.human_avoid_offset)
            reason = f"avoid_human_{avoid_side}"
        elif state == TaskState.AVOID_CAR.value:
            target = intent.get("target") or {}
            avoid_side = self._choose_avoid_side(
                segmentation,
                target,
                frame_w,
                frame_h,
                prefer_away=False,
                category="car",
                now=now,
            )
            path_offset = self._side_sign(avoid_side) * self._avoid_offset(target, self.car_avoid_offset)
            reason = f"avoid_car_{avoid_side}"
        elif state == TaskState.COLLECT_GOLD.value:
            target = intent.get("target") or {}
            path_offset = self._gold_offset(base_error, target, center_x)
            reason = "collect_gold"

        raw_target_path = []
        if line_valid and final_error is not None:
            raw_target_path = self._build_target_path(segmentation, path_offset, frame_w, frame_h)
            if state != TaskState.NORMAL_TRACK.value and raw_target_path:
                raw_final_error = self._path_error_at_lookahead(raw_target_path, center_x, frame_h)
                if raw_final_error is None and base_error is not None:
                    lookahead_y = frame_h * self.control_lookahead_y_ratio
                    raw_final_error = base_error + path_offset * self._path_bias_gain(lookahead_y, frame_h)

        if final_error is not None:
            if self.last_output_error is None and state != TaskState.NORMAL_TRACK.value and base_error is not None:
                self.last_output_error = base_error
            final_error = self._smooth_final_error(
                _clamp(raw_final_error, -self.max_track_error, self.max_track_error),
                state,
            )
            if line_valid:
                self.last_final_error = final_error
                self.last_valid_ts = now

        effective_path_offset = path_offset
        target_path = raw_target_path
        if line_valid and final_error is not None and state != TaskState.NORMAL_TRACK.value:
            effective_path_offset = self._effective_path_offset_for_error(
                segmentation,
                final_error,
                center_x,
                path_offset,
                frame_w,
                frame_h,
            )
            target_path = self._build_target_path(segmentation, effective_path_offset, frame_w, frame_h)
        elif line_valid and final_error is not None and not target_path:
            target_path = self._build_target_path(segmentation, 0.0, frame_w, frame_h)

        target_x = None
        if final_error is not None:
            target_x = _clamp(center_x + final_error, 0, frame_w - 1)

        return {
            "final_track_error": float(final_error) if final_error is not None else None,
            "raw_final_track_error": float(raw_final_error) if raw_final_error is not None else None,
            "base_track_error": float(base_error) if base_error is not None else None,
            "target_x": float(target_x) if target_x is not None else None,
            "target_path": target_path,
            "center_x": float(center_x),
            "desired_path_offset": float(path_offset),
            "effective_path_offset": float(effective_path_offset),
            "avoid_side": avoid_side,
            "planner_reason": reason,
            "line_valid": line_valid,
        }

    def _recover_error(self, now):
        if self.last_valid_ts <= 0.0:
            return 0.0
        age = max(0.0, float(now) - self.last_valid_ts)
        decay = max(0.0, 1.0 - age / max(0.01, self.recover_decay_seconds))
        return self.last_final_error * decay

    def _smooth_final_error(self, target_error, state):
        if state == TaskState.LINE_LOSS_SAFE_STOP.value:
            return target_error

        previous = self.last_output_error
        if previous is None or state == TaskState.NORMAL_TRACK.value:
            smoothed = target_error
        else:
            alpha = _clamp(self.final_error_smooth_alpha, 0.0, 1.0)
            smoothed = previous + (target_error - previous) * alpha
            max_step = max(1.0, self.max_error_step_per_frame)
            smoothed = previous + _clamp(smoothed - previous, -max_step, max_step)

        smoothed = _clamp(smoothed, -self.max_track_error, self.max_track_error)
        self.last_output_error = smoothed
        return smoothed

    def _build_target_path(self, segmentation, path_offset, frame_w, frame_h):
        mid_points = segmentation.get("mid_points") or []
        if len(mid_points) < 2:
            return []

        points = self._sample_mid_points(mid_points)
        road_mask = segmentation.get("road_mask")
        road_bounds = self._road_bounds_for_points(road_mask, points, frame_w)
        path = []
        for point in points:
            try:
                x_raw, y_raw = point
                y = int(_clamp(y_raw, 0, frame_h - 1))
                bias_gain = self._path_bias_gain(y, frame_h)
                shifted_x = _finite_float(x_raw, frame_w * 0.5) + float(path_offset) * bias_gain
            except Exception:
                continue
            x = self._clamp_path_x_to_road(shifted_x, y, road_bounds, frame_w)
            path.append((int(round(x)), y))
        return path

    def _sample_mid_points(self, mid_points):
        max_points = max(2, int(self.target_path_max_points))
        count = len(mid_points)
        if count <= max_points:
            return list(mid_points)
        if max_points == 2:
            return [mid_points[0], mid_points[-1]]

        sampled = []
        last_idx = None
        step = (count - 1) / float(max_points - 1)
        for i in range(max_points):
            idx = int(round(i * step))
            idx = max(0, min(count - 1, idx))
            if idx != last_idx:
                sampled.append(mid_points[idx])
                last_idx = idx
        return sampled

    def _path_bias_gain(self, y, frame_h):
        near_y = float(frame_h) * _clamp(self.path_near_anchor_y_ratio, 0.0, 1.0)
        full_y = float(frame_h) * _clamp(self.path_full_bias_y_ratio, 0.0, 1.0)
        if near_y <= full_y:
            near_y, full_y = full_y, near_y
        span = max(1.0, near_y - full_y)
        t = _clamp((near_y - float(y)) / span, 0.0, 1.0)
        smooth = t * t * (3.0 - 2.0 * t)
        near_gain = _clamp(self.path_near_bias_gain, 0.0, 1.0)
        return near_gain + (1.0 - near_gain) * smooth

    def _road_bounds_for_points(self, road_mask, points, frame_w):
        if road_mask is None or not hasattr(road_mask, "shape"):
            return {}
        mask = np.asarray(road_mask)
        if mask.ndim != 2:
            return {}

        bounds = {}
        margin = max(0.0, self.target_path_road_margin)
        for _x, y_raw in points:
            y = int(_clamp(y_raw, 0, mask.shape[0] - 1))
            if y in bounds:
                continue
            xs = np.flatnonzero(mask[y])
            if xs.size < 2:
                continue
            left = min(frame_w - 1, float(xs[0]) + margin)
            right = max(0.0, float(xs[-1]) - margin)
            if left > right:
                left = float(xs[0])
                right = float(xs[-1])
            bounds[y] = (left, right)
        return bounds

    def _clamp_path_x_to_road(self, x, y, road_bounds, frame_w):
        x = _clamp(x, 0, frame_w - 1)
        bounds = road_bounds.get(int(y)) if road_bounds else None
        if bounds is None:
            return x
        left, right = bounds
        return _clamp(x, left, right)

    def _path_error_at_lookahead(self, path, center_x, frame_h):
        if not path:
            return None
        lookahead_y = int(_clamp(frame_h * self.control_lookahead_y_ratio, 0, frame_h - 1))
        idx = min(range(len(path)), key=lambda i: abs(path[i][1] - lookahead_y))
        return float(path[idx][0]) - float(center_x)

    def _effective_path_offset_for_error(self, segmentation, final_error, center_x, desired_offset, frame_w, frame_h):
        mid_points = segmentation.get("mid_points") or []
        if len(mid_points) < 2:
            return desired_offset

        lookahead_y = int(_clamp(frame_h * self.control_lookahead_y_ratio, 0, frame_h - 1))
        try:
            idx = min(range(len(mid_points)), key=lambda i: abs(mid_points[i][1] - lookahead_y))
            base_x = _finite_float(mid_points[idx][0], center_x)
        except Exception:
            return desired_offset

        gain = self._path_bias_gain(lookahead_y, frame_h)
        if gain < 0.05:
            return desired_offset

        target_x = _clamp(float(center_x) + float(final_error), 0, frame_w - 1)
        effective = (target_x - base_x) / gain
        max_abs = max(abs(float(desired_offset)), 1.0)
        return _clamp(effective, -max_abs, max_abs)

    def _avoid_offset(self, target, default_offset):
        size = target.get("size") or [0.0, 0.0]
        box_w = _finite_float(size[0], 0.0)
        return max(float(default_offset), min(self.max_track_error, box_w * self.avoid_box_width_gain))

    def _gold_error(self, base_error, target, center_x):
        center = target.get("center") or [None, None]
        target_x = _finite_float(center[0])
        if target_x is None:
            return base_error
        target_error = target_x - center_x
        bias = _clamp((target_error - base_error) * self.gold_gain, -self.gold_max_bias, self.gold_max_bias)
        return base_error + bias

    def _gold_offset(self, base_error, target, center_x):
        if base_error is None:
            return 0.0
        gold_error = self._gold_error(base_error, target, center_x)
        if gold_error is None:
            return 0.0
        return _clamp(gold_error - base_error, -self.gold_max_bias, self.gold_max_bias)

    def _choose_avoid_side(self, segmentation, target, frame_w, frame_h, prefer_away, category, now):
        center_x = _finite_float(segmentation.get("center_x"), frame_w * 0.5)
        target_center = target.get("center") or [center_x, frame_h * 0.5]
        target_x = _finite_float(target_center[0], center_x)
        preferred = "right" if target_x <= center_x else "left"
        if not prefer_away:
            preferred = self._road_free_side(segmentation, target, preferred)
        else:
            road_side = self._road_free_side(segmentation, target, preferred)
            preferred = road_side or preferred
        return self._apply_avoid_side_hysteresis(category, preferred, target_x, center_x, now)

    def _apply_avoid_side_hysteresis(self, category, preferred, target_x, center_x, now):
        previous = self.last_avoid_side
        same_category = self.last_avoid_category == category
        age = max(0.0, float(now) - self.last_avoid_side_ts) if self.last_avoid_side_ts > 0.0 else None
        in_hold = same_category and previous in ("left", "right") and age is not None and age <= self.avoid_side_hold_seconds

        side = preferred
        if in_hold and preferred != previous:
            lateral = abs(float(target_x) - float(center_x))
            if lateral < self.avoid_side_switch_margin:
                side = previous

        self.last_avoid_side = side
        self.last_avoid_category = category
        self.last_avoid_side_ts = float(now)
        return side

    def _road_free_side(self, segmentation, target, default_side):
        road_mask = segmentation.get("road_mask")
        if road_mask is None or not hasattr(road_mask, "shape"):
            return default_side
        mask = np.asarray(road_mask)
        if mask.ndim != 2 or not np.any(mask):
            return default_side

        h, w = mask.shape[:2]
        center = target.get("center") or [w * 0.5, h * 0.6]
        bbox = target.get("bbox") or [0.0, h * 0.35, w, h * 0.75]
        target_x = int(_clamp(_finite_float(center[0], w * 0.5), 0, w - 1))
        bottom = _finite_float(bbox[3], h * 0.65)
        box_h = _finite_float((target.get("size") or [0.0, h * 0.2])[1], h * 0.2)
        y0 = int(_clamp(bottom - box_h * self.road_side_roi_top_box_gain, h * self.road_side_roi_min_y_ratio, h - 1))
        y1 = int(_clamp(bottom + box_h * self.road_side_roi_bottom_box_gain, y0 + 1, h))
        roi = mask[y0:y1, :]
        left_score = float(np.count_nonzero(roi[:, :target_x]))
        right_score = float(np.count_nonzero(roi[:, target_x:]))
        if left_score <= 0.0 and right_score <= 0.0:
            return default_side
        if abs(left_score - right_score) < max(
            self.road_side_balance_min_delta,
            self.road_side_balance_ratio * (left_score + right_score),
        ):
            return default_side
        return "left" if left_score > right_score else "right"

    @staticmethod
    def _side_sign(side):
        return -1.0 if side == "left" else 1.0

    def draw_debug(self, frame, task_decision, plan_result):
        if frame is None or plan_result is None:
            return frame
        out = frame
        h, w = out.shape[:2]
        target_x = _finite_float(plan_result.get("target_x"))
        target_path = list(plan_result.get("target_path") or [])
        if len(target_path) >= 2:
            pts = np.array(target_path, np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], False, (255, 0, 255), 3)
            lookahead_y = int(h * self.control_lookahead_y_ratio)
            idx = min(range(len(target_path)), key=lambda i: abs(target_path[i][1] - lookahead_y))
            cv2.circle(out, target_path[idx], 8, (255, 0, 255), -1)
            cv2.circle(out, target_path[0], 5, (255, 0, 255), -1)
        elif target_x is not None:
            x = int(_clamp(target_x, 0, w - 1))
            cv2.circle(out, (x, int(h * 0.68)), 8, (255, 0, 255), -1)

        state = str((task_decision or {}).get("task_state") or "N/A")
        err = plan_result.get("final_track_error")
        err_text = "N/A" if err is None else f"{float(err):.1f}"
        cv2.putText(out, f"Task: {state}", (10, 172), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)
        cv2.putText(out, f"Final err: {err_text}", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)
        return out
