import math

import cv2
import numpy as np

from control_states import TaskState

# ===== 局部规划调车参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# 当前不使用环境变量覆盖，避免源码默认值和实际运行值不一致。
PLANNER_DEFAULTS = {
    # AVOID_CAR 基础绕行偏置，单位 px；调大绕得更远，但更容易压出赛道中心区域。
    "CAR_AVOID_OFFSET": 55.0,

    # AVOID_HUMAN 基础绕行偏置，单位 px；人是动态障碍，默认比 car 更远离一些。
    "HUMAN_AVOID_OFFSET": 75.0,

    # Human motion planning: vx is measured in image pixels per second.
    # If reliable, the planner passes behind the human: vx>0 -> left, vx<0 -> right.
    "HUMAN_MOTION_MIN_VX": 35.0,
    "HUMAN_MOTION_SMOOTH_ALPHA": 0.45,
    "HUMAN_MOTION_MAX_DT": 0.50,
    "HUMAN_MOTION_MAX_MATCH_DX_RATIO": 0.35,

    # Predict the short-term occupied bbox for moving humans before scoring corridors.
    "HUMAN_MOTION_PREDICT_SECONDS": 0.45,

    # Human yield/wait action.  If passing would require a large steering jump,
    # or the car is already offset toward the requested passing side, stop and
    # wait for the moving person to open a safer corridor.
    "HUMAN_YIELD_ENABLE": True,
    "HUMAN_YIELD_MIN_BOTTOM_RATIO": 0.58,
    "HUMAN_YIELD_BODY_OFFSET_RATIO": 0.14,
    "HUMAN_YIELD_MAX_REQUIRED_SHIFT_RATIO": 0.30,
    "HUMAN_YIELD_MIN_SIDE_SHARE": 0.22,
    "HUMAN_YIELD_HOLD_SECONDS": 0.35,

    # 障碍框越宽，额外绕行越大；实际偏置取 max(基础偏置, bbox_width * 该系数)。
    "AVOID_BOX_WIDTH_GAIN": 0.35,

    # Human uses a wider bbox-based clearance because the occupied area is dynamic.
    "HUMAN_BOX_WIDTH_GAIN": 0.65,

    # COLLECT_GOLD 向金币吸引的比例；调大更主动靠金币，调小更偏向继续巡线。
    "GOLD_BIAS_GAIN": 0.45,

    # 单帧金币吸引最大偏置，单位 px；限制金币目标线不会突然偏得太狠。
    "GOLD_MAX_BIAS": 75.0,

    # RECOVER_LINE 保持上一帧 final_track_error 的衰减时间，单位 s；调大丢线后更久保持原方向。
    "RECOVER_DECAY_SECONDS": 0.5,

    # 绘制偏移目标路径时，尽量让路径点留在 road_mask 边界内侧这么多像素。
    "TARGET_PATH_ROAD_MARGIN": 8.0,

    # 最终控制误差从紫色规划线的这个 y 位置读取；数值越小越偏向画面远处，转向会更提前。
    "CONTROL_LOOKAHEAD_Y": 300.0,

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

    # stone 分岔候选至少需要多少个采样行同时出现两个 segment，避免噪声误判为分岔。
    "STONE_BRANCH_MIN_SPLIT_ROWS": 4,

    # stone 是否在外圈候选路径上的横向走廊阈值，按画面宽度取比例。
    "STONE_PATH_HIT_LATERAL_RATIO": 0.08,

    # stone bbox 越宽，路径走廊越宽；用于近处大目标。
    "STONE_PATH_HIT_BOX_GAIN": 0.75,

    # stone 路径走廊最小像素宽度。
    "STONE_PATH_HIT_MIN_PX": 32.0,

    # Once a branch is selected, keep its identity while fork candidates remain visible.
    "BRANCH_LOCK_RELEASE_FRAMES": 12,
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
        # max_track_error is accepted only for old call-site compatibility.
        self.car_avoid_offset = float(car_avoid_offset if car_avoid_offset is not None else self.params["CAR_AVOID_OFFSET"])
        self.human_avoid_offset = float(human_avoid_offset if human_avoid_offset is not None else self.params["HUMAN_AVOID_OFFSET"])
        self.gold_gain = float(gold_gain if gold_gain is not None else self.params["GOLD_BIAS_GAIN"])
        self.gold_max_bias = float(gold_max_bias if gold_max_bias is not None else self.params["GOLD_MAX_BIAS"])
        self.recover_decay_seconds = float(
            recover_decay_seconds if recover_decay_seconds is not None else self.params["RECOVER_DECAY_SECONDS"]
        )
        self.avoid_box_width_gain = float(self.params["AVOID_BOX_WIDTH_GAIN"])
        self.human_box_width_gain = float(self.params.get("HUMAN_BOX_WIDTH_GAIN", 0.65))
        self.target_path_road_margin = float(self.params["TARGET_PATH_ROAD_MARGIN"])
        self.control_lookahead_y = float(self.params["CONTROL_LOOKAHEAD_Y"])
        self.path_near_anchor_y_ratio = float(self.params["PATH_NEAR_ANCHOR_Y_RATIO"])
        self.path_full_bias_y_ratio = float(self.params["PATH_FULL_BIAS_Y_RATIO"])
        self.path_near_bias_gain = float(self.params["PATH_NEAR_BIAS_GAIN"])
        self.target_path_max_points = int(self.params["TARGET_PATH_MAX_POINTS"])
        self.avoid_side_hold_seconds = float(self.params["AVOID_SIDE_HOLD_SECONDS"])
        self.avoid_side_switch_margin = float(self.params["AVOID_SIDE_SWITCH_MARGIN"])
        self.human_motion_min_vx = float(self.params["HUMAN_MOTION_MIN_VX"])
        self.human_motion_smooth_alpha = float(self.params["HUMAN_MOTION_SMOOTH_ALPHA"])
        self.human_motion_max_dt = float(self.params["HUMAN_MOTION_MAX_DT"])
        self.human_motion_max_match_dx_ratio = float(self.params["HUMAN_MOTION_MAX_MATCH_DX_RATIO"])
        self.human_motion_predict_seconds = float(self.params.get("HUMAN_MOTION_PREDICT_SECONDS", 0.45))
        self.human_yield_enable = bool(self.params.get("HUMAN_YIELD_ENABLE", True))
        self.human_yield_min_bottom_ratio = float(self.params.get("HUMAN_YIELD_MIN_BOTTOM_RATIO", 0.58))
        self.human_yield_body_offset_ratio = float(self.params.get("HUMAN_YIELD_BODY_OFFSET_RATIO", 0.14))
        self.human_yield_max_required_shift_ratio = float(
            self.params.get("HUMAN_YIELD_MAX_REQUIRED_SHIFT_RATIO", 0.30)
        )
        self.human_yield_min_side_share = float(self.params.get("HUMAN_YIELD_MIN_SIDE_SHARE", 0.22))
        self.human_yield_hold_seconds = float(self.params.get("HUMAN_YIELD_HOLD_SECONDS", 0.35))
        self.road_side_roi_min_y_ratio = float(self.params["ROAD_SIDE_ROI_MIN_Y_RATIO"])
        self.road_side_roi_top_box_gain = float(self.params["ROAD_SIDE_ROI_TOP_BOX_GAIN"])
        self.road_side_roi_bottom_box_gain = float(self.params["ROAD_SIDE_ROI_BOTTOM_BOX_GAIN"])
        self.road_side_balance_min_delta = float(self.params["ROAD_SIDE_BALANCE_MIN_DELTA"])
        self.road_side_balance_ratio = float(self.params["ROAD_SIDE_BALANCE_RATIO"])
        self.stone_branch_min_split_rows = int(self.params["STONE_BRANCH_MIN_SPLIT_ROWS"])
        self.stone_path_hit_lateral_ratio = float(self.params["STONE_PATH_HIT_LATERAL_RATIO"])
        self.stone_path_hit_box_gain = float(self.params["STONE_PATH_HIT_BOX_GAIN"])
        self.stone_path_hit_min_px = float(self.params["STONE_PATH_HIT_MIN_PX"])
        self.branch_lock_release_frames = int(self.params["BRANCH_LOCK_RELEASE_FRAMES"])
        self.last_final_error = 0.0
        self.last_avoid_side = None
        self.last_avoid_category = None
        self.last_avoid_side_ts = None
        self.last_human_yield_ts = None
        self.last_human_motion_x = None
        self.last_human_motion_ts = None
        self.last_human_motion_vx = 0.0
        self.last_valid_ts = 0.0
        self.locked_branch = None
        self.branch_lock_missing_frames = 0

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
        stop_state = state in (
            TaskState.LINE_LOSS_SAFE_STOP.value,
            TaskState.TRAFFIC_LIGHT_STOP.value,
            TaskState.ENDSIGN_STOP.value,
        )

        path_offset = 0.0
        raw_final_error = None
        final_error = base_error
        reason = "track_center"
        avoid_side = None
        avoid_motion = None
        speed_override = None
        yield_wait = False
        branch_candidates = self._build_stone_branch_candidates(
            segmentation, frame_w, frame_h, center_x, base_error
        )
        selected_branch = self.locked_branch
        self._update_branch_lock_visibility(branch_candidates)

        if stop_state:
            final_error = None
            raw_final_error = None
            reason = "safe_stop"
            if state == TaskState.TRAFFIC_LIGHT_STOP.value:
                final_error = 0.0
                raw_final_error = 0.0
                reason = "traffic_light_stop"
            elif state == TaskState.ENDSIGN_STOP.value:
                final_error = 0.0
                raw_final_error = 0.0
                reason = "endsign_stop"
        elif state == TaskState.RECOVER_LINE.value or not line_valid:
            final_error = self._recover_error(now)
            raw_final_error = final_error
            reason = "hold_last_line"
        elif state == TaskState.AVOID_HUMAN.value:
            target = intent.get("target") or {}
            avoid_side, avoid_motion = self._choose_human_avoid_side(
                segmentation,
                target,
                frame_w,
                frame_h,
                now=now,
                held=bool(intent.get("held")),
            )
            avoid_offset = self._avoid_offset(target, self.human_avoid_offset, width_gain=self.human_box_width_gain)
            yield_wait, yield_reason = self._should_yield_for_human(
                segmentation,
                target,
                frame_w,
                frame_h,
                center_x,
                base_error,
                avoid_side,
                avoid_offset,
                avoid_motion,
                now,
            )
            if yield_wait:
                speed_override = 0.0
                final_error = 0.0
                raw_final_error = 0.0
                path_offset = 0.0
                self.last_human_yield_ts = float(now)
                if avoid_motion is None:
                    avoid_motion = {}
                avoid_motion["yield_wait"] = True
                avoid_motion["yield_reason"] = yield_reason
                reason = f"yield_human_{yield_reason}"
            else:
                path_offset = self._side_sign(avoid_side) * avoid_offset
                motion_source = (avoid_motion or {}).get("source") or "static"
                reason = f"avoid_human_{avoid_side}_{motion_source}"
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
        elif state == TaskState.AVOID_STONE.value:
            reason = "stone_branch_select"

        raw_target_path = []
        if line_valid and final_error is not None and not stop_state and not yield_wait:
            if state == TaskState.AVOID_STONE.value:
                target = intent.get("target") or {}
                branch_plan = self._select_stone_branch(branch_candidates, target, frame_w, frame_h)
                if branch_plan is not None:
                    raw_target_path = list(branch_plan["path"])
                    selected_branch = branch_plan["branch"]
                    self.locked_branch = selected_branch
                    self.branch_lock_missing_frames = 0
                    reason = branch_plan["reason"]
                    raw_final_error = self._path_error_at_lookahead(raw_target_path, center_x, frame_h)
                if not raw_target_path:
                    raw_target_path = self._build_target_path(segmentation, 0.0, frame_w, frame_h)
                    reason = "stone_branch_fallback"
                    if raw_target_path:
                        raw_final_error = self._path_error_at_lookahead(raw_target_path, center_x, frame_h)
            elif self.locked_branch:
                locked = self._candidate_for_branch(branch_candidates, self.locked_branch)
                if locked is not None:
                    raw_target_path = list(locked.get("path") or [])
                    selected_branch = self.locked_branch
                    reason = f"branch_locked_{self.locked_branch}"
                    raw_final_error = self._path_error_at_lookahead(raw_target_path, center_x, frame_h)
                if not raw_target_path:
                    raw_target_path = self._build_target_path(segmentation, path_offset, frame_w, frame_h)
                    raw_final_error = self._path_error_at_lookahead(raw_target_path, center_x, frame_h)
                    reason = f"branch_locked_{self.locked_branch}_single_path"
            else:
                raw_target_path = self._build_target_path(segmentation, path_offset, frame_w, frame_h)
                raw_final_error = self._path_error_at_lookahead(raw_target_path, center_x, frame_h)

        if raw_final_error is not None and math.isfinite(float(raw_final_error)):
            final_error = float(raw_final_error)
        else:
            final_error = None

        if final_error is not None and line_valid and not stop_state:
            self.last_final_error = final_error
            self.last_valid_ts = now

        effective_path_offset = path_offset
        target_path = raw_target_path
        if stop_state or yield_wait:
            target_path = []
        elif line_valid and final_error is not None and state == TaskState.AVOID_STONE.value:
            target_path = raw_target_path
        elif line_valid and final_error is not None and state != TaskState.NORMAL_TRACK.value:
            target_path = raw_target_path
        elif line_valid and final_error is not None and not target_path:
            target_path = self._build_target_path(segmentation, 0.0, frame_w, frame_h)

        target_x = None
        if final_error is not None:
            target_x = center_x + final_error

        return {
            "final_track_error": float(final_error) if final_error is not None else None,
            "raw_final_track_error": float(raw_final_error) if raw_final_error is not None else None,
            "base_track_error": float(base_error) if base_error is not None else None,
            "target_x": float(target_x) if target_x is not None else None,
            "target_path": target_path,
            "center_x": float(center_x),
            "control_lookahead_y": float(self._control_lookahead_y(frame_h)),
            "desired_path_offset": float(path_offset),
            "effective_path_offset": float(effective_path_offset),
            "track_error_limit": None,
            "avoid_side": avoid_side,
            "avoid_motion": dict(avoid_motion or {}),
            "speed_override": speed_override,
            "yield_wait": bool(yield_wait),
            "branch_candidates": branch_candidates,
            "selected_branch": selected_branch,
            "planner_reason": reason,
            "line_valid": line_valid,
        }

    def _recover_error(self, now):
        if self.last_valid_ts <= 0.0:
            return 0.0
        age = max(0.0, float(now) - self.last_valid_ts)
        decay = max(0.0, 1.0 - age / max(0.01, self.recover_decay_seconds))
        return self.last_final_error * decay

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

    def _control_lookahead_y(self, frame_h):
        return _clamp(self.control_lookahead_y, 0, max(0, int(frame_h) - 1))

    def _path_error_at_lookahead(self, path, center_x, frame_h):
        if not path:
            return None
        lookahead_y = int(self._control_lookahead_y(frame_h))
        idx = min(range(len(path)), key=lambda i: abs(path[i][1] - lookahead_y))
        return float(path[idx][0]) - float(center_x)

    def _build_stone_branch_candidates(self, segmentation, frame_w, frame_h, center_x, base_error):
        # Early/candidate states are visual warnings only. Steering may branch
        # only after the positive-Y classifier and strict geometry agree.
        if str(segmentation.get("fork_state") or "") != "FORK_CONFIRMED":
            return []

        direct_candidates = []
        for candidate in (segmentation.get("fork_candidates") or []):
            side = str(candidate.get("side") or "")
            if side not in ("left", "right"):
                continue
            raw_path = candidate.get("points") or candidate.get("path") or []
            path = []
            for point in raw_path:
                try:
                    x_raw, y_raw = point
                    x = int(round(_clamp(_finite_float(x_raw, center_x), 0, frame_w - 1)))
                    y = int(round(_clamp(_finite_float(y_raw, 0.0), 0, frame_h - 1)))
                except Exception:
                    continue
                path.append((x, y))
            if len(path) < 2:
                continue
            err = self._path_error_at_lookahead(path, center_x, frame_h)
            if err is None:
                continue
            direct_candidates.append({
                "side": side,
                "branch": "outer" if side == "left" else "inner",
                "path": path,
                "error": float(err),
                "split_rows": int(segmentation.get("fork_split_rows") or 0),
                "source": "segmentation_candidates",
            })

        if any(c.get("branch") == "outer" for c in direct_candidates) and any(
            c.get("branch") == "inner" for c in direct_candidates
        ):
            direct_candidates.sort(key=lambda item: 0 if item.get("branch") == "outer" else 1)
            return direct_candidates

        mid_points = segmentation.get("mid_points") or []
        road_mask = segmentation.get("road_mask")
        if len(mid_points) < 2 or road_mask is None or not hasattr(road_mask, "shape"):
            return []

        mask = np.asarray(road_mask)
        if mask.ndim != 2 or not np.any(mask):
            return []

        points = self._sample_mid_points(mid_points)
        left_path = []
        right_path = []
        split_rows = 0
        min_width = max(8, int(frame_w * 0.025))

        for point in points:
            try:
                base_x, y_raw = point
                y = int(_clamp(y_raw, 0, min(frame_h - 1, mask.shape[0] - 1)))
                base_x = _finite_float(base_x, center_x)
            except Exception:
                continue

            segments = self._row_segments(mask[y], min_width)
            if len(segments) >= 2:
                split_rows += 1
                left_path.append((int(round(segments[0]["center"])), y))
                right_path.append((int(round(segments[-1]["center"])), y))
            else:
                left_path.append((int(round(base_x)), y))
                right_path.append((int(round(base_x)), y))

        if split_rows < max(1, self.stone_branch_min_split_rows):
            return []
        if len(left_path) < 2 or len(right_path) < 2:
            return []

        candidates = []
        for label, path in (("left", left_path), ("right", right_path)):
            err = self._path_error_at_lookahead(path, center_x, frame_h)
            if err is None:
                continue
            candidates.append({
                "side": label,
                "branch": "outer" if label == "left" else "inner",
                "path": path,
                "error": float(err),
                "split_rows": int(split_rows),
                "source": "road_mask_scan",
            })

        if len(candidates) < 2:
            return []

        return candidates

    @staticmethod
    def _row_segments(row, min_width):
        xs = np.flatnonzero(row)
        if xs.size < min_width:
            return []
        splits = np.where(np.diff(xs) > 1)[0] + 1
        result = []
        for seg in np.split(xs, splits):
            width = int(seg[-1] - seg[0] + 1)
            if width < min_width:
                continue
            result.append({
                "left": int(seg[0]),
                "right": int(seg[-1]),
                "center": (float(seg[0]) + float(seg[-1])) * 0.5,
                "width": width,
            })
        return result

    def _select_stone_branch(self, branch_candidates, target, frame_w, frame_h):
        if not branch_candidates:
            return None

        outer = None
        inner = None
        for candidate in branch_candidates:
            if candidate.get("branch") == "outer":
                outer = candidate
            elif candidate.get("branch") == "inner":
                inner = candidate

        if outer is None:
            outer = branch_candidates[0]
        if inner is None and len(branch_candidates) >= 2:
            inner = branch_candidates[1]

        if inner is not None and self._stone_hits_path(target, outer.get("path") or [], frame_w, frame_h):
            return {
                "branch": "inner",
                "path": inner.get("path") or [],
                "reason": "stone_on_outer_choose_inner",
            }

        return {
            "branch": "outer",
            "path": outer.get("path") or [],
            "reason": "stone_not_on_outer_keep_outer",
        }

    @staticmethod
    def _candidate_for_branch(branch_candidates, branch):
        for candidate in branch_candidates or []:
            if candidate.get("branch") == branch:
                return candidate
        return None

    def _update_branch_lock_visibility(self, branch_candidates):
        if not self.locked_branch:
            self.branch_lock_missing_frames = 0
            return
        if self._candidate_for_branch(branch_candidates, self.locked_branch) is not None:
            self.branch_lock_missing_frames = 0
            return
        self.branch_lock_missing_frames += 1
        if self.branch_lock_missing_frames >= max(1, self.branch_lock_release_frames):
            self.locked_branch = None
            self.branch_lock_missing_frames = 0

    def _stone_hits_path(self, target, path, frame_w, frame_h):
        if not target or not path:
            return False
        center = target.get("center") or [None, None]
        size = target.get("size") or [0.0, 0.0]
        stone_x = _finite_float(center[0])
        stone_y = _finite_float(center[1])
        if stone_x is None or stone_y is None:
            return False

        try:
            idx = min(range(len(path)), key=lambda i: abs(path[i][1] - stone_y))
            path_x = float(path[idx][0])
        except Exception:
            return False

        box_w = _finite_float(size[0], 0.0)
        threshold = max(
            frame_w * self.stone_path_hit_lateral_ratio,
            box_w * self.stone_path_hit_box_gain,
            self.stone_path_hit_min_px,
        )
        return abs(float(stone_x) - path_x) <= threshold

    def _avoid_offset(self, target, default_offset, width_gain=None):
        size = target.get("size") or [0.0, 0.0]
        box_w = _finite_float(size[0], 0.0)
        gain = self.avoid_box_width_gain if width_gain is None else float(width_gain)
        return max(float(default_offset), box_w * gain)

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

    def _choose_human_avoid_side(self, segmentation, target, frame_w, frame_h, now, held=False):
        center_x = _finite_float(segmentation.get("center_x"), frame_w * 0.5)
        target_center = target.get("center") or [center_x, frame_h * 0.5]
        target_x = _finite_float(target_center[0], center_x)
        if held and self.last_avoid_category == "human" and self.last_avoid_side in ("left", "right"):
            return self.last_avoid_side, {
                "reliable": False,
                "vx": float(self.last_human_motion_vx),
                "raw_vx": 0.0,
                "dt": 0.0,
                "preferred_side": self.last_avoid_side,
                "road_checked_side": self.last_avoid_side,
                "source": "held_side",
            }

        motion = self._update_human_motion(target, now, frame_w)
        static_side = "right" if target_x <= center_x else "left"
        motion_side = None
        if motion.get("reliable"):
            motion_side = "left" if motion.get("vx", 0.0) > 0.0 else "right"

        preferred, corridor = self._human_corridor_side(segmentation, target, frame_w, frame_h, static_side, motion)
        motion["preferred_side"] = motion_side or static_side
        motion["road_checked_side"] = preferred
        motion["corridor_left_score"] = float(corridor.get("left_score", 0.0))
        motion["corridor_right_score"] = float(corridor.get("right_score", 0.0))
        motion["predicted_shift_px"] = float(corridor.get("predicted_shift_px", 0.0))
        motion["source"] = corridor.get("source") or "corridor"
        side = self._apply_avoid_side_hysteresis("human", preferred, target_x, center_x, now)
        return side, motion

    def _human_corridor_side(self, segmentation, target, frame_w, frame_h, static_side, motion=None):
        motion = dict(motion or {})
        vx = _finite_float(motion.get("vx"), 0.0)
        motion_reliable = bool(motion.get("reliable"))
        motion_side = None
        if motion_reliable:
            motion_side = "left" if vx > 0.0 else "right"

        road_mask = segmentation.get("road_mask")
        if road_mask is None or not hasattr(road_mask, "shape"):
            return motion_side or static_side, {
                "left_score": 0.0,
                "right_score": 0.0,
                "predicted_shift_px": 0.0,
                "source": "no_road_mask_motion_tie" if motion_side else "no_road_mask_static",
            }

        mask = np.asarray(road_mask)
        if mask.ndim != 2 or not np.any(mask):
            return motion_side or static_side, {
                "left_score": 0.0,
                "right_score": 0.0,
                "predicted_shift_px": 0.0,
                "source": "empty_road_mask_motion_tie" if motion_side else "empty_road_mask_static",
            }

        h, w = mask.shape[:2]
        bbox = target.get("bbox") or [0.0, h * 0.35, w, h * 0.75]
        try:
            left, top, right, bottom = [float(v) for v in bbox]
        except Exception:
            left, top, right, bottom = 0.0, h * 0.35, float(w), h * 0.75

        size = target.get("size") or [max(0.0, right - left), max(0.0, bottom - top)]
        box_w = max(0.0, _finite_float(size[0], right - left))
        box_h = max(1.0, _finite_float(size[1], bottom - top))
        predicted_shift = vx * self.human_motion_predict_seconds if motion_reliable else 0.0
        predicted_left = left + predicted_shift
        predicted_right = right + predicted_shift
        occupied_left = min(left, predicted_left)
        occupied_right = max(right, predicted_right)
        clearance = max(16.0, frame_w * 0.04, box_w * 0.25)
        x_left = int(_clamp(occupied_left - clearance, 0, w - 1))
        x_right = int(_clamp(occupied_right + clearance, 0, w - 1))
        y0 = int(_clamp(bottom - box_h * self.road_side_roi_top_box_gain, h * self.road_side_roi_min_y_ratio, h - 1))
        y1 = int(_clamp(bottom + box_h * self.road_side_roi_bottom_box_gain, y0 + 1, h))
        roi = mask[y0:y1, :]
        left_score = float(np.count_nonzero(roi[:, :x_left]))
        right_score = float(np.count_nonzero(roi[:, x_right:]))
        total = max(1.0, left_score + right_score)
        min_delta = max(self.road_side_balance_min_delta, self.road_side_balance_ratio * total)

        if abs(left_score - right_score) >= min_delta:
            side = "left" if left_score > right_score else "right"
            source = "corridor_motion_prediction" if motion_reliable else "corridor_road"
        elif motion_side in ("left", "right"):
            side = motion_side
            source = "corridor_motion_tie"
        else:
            side = static_side
            source = "corridor_static_tie"

        return side, {
            "left_score": left_score,
            "right_score": right_score,
            "predicted_shift_px": float(predicted_shift),
            "source": source,
        }

    def _update_human_motion(self, target, now, frame_w):
        center = target.get("center") or [None, None]
        target_x = _finite_float(center[0])
        info = {
            "reliable": False,
            "vx": 0.0,
            "raw_vx": 0.0,
            "dt": 0.0,
        }
        if target_x is None:
            self.last_human_motion_x = None
            self.last_human_motion_ts = None
            self.last_human_motion_vx = 0.0
            return info

        now = float(now)
        last_x = self.last_human_motion_x
        last_ts = self.last_human_motion_ts
        self.last_human_motion_x = float(target_x)
        self.last_human_motion_ts = now

        if last_x is None or last_ts is None:
            self.last_human_motion_vx = 0.0
            return info

        dt = max(0.0, now - float(last_ts))
        info["dt"] = float(dt)
        if dt <= 1e-3 or dt > max(1e-3, self.human_motion_max_dt):
            self.last_human_motion_vx = 0.0
            return info

        dx = float(target_x) - float(last_x)
        if abs(dx) > max(1.0, float(frame_w) * self.human_motion_max_match_dx_ratio):
            self.last_human_motion_vx = 0.0
            return info

        raw_vx = dx / dt
        alpha = _clamp(self.human_motion_smooth_alpha, 0.0, 1.0)
        vx = self.last_human_motion_vx * (1.0 - alpha) + raw_vx * alpha
        self.last_human_motion_vx = vx
        info["raw_vx"] = float(raw_vx)
        info["vx"] = float(vx)
        info["reliable"] = abs(vx) >= max(1.0, self.human_motion_min_vx)
        return info

    def _should_yield_for_human(
        self,
        segmentation,
        target,
        frame_w,
        frame_h,
        center_x,
        base_error,
        avoid_side,
        avoid_offset,
        motion,
        now,
    ):
        if not self.human_yield_enable or avoid_side not in ("left", "right"):
            return False, "disabled"

        if (
            self.last_human_yield_ts is not None
            and float(now) - float(self.last_human_yield_ts) <= self.human_yield_hold_seconds
        ):
            return True, "hold"

        bbox = target.get("bbox") or [0.0, frame_h * 0.35, frame_w, frame_h * 0.75]
        bottom = _finite_float(bbox[3], frame_h * 0.5)
        bottom_ratio = float(bottom) / float(max(1, frame_h))
        if bottom_ratio < self.human_yield_min_bottom_ratio:
            return False, "far"

        side_sign = self._side_sign(avoid_side)
        current_error = _finite_float(base_error, 0.0)
        body_offset_limit = max(1.0, float(frame_w) * self.human_yield_body_offset_ratio)
        if abs(current_error) >= body_offset_limit and current_error * side_sign > 0.0:
            return True, "body_offset"

        lookahead_y = self._control_lookahead_y(frame_h)
        lookahead_gain = self._path_bias_gain(lookahead_y, frame_h)
        planned_error = current_error + side_sign * float(avoid_offset) * lookahead_gain
        max_required_shift = max(1.0, float(frame_w) * self.human_yield_max_required_shift_ratio)
        if abs(planned_error) >= max_required_shift:
            return True, "large_shift"

        motion = dict(motion or {})
        left_score = _finite_float(motion.get("corridor_left_score"), 0.0)
        right_score = _finite_float(motion.get("corridor_right_score"), 0.0)
        total = max(0.0, left_score + right_score)
        if total > 0.0:
            selected_score = left_score if avoid_side == "left" else right_score
            if selected_score / total < _clamp(self.human_yield_min_side_share, 0.0, 1.0):
                return True, "narrow_corridor"

        return False, "pass"

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
        age = max(0.0, float(now) - self.last_avoid_side_ts) if self.last_avoid_side_ts is not None else None
        in_hold = same_category and previous in ("left", "right") and age is not None and age <= self.avoid_side_hold_seconds

        side = preferred
        if in_hold and preferred != previous:
            lateral = abs(float(target_x) - float(center_x))
            if lateral < self.avoid_side_switch_margin:
                side = previous

        if not same_category or side != previous:
            self.last_avoid_side_ts = float(now)
        self.last_avoid_side = side
        self.last_avoid_category = category
        return side

    def _road_free_side(self, segmentation, target, default_side):
        scores = self._road_side_scores(segmentation, target)
        if scores is None:
            return default_side
        left_score, right_score = scores
        if left_score <= 0.0 and right_score <= 0.0:
            return default_side
        if abs(left_score - right_score) < max(
            self.road_side_balance_min_delta,
            self.road_side_balance_ratio * (left_score + right_score),
        ):
            return default_side
        return "left" if left_score > right_score else "right"

    def _road_side_scores(self, segmentation, target):
        road_mask = segmentation.get("road_mask")
        if road_mask is None or not hasattr(road_mask, "shape"):
            return None
        mask = np.asarray(road_mask)
        if mask.ndim != 2 or not np.any(mask):
            return None

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
        return left_score, right_score

    @staticmethod
    def _side_sign(side):
        return -1.0 if side == "left" else 1.0

    def draw_debug(self, frame, task_decision, plan_result, draw_text=True):
        if frame is None or plan_result is None:
            return frame
        out = frame
        h, w = out.shape[:2]
        target_x = _finite_float(plan_result.get("target_x"))
        branch_candidates = list(plan_result.get("branch_candidates") or [])
        selected_branch = plan_result.get("selected_branch")
        for candidate in branch_candidates:
            path = list(candidate.get("path") or [])
            if len(path) < 2:
                continue
            color = (160, 160, 160)
            thickness = 1
            if candidate.get("branch") == selected_branch:
                color = (255, 0, 255)
                thickness = 2
            pts = np.array(path, np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], False, color, thickness)

        target_path = list(plan_result.get("target_path") or [])
        if len(target_path) >= 2:
            pts = np.array(target_path, np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], False, (255, 0, 255), 3)
            lookahead_y = int(self._control_lookahead_y(h))
            cv2.line(out, (0, lookahead_y), (w - 1, lookahead_y), (255, 0, 255), 1)
            idx = min(range(len(target_path)), key=lambda i: abs(target_path[i][1] - lookahead_y))
            cv2.circle(out, target_path[idx], 8, (255, 0, 255), -1)
            cv2.circle(out, target_path[0], 5, (255, 0, 255), -1)
        elif target_x is not None:
            x = int(_clamp(target_x, 0, w - 1))
            cv2.circle(out, (x, int(h * 0.68)), 8, (255, 0, 255), -1)

        if draw_text:
            state = str((task_decision or {}).get("task_state") or "N/A")
            err = plan_result.get("final_track_error")
            err_text = "N/A" if err is None else f"{float(err):.1f}"
            cv2.putText(out, f"Task: {state}", (10, 172), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)
            cv2.putText(out, f"Final err: {err_text}", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)
            if selected_branch:
                cv2.putText(out, f"Branch: {selected_branch}", (10, 228), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)
        return out
