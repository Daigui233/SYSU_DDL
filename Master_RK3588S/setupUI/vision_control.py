import math
import os
import time
from dataclasses import dataclass

import cv2
import numpy as np


STATE_TRACK = 1
STATE_AVOID_CAR = 2
STATE_AVOID_HUMAN = 3
STATE_COLLECT_GOLD = 4
STATE_RECOVER_LINE = 5
STATE_LINE_LOSS_SAFE_STOP = 6
STATE_SAFE_STOP = 7
CONTROL_FLAG_USE_TARGET_SPEED = 0x01

ROUTE_NONE = "NONE"
ROUTE_SINGLE = "SINGLE"
ROUTE_MULTI_FORK = "MULTI_FORK"
ROUTE_AMBIGUOUS = "AMBIGUOUS"

_PAIR_FRACTIONS = np.arange(24, dtype=np.float32) / 23.0


def _env_float(name, default):
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return float(default)


def _env_int(name, default):
    try:
        return int(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return int(default)


def _env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _finite_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


def _soft_limit(value, limit, margin):
    """Preserve small values and smoothly compress only the excess."""
    value = float(value)
    limit = max(0.0, float(limit))
    margin = max(0.0, float(margin))
    magnitude = abs(value)
    if magnitude <= limit:
        return value
    if margin <= 1e-6:
        return math.copysign(limit, value)
    compressed = (
        limit + margin *
        (1.0 - math.exp(-(magnitude - limit) / margin)))
    return float(math.copysign(compressed, value))


def _interp_path_x(points, target_y):
    if points is None or len(points) == 0:
        return None
    points = np.asarray(points, dtype=np.float32)
    order = np.argsort(points[:, 1])
    ys = points[order, 1]
    xs = points[order, 0]
    target_y = float(target_y)
    if target_y <= float(ys[0]):
        return float(xs[0])
    if target_y >= float(ys[-1]):
        return float(xs[-1])
    return float(np.interp(target_y, ys, xs))


def _interp_path_x_many(points, target_ys):
    points = np.asarray(points, dtype=np.float32)
    target_ys = np.asarray(target_ys, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        return np.full(target_ys.shape, np.nan, dtype=np.float32)
    order = np.argsort(points[:, 1])
    ys = points[order, 1]
    xs = points[order, 0]
    ys, unique_indices = np.unique(ys, return_index=True)
    xs = xs[unique_indices]
    if len(ys) == 1:
        return np.full(target_ys.shape, xs[0], dtype=np.float32)
    return np.interp(target_ys, ys, xs).astype(np.float32)


@dataclass
class VisionControlConfig:
    visual_center_x: float = 0.50
    lookahead_y_ratio: float = 0.625
    max_track_error_640: float = 160.0
    max_error_step_640: float = 24.0
    track_deadband_px_640: float = 0.0
    track_small_error_px_640: float = 32.0
    track_small_error_gain: float = 0.20
    track_fast_error_px_640: float = 64.0
    track_fast_step_scale: float = 2.0
    track_reverse_step_scale: float = 0.50
    curve_track_step_scale: float = 1.50
    curve_state_enter_curvature_640: float = 8.0
    curve_state_exit_curvature_640: float = 4.0
    curve_state_enter_frames: int = 2
    curve_state_exit_frames: int = 5
    curve_evidence_stable_frames: int = 2
    curve_evidence_max_delta_640: float = 20.0
    path_heading_feedforward_gain: float = 0.35
    path_heading_feedforward_max_px_640: float = 80.0
    path_heading_deadband_px_640: float = 6.0
    curve_feedforward_gain: float = 0.85
    curve_feedforward_max_px_640: float = 36.0
    # Track-specific steering feedforward: straight or right 3 m circle.
    right_circle_feedforward_640: float = 66.5
    right_circle_trim_max_640: float = 10.0
    right_circle_capture_range_640: float = 24.0
    right_circle_compensation_gain: float = 0.30
    straight_error_limit_ratio: float = 0.50
    curve_task_adjust_limit_ratio: float = 0.50
    curve_task_adjust_soft_margin_ratio: float = 0.25
    straight_min_span_ratio: float = 0.50
    straight_path_max_residual_640: float = 6.0
    straight_edge_max_residual_640: float = 10.0
    normal_speed_mps: float = 0.06
    recover_speed_mps: float = 0.04
    human_pass_speed_mps: float = 0.30
    collect_speed_mps: float = 0.06
    turnsign_slow_speed_mps: float = 0.08
    route_confirm_frames: int = 6
    branch_separation_px_640: float = 70.0
    branch_separation_rows: int = 8
    overlap_px_640: float = 28.0
    ocr_lock_lifetime_s: float = 10.0
    ocr_confirm_frames: int = 1
    curve_merge_support_ratio: float = 0.70
    curve_merge_near_px_640: float = 36.0
    curve_merge_enter_evidence: int = 4
    curve_merge_release_frames: int = 30
    recover_hold_s: float = 0.5
    line_anchor_y_ratio: float = 0.78
    line_anchor_max_offset_px_640: float = 200.0
    hazard_bottom_ratio: float = 0.58
    hazard_lateral_ratio: float = 0.18
    coin_bottom_ratio: float = 0.55
    coin_lateral_ratio: float = 0.24
    # Measured from the live 640x480 AR Preview: the desired stop-size
    # TurnSign box is about 140x69 px (9660 px^2, or 3.14% of the frame).
    sign_stop_area_ratio: float = 0.031
    sign_slow_min_score: float = 0.20
    sign_latch_frames: int = 3
    turnsign_steer_gain: float = 1.5
    turnsign_reverse_speed_mps: float = -0.08
    turnsign_reverse_duration_s: float = 0.5
    turnsign_trim_far_y_ratio: float = 0.35
    # Keep OCR parking micro-adjustment disabled until it is revalidated.
    # OCR route selection remains enabled independently of this switch.
    turnsign_trim_enabled: bool = False
    turnsign_trim_sample_rows: int = 6
    turnsign_trim_low_separation_px_640: float = 175.0
    turnsign_trim_high_separation_px_640: float = 220.0
    turnsign_trim_settle_s: float = 0.8
    turnsign_trim_stable_frames: int = 3
    turnsign_trim_center_deadband_px_640: float = 24.0
    turnsign_trim_split_px_640: float = 36.0
    turnsign_trim_split_frames: int = 3
    turnsign_trim_collapse_frames: int = 2
    turnsign_trim_drop_px_640: float = 45.0
    turnsign_trim_steer_deadband_px_640: float = 4.0
    turnsign_trim_min_steer_px_640: float = 44.0
    turnsign_trim_steer_gain: float = 0.75
    turnsign_trim_max_steer_px_640: float = 100.0
    human_stop_line_margin_ratio: float = 0.0
    # Larger values move the human stop line upward, so braking starts sooner.
    human_stop_progress_ratio: float = 0.84
    human_preline_missing_px_480: float = 20.0
    human_pass_offset_px_640: float = 38.0
    human_speed_hold_s: float = 0.5
    human_path_return_guard_s: float = 0.4
    human_absence_confirm_s: float = 1.5
    human_stop_reverse_speed_mps: float = -0.05
    human_stop_reverse_duration_s: float = 0.3
    car_avoid_offset_px_640: float = 55.0
    car_avoid_steer_gain: float = 0.70
    car_avoid_max_offset_px_640: float = 44.0
    car_avoid_hold_s: float = 2.0
    car_human_pass_speed_mps: float = 0.30
    car_human_pass_hold_s: float = 2.0
    human_avoid_offset_px_640: float = 75.0
    avoid_box_width_gain: float = 0.35
    gold_bias_gain: float = 0.45
    gold_max_bias_px_640: float = 75.0
    min_human_score: float = 0.35
    min_car_score: float = 0.35
    min_coin_score: float = 0.35

    @classmethod
    def from_env(cls):
        return cls(
            visual_center_x=_clamp(_env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8),
            lookahead_y_ratio=_clamp(_env_float("VISION_CONTROL_LOOKAHEAD_Y_RATIO", 0.625), 0.25, 0.95),
            max_track_error_640=max(1.0, _env_float("VISION_CONTROL_MAX_ERROR_640", 160.0)),
            max_error_step_640=max(1.0, _env_float("VISION_CONTROL_MAX_STEP_640", 24.0)),
            track_deadband_px_640=max(
                0.0, _env_float("VISION_CONTROL_TRACK_DEADBAND_640", 0.0)),
            track_small_error_px_640=max(
                1.0, _env_float("VISION_CONTROL_TRACK_SMALL_ERROR_640", 32.0)),
            track_small_error_gain=_clamp(
                _env_float("VISION_CONTROL_TRACK_SMALL_GAIN", 0.20), 0.0, 1.0),
            track_fast_error_px_640=max(
                1.0, _env_float("VISION_CONTROL_TRACK_FAST_ERROR_640", 64.0)),
            track_fast_step_scale=max(
                1.0, _env_float("VISION_CONTROL_TRACK_FAST_STEP_SCALE", 2.0)),
            track_reverse_step_scale=_clamp(
                _env_float("VISION_CONTROL_TRACK_REVERSE_STEP_SCALE", 0.50),
                0.1, 1.0),
            curve_track_step_scale=max(
                1.0, _env_float(
                    "VISION_CONTROL_CURVE_TRACK_STEP_SCALE", 1.50)),
            curve_state_enter_curvature_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_CURVE_STATE_ENTER_640", 8.0)),
            curve_state_exit_curvature_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_CURVE_STATE_EXIT_640", 4.0)),
            curve_state_enter_frames=max(
                1, _env_int(
                    "VISION_CONTROL_CURVE_STATE_ENTER_FRAMES", 2)),
            curve_state_exit_frames=max(
                1, _env_int(
                    "VISION_CONTROL_CURVE_STATE_EXIT_FRAMES", 5)),
            curve_evidence_stable_frames=max(
                1, _env_int(
                    "VISION_CONTROL_CURVE_EVIDENCE_STABLE_FRAMES", 2)),
            curve_evidence_max_delta_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_CURVE_EVIDENCE_MAX_DELTA_640", 20.0)),
            path_heading_feedforward_gain=max(
                0.0, _env_float(
                    "VISION_CONTROL_PATH_HEADING_FF_GAIN", 0.35)),
            path_heading_feedforward_max_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_PATH_HEADING_FF_MAX_640", 80.0)),
            path_heading_deadband_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_PATH_HEADING_DEADBAND_640", 6.0)),
            curve_feedforward_gain=_clamp(
                _env_float("VISION_CONTROL_CURVE_FF_GAIN", 0.85),
                0.0, 1.0),
            curve_feedforward_max_px_640=max(
                0.0, _env_float("VISION_CONTROL_CURVE_FF_MAX_640", 36.0)),
            right_circle_feedforward_640=_clamp(
                _env_float("VISION_CONTROL_RIGHT_CIRCLE_FF_640", 66.5),
                -160.0, 160.0),
            right_circle_trim_max_640=max(
                0.0, _env_float("VISION_CONTROL_RIGHT_CIRCLE_TRIM_MAX_640", 10.0)),
            right_circle_capture_range_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_RIGHT_CIRCLE_CAPTURE_RANGE_640", 24.0)),
            right_circle_compensation_gain=_clamp(
                _env_float(
                    "VISION_CONTROL_RIGHT_CIRCLE_COMPENSATION_GAIN", 0.30),
                0.0, 1.0),
            straight_error_limit_ratio=_clamp(
                _env_float(
                    "VISION_CONTROL_STRAIGHT_ERROR_LIMIT_RATIO", 0.50),
                0.0, 1.0),
            curve_task_adjust_limit_ratio=_clamp(
                _env_float(
                    "VISION_CONTROL_CURVE_TASK_ADJUST_LIMIT_RATIO", 0.50),
                0.0, 1.0),
            curve_task_adjust_soft_margin_ratio=_clamp(
                _env_float(
                    "VISION_CONTROL_CURVE_TASK_ADJUST_SOFT_MARGIN_RATIO",
                    0.25),
                0.0, 1.0),
            straight_min_span_ratio=_clamp(
                _env_float(
                    "VISION_CONTROL_STRAIGHT_MIN_SPAN_RATIO", 0.50),
                0.10, 0.95),
            straight_path_max_residual_640=max(
                0.5, _env_float(
                    "VISION_CONTROL_STRAIGHT_PATH_MAX_RESIDUAL_640", 6.0)),
            straight_edge_max_residual_640=max(
                0.5, _env_float(
                    "VISION_CONTROL_STRAIGHT_EDGE_MAX_RESIDUAL_640", 10.0)),
            normal_speed_mps=max(0.0, _env_float("VISION_CONTROL_NORMAL_SPEED", 0.06)),
            recover_speed_mps=max(0.031, _env_float("VISION_CONTROL_RECOVER_SPEED", 0.04)),
            human_pass_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_SPEED", 0.30)),
            collect_speed_mps=max(0.0, _env_float("VISION_CONTROL_COLLECT_SPEED", 0.06)),
            turnsign_slow_speed_mps=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_SLOW_SPEED", 0.08)),
            route_confirm_frames=max(1, _env_int("VISION_CONTROL_ROUTE_CONFIRM_FRAMES", 6)),
            branch_separation_px_640=max(1.0, _env_float("VISION_CONTROL_BRANCH_SEP_640", 70.0)),
            branch_separation_rows=max(1, _env_int("VISION_CONTROL_BRANCH_SEP_ROWS", 8)),
            overlap_px_640=max(1.0, _env_float("VISION_CONTROL_OVERLAP_640", 28.0)),
            ocr_lock_lifetime_s=max(
                0.1, _env_float("VISION_CONTROL_OCR_LOCK_LIFETIME_S", 10.0)),
            ocr_confirm_frames=max(1, _env_int("VISION_CONTROL_OCR_CONFIRM_FRAMES", 1)),
            curve_merge_support_ratio=_clamp(
                _env_float("VISION_CONTROL_CURVE_MERGE_SUPPORT_RATIO", 0.70),
                0.25, 1.0),
            curve_merge_near_px_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_CURVE_MERGE_NEAR_640", 36.0)),
            curve_merge_enter_evidence=max(
                2, _env_int(
                    "VISION_CONTROL_CURVE_MERGE_ENTER_EVIDENCE", 4)),
            curve_merge_release_frames=max(
                1, _env_int(
                    "VISION_CONTROL_CURVE_MERGE_RELEASE_FRAMES", 30)),
            recover_hold_s=max(0.0, _env_float("VISION_CONTROL_RECOVER_HOLD_S", 0.5)),
            line_anchor_y_ratio=_clamp(
                _env_float("VISION_CONTROL_LINE_ANCHOR_Y_RATIO", 0.78),
                0.50, 0.98),
            line_anchor_max_offset_px_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_LINE_ANCHOR_MAX_OFFSET_640", 200.0)),
            hazard_bottom_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_BOTTOM_RATIO", 0.58), 0.0, 1.0),
            hazard_lateral_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_LATERAL_RATIO", 0.18), 0.01, 0.5),
            coin_bottom_ratio=_clamp(_env_float("VISION_CONTROL_COIN_BOTTOM_RATIO", 0.55), 0.0, 1.0),
            coin_lateral_ratio=_clamp(_env_float("VISION_CONTROL_COIN_LATERAL_RATIO", 0.24), 0.01, 0.5),
            sign_stop_area_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_AREA_RATIO", 0.031), 0.001, 1.0),
            sign_slow_min_score=_clamp(_env_float("VISION_CONTROL_SIGN_SLOW_MIN_SCORE", 0.20), 0.0, 1.0),
            sign_latch_frames=max(1, _env_int("VISION_CONTROL_SIGN_LATCH_FRAMES", 3)),
            turnsign_steer_gain=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_STEER_GAIN", 1.5)),
            turnsign_reverse_speed_mps=-abs(_env_float("VISION_CONTROL_TURNSIGN_REVERSE_SPEED", -0.08)),
            turnsign_reverse_duration_s=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_REVERSE_DURATION_S", 0.5)),
            turnsign_trim_far_y_ratio=_clamp(
                _env_float("VISION_CONTROL_TURNSIGN_TRIM_FAR_Y_RATIO", 0.35),
                0.05, 0.90),
            turnsign_trim_enabled=_env_bool(
                "VISION_CONTROL_TURNSIGN_TRIM_ENABLED", False),
            turnsign_trim_sample_rows=max(
                2, _env_int("VISION_CONTROL_TURNSIGN_TRIM_SAMPLE_ROWS", 6)),
            turnsign_trim_low_separation_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_LOW_SEPARATION_640", 175.0)),
            turnsign_trim_high_separation_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_HIGH_SEPARATION_640", 220.0)),
            turnsign_trim_settle_s=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_SETTLE_S", 0.8)),
            turnsign_trim_stable_frames=max(
                1, _env_int(
                    "VISION_CONTROL_TURNSIGN_TRIM_STABLE_FRAMES", 3)),
            turnsign_trim_center_deadband_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_CENTER_DEADBAND_640", 24.0)),
            turnsign_trim_split_px_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_SPLIT_640", 36.0)),
            turnsign_trim_split_frames=max(
                1, _env_int(
                    "VISION_CONTROL_TURNSIGN_TRIM_SPLIT_FRAMES", 3)),
            turnsign_trim_collapse_frames=max(
                1, _env_int(
                    "VISION_CONTROL_TURNSIGN_TRIM_COLLAPSE_FRAMES", 2)),
            turnsign_trim_drop_px_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_DROP_640", 45.0)),
            turnsign_trim_steer_deadband_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_STEER_DEADBAND_640", 4.0)),
            turnsign_trim_min_steer_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_MIN_STEER_640", 44.0)),
            turnsign_trim_steer_gain=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_STEER_GAIN", 0.75)),
            turnsign_trim_max_steer_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_TURNSIGN_TRIM_MAX_STEER_640", 100.0)),
            human_stop_line_margin_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_LINE_MARGIN_RATIO", 0.0), 0.0, 0.5),
            human_stop_progress_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_PROGRESS_RATIO", 0.84), 0.0, 1.0),
            human_preline_missing_px_480=max(0.0, _env_float("VISION_CONTROL_HUMAN_PRELINE_MISSING_PX_480", 20.0)),
            human_pass_offset_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_OFFSET_640", 38.0)),
            human_speed_hold_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED_HOLD_S", 0.5)),
            human_path_return_guard_s=max(0.0, _env_float(
                "VISION_CONTROL_HUMAN_PATH_RETURN_GUARD_S", 0.4)),
            human_absence_confirm_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_ABSENCE_CONFIRM_S", 1.5)),
            human_stop_reverse_speed_mps=-abs(_env_float(
                "VISION_CONTROL_HUMAN_STOP_REVERSE_SPEED", -0.05)),
            human_stop_reverse_duration_s=max(0.0, _env_float(
                "VISION_CONTROL_HUMAN_STOP_REVERSE_DURATION_S", 0.3)),
            car_avoid_offset_px_640=max(0.0, _env_float("VISION_CONTROL_CAR_AVOID_OFFSET_640", 55.0)),
            car_avoid_steer_gain=_clamp(
                _env_float("VISION_CONTROL_CAR_AVOID_STEER_GAIN", 0.70),
                0.0, 1.0),
            car_avoid_max_offset_px_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_CAR_AVOID_MAX_OFFSET_640", 44.0)),
            car_avoid_hold_s=max(0.0, _env_float("VISION_CONTROL_CAR_AVOID_HOLD_S", 2.0)),
            car_human_pass_speed_mps=max(0.0, _env_float("VISION_CONTROL_CAR_HUMAN_PASS_SPEED", 0.30)),
            car_human_pass_hold_s=max(0.0, _env_float("VISION_CONTROL_CAR_HUMAN_PASS_HOLD_S", 2.0)),
            human_avoid_offset_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_AVOID_OFFSET_640", 75.0)),
            avoid_box_width_gain=max(0.0, _env_float("VISION_CONTROL_AVOID_BOX_WIDTH_GAIN", 0.35)),
            gold_bias_gain=max(0.0, _env_float("VISION_CONTROL_GOLD_BIAS_GAIN", 0.45)),
            gold_max_bias_px_640=max(0.0, _env_float("VISION_CONTROL_GOLD_MAX_BIAS_640", 75.0)),
            min_human_score=_clamp(_env_float("VISION_CONTROL_HUMAN_MIN_SCORE", 0.35), 0.0, 1.0),
            min_car_score=_clamp(_env_float("VISION_CONTROL_CAR_MIN_SCORE", 0.35), 0.0, 1.0),
            min_coin_score=_clamp(_env_float("VISION_CONTROL_COIN_MIN_SCORE", 0.35), 0.0, 1.0),
        )


class VisionControlPlanner:
    """Convert pure visual perception into one TC264D command and debug packet."""

    def __init__(self, config=None):
        self.config = config or VisionControlConfig.from_env()
        self.fitted_control_tracker = _FittedControlPathTracker()
        self.last_path_target_x = None
        self.last_path_target_y = None
        self.last_path_target_slot = None
        self.last_path_target_ts = 0.0
        self.last_valid_ts = 0.0
        self.last_error = 0.0
        self.last_forward_speed_mps = float(self.config.normal_speed_mps)
        self.track_error_trend_sign = 0
        self.track_error_trend_frames = 0
        self.track_error_response = "initial"
        self.road_shape_state = "straight"
        self.road_shape_valid = False
        self.road_heading_delta_640 = 0.0
        self.road_curvature_640 = 0.0
        self.road_curve_enter_frames = 0
        self.road_curve_exit_frames = 0
        self.road_curve_path_states = {}
        self.road_shape_metrics = {}
        self.route_state = ROUTE_NONE
        self.route_reason = "initial"
        self.route_initialized = False
        self.pending_route_state = None
        self.pending_route_frames = 0
        self.branch_lock = "left"
        self.branch_lock_source = "default"
        self.selected_slot_lock = 0
        self.curve_merge_override = False
        self.curve_merge_bad_evidence = 0
        self.curve_merge_blue_stable_frames = 0
        self.curve_merge_reason = "inactive"
        self.curve_merge_metrics = {}
        self.selection_reason = "initial"
        self.last_valid_ocr_ts = 0.0
        self.ocr_pending_direction = None
        self.ocr_pending_frames = 0
        self.ocr_confirmed_current = False
        self.human_waiting_cross = False
        self.human_last_side = None
        self.human_pass_active = False
        self.human_pass_offset_x = 0.0
        self.human_speed_hold_until = 0.0
        self.human_path_return_guard_until = 0.0
        self.human_path_return_guard_start_x = None
        self.human_path_return_pending = False
        self.human_last_avoid_target_x = None
        self.human_detected_latched = False
        self.human_last_seen_ts = 0.0
        self.human_preline_last_gap_px_480 = None
        self.human_preline_wait_until = 0.0
        self.human_stop_reverse_until = 0.0
        self.human_stop_reverse_started = False
        self.car_avoid_side = 0
        self.car_avoid_offset_px_640 = 0.0
        self.car_avoid_hold_until = 0.0
        self.car_human_active = False
        self.car_human_waiting_cross = False
        self.car_human_seen_avoid_side = False
        self.car_human_last_seen_ts = 0.0
        self.car_human_pass_until = 0.0
        self.sign_seen_frames = 0
        self.sign_latched_since = None
        self.turnsign_control_session_id = None
        self.turnsign_new_session_pending = False
        self.turnsign_trim_pulse_until = 0.0
        self.turnsign_trim_direction = 0
        self.turnsign_trim_session_adaptive = False
        self.turnsign_trim_settle_until = 0.0
        self.turnsign_trim_separation_640 = None
        self.turnsign_trim_lookahead_separation_640 = None
        self.turnsign_trim_separation_samples_640 = []
        self.turnsign_last_seen_delta_640 = None
        self.turnsign_last_lost_delta_640 = None
        self.turnsign_trim_position_delta_640 = None
        self.turnsign_detection_was_fresh = False
        self.turnsign_trim_fresh_frames = 0
        self.turnsign_trim_current_fresh = False
        self.turnsign_trim_current_centered = False
        self.turnsign_trim_stop_ready = False
        self.turnsign_trim_line_split_frames = 0
        self.turnsign_trim_line_collapse_frames = 0
        self.turnsign_trim_line_ever_split = False
        self.turnsign_trim_line_split_ready = False
        self.turnsign_trim_line_clear_current = False
        self.turnsign_trim_overshoot_latched = False
        self.turnsign_trim_max_lookahead_separation_640 = 0.0
        self.turnsign_trim_line_history_ts = None
        self.turnsign_trim_pending_ocr_direction = None
        self.turnsign_trim_api_ready = False
        self.turnsign_trim_accept_ready = False

    def update(self, perception_result, ocr_response=None, now=None):
        now = float(time.monotonic() if now is None else now)
        start = time.perf_counter()
        perception_result = (
            perception_result if isinstance(perception_result, dict) else {})
        image_shape = self._image_shape(perception_result)
        candidates, search_ms = self._extract_candidates(
            perception_result, image_shape, now=now)
        self._update_turnsign_curve_separation(candidates, image_shape)

        response = ocr_response if isinstance(ocr_response, dict) else {}
        incoming_session_id = response.get("session_id")
        if (
            bool(response.get("session_active"))
            and incoming_session_id is not None
            and incoming_session_id != self.turnsign_control_session_id
        ):
            self.turnsign_control_session_id = incoming_session_id
            self._reset_turnsign_trim_runtime()
            self.turnsign_trim_session_adaptive = bool(
                self.config.turnsign_trim_enabled)
            self.turnsign_new_session_pending = bool(
                self.config.turnsign_trim_enabled)
        if (not self.config.turnsign_trim_enabled and
                (self.turnsign_trim_session_adaptive or
                 self.turnsign_trim_pulse_until > 0.0)):
            self._reset_turnsign_trim_runtime()
        if (
            self.config.turnsign_trim_enabled
            and self.turnsign_trim_session_adaptive
        ):
            self._update_turnsign_line_history(now)

        raw_route_state, raw_route_reason = self._classify_routes(
            candidates, image_shape)
        route_state, route_reason = self._stabilize_route_state(
            raw_route_state, raw_route_reason)
        raw_ocr_direction, raw_ocr_current = self._extract_ocr_direction(
            ocr_response)
        ocr_direction, ocr_current = self._confirm_ocr_direction(
            raw_ocr_direction, raw_ocr_current)
        ocr_lock_expired = False
        if ocr_current:
            self.last_valid_ocr_ts = now
            self.turnsign_trim_api_ready = True
            if (
                self.config.turnsign_trim_enabled
                and self.turnsign_trim_session_adaptive
                and not self.turnsign_trim_line_split_ready
            ):
                self.turnsign_trim_pending_ocr_direction = ocr_direction
            else:
                self._apply_ocr_direction_lock(ocr_direction)
        else:
            ocr_lock_expired = self._expire_ocr_lock(now)
        if (
            self.config.turnsign_trim_enabled
            and self.turnsign_trim_pending_ocr_direction in {"left", "right"}
            and self.turnsign_trim_line_split_ready
        ):
            self._apply_ocr_direction_lock(
                self.turnsign_trim_pending_ocr_direction)
            self.turnsign_trim_pending_ocr_direction = None

        self._update_curve_merge_continuity(candidates, image_shape)
        raw_selected = self._select_candidate(candidates)
        line_connected, line_connection_reason, line_connection_metrics = (
            self._path_connection_status(raw_selected, image_shape))
        line_loss_reason = None
        selected = raw_selected
        if raw_selected is not None and not line_connected:
            line_loss_reason = line_connection_reason
            selected = None
            self.selection_reason = "line_loss_{}".format(
                line_connection_reason)
        elif raw_selected is None and not candidates:
            line_loss_reason = "no_candidate"
        command, control_target = self._build_command(
            selected, route_reason, perception_result,
            image_shape, now, ocr_response,
            line_loss_reason=line_loss_reason)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        debug = {
            "enabled": True,
            "route_state": route_state,
            "route_reason": route_reason,
            "raw_route_state": raw_route_state,
            "raw_route_reason": raw_route_reason,
            "pending_route_state": self.pending_route_state,
            "pending_route_frames": self.pending_route_frames,
            "branch_lock": self.branch_lock,
            "branch_lock_source": self.branch_lock_source,
            "selected_slot_lock": self.selected_slot_lock,
            "selection_reason": self.selection_reason,
            "line_loss_active": line_loss_reason is not None,
            "line_loss_reason": line_loss_reason,
            "line_connected": bool(line_connected),
            "line_connection_reason": line_connection_reason,
            "line_connection_metrics": line_connection_metrics,
            "road_shape_state": self.road_shape_state,
            "road_shape_valid": bool(self.road_shape_valid),
            "road_heading_delta_640": float(
                self.road_heading_delta_640),
            "road_curvature_640": float(self.road_curvature_640),
            "road_curve_enter_frames": int(
                self.road_curve_enter_frames),
            "road_curve_exit_frames": int(
                self.road_curve_exit_frames),
            "road_curve_evidence": dict(self.road_shape_metrics),
            "raw_selected_slot": (
                None if raw_selected is None
                else int(raw_selected.get("slot", -1))),
            "curve_merge_override": bool(self.curve_merge_override),
            "curve_merge_reason": self.curve_merge_reason,
            "curve_merge_bad_evidence": self.curve_merge_bad_evidence,
            "curve_merge_blue_stable_frames": (
                self.curve_merge_blue_stable_frames),
            "curve_merge_metrics": dict(self.curve_merge_metrics),
            "raw_ocr_direction": (
                raw_ocr_direction if raw_ocr_current else None),
            "raw_ocr_current": bool(raw_ocr_current),
            "ocr_pending_direction": self.ocr_pending_direction,
            "ocr_pending_frames": self.ocr_pending_frames,
            "ocr_confirm_frames": self.config.ocr_confirm_frames,
            "ocr_direction": ocr_direction if ocr_current else None,
            "ocr_current": bool(ocr_current),
            "ocr_lock_expired": bool(ocr_lock_expired),
            "ocr_lock_age_s": self._ocr_lock_age(now),
            "ocr_lock_remaining_s": self._ocr_lock_remaining(now),
            "turnsign_trim_separation_640": self.turnsign_trim_separation_640,
            "turnsign_trim_enabled": bool(
                self.config.turnsign_trim_enabled),
            "turnsign_trim_lookahead_separation_640": (
                self.turnsign_trim_lookahead_separation_640),
            "turnsign_trim_separation_samples_640": list(
                self.turnsign_trim_separation_samples_640),
            "turnsign_trim_direction": int(self.turnsign_trim_direction),
            "turnsign_trim_pulse_remaining_s": max(
                0.0, float(self.turnsign_trim_pulse_until) - now),
            "turnsign_trim_settle_remaining_s": max(
                0.0, float(self.turnsign_trim_settle_until) - now),
            "turnsign_last_seen_delta_640": self.turnsign_last_seen_delta_640,
            "turnsign_last_lost_delta_640": self.turnsign_last_lost_delta_640,
            "turnsign_trim_position_delta_640": (
                self.turnsign_trim_position_delta_640),
            "turnsign_trim_fresh_frames": int(
                self.turnsign_trim_fresh_frames),
            "turnsign_trim_current_fresh": bool(
                self.turnsign_trim_current_fresh),
            "turnsign_trim_current_centered": bool(
                self.turnsign_trim_current_centered),
            "turnsign_trim_stop_ready": bool(
                self.turnsign_trim_stop_ready),
            "turnsign_trim_line_split_frames": int(
                self.turnsign_trim_line_split_frames),
            "turnsign_trim_line_collapse_frames": int(
                self.turnsign_trim_line_collapse_frames),
            "turnsign_trim_line_ever_split": bool(
                self.turnsign_trim_line_ever_split),
            "turnsign_trim_line_split_ready": bool(
                self.turnsign_trim_line_split_ready),
            "turnsign_trim_line_clear_current": bool(
                self.turnsign_trim_line_clear_current),
            "turnsign_trim_overshoot_latched": bool(
                self.turnsign_trim_overshoot_latched),
            "turnsign_trim_max_lookahead_separation_640": float(
                self.turnsign_trim_max_lookahead_separation_640),
            "turnsign_trim_pending_ocr_direction": (
                self.turnsign_trim_pending_ocr_direction),
            "turnsign_trim_api_ready": bool(self.turnsign_trim_api_ready),
            "turnsign_trim_accept_ready": bool(
                self.turnsign_trim_accept_ready),
            "selected_slot": (
                None if selected is None else int(selected.get("slot", -1))),
            "candidate_count": len(candidates),
            "detected_path_count": int(perception_result.get(
                "detected_path_count", 2 if len(candidates) >= 2 else 1)),
            "valid_path_count": len(candidates),
            "candidates": [
                self._summarize_candidate(item, image_shape)
                for item in candidates
            ],
            "candidate_paths": [
                self._debug_path_points(item) for item in candidates],
            "selected_path": (
                None if selected is None
                else self._debug_path_points(selected)),
            "control_target": control_target,
            "command": dict(command) if command else None,
            "timings_ms": {
                "path_search": float(search_ms),
                "control_total": float(elapsed_ms),
            },
        }
        perception_result["vision_control"] = debug
        return command, debug

    def _extract_candidates(self, result, image_shape, now=None):
        started = time.perf_counter()
        centerline = result.get("centerline") or {}
        road = (result.get("road") or {}).get("mask")
        if road is None:
            road = result.get("road_mask")
        raw_curve_paths = result.get("raw_curve_paths")
        if raw_curve_paths is None:
            raw_curve_paths = centerline.get("raw_curve_paths")
        candidates = self._build_fitted_control_paths(
            raw_curve_paths or [], road, image_shape, now=now)
        candidates.sort(key=lambda item: int(item.get("slot", 99)))
        self._publish_filtered_paths(result, candidates)
        elapsed = (time.perf_counter() - started) * 1000.0
        return candidates, elapsed

    def _build_fitted_control_paths(
            self, raw_paths, road_mask, image_shape, now=None):
        height, width = image_shape[:2]
        lookahead_y = float(height) * self.config.lookahead_y_ratio
        confidence_split_y = max(0.0, lookahead_y - 10.0)
        timestamp = time.monotonic() if now is None else float(now)
        fitted_paths = []
        present_slots = set()
        for raw_path in list(raw_paths)[:2]:
            slot = int(raw_path.get("slot", len(fitted_paths)))
            present_slots.add(slot)
            points = np.asarray(
                raw_path.get("points_xy"), dtype=np.float32)
            if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
                self.fitted_control_tracker.update(
                    slot, np.empty((0, 2)), np.empty((0,)),
                    image_shape, now=timestamp)
                continue
            points = points.copy()
            points[:, 0] = np.clip(points[:, 0], 0, width - 1)
            points[:, 1] = np.clip(points[:, 1], 0, height - 1)
            probabilities = np.asarray(
                raw_path.get("point_confidences", []), dtype=np.float32)
            if len(probabilities) != len(points):
                probabilities = np.full(
                    len(points), float(raw_path.get("score", 1.0)),
                    dtype=np.float32)
            inside_road = _semantic_road_point_mask(
                points, road_mask, image_shape)
            associated = _associated_point_mask(
                points, width, eligible_mask=inside_road)
            points, probabilities = _densify_associated_lower_points(
                points, probabilities, associated, width, height,
                lower_boundary_y=confidence_split_y)
            inside_road = _semantic_road_point_mask(
                points, road_mask, image_shape)
            points = points[inside_road]
            probabilities = probabilities[inside_road]
            points, probabilities, inliers = _fit_smooth_majority_curve(
                points, probabilities, width, extend_to_y=lookahead_y)
            points, probabilities = self.fitted_control_tracker.update(
                slot, points, probabilities, image_shape, now=timestamp)
            inside_road = _semantic_road_point_mask(
                points, road_mask, image_shape)
            points, probabilities = _select_control_curve_segment(
                points, probabilities, inside_road, lookahead_y)
            if len(points) < 3:
                continue
            fitted_paths.append({
                "slot": slot,
                "role": "left" if slot == 0 else "right",
                "identity": "left" if slot == 0 else "right",
                "source": "fitted_control_curve",
                "score": float(raw_path.get("score", 1.0)),
                "coverage": float(len(points)) / max(1.0, float(height)),
                "row_support": int(np.count_nonzero(inliers)),
                "point_confidences": probabilities.astype(np.float32),
                "points_xy": points.astype(np.float32),
                "spatial_prefiltered": True,
                "fitted_control": True,
            })
        for slot in (0, 1):
            if slot not in present_slots:
                self.fitted_control_tracker.update(
                    slot, np.empty((0, 2)), np.empty((0,)),
                    image_shape, now=timestamp)
        fitted_paths.sort(key=lambda item: int(item.get("slot", 99)))
        return fitted_paths[:2]

    def _publish_filtered_paths(self, result, candidates):
        # From this point on ``paths`` has one meaning only: the fitted curves
        # used for route selection and steering.
        result["paths"] = candidates
        result["path_count"] = len(candidates)
        # ARPreview lets the vision-control overlay draw these once with
        # probability-segmented identity colors.
        result["vision_control_path_overlay"] = True
        result["temporal"] = {
            "enabled": True,
            "status": (
                "fitted_curve_tracking" if candidates
                else "path_unavailable"),
            "mode": "adaptive_curve_tracking",
            "source": "fitted_control_curve",
        }
        centerline = result.get("centerline")
        if isinstance(centerline, dict):
            centerline["paths"] = candidates
            centerline["path_count"] = len(candidates)
            centerline["valid_path_count"] = len(candidates)
            centerline["temporal"] = result["temporal"]

    def _stabilize_route_state(self, raw_state, raw_reason):
        if not self.route_initialized:
            self.route_initialized = True
            self.route_state = raw_state
            self.route_reason = raw_reason
            return self.route_state, self.route_reason
        if raw_state == self.route_state:
            self.pending_route_state = None
            self.pending_route_frames = 0
            self.route_reason = raw_reason
            return self.route_state, self.route_reason
        if raw_state != self.pending_route_state:
            self.pending_route_state = raw_state
            self.pending_route_frames = 1
        else:
            self.pending_route_frames += 1
        if self.pending_route_frames >= self.config.route_confirm_frames:
            self.route_state = raw_state
            self.route_reason = raw_reason
            self.pending_route_state = None
            self.pending_route_frames = 0
            return self.route_state, self.route_reason
        return self.route_state, "hold_{}_pending_{}".format(
            self.route_state.lower(), raw_state.lower())

    def _classify_routes(self, candidates, image_shape):
        if not candidates:
            return ROUTE_NONE, "no_candidate"
        if len(candidates) == 1:
            return ROUTE_SINGLE, "one_candidate"
        first, second = candidates[:2]
        first_points = first.get("branch_points_xy", first["points_xy"])
        second_points = second.get("branch_points_xy", second["points_xy"])
        stats = self._path_pair_stats(
            first_points, second_points, image_shape)
        if stats["mean_distance_640"] <= self.config.overlap_px_640:
            return ROUTE_SINGLE, "overlap_dedup"
        if stats["separated_rows"] >= self.config.branch_separation_rows:
            return ROUTE_MULTI_FORK, "separated_candidates"
        return ROUTE_AMBIGUOUS, "weak_separation"

    def _update_curve_merge_continuity(self, candidates, image_shape):
        """Temporarily follow stable slot 1 when slot 0 collapses at a merge.

        Curve-head slot identity is normally authoritative.  In real merge
        frames, however, slot 0 alternates between a full route and a short
        piece of the shared trunk while slot 1 remains continuous.  Requiring
        both paths to meet near the vehicle keeps this exception out of
        unrelated missing-line and ordinary two-lane cases.
        """
        self.curve_merge_metrics = {}
        if self.branch_lock != "left":
            self.curve_merge_override = False
            self.curve_merge_bad_evidence = 0
            self.curve_merge_blue_stable_frames = 0
            self.curve_merge_reason = (
                "locked_right" if self.branch_lock == "right" else "inactive")
            return

        by_slot = {
            int(candidate.get("slot", -1)): candidate
            for candidate in candidates
        }
        blue = by_slot.get(0)
        green = by_slot.get(1)
        if blue is None or green is None:
            # A merge takeover is valid only while both candidates exist.
            # If green disappears during takeover, immediately return slot
            # ownership to blue instead of steering from stale green state.
            if not self.curve_merge_override:
                self.curve_merge_bad_evidence = max(
                    0, self.curve_merge_bad_evidence - 1)
            self.curve_merge_blue_stable_frames = 0
            self.curve_merge_reason = "waiting_for_both_paths"
            if self.curve_merge_override:
                self.curve_merge_override = False
                self.curve_merge_bad_evidence = 0
                self.selected_slot_lock = 0
                self.curve_merge_reason = "green_missing"
            return

        blue_points = np.asarray(blue.get("points_xy"), dtype=np.float32)
        green_points = np.asarray(green.get("points_xy"), dtype=np.float32)
        if not len(blue_points) or not len(green_points):
            if self.curve_merge_override:
                self.curve_merge_override = False
                self.curve_merge_bad_evidence = 0
                self.curve_merge_blue_stable_frames = 0
                self.selected_slot_lock = 0
                self.curve_merge_reason = "merge_path_empty"
            return
        height, width = image_shape[:2]
        lookahead_y = float(height) * self.config.lookahead_y_ratio
        near_y = float(height) * 0.875
        blue_span = float(np.ptp(blue_points[:, 1]))
        green_span = float(np.ptp(green_points[:, 1]))
        blue_covers_lookahead = self._path_covers_y(
            blue_points, lookahead_y)
        green_covers_lookahead = self._path_covers_y(
            green_points, lookahead_y)
        blue_covers_control_geometry = self._path_covers_control_geometry(
            blue_points, lookahead_y, height)
        green_covers_control_geometry = self._path_covers_control_geometry(
            green_points, lookahead_y, height)
        blue_covers_near = self._path_covers_y(blue_points, near_y)
        green_covers_near = self._path_covers_y(green_points, near_y)
        blue_near_x = _interp_path_x(blue_points, near_y)
        green_near_x = _interp_path_x(green_points, near_y)
        near_distance_640 = 1e9
        if blue_near_x is not None and green_near_x is not None:
            near_distance_640 = (
                abs(float(blue_near_x) - float(green_near_x)) * 640.0 /
                float(max(1, width)))
        shared_near = (
            blue_covers_near and green_covers_near and
            near_distance_640 <= self.config.curve_merge_near_px_640)
        green_good = green_covers_control_geometry and green_span > 0.0
        blue_bad = (
            not blue_covers_control_geometry or
            blue_span < (
                green_span * self.config.curve_merge_support_ratio))
        bad_merge_frame = shared_near and green_good and blue_bad
        blue_recovered = (
            blue_covers_control_geometry and
            blue_span >= green_span * max(
                0.85, self.config.curve_merge_support_ratio))
        self.curve_merge_metrics = {
            "shared_near": bool(shared_near),
            "near_distance_640": float(near_distance_640),
            "blue_span": blue_span,
            "green_span": green_span,
            "blue_covers_lookahead": bool(blue_covers_lookahead),
            "green_covers_lookahead": bool(green_covers_lookahead),
            "blue_covers_control_geometry": bool(
                blue_covers_control_geometry),
            "green_covers_control_geometry": bool(
                green_covers_control_geometry),
            "blue_bad": bool(blue_bad),
        }

        if self.curve_merge_override:
            if not green_good:
                self.curve_merge_override = False
                self.curve_merge_bad_evidence = 0
                self.curve_merge_blue_stable_frames = 0
                self.selected_slot_lock = 0
                self.curve_merge_reason = "green_support_lost"
                return
            if blue_recovered:
                self.curve_merge_blue_stable_frames += 1
            else:
                self.curve_merge_blue_stable_frames = 0
            if (self.curve_merge_blue_stable_frames >=
                    self.config.curve_merge_release_frames):
                self.curve_merge_override = False
                self.curve_merge_bad_evidence = 0
                self.curve_merge_blue_stable_frames = 0
                self.curve_merge_reason = "blue_recovered"
                self.selected_slot_lock = 0
            else:
                self.curve_merge_reason = "stable_green_takeover"
                self.selected_slot_lock = 1
            return

        if bad_merge_frame:
            # Bad frames add evidence faster than good frames remove it.  The
            # observed failure alternates full and truncated blue paths, so a
            # consecutive-frame counter would never latch reliably.
            self.curve_merge_bad_evidence += 2
            self.curve_merge_reason = "blue_support_unstable"
        else:
            self.curve_merge_bad_evidence = max(
                0, self.curve_merge_bad_evidence - 1)
            self.curve_merge_reason = "default_blue"
        if (self.curve_merge_bad_evidence >=
                self.config.curve_merge_enter_evidence):
            self.curve_merge_override = True
            self.curve_merge_blue_stable_frames = 0
            self.curve_merge_reason = "enter_green_takeover"
            self.selected_slot_lock = 1
        else:
            self.selected_slot_lock = 0

    @staticmethod
    def _path_covers_y(points, target_y):
        points = np.asarray(points, dtype=np.float32)
        return bool(
            points.ndim == 2 and points.shape[1] == 2 and len(points) and
            float(np.min(points[:, 1])) <= float(target_y) <=
            float(np.max(points[:, 1])))

    def _update_turnsign_curve_separation(self, candidates, image_shape):
        """Measure blue/green separation from lookahead toward image top."""
        self.turnsign_trim_separation_640 = None
        self.turnsign_trim_lookahead_separation_640 = None
        self.turnsign_trim_separation_samples_640 = []
        by_slot = {
            int(item.get("slot", -1)): item
            for item in candidates
            if int(item.get("slot", -1)) in {0, 1}
        }
        if 0 not in by_slot or 1 not in by_slot:
            return
        blue = np.asarray(by_slot[0].get("points_xy", ()), dtype=np.float32)
        green = np.asarray(by_slot[1].get("points_xy", ()), dtype=np.float32)
        height, width = image_shape[:2]
        lookahead_y = float(height) * self.config.lookahead_y_ratio
        far_y = min(
            lookahead_y,
            float(height) * self.config.turnsign_trim_far_y_ratio)
        rows = np.linspace(
            far_y, lookahead_y,
            self.config.turnsign_trim_sample_rows, dtype=np.float32)
        scale = 640.0 / float(max(1, width))
        samples = []
        for row_y in rows:
            row_y = float(row_y)
            if not (
                self._path_covers_y(blue, row_y)
                and self._path_covers_y(green, row_y)
            ):
                continue
            blue_x = _interp_path_x(blue, row_y)
            green_x = _interp_path_x(green, row_y)
            if blue_x is None or green_x is None:
                continue
            samples.append(abs(float(green_x) - float(blue_x)) * scale)
        self.turnsign_trim_separation_samples_640 = [
            float(value) for value in samples]
        if samples:
            # A high quantile sees a fork visible toward the image top without
            # allowing one noisy row to own the longitudinal decision.
            ordered = sorted(samples)
            index = int(math.ceil(0.75 * float(len(ordered) - 1)))
            self.turnsign_trim_separation_640 = float(ordered[index])
        if (
            self._path_covers_y(blue, lookahead_y)
            and self._path_covers_y(green, lookahead_y)
        ):
            blue_x = _interp_path_x(blue, lookahead_y)
            green_x = _interp_path_x(green, lookahead_y)
            if blue_x is not None and green_x is not None:
                self.turnsign_trim_lookahead_separation_640 = (
                    abs(float(green_x) - float(blue_x)) * scale)

    def _update_turnsign_line_history(self, now):
        """Remember split-then-collapse geometry for one TurnSign session."""
        now = float(now)
        if self.turnsign_trim_line_history_ts == now:
            return
        self.turnsign_trim_line_history_ts = now
        separation = _finite_float(
            self.turnsign_trim_lookahead_separation_640)
        clear_now = bool(
            separation is not None
            and separation >= self.config.turnsign_trim_split_px_640)
        self.turnsign_trim_line_clear_current = clear_now
        previous_max = float(
            self.turnsign_trim_max_lookahead_separation_640)
        if separation is not None:
            self.turnsign_trim_max_lookahead_separation_640 = max(
                previous_max, float(separation))

        if self.turnsign_trim_overshoot_latched:
            if clear_now:
                self.turnsign_trim_line_split_frames += 1
                if (
                    self.turnsign_trim_line_split_frames >=
                    self.config.turnsign_trim_split_frames
                ):
                    self.turnsign_trim_line_split_ready = True
                    self.turnsign_trim_line_ever_split = True
                    self.turnsign_trim_overshoot_latched = False
                    self.turnsign_trim_line_collapse_frames = 0
                    self.turnsign_trim_max_lookahead_separation_640 = float(
                        separation)
            else:
                self.turnsign_trim_line_split_frames = 0
                self.turnsign_trim_line_split_ready = False
            return

        dropped_after_split = bool(
            self.turnsign_trim_line_ever_split
            and separation is not None
            and previous_max - float(separation) >=
                self.config.turnsign_trim_drop_px_640)
        collapse_now = bool(
            self.turnsign_trim_line_ever_split
            and (not clear_now or dropped_after_split))
        if collapse_now:
            self.turnsign_trim_line_collapse_frames += 1
            self.turnsign_trim_line_split_frames = 0
            self.turnsign_trim_line_split_ready = False
            if (
                self.turnsign_trim_line_collapse_frames >=
                self.config.turnsign_trim_collapse_frames
            ):
                self.turnsign_trim_overshoot_latched = True
            return

        self.turnsign_trim_line_collapse_frames = 0
        if clear_now:
            self.turnsign_trim_line_split_frames += 1
            if (
                self.turnsign_trim_line_split_frames >=
                self.config.turnsign_trim_split_frames
            ):
                self.turnsign_trim_line_split_ready = True
                self.turnsign_trim_line_ever_split = True
        else:
            self.turnsign_trim_line_split_frames = 0
            self.turnsign_trim_line_split_ready = False

    def _select_candidate(self, candidates):
        if not candidates:
            self.selection_reason = "no_candidate"
            return None
        wanted_slot = (
            1 if (self.branch_lock == "right" or
                  (self.branch_lock == "left" and
                   self.curve_merge_override)) else 0)
        self.selected_slot_lock = wanted_slot
        self.selection_reason = (
            "merge_continuity_green" if
            self.branch_lock == "left" and self.curve_merge_override
            else "locked_{}".format(self.branch_lock or "left"))
        for candidate in candidates:
            if int(candidate.get("slot", -1)) == wanted_slot:
                return candidate
        return None

    def _path_connection_status(self, selected, image_shape):
        """Reject a fitted path whose near end no longer reaches the car."""
        if selected is None:
            return False, "no_selected_route", {}
        points = np.asarray(
            selected.get("points_xy", ()), dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
            return False, "invalid_path", {}
        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        if len(points) < 3:
            return False, "invalid_path", {}
        height, width = image_shape[:2]
        maximum_y = float(np.max(points[:, 1]))
        near_band = max(18.0, 0.05 * float(height))
        near_points = points[points[:, 1] >= maximum_y - near_band]
        near_x = float(np.median(near_points[:, 0]))
        visual_center = float(width) * self.config.visual_center_x
        near_offset_640 = (
            (near_x - visual_center) * 640.0 / float(max(1, width)))
        reaches_vehicle = bool(
            maximum_y >=
            float(height) * self.config.line_anchor_y_ratio)
        inside_anchor_gate = bool(
            abs(near_offset_640) <=
            self.config.line_anchor_max_offset_px_640)
        metrics = {
            "maximum_y": maximum_y,
            "maximum_y_ratio": maximum_y / float(max(1, height)),
            "near_x": near_x,
            "near_offset_640": near_offset_640,
            "reaches_vehicle": reaches_vehicle,
            "inside_anchor_gate": inside_anchor_gate,
        }
        if not reaches_vehicle:
            return False, "short_near_support", metrics
        if not inside_anchor_gate:
            return False, "detached_near_anchor", metrics
        return True, "connected", metrics

    @staticmethod
    def _is_human_stop_reason(task_reason):
        return str(task_reason or "") in {
            "human_half_lookahead_stop",
            "human_wait_cross",
            "human_absence_check",
            "human_preline_absence_check",
            "car_human_same_side_wait",
            "car_human_absence_check",
            "car_human_preline_absence_check",
        }

    def _update_human_stop_reverse(self, task_reason, now):
        if not self._is_human_stop_reason(task_reason):
            self.human_stop_reverse_until = 0.0
            self.human_stop_reverse_started = False
            return False
        if not self.human_stop_reverse_started:
            self.human_stop_reverse_started = True
            self.human_stop_reverse_until = (
                float(now) + self.config.human_stop_reverse_duration_s)
        return float(now) < self.human_stop_reverse_until

    def _build_command(
            self, selected, route_reason, result, image_shape, now,
            ocr_response=None, line_loss_reason=None):
        lookahead_y = image_shape[0] * self.config.lookahead_y_ratio
        path_geometry = self._path_geometry_metrics(
            selected, lookahead_y, image_shape)
        road_shape_geometry = self._road_shape_consensus_geometry(
            result, selected, lookahead_y, image_shape)
        self._update_road_shape_state(road_shape_geometry)
        ocr_route_locked = (
            self.branch_lock_source == "ocr"
            and self.branch_lock in {"left", "right"}
            and self.last_valid_ocr_ts > 0.0
            and float(now) - self.last_valid_ocr_ts <
                self.config.ocr_lock_lifetime_s)
        if (
            ocr_route_locked and selected is None
            and line_loss_reason is None
        ):
            if isinstance(ocr_response, dict):
                ocr_response["control_phase"] = "ocr_wait_route"
            return self._line_loss_hold_command(
                "ocr_selected_route_unavailable", "ocr_wait_route",
                now, lookahead_y)
        if ocr_route_locked and isinstance(ocr_response, dict):
            ocr_response["control_phase"] = "ocr_route_ready"

        path_target = (
            self._path_target_on_selected(selected, lookahead_y)
            if selected is not None else None)
        path_target_held = False
        adaptive_path_without_hold = False
        if path_target is not None and path_target[2]:
            held_target = self._held_path_target(now)
            if held_target is not None:
                path_target = (
                    float(held_target[0]), float(held_target[1]), True)
                path_target_held = True
            else:
                # A nearest endpoint is only a preview fallback. Never use it
                # as a fresh steering target when the control row is absent.
                adaptive_path_without_hold = True
                path_target = None
        task_path_target_x = None if path_target is None else path_target[0]
        task_state, speed, task_reason, target_override_x = (
            self._task_from_detections(
                result, selected, image_shape, lookahead_y, now,
                ocr_response=ocr_response,
                path_target_x=task_path_target_x))
        human_reverse_active = self._update_human_stop_reverse(
            task_reason, now)
        if adaptive_path_without_hold:
            if self._is_human_stop_reason(task_reason):
                reverse_state = (
                    STATE_AVOID_HUMAN if human_reverse_active
                    else STATE_SAFE_STOP)
                reverse_speed = (
                    self.config.human_stop_reverse_speed_mps
                    if human_reverse_active else 0.0)
                return self._command(
                    self.last_error, reverse_speed, reverse_state,
                    flags=(
                        CONTROL_FLAG_USE_TARGET_SPEED
                        if human_reverse_active else 0)), {
                        "target_x": None,
                        "path_target_x": None,
                        "path_target_y": float(lookahead_y),
                        "path_target_adaptive_y": True,
                        "path_target_held": False,
                        "track_error_640": float(self.last_error),
                        "reason": "adaptive_path_human_stop",
                        "task_reason": task_reason,
                        "human_stop_reverse_active": bool(
                            human_reverse_active),
                    }
            command, target = self._line_loss_hold_command(
                "adaptive_path_no_previous_target", task_reason,
                now, lookahead_y)
            target.update({
                "path_target_adaptive_y": True,
                "path_target_held": False,
                "human_stop_reverse_active": False,
            })
            return command, target
        if selected is None:
            return self._line_loss_hold_command(
                line_loss_reason or route_reason or "line_unavailable",
                task_reason, now, lookahead_y)
        if path_target is None:
            return self._line_loss_hold_command(
                "missing_target_x", task_reason, now, lookahead_y)

        path_target_x, path_target_y, path_target_adaptive = path_target
        requested_target_x = _clamp(
            float(target_override_x) if target_override_x is not None
            else float(path_target_x),
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        width = float(max(1, image_shape[1]))
        requested_task_adjustment_640 = (
            (float(requested_target_x) - float(path_target_x))
            * 640.0 / width)
        curve_task_adjust_limit_640 = (
            abs(float(self.config.right_circle_feedforward_640))
            * float(self.config.curve_task_adjust_limit_ratio))
        offset_task = bool(
            target_override_x is not None
            and int(task_state) in {
                STATE_AVOID_CAR,
                STATE_AVOID_HUMAN,
                STATE_COLLECT_GOLD,
            }
        )
        curve_offset_task = bool(
            offset_task
            and self.road_shape_state == "curve"
            and road_shape_geometry["valid"]
        )
        applied_task_adjustment_640 = float(
            requested_task_adjustment_640)
        curve_task_adjust_soft_margin_640 = (
            curve_task_adjust_limit_640
            * float(self.config.curve_task_adjust_soft_margin_ratio))
        if curve_offset_task:
            applied_task_adjustment_640 = _soft_limit(
                requested_task_adjustment_640,
                curve_task_adjust_limit_640,
                curve_task_adjust_soft_margin_640,
            )
        target_x = _clamp(
            float(path_target_x)
            + applied_task_adjustment_640 * width / 640.0,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        task_offset_x = float(target_x) - float(path_target_x)
        path_raw_error = (
            float(target_x) / width
            - self.config.visual_center_x) * 640.0
        heading_feedforward = 0.0
        curve_feedforward = 0.0
        right_circle_feedforward = 0.0
        right_circle_trim = 0.0
        right_circle_compensation = 0.0
        right_circle_feedforward_active = False
        right_circle_anchor_weight = 0.0
        line_feedforward_enabled = bool(
            (task_reason == "track" and target_override_x is None)
            or curve_offset_task)
        if (line_feedforward_enabled and
                not path_target_adaptive and path_geometry["valid"]):
            heading_delta_640 = float(
                path_geometry["heading_delta_640"])
            if (
                abs(heading_delta_640) >
                self.config.path_heading_deadband_px_640
            ):
                heading_feedforward = _clamp(
                    heading_delta_640 * float(
                        self.config.path_heading_feedforward_gain),
                    -float(self.config.path_heading_feedforward_max_px_640),
                    float(self.config.path_heading_feedforward_max_px_640),
                )
            curve_feedforward = _clamp(
                float(road_shape_geometry["curvature_640"]) * float(
                    self.config.curve_feedforward_gain),
                -float(self.config.curve_feedforward_max_px_640),
                float(self.config.curve_feedforward_max_px_640),
            )
            # The course has only a straight and a right-turn 3 m circle.
            # Once the curve state is latched, hold the measured steering
            # baseline and permit only a small visual trim around it.
            if (
                self.road_shape_state == "curve"
                and road_shape_geometry["valid"]
                and float(road_shape_geometry["curvature_640"]) > 0.0
            ):
                right_circle_feedforward = float(
                    self.config.right_circle_feedforward_640)
        total_feedforward = (
            float(heading_feedforward) + float(curve_feedforward))
        line_based_error = float(path_raw_error) + total_feedforward
        raw_error = line_based_error
        if right_circle_feedforward != 0.0:
            feedforward_delta = (
                line_based_error - right_circle_feedforward)
            capture_range = float(
                self.config.right_circle_capture_range_640)
            if capture_range > 0.0 and abs(feedforward_delta) < capture_range:
                # Keep the selected blue/green line as the steering source.
                # The measured circle value is only a soft nearby anchor.
                # Its bounded correction fades continuously to zero at the
                # capture boundary, so the line-derived command never jumps
                # or gets replaced by a hard-coded steering value.
                right_circle_feedforward_active = True
                right_circle_anchor_weight = _clamp(
                    1.0 - abs(feedforward_delta) / capture_range,
                    0.0,
                    1.0,
                )
                right_circle_trim = _clamp(
                    feedforward_delta,
                    -float(self.config.right_circle_trim_max_640),
                    float(self.config.right_circle_trim_max_640),
                )
                right_circle_compensation = (
                    _clamp(
                        -feedforward_delta,
                        -float(self.config.right_circle_trim_max_640),
                        float(self.config.right_circle_trim_max_640),
                    ) * right_circle_anchor_weight * float(
                        self.config.right_circle_compensation_gain))
                raw_error = (
                    line_based_error + right_circle_compensation)
        uncapped_raw_error = float(raw_error)
        straight_error_limit_640 = (
            abs(float(self.config.right_circle_feedforward_640))
            * float(self.config.straight_error_limit_ratio))
        high_confidence_straight = bool(
            road_shape_geometry["valid"]
            and road_shape_geometry.get("reason") == "straight_consensus"
            and self.road_shape_metrics.get("stable_path_ready", False)
            and self.road_shape_metrics.get("segmentation_reason") == "valid"
        )
        straight_hard_limit_enabled = bool(
            high_confidence_straight and not offset_task)
        straight_error_limited = bool(
            straight_hard_limit_enabled
            and abs(raw_error) > straight_error_limit_640)
        if straight_hard_limit_enabled:
            raw_error = _clamp(
                raw_error,
                -straight_error_limit_640,
                straight_error_limit_640,
            )
        applied_steering_feedforward = (
            float(raw_error) - float(path_raw_error))
        control_target_x = _clamp(
            float(target_x) + applied_steering_feedforward *
            float(max(1, image_shape[1])) / 640.0,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        if task_reason in {
            "turnsign_trim_forward", "turnsign_trim_reverse"
        }:
            error = _clamp(
                raw_error,
                -self.config.max_track_error_640,
                self.config.max_track_error_640)
            self._reset_track_error_response("turnsign_trim")
        else:
            error = self._limit_error(
                raw_error,
                adaptive=(task_reason == "track"),
                curve_mode=(
                    task_reason == "track"
                    and road_shape_geometry["valid"]
                    and self.road_shape_state == "curve"),
            )
        self.last_error = error
        if not path_target_held:
            self.last_valid_ts = now
            self.last_path_target_x = float(path_target_x)
            self.last_path_target_y = float(path_target_y)
            self.last_path_target_slot = int(selected["slot"])
            self.last_path_target_ts = float(now)
        output_speed = (
            self.config.human_stop_reverse_speed_mps
            if human_reverse_active else speed)
        output_state = STATE_AVOID_HUMAN if human_reverse_active else task_state
        if float(output_speed) > 0.0:
            self.last_forward_speed_mps = float(output_speed)
        return self._command(error, output_speed, output_state), {
            "target_x": float(control_target_x),
            "base_target_x": float(target_x),
            "path_target_x": float(path_target_x),
            "path_target_y": float(path_target_y),
            "path_target_held": bool(path_target_held),
            "path_target_adaptive_y": bool(path_target_adaptive),
            "lookahead_y": float(path_target_y),
            "human_stop_line_y": self._human_stop_line_y(
                image_shape, path_target_y),
            "task_offset_x": float(task_offset_x),
            "task_target_applied": target_override_x is not None,
            "requested_task_adjustment_640": float(
                requested_task_adjustment_640),
            "applied_task_adjustment_640": float(
                applied_task_adjustment_640),
            "curve_task_adjust_limit_640": float(
                curve_task_adjust_limit_640),
            "curve_task_adjust_soft_margin_640": float(
                curve_task_adjust_soft_margin_640),
            "curve_task_adjustment_limited": bool(
                curve_offset_task
                and abs(applied_task_adjustment_640
                        - requested_task_adjustment_640) > 1e-6),
            "track_error_640": float(error),
            "raw_track_error_640": float(raw_error),
            "uncapped_raw_track_error_640": float(uncapped_raw_error),
            "straight_error_limit_640": float(straight_error_limit_640),
            "straight_error_limited": bool(straight_error_limited),
            "high_confidence_straight": bool(
                high_confidence_straight),
            "straight_hard_limit_enabled": bool(
                straight_hard_limit_enabled),
            "straight_limit_suppressed_by_task": bool(
                high_confidence_straight and offset_task),
            "path_raw_track_error_640": float(path_raw_error),
            "path_heading_feedforward_640": float(
                heading_feedforward),
            "curve_feedforward_640": float(curve_feedforward),
            "right_circle_feedforward_640": float(
                right_circle_feedforward),
            "right_circle_trim_640": float(right_circle_trim),
            "right_circle_feedforward_active": bool(
                right_circle_feedforward_active),
            "right_circle_compensation_640": float(
                right_circle_compensation),
            "right_circle_anchor_weight": float(
                right_circle_anchor_weight),
            "right_circle_compensation_gain": float(
                self.config.right_circle_compensation_gain),
            "right_circle_output_offset_640": float(
                raw_error - right_circle_feedforward
                if right_circle_feedforward != 0.0 else 0.0),
            "line_based_track_error_640": float(line_based_error),
            "total_feedforward_640": float(total_feedforward),
            "applied_steering_feedforward_640": float(
                applied_steering_feedforward),
            "road_shape_state": self.road_shape_state,
            "road_shape_valid": bool(road_shape_geometry["valid"]),
            "road_geometry_reason": road_shape_geometry.get("reason"),
            "road_geometry_sample_rows_y": list(
                road_shape_geometry.get("sample_rows_y") or ()),
            "road_geometry_support_y": list(
                road_shape_geometry.get("support_y") or ()),
            "road_heading_delta_640": float(
                road_shape_geometry["heading_delta_640"]),
            "road_curvature_640": float(
                road_shape_geometry["curvature_640"]),
            "road_curve_evidence": dict(self.road_shape_metrics),
            "selected_path_curvature_640": float(
                path_geometry["curvature_640"]),
            "track_error_response": self.track_error_response,
            "track_error_trend_frames": int(self.track_error_trend_frames),
            "reason": route_reason,
            "task_reason": task_reason,
            "human_stop_reverse_active": bool(human_reverse_active),
        }

    def _line_loss_hold_command(
            self, reason, task_reason, now, lookahead_y):
        """Hold the latest steering, but slow down until the path returns."""
        age = now - self.last_valid_ts if self.last_valid_ts else 1e9
        held_target = self._held_path_target(now)
        error = float(self.last_error)
        previous_speed = float(self.last_forward_speed_mps)
        if previous_speed <= 0.0:
            previous_speed = max(0.031, float(self.config.normal_speed_mps))
        recover_speed = max(0.031, float(self.config.recover_speed_mps))
        speed = min(previous_speed, recover_speed)
        return self._command(error, speed, STATE_RECOVER_LINE), {
            "target_x": (
                None if held_target is None else held_target[0]),
            "path_target_x": (
                None if held_target is None else held_target[0]),
            "path_target_y": (
                float(lookahead_y)
                if held_target is None else held_target[1]),
            "path_target_held": held_target is not None,
            "track_error_640": error,
            "reason": str(reason or "line_unavailable"),
            "task_reason": str(task_reason or "no_path"),
            "line_loss_hold": True,
            "line_loss_indefinite": True,
            "line_loss_age_s": float(age),
            "line_loss_previous_speed_mps": float(previous_speed),
            "line_loss_held_speed_mps": float(speed),
            "line_loss_speed_reduced": bool(speed < previous_speed),
        }

    @staticmethod
    def _path_target_on_selected(selected, preferred_y):
        points = np.asarray(
            (selected or {}).get("points_xy", ()), dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 2 or not len(points):
            return None
        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        if not len(points):
            return None
        minimum_y = float(np.min(points[:, 1]))
        maximum_y = float(np.max(points[:, 1]))
        target_x = _interp_path_x(points, preferred_y)
        if (target_x is not None and
                minimum_y <= float(preferred_y) <= maximum_y):
            return float(target_x), float(preferred_y), False
        nearest = int(np.argmin(np.abs(points[:, 1] - float(preferred_y))))
        # Keep the displayed lookahead row fixed and tag the nearest endpoint
        # as adaptive. The caller may use it for preview or a recent-target
        # hold, but never as a fresh normal steering target.
        return float(points[nearest, 0]), float(preferred_y), True

    @staticmethod
    def _road_geometry_sample_rows(lookahead_y, image_height):
        span = min(72.0, max(32.0, 0.15 * float(image_height)))
        sample_step = 0.5 * span
        return (
            float(lookahead_y),
            float(lookahead_y) + sample_step,
            float(lookahead_y) + span,
        )

    @classmethod
    def _path_covers_control_geometry(cls, points, lookahead_y, image_height):
        rows = cls._road_geometry_sample_rows(lookahead_y, image_height)
        return all(cls._path_covers_y(points, row) for row in rows)

    def _path_geometry_metrics(self, selected, lookahead_y, image_shape):
        """Measure tangent and bend once for feedforward and road shape."""
        empty = {
            "valid": False,
            "reason": "insufficient_points",
            "heading_delta_640": 0.0,
            "curvature_640": 0.0,
            "sample_rows_y": (),
            "support_y": (),
        }
        points = np.asarray(
            (selected or {}).get("points_xy", ()), dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 4:
            return empty
        height, width = image_shape[:2]
        # The fitted control curve is guaranteed to reach the lookahead row,
        # not a row farther toward the top of the image.  Sample from the
        # lookahead toward the vehicle so normal fitted curves can provide
        # geometry without extrapolation.
        rows = self._road_geometry_sample_rows(lookahead_y, height)
        span = float(rows[-1] - rows[0])
        sample_step = float(rows[1] - rows[0])
        support_y = (
            float(np.min(points[:, 1])),
            float(np.max(points[:, 1])),
        )
        base = {
            "valid": False,
            "reason": "insufficient_vertical_support",
            "heading_delta_640": 0.0,
            "curvature_640": 0.0,
            "sample_rows_y": rows,
            "support_y": support_y,
        }
        if (
            rows[0] < support_y[0]
            or rows[-1] > support_y[1]
        ):
            return base
        samples = [_interp_path_x(points, row) for row in rows]
        if any(
            value is None or not math.isfinite(float(value))
            for value in samples
        ):
            base["reason"] = "invalid_interpolation"
            return base
        width_scale = 640.0 / float(max(1, width))
        # Preserve the former 72 px adjacent-row scale.  The new one-sided
        # samples are 36 px apart at 480p, so slope and second difference are
        # normalized to keep existing thresholds/gains comparable.
        heading_scale = (2.0 * span) / max(1.0, rows[2] - rows[0])
        curvature_scale = (span / max(1.0, sample_step)) ** 2
        heading_delta_640 = (
            float(samples[0] - samples[2]) * width_scale * heading_scale)
        curvature_640 = (
            float(samples[0] - 2.0 * samples[1] + samples[2]) *
            width_scale * curvature_scale)
        return {
            "valid": True,
            "reason": "valid",
            "heading_delta_640": float(heading_delta_640),
            "curvature_640": float(curvature_640),
            "sample_rows_y": rows,
            "support_y": support_y,
        }

    @staticmethod
    def _curvature_from_samples(samples, rows, image_width):
        span = float(rows[-1] - rows[0])
        sample_step = float(rows[1] - rows[0])
        width_scale = 640.0 / float(max(1, image_width))
        heading_scale = (2.0 * span) / max(1.0, rows[2] - rows[0])
        curvature_scale = (span / max(1.0, sample_step)) ** 2
        heading_delta_640 = (
            float(samples[0] - samples[2]) *
            width_scale * heading_scale)
        curvature_640 = (
            float(samples[0] - 2.0 * samples[1] + samples[2]) *
            width_scale * curvature_scale)
        return float(heading_delta_640), float(curvature_640)

    @staticmethod
    def _road_mask_run(road_mask, row, reference_x):
        height, width = road_mask.shape
        offsets = (0, -1, 1, -2, 2)
        for offset in offsets:
            center_row = int(np.clip(int(row) + offset, 0, height - 1))
            row_start = max(0, center_row - 1)
            row_end = min(height, center_row + 2)
            occupied = np.any(
                road_mask[row_start:row_end] != 0, axis=0)
            padded = np.pad(occupied.astype(np.int8), (1, 1))
            transitions = np.diff(padded)
            starts = np.flatnonzero(transitions == 1)
            ends = np.flatnonzero(transitions == -1) - 1
            runs = [
                (int(start), int(end))
                for start, end in zip(starts, ends)
                if end - start + 1 >= 3
            ]
            if not runs:
                continue

            def distance(run):
                start, end = run
                if start <= reference_x <= end:
                    return 0.0
                return min(
                    abs(float(reference_x) - start),
                    abs(float(reference_x) - end),
                )

            return min(
                runs,
                key=lambda run: (
                    distance(run),
                    abs(0.5 * (run[0] + run[1]) - reference_x),
                    -(run[1] - run[0]),
                ),
            )
        return None

    def _stable_curve_path_geometry(
            self, candidates, selected, lookahead_y, image_shape):
        selected_slot = (
            None if selected is None
            else int(selected.get("slot", -1)))
        next_states = {}
        evaluated = []
        for candidate in list(candidates or ())[:2]:
            slot = int(candidate.get("slot", -1))
            connected, _reason, _metrics = self._path_connection_status(
                candidate, image_shape)
            geometry = self._path_geometry_metrics(
                candidate, lookahead_y, image_shape)
            if not connected or not geometry["valid"]:
                continue
            curvature = float(geometry["curvature_640"])
            previous = self.road_curve_path_states.get(slot)
            consistent = bool(
                previous is not None
                and abs(curvature - float(previous["curvature_640"]))
                <= float(self.config.curve_evidence_max_delta_640)
            )
            stable_frames = (
                min(1000, int(previous["stable_frames"]) + 1)
                if consistent else 1)
            next_states[slot] = {
                "curvature_640": curvature,
                "stable_frames": stable_frames,
            }
            confidences = np.asarray(
                candidate.get("point_confidences", ()),
                dtype=np.float32)
            mean_confidence = (
                float(np.mean(confidences))
                if len(confidences) else float(candidate.get("score", 0.0)))
            support_span = float(
                geometry["support_y"][1] - geometry["support_y"][0])
            quality = (
                stable_frames,
                int(candidate.get("row_support", 0)),
                mean_confidence,
                support_span,
                1 if slot == selected_slot else 0,
            )
            evaluated.append(
                (quality, candidate, geometry, mean_confidence))
        self.road_curve_path_states = next_states
        if not evaluated:
            return None, None, {
                "stable_path_slot": None,
                "stable_path_frames": 0,
                "stable_path_ready": False,
            }
        _quality, candidate, geometry, confidence = max(
            evaluated, key=lambda item: item[0])
        slot = int(candidate.get("slot", -1))
        stable_frames = int(next_states[slot]["stable_frames"])
        return candidate, geometry, {
            "stable_path_slot": slot,
            "stable_path_frames": stable_frames,
            "stable_path_ready": bool(
                stable_frames >=
                self.config.curve_evidence_stable_frames),
            "stable_path_row_support": int(
                candidate.get("row_support", 0)),
            "stable_path_confidence": float(confidence),
            "stable_path_curvature_640": float(
                geometry["curvature_640"]),
        }

    def _segmentation_edge_geometry(
            self, result, reference_path, path_geometry, image_shape):
        road = (result.get("road") or {}).get("mask")
        if road is None:
            road = result.get("road_mask")
        road_mask = np.asarray(
            road) if road is not None else np.empty((0, 0))
        if road_mask.ndim != 2 or not road_mask.size or not np.any(road_mask):
            return None, {"segmentation_reason": "missing_road_mask"}
        points = np.asarray(
            (reference_path or {}).get("points_xy", ()),
            dtype=np.float32)
        rows = tuple(path_geometry.get("sample_rows_y") or ())
        if len(rows) != 3 or len(points) < 2:
            return None, {"segmentation_reason": "missing_reference_path"}
        image_height, image_width = image_shape[:2]
        mask_height, mask_width = road_mask.shape
        edge_rows = np.linspace(
            float(rows[0]), float(rows[-1]), 7, dtype=np.float32)
        left_samples = []
        right_samples = []
        widths_640 = []
        for image_y in edge_rows:
            reference_image_x = _interp_path_x(points, image_y)
            if reference_image_x is None:
                return None, {
                    "segmentation_reason": "reference_interpolation_failed"}
            mask_x = (
                float(reference_image_x) *
                float(max(1, mask_width - 1)) /
                float(max(1, image_width - 1)))
            mask_y = int(round(
                float(image_y) * float(max(1, mask_height - 1)) /
                float(max(1, image_height - 1))))
            run = self._road_mask_run(road_mask, mask_y, mask_x)
            if run is None:
                return None, {
                    "segmentation_reason": "road_edge_missing"}
            image_scale = (
                float(max(1, image_width - 1)) /
                float(max(1, mask_width - 1)))
            left_x = float(run[0]) * image_scale
            right_x = float(run[1]) * image_scale
            left_samples.append(left_x)
            right_samples.append(right_x)
            widths_640.append(
                (right_x - left_x) * 640.0 /
                float(max(1, image_width)))
        if min(widths_640) < 12.0:
            return None, {"segmentation_reason": "road_component_too_narrow"}
        center_samples = [
            0.5 * (left + right)
            for left, right in zip(left_samples, right_samples)
        ]
        def smoothed_curvature(samples):
            try:
                coefficients = np.polyfit(
                    edge_rows.astype(np.float64),
                    np.asarray(samples, dtype=np.float64),
                    2,
                )
            except (TypeError, ValueError, np.linalg.LinAlgError):
                return 0.0, 0.0
            fitted = np.polyval(
                coefficients, np.asarray(rows, dtype=np.float64))
            return self._curvature_from_samples(
                fitted, rows, image_width)

        left_heading, left_curvature = smoothed_curvature(left_samples)
        right_heading, right_curvature = smoothed_curvature(right_samples)
        center_heading, center_curvature = smoothed_curvature(center_samples)
        return {
            "valid": True,
            "heading_delta_640": float(center_heading),
            "curvature_640": float(center_curvature),
            "left_curvature_640": float(left_curvature),
            "right_curvature_640": float(right_curvature),
            "sample_rows_y": rows,
            "support_y": tuple(path_geometry.get("support_y") or ()),
        }, {
            "segmentation_reason": "valid",
            "segmentation_center_curvature_640": float(center_curvature),
            "segmentation_left_curvature_640": float(left_curvature),
            "segmentation_right_curvature_640": float(right_curvature),
            "segmentation_widths_640": [
                float(value) for value in widths_640],
        }

    @staticmethod
    def _line_fit_max_residual_640(y_values, x_values, image_width):
        y_values = np.asarray(y_values, dtype=np.float64)
        x_values = np.asarray(x_values, dtype=np.float64)
        if len(y_values) < 3 or len(x_values) != len(y_values):
            return 1e9
        try:
            coefficients = np.polyfit(y_values, x_values, 1)
        except (TypeError, ValueError, np.linalg.LinAlgError):
            return 1e9
        predicted = np.polyval(coefficients, y_values)
        return float(
            np.max(np.abs(x_values - predicted))
            * 640.0 / float(max(1, image_width)))

    def _long_straight_evidence(
            self, result, reference_path, image_shape):
        points = np.asarray(
            (reference_path or {}).get("points_xy", ()),
            dtype=np.float32)
        finite = (
            np.all(np.isfinite(points), axis=1)
            if points.ndim == 2 and points.shape[1:] == (2,)
            else np.zeros(0, dtype=bool))
        points = points[finite] if len(finite) else np.empty((0, 2))
        image_height, image_width = image_shape[:2]
        minimum_span = (
            float(image_height) * float(
                self.config.straight_min_span_ratio))
        if len(points) < 4:
            return {
                "long_straight_ready": False,
                "long_straight_reason": "insufficient_path_points",
                "long_path_span_640": 0.0,
            }
        order = np.argsort(points[:, 1], kind="stable")
        points = points[order]
        minimum_y = float(points[0, 1])
        maximum_y = float(points[-1, 1])
        path_span = maximum_y - minimum_y
        path_residual = self._line_fit_max_residual_640(
            points[:, 1], points[:, 0], image_width)
        metrics = {
            "long_path_span_px": float(path_span),
            "long_path_min_span_px": float(minimum_span),
            "long_path_residual_640": float(path_residual),
        }
        if path_span < minimum_span:
            metrics.update({
                "long_straight_ready": False,
                "long_straight_reason": "path_too_short",
            })
            return metrics
        if path_residual > self.config.straight_path_max_residual_640:
            metrics.update({
                "long_straight_ready": False,
                "long_straight_reason": "path_not_straight_end_to_end",
            })
            return metrics

        road = (result.get("road") or {}).get("mask")
        if road is None:
            road = result.get("road_mask")
        road_mask = np.asarray(
            road) if road is not None else np.empty((0, 0))
        if road_mask.ndim != 2 or not road_mask.size or not np.any(road_mask):
            metrics.update({
                "long_straight_ready": False,
                "long_straight_reason": "missing_long_road_edges",
            })
            return metrics
        mask_height, mask_width = road_mask.shape
        sample_rows = np.linspace(
            minimum_y, maximum_y, 11, dtype=np.float32)
        left_samples = []
        right_samples = []
        for image_y in sample_rows:
            reference_x = _interp_path_x(points, float(image_y))
            if reference_x is None:
                metrics.update({
                    "long_straight_ready": False,
                    "long_straight_reason": "long_reference_failed",
                })
                return metrics
            mask_x = (
                float(reference_x) * float(max(1, mask_width - 1))
                / float(max(1, image_width - 1)))
            mask_y = int(round(
                float(image_y) * float(max(1, mask_height - 1))
                / float(max(1, image_height - 1))))
            run = self._road_mask_run(road_mask, mask_y, mask_x)
            if run is None:
                metrics.update({
                    "long_straight_ready": False,
                    "long_straight_reason": "long_road_edge_missing",
                })
                return metrics
            image_scale = (
                float(max(1, image_width - 1))
                / float(max(1, mask_width - 1)))
            left_samples.append(float(run[0]) * image_scale)
            right_samples.append(float(run[1]) * image_scale)
        center_samples = [
            0.5 * (left + right)
            for left, right in zip(left_samples, right_samples)
        ]
        left_residual = self._line_fit_max_residual_640(
            sample_rows, left_samples, image_width)
        right_residual = self._line_fit_max_residual_640(
            sample_rows, right_samples, image_width)
        center_residual = self._line_fit_max_residual_640(
            sample_rows, center_samples, image_width)
        maximum_edge_residual = max(
            left_residual, right_residual, center_residual)
        metrics.update({
            "long_left_edge_residual_640": float(left_residual),
            "long_right_edge_residual_640": float(right_residual),
            "long_center_residual_640": float(center_residual),
            "long_straight_ready": bool(
                maximum_edge_residual <=
                self.config.straight_edge_max_residual_640),
            "long_straight_reason": (
                "whole_segment_straight"
                if maximum_edge_residual <=
                    self.config.straight_edge_max_residual_640
                else "road_edges_not_straight_end_to_end"),
        })
        return metrics

    def _road_shape_consensus_geometry(
            self, result, selected, lookahead_y, image_shape):
        candidates = result.get("paths")
        if candidates is None:
            candidates = (result.get("centerline") or {}).get("paths") or []
        candidates = list(candidates or ())
        if selected is not None:
            selected_slot = int(selected.get("slot", -1))
            if not any(
                int(candidate.get("slot", -1)) == selected_slot
                for candidate in candidates
            ):
                candidates.append(selected)
        stable_path, path_geometry, metrics = (
            self._stable_curve_path_geometry(
                candidates, selected, lookahead_y, image_shape))
        empty = {
            "valid": False,
            "reason": "no_stable_path",
            "heading_delta_640": 0.0,
            "curvature_640": 0.0,
            "sample_rows_y": (),
            "support_y": (),
        }
        if stable_path is None or path_geometry is None:
            self.road_shape_metrics = metrics
            return empty
        if not metrics["stable_path_ready"]:
            pending = dict(empty)
            pending.update({
                "reason": "stable_path_pending",
                "sample_rows_y": path_geometry.get("sample_rows_y", ()),
                "support_y": path_geometry.get("support_y", ()),
            })
            self.road_shape_metrics = metrics
            return pending
        segmentation, segmentation_metrics = (
            self._segmentation_edge_geometry(
                result, stable_path, path_geometry, image_shape))
        metrics.update(segmentation_metrics)
        if segmentation is None:
            missing = dict(empty)
            missing.update({
                "reason": segmentation_metrics["segmentation_reason"],
                "sample_rows_y": path_geometry.get("sample_rows_y", ()),
                "support_y": path_geometry.get("support_y", ()),
            })
            self.road_shape_metrics = metrics
            return missing
        long_straight_metrics = self._long_straight_evidence(
            result, stable_path, image_shape)
        metrics.update(long_straight_metrics)

        path_curvature = float(path_geometry["curvature_640"])
        center_curvature = float(segmentation["curvature_640"])
        edge_curvatures = (
            float(segmentation["left_curvature_640"]),
            float(segmentation["right_curvature_640"]),
        )
        exit_threshold = max(
            0.0, float(self.config.curve_state_exit_curvature_640))
        path_sign = self._sign(path_curvature)
        center_sign = self._sign(center_curvature)
        agreeing_edges = [
            value for value in edge_curvatures
            if self._sign(value) == path_sign
            and abs(value) >= exit_threshold
        ]
        straight_consensus = bool(
            abs(path_curvature) <= exit_threshold
            and abs(center_curvature) <= exit_threshold
            and all(
                abs(value) <= exit_threshold
                for value in edge_curvatures
            )
            and metrics.get("long_straight_ready", False)
        )
        curve_consensus = bool(
            path_sign != 0
            and path_sign == center_sign
            and agreeing_edges
        )
        metrics.update({
            "curve_consensus": curve_consensus,
            "straight_consensus": straight_consensus,
            "agreeing_segmentation_edges": len(agreeing_edges),
        })
        if straight_consensus:
            consensus_curvature = 0.0
            reason = "straight_consensus"
        elif curve_consensus:
            edge_curvature = min(
                agreeing_edges,
                key=lambda value: abs(
                    abs(value) - abs(path_curvature)))
            consensus_magnitude = min(
                abs(path_curvature),
                abs(center_curvature),
                abs(edge_curvature),
            )
            consensus_curvature = (
                float(path_sign) * consensus_magnitude)
            metrics["selected_segmentation_edge_curvature_640"] = float(
                edge_curvature)
            reason = "curve_consensus"
        else:
            consensus_curvature = 0.0
            reason = "curve_evidence_disagrees"
        self.road_shape_metrics = metrics
        return {
            "valid": True,
            "reason": reason,
            "heading_delta_640": float(
                segmentation["heading_delta_640"]),
            "curvature_640": float(consensus_curvature),
            "sample_rows_y": path_geometry.get("sample_rows_y", ()),
            "support_y": path_geometry.get("support_y", ()),
        }

    def _update_road_shape_state(self, geometry):
        """Latch a real bend from path curvature without reacting to slope."""
        valid = bool((geometry or {}).get("valid"))
        self.road_shape_valid = valid
        if not valid:
            self.road_heading_delta_640 = 0.0
            self.road_curvature_640 = 0.0
            self.road_curve_enter_frames = 0
            if self.road_shape_state == "curve":
                self.road_curve_exit_frames += 1
                if (
                    self.road_curve_exit_frames >=
                    self.config.curve_state_exit_frames
                ):
                    self.road_shape_state = "straight"
                    self.road_curve_exit_frames = 0
            else:
                self.road_curve_exit_frames = 0
            return

        self.road_heading_delta_640 = float(
            geometry.get("heading_delta_640", 0.0))
        self.road_curvature_640 = float(
            geometry.get("curvature_640", 0.0))
        magnitude = abs(self.road_curvature_640)
        enter_threshold = max(
            float(self.config.curve_state_enter_curvature_640),
            float(self.config.curve_state_exit_curvature_640),
        )
        exit_threshold = min(
            enter_threshold,
            float(self.config.curve_state_exit_curvature_640),
        )

        if self.road_shape_state == "straight":
            self.road_curve_exit_frames = 0
            if magnitude >= enter_threshold:
                self.road_curve_enter_frames += 1
            else:
                self.road_curve_enter_frames = 0
            if (
                self.road_curve_enter_frames >=
                self.config.curve_state_enter_frames
            ):
                self.road_shape_state = "curve"
                self.road_curve_enter_frames = 0
        else:
            self.road_curve_enter_frames = 0
            if magnitude <= exit_threshold:
                self.road_curve_exit_frames += 1
            else:
                self.road_curve_exit_frames = 0
            if (
                self.road_curve_exit_frames >=
                self.config.curve_state_exit_frames
            ):
                self.road_shape_state = "straight"
                self.road_curve_exit_frames = 0

    def _held_path_target(self, now):
        if (self.last_path_target_x is None or
                self.last_path_target_y is None or
                self.last_path_target_slot is None or
                self.selected_slot_lock != self.last_path_target_slot or
                float(now) - self.last_path_target_ts >
                self.config.recover_hold_s):
            return None
        return (float(self.last_path_target_x),
                float(self.last_path_target_y))

    def _task_from_detections(
            self, result, selected, image_shape, lookahead_y, now,
            ocr_response=None, path_target_x=None):
        detections = result.get("detections") or []
        if selected is None:
            return (
                STATE_RECOVER_LINE, self.config.recover_speed_mps,
                "no_path", None)
        target_path_x = _finite_float(path_target_x)
        if target_path_x is None:
            target_path_x = _interp_path_x(
                selected["points_xy"], lookahead_y)
        if target_path_x is None:
            target_path_x = image_shape[1] * self.config.visual_center_x
        lookahead_path_x = float(target_path_x)

        sign_action = self._turnsign_action(
            detections, image_shape, ocr_response, now,
            lookahead_path_x)
        if sign_action is not None:
            return sign_action
        car_action = self._car_avoidance_action(
            detections, image_shape, lookahead_y, lookahead_path_x, now)
        if car_action is not None:
            if str(car_action[2]) in {
                "car_human_same_side_pass",
                "car_human_same_side_pass_hold",
            }:
                self.human_path_return_guard_until = 0.0
                self.human_path_return_guard_start_x = None
                self.human_path_return_pending = True
                self.human_last_avoid_target_x = _finite_float(
                    car_action[3])
            return car_action
        human_action = self._human_action(
            detections, selected, image_shape, lookahead_y,
            lookahead_path_x, now)
        if human_action is not None:
            if int(human_action[0]) == STATE_AVOID_HUMAN:
                self.human_speed_hold_until = (
                    float(now) + self.config.human_speed_hold_s)
            return human_action
        if now < self.human_speed_hold_until:
            held_target_x = _clamp(
                float(target_path_x) + float(self.human_pass_offset_x),
                0.0,
                float(max(0, image_shape[1] - 1)),
            )
            self.human_path_return_pending = True
            self.human_last_avoid_target_x = float(held_target_x)
            return (
                STATE_AVOID_HUMAN,
                self.config.human_pass_speed_mps,
                "human_speed_hold",
                held_target_x,
            )
        if self.human_path_return_pending:
            # Keep the last physical pass target as the start of a short
            # return ramp.  The newly selected path may be noisy or sit at
            # the image edge immediately after the person leaves.
            return_start_x = _finite_float(
                self.human_last_avoid_target_x)
            if return_start_x is None:
                return_start_x = (
                    float(target_path_x) + float(self.human_pass_offset_x))
            self.human_path_return_guard_start_x = _clamp(
                float(return_start_x),
                0.0,
                float(max(0, image_shape[1] - 1)),
            )
            self.human_path_return_guard_until = (
                float(now) + self.config.human_path_return_guard_s)
            self.human_path_return_pending = False
        self.human_pass_offset_x = 0.0
        guard_duration = float(self.config.human_path_return_guard_s)
        if (
            self.human_path_return_guard_start_x is not None
            and float(now) < self.human_path_return_guard_until
            and guard_duration > 0.0
        ):
            remaining = _clamp(
                (self.human_path_return_guard_until - float(now)) /
                guard_duration,
                0.0, 1.0)
            return_target_x = (
                float(target_path_x)
                + (float(self.human_path_return_guard_start_x)
                   - float(target_path_x)) * remaining
            )
            return (
                STATE_TRACK,
                self.config.normal_speed_mps,
                "human_path_return_guard",
                _clamp(
                    return_target_x,
                    0.0,
                    float(max(0, image_shape[1] - 1)),
                ),
            )
        self.human_path_return_guard_until = 0.0
        self.human_path_return_guard_start_x = None
        self.human_last_avoid_target_x = None
        coin = self._best_coin(detections, image_shape, target_path_x)
        if coin is not None:
            return (
                STATE_COLLECT_GOLD,
                self.config.collect_speed_mps,
                "coin_bias",
                self._coin_target_x(
                    coin, target_path_x, image_shape),
            )
        return STATE_TRACK, self.config.normal_speed_mps, "track", None

    def _turnsign_action(
            self, detections, image_shape, ocr_response, now,
            path_x):
        response = ocr_response if isinstance(ocr_response, dict) else {}
        phase = str(response.get("control_phase") or "")
        if not self.config.turnsign_trim_enabled:
            # Keep the sign safety phases, but do not let OCR parking control
            # change longitudinal motion or replace the selected path target.
            if phase in {
                "turnsign_edge_over_line", "turnsign_ocr_wait",
                "turnsign_position_ready", "turnsign_missing_stop",
            }:
                return STATE_SAFE_STOP, 0.0, phase, None
            if phase in {
                "turnsign_edge_left", "turnsign_edge_right",
                "turnsign_approach", "turnsign_missing_hold",
                "turnsign_class_continuation",
                "turnsign_trim_forward", "turnsign_trim_reverse",
                "turnsign_trim_settle", "turnsign_trim_verify",
                "turnsign_trim_ready",
            }:
                return (
                    STATE_TRACK,
                    self.config.turnsign_slow_speed_mps,
                    "turnsign_trim_disabled",
                    None,
                )
            if phase:
                return None
            return self._legacy_turnsign_action(
                detections, image_shape, response, now)
        session_active = bool(response.get("session_active"))
        session_id = response.get("session_id")
        new_session = bool(
            session_active and session_id is not None
            and session_id == self.turnsign_control_session_id
            and self.turnsign_new_session_pending)
        if new_session:
            self.turnsign_new_session_pending = False
        self._remember_turnsign_position(response, phase, image_shape)
        if self.turnsign_trim_session_adaptive:
            self._update_turnsign_line_history(now)
        api_result_ready = bool(
            response.get("turnsign_resolved")
            or self._ocr_has_current_direction()
            or self.turnsign_trim_pending_ocr_direction in {"left", "right"})
        if api_result_ready:
            self.turnsign_trim_api_ready = True
        self._refresh_turnsign_trim_accept_ready()
        self._publish_turnsign_trim_info(response)

        if new_session:
            trim_direction = self._turnsign_trim_required_direction()
            if trim_direction:
                self._start_turnsign_trim_pulse(now, trim_direction)

        if not session_active and self.turnsign_trim_pulse_until > 0.0:
            self.turnsign_trim_pulse_until = 0.0
            self.turnsign_trim_direction = 0

        # A resolved OCR result cannot shorten a pulse that has already begun.
        if self.turnsign_trim_pulse_until > 0.0:
            if float(now) < self.turnsign_trim_pulse_until:
                return self._active_turnsign_trim_action(
                    response, image_shape, path_x)
            self.turnsign_trim_pulse_until = 0.0
            self.turnsign_trim_direction = 0
            self.turnsign_trim_settle_until = (
                float(now) + self.config.turnsign_trim_settle_s)

        if api_result_ready and (
            not self.turnsign_trim_session_adaptive
            or self.turnsign_trim_line_split_ready
        ):
            # Once a valid OCR result is locked, the consumed physical sign no
            # longer changes speed or steering, but a curve session must first
            # expose both route slots at the lookahead row.
            self.turnsign_trim_pulse_until = 0.0
            self._clear_turnsign_state()
            if phase != "ocr_route_ready":
                response["control_phase"] = "turnsign_consumed"
            return None

        if (
            self.turnsign_trim_session_adaptive
            and (session_active or self.turnsign_trim_api_ready)
        ):
            if self.turnsign_trim_accept_ready:
                response["control_phase"] = "turnsign_trim_ready"
                self._publish_turnsign_trim_info(response)
                return (
                    STATE_SAFE_STOP, 0.0,
                    "turnsign_trim_ready", None)
            if float(now) < self.turnsign_trim_settle_until:
                response["control_phase"] = "turnsign_trim_settle"
                self._publish_turnsign_trim_info(response)
                return (
                    STATE_SAFE_STOP, 0.0,
                    "turnsign_trim_settle", None)
            trim_direction = self._turnsign_trim_required_direction()
            if trim_direction:
                self._start_turnsign_trim_pulse(
                    now, trim_direction)
                return self._active_turnsign_trim_action(
                    response, image_shape, path_x)
            if trim_direction == 0 and not self.turnsign_trim_stop_ready:
                response["control_phase"] = "turnsign_trim_verify"
                self._publish_turnsign_trim_info(response)
                return (
                    STATE_SAFE_STOP, 0.0,
                    "turnsign_trim_verify", None)

        if phase:
            if phase == "turnsign_edge_over_line":
                return (
                    STATE_SAFE_STOP, 0.0,
                    "turnsign_edge_over_line", None)

            if phase in {
                "turnsign_ocr_wait", "turnsign_position_ready",
                "turnsign_missing_stop",
            }:
                return STATE_SAFE_STOP, 0.0, phase, None

            if phase in {"turnsign_edge_left", "turnsign_edge_right"}:
                sign_center_x = _finite_float(
                    response.get("bbox_center_x"))
                if sign_center_x is None:
                    geom = self._detection_geom(
                        response.get("detection") or {}, image_shape)
                    sign_center_x = (
                        None if geom is None else float(geom["cx"]))
                if sign_center_x is None:
                    return (
                        STATE_SAFE_STOP, 0.0,
                        "turnsign_missing_stop", None)
                width = float(max(1, image_shape[1]))
                visual_center = width * self.config.visual_center_x
                sign_delta = float(sign_center_x) - visual_center
                target_x = _clamp(
                    float(path_x) +
                    self.config.turnsign_steer_gain * sign_delta,
                    0.0,
                    float(max(0, image_shape[1] - 1)),
                )
                return (
                    STATE_TRACK,
                    self.config.turnsign_slow_speed_mps,
                    "turnsign_edge_steer",
                    target_x,
                )

            if phase in {
                "turnsign_approach", "turnsign_missing_hold",
                "turnsign_class_continuation",
            }:
                return (
                    STATE_TRACK,
                    self.config.turnsign_slow_speed_mps,
                    "turnsign_approach",
                    None,
                )

            # Too-small, too-far, low-score and pre-confirmation detections do
            # not count as a recognized sign and must not affect the vehicle.
            return None

        # Compatibility fallback when OCR is disabled or an older caller does
        # not provide the explicit position-control phase.
        return self._legacy_turnsign_action(
            detections, image_shape, response, now)

    def _turnsign_trim_motion_direction(self):
        if self.turnsign_trim_overshoot_latched:
            return -1
        separation = _finite_float(self.turnsign_trim_separation_640)
        if separation is None:
            return -1 if self.turnsign_trim_line_ever_split else 1
        low = min(
            self.config.turnsign_trim_low_separation_px_640,
            self.config.turnsign_trim_high_separation_px_640)
        high = max(
            self.config.turnsign_trim_low_separation_px_640,
            self.config.turnsign_trim_high_separation_px_640)
        target = 0.5 * (float(low) + float(high))
        return 1 if separation <= target else -1

    def _turnsign_trim_required_direction(self):
        distance_direction = self._turnsign_trim_motion_direction()
        if distance_direction is None:
            return None
        if self.turnsign_trim_accept_ready:
            return 0
        # Once both branches are visible, give a centered sign a few stationary
        # frames to prove stability. Before the split is clear, longitudinal
        # micro-adjustment must continue even if the sign is already centered.
        if (
            self.turnsign_trim_line_clear_current
            and (self.turnsign_trim_api_ready
                 or (self.turnsign_trim_current_fresh
                     and self.turnsign_trim_current_centered))
        ):
            return 0
        # Separation owns only the longitudinal direction; it is deliberately
        # not part of the final parking acceptance condition.
        return distance_direction

    def _refresh_turnsign_trim_accept_ready(self):
        self.turnsign_trim_accept_ready = bool(
            self.turnsign_trim_line_split_ready
            and (self.turnsign_trim_stop_ready
                 or self.turnsign_trim_api_ready))

    def _start_turnsign_trim_pulse(self, now, direction):
        self.turnsign_trim_direction = self._sign(direction)
        self.turnsign_trim_settle_until = 0.0
        self.turnsign_trim_pulse_until = (
            float(now) + self.config.turnsign_reverse_duration_s)

    def _active_turnsign_trim_action(
            self, response, image_shape, path_x):
        direction = self._sign(self.turnsign_trim_direction)
        reason = (
            "turnsign_trim_forward" if direction > 0
            else "turnsign_trim_reverse")
        response["control_phase"] = reason
        target_x = self._turnsign_trim_target_x(
            image_shape, direction, path_x)
        self._publish_turnsign_trim_info(response)
        speed = abs(float(self.config.turnsign_reverse_speed_mps)) * direction
        return STATE_TRACK, speed, reason, target_x

    def _turnsign_trim_target_x(self, image_shape, direction, path_x):
        delta_640 = _finite_float(self.turnsign_trim_position_delta_640)
        width = float(max(1, image_shape[1]))
        if not self.turnsign_trim_current_fresh:
            # Do not keep steering toward a stale sign position after the
            # detector loses the sign. The longitudinal OCR pulse can continue
            # while lateral control remains on the selected path.
            return _clamp(
                float(path_x), 0.0, float(max(0, image_shape[1] - 1)))
        if (
            delta_640 is None
            or abs(delta_640) <=
                self.config.turnsign_trim_steer_deadband_px_640
        ):
            return _clamp(
                float(path_x), 0.0, float(max(0, image_shape[1] - 1)))
        steer_gain = self.config.turnsign_trim_steer_gain
        max_steer = self.config.turnsign_trim_max_steer_px_640
        correction_640 = (
            float(direction) * float(steer_gain)
            * float(delta_640))
        if correction_640 != 0.0:
            correction_640 = math.copysign(
                max(
                    abs(float(correction_640)),
                    float(self.config.turnsign_trim_min_steer_px_640)),
                correction_640)
        correction_640 = _clamp(
            correction_640, -float(max_steer), float(max_steer))
        return _clamp(
            float(path_x) + correction_640 * width / 640.0,
            0.0, float(max(0, image_shape[1] - 1)))

    def _remember_turnsign_position(self, response, phase, image_shape):
        center_x = _finite_float(response.get("bbox_center_x"))
        if center_x is None:
            geom = self._detection_geom(
                response.get("detection") or {}, image_shape)
            center_x = None if geom is None else float(geom["cx"])
        fresh_marker = response.get("current_detection_fresh")
        if fresh_marker is None:
            fresh_marker = response.get("detection_fresh")
        if fresh_marker is None:
            fresh_marker = not str(phase).startswith("turnsign_missing")
        is_fresh = bool(center_x is not None and fresh_marker)
        if is_fresh:
            width = float(max(1, image_shape[1]))
            visual_center = width * self.config.visual_center_x
            self.turnsign_last_seen_delta_640 = (
                (float(center_x) - visual_center) * 640.0 / width)
            try:
                confirmed_frames = int(response.get("confirm_count") or 0)
            except (TypeError, ValueError):
                confirmed_frames = 0
            self.turnsign_trim_fresh_frames = min(
                self.config.turnsign_trim_stable_frames,
                max(
                    self.turnsign_trim_fresh_frames + 1,
                    confirmed_frames))
        elif (
            self.turnsign_detection_was_fresh
            and self.turnsign_last_seen_delta_640 is not None
        ):
            self.turnsign_last_lost_delta_640 = float(
                self.turnsign_last_seen_delta_640)
        if (
            not is_fresh
            and self.turnsign_last_lost_delta_640 is None
            and self.turnsign_last_seen_delta_640 is not None
        ):
            self.turnsign_last_lost_delta_640 = float(
                self.turnsign_last_seen_delta_640)
        if not is_fresh:
            self.turnsign_trim_fresh_frames = 0
        self.turnsign_detection_was_fresh = is_fresh
        self.turnsign_trim_current_fresh = is_fresh
        self.turnsign_trim_current_centered = bool(
            is_fresh
            and self.turnsign_last_seen_delta_640 is not None
            and abs(float(self.turnsign_last_seen_delta_640)) <=
                self.config.turnsign_trim_center_deadband_px_640)
        self.turnsign_trim_position_delta_640 = (
            self.turnsign_last_seen_delta_640 if is_fresh
            else self.turnsign_last_lost_delta_640)
        self.turnsign_trim_stop_ready = bool(
            self.turnsign_trim_current_centered
            and self.turnsign_trim_fresh_frames >=
                self.config.turnsign_trim_stable_frames)

    def _publish_turnsign_trim_info(self, response):
        response["turnsign_trim_separation_640"] = (
            self.turnsign_trim_separation_640)
        response["turnsign_trim_lookahead_separation_640"] = (
            self.turnsign_trim_lookahead_separation_640)
        response["turnsign_trim_direction"] = int(
            self.turnsign_trim_direction)
        response["turnsign_trim_position_delta_640"] = (
            self.turnsign_trim_position_delta_640)
        response["turnsign_trim_fresh_frames"] = int(
            self.turnsign_trim_fresh_frames)
        response["turnsign_trim_current_centered"] = bool(
            self.turnsign_trim_current_centered)
        response["turnsign_trim_stop_ready"] = bool(
            self.turnsign_trim_stop_ready)
        response["turnsign_trim_line_split_ready"] = bool(
            self.turnsign_trim_line_split_ready)
        response["turnsign_trim_line_clear_current"] = bool(
            self.turnsign_trim_line_clear_current)
        response["turnsign_trim_line_ever_split"] = bool(
            self.turnsign_trim_line_ever_split)
        response["turnsign_trim_overshoot_latched"] = bool(
            self.turnsign_trim_overshoot_latched)
        response["turnsign_trim_accept_ready"] = bool(
            self.turnsign_trim_accept_ready)

    def _reset_turnsign_trim_runtime(self):
        self.turnsign_trim_pulse_until = 0.0
        self.turnsign_trim_direction = 0
        self.turnsign_trim_session_adaptive = False
        self.turnsign_trim_settle_until = 0.0
        self.turnsign_last_seen_delta_640 = None
        self.turnsign_last_lost_delta_640 = None
        self.turnsign_trim_position_delta_640 = None
        self.turnsign_detection_was_fresh = False
        self.turnsign_trim_fresh_frames = 0
        self.turnsign_trim_current_fresh = False
        self.turnsign_trim_current_centered = False
        self.turnsign_trim_stop_ready = False
        self.turnsign_trim_line_split_frames = 0
        self.turnsign_trim_line_collapse_frames = 0
        self.turnsign_trim_line_ever_split = False
        self.turnsign_trim_line_split_ready = False
        self.turnsign_trim_line_clear_current = False
        self.turnsign_trim_overshoot_latched = False
        self.turnsign_trim_max_lookahead_separation_640 = 0.0
        self.turnsign_trim_line_history_ts = None
        self.turnsign_trim_pending_ocr_direction = None
        self.turnsign_trim_api_ready = False
        self.turnsign_trim_accept_ready = False

    def _legacy_turnsign_action(
            self, detections, image_shape, response, now):
        has_active_ocr = bool(response.get("active"))
        has_sign = False
        should_stop = False
        for det in detections:
            if not self._is_turnsign_detection(det):
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.sign_slow_min_score:
                continue
            has_sign = True
            geom = self._detection_geom(det, image_shape)
            if geom is not None and self._sign_should_stop(
                    geom, image_shape):
                should_stop = True
        if has_sign:
            self.sign_seen_frames += 1
            if self.sign_seen_frames >= self.config.sign_latch_frames:
                if self.sign_latched_since is None:
                    self.sign_latched_since = float(now)
        else:
            self.sign_seen_frames = 0
        if self.sign_latched_since is not None and not has_sign:
            return STATE_SAFE_STOP, 0.0, "turnsign_stop", None
        if has_active_ocr and not has_sign:
            return STATE_SAFE_STOP, 0.0, "turnsign_stop", None
        if should_stop:
            return STATE_SAFE_STOP, 0.0, "turnsign_stop", None
        if has_sign or has_active_ocr:
            return (
                STATE_TRACK,
                min(
                    self.config.normal_speed_mps,
                    self.config.turnsign_slow_speed_mps),
                "turnsign_slow",
                None,
            )
        return None

    def _clear_turnsign_state(self):
        self._reset_turnsign_trim_runtime()
        self.turnsign_new_session_pending = False
        self.sign_seen_frames = 0
        self.sign_latched_since = None

    def _car_avoidance_action(
            self, detections, image_shape, lookahead_y, path_x, now):
        """Keep one continuous avoidance side across car and human phases."""
        if (
            self.car_human_pass_until > 0.0
            and float(now) >= self.car_human_pass_until
        ):
            self._clear_car_avoidance_state()
        car = self._best_car(detections, image_shape, path_x)
        pass_holding = float(now) < self.car_human_pass_until
        car_holding = float(now) < self.car_avoid_hold_until

        if car is not None:
            desired_target_x = self._avoid_target_x(
                "car", car, path_x, image_shape)
            desired_offset = float(desired_target_x) - float(path_x)
            desired_side = self._sign(desired_offset)
            new_sequence = (
                self.car_avoid_side == 0
                or (not car_holding
                    and not self.car_human_active
                    and not pass_holding)
            )
            if new_sequence:
                self._clear_car_avoidance_state()
                self.car_avoid_side = desired_side or 1
                self._clear_human_state()
                self.human_pass_offset_x = 0.0
                self.human_speed_hold_until = 0.0

            # Never cross the baseline while one avoidance sequence is active.
            # The magnitude may grow as the car box grows, but its sign remains
            # the originally selected side away from the car.
            scale = 640.0 / float(max(1, image_shape[1]))
            desired_magnitude_640 = abs(desired_offset) * scale
            self.car_avoid_offset_px_640 = max(
                self.car_avoid_offset_px_640,
                desired_magnitude_640,
            )
            self.car_avoid_hold_until = (
                float(now) + self.config.car_avoid_hold_s)
            car_holding = True

        context_active = bool(
            car is not None
            or car_holding
            or self.car_human_active
            or pass_holding)
        if not context_active or self.car_avoid_side == 0:
            self._clear_car_avoidance_state()
            return None

        avoid_target_x = self._latched_car_target_x(
            path_x, image_shape, now,
            full_hold=bool(
                car is not None or self.car_human_active or pass_holding))
        human = self._best_car_context_human(
            detections, image_shape)

        if pass_holding:
            return (
                STATE_AVOID_HUMAN,
                self.config.car_human_pass_speed_mps,
                "car_human_same_side_pass_hold",
                avoid_target_x,
            )

        if human is not None:
            geom = human["geom"]
            reached_line = self._human_on_stop_line(
                geom, image_shape, lookahead_y)
            if not self.car_human_active and not reached_line:
                preline_side = self._sign(
                    float(geom["cx"]) - float(avoid_target_x))
                if preline_side == self.car_avoid_side:
                    self.car_human_seen_avoid_side = True
                # A visible person before the stop line does not take control
                # away from the car route. Remember only the vertical gap so a
                # near-line detector dropout can trigger the 1.5 s safety wait.
                self._record_human_preline_gap(
                    geom, image_shape, lookahead_y)
                return (
                    STATE_AVOID_CAR,
                    self.config.normal_speed_mps,
                    "car_human_preline_approach",
                    avoid_target_x,
                )

            if not self.car_human_active:
                self.car_human_active = True
                self._clear_human_state()
                self.human_pass_offset_x = 0.0
                self.human_speed_hold_until = 0.0
            self._clear_human_preline_state()
            self.car_human_last_seen_ts = float(now)

            human_side = self._sign(
                float(geom["cx"]) - float(avoid_target_x))
            if human_side == self.car_avoid_side:
                self.car_human_seen_avoid_side = True

            crossed_to_other_side = (
                self.car_human_waiting_cross
                and self.car_human_seen_avoid_side
                and human_side != 0
                and human_side == -self.car_avoid_side
            )
            if crossed_to_other_side:
                self.car_human_pass_until = (
                    float(now) + self.config.car_human_pass_hold_s)
                self.car_human_waiting_cross = False
                return (
                    STATE_AVOID_HUMAN,
                    self.config.car_human_pass_speed_mps,
                    "car_human_same_side_pass",
                    avoid_target_x,
                )

            if reached_line:
                self.car_human_waiting_cross = True
            if self.car_human_waiting_cross:
                return (
                    STATE_SAFE_STOP,
                    0.0,
                    "car_human_same_side_wait",
                    avoid_target_x,
                )

        if self.car_human_active:
            absence_age = max(
                0.0, float(now) - self.car_human_last_seen_ts)
            if absence_age < self.config.human_absence_confirm_s:
                return (
                    STATE_SAFE_STOP,
                    0.0,
                    "car_human_absence_check",
                    avoid_target_x,
                )

            # No Human has been detected for the complete confirmation window.
            # Release only the pedestrian part of the state; a remaining car
            # hold still owns the same avoidance side until its own 2 s expires.
            self.car_human_active = False
            self.car_human_waiting_cross = False
            self.car_human_seen_avoid_side = False
            self.car_human_last_seen_ts = 0.0
            if car is None and not car_holding:
                self._clear_car_avoidance_state()
                return None

        elif self._human_preline_missing_waiting(now):
            return (
                STATE_SAFE_STOP,
                0.0,
                "car_human_preline_absence_check",
                avoid_target_x,
            )

        reason = "car_in_path_bias" if car is not None else "car_avoid_hold"
        return (
            STATE_AVOID_CAR,
            self.config.normal_speed_mps,
            reason,
            avoid_target_x,
        )

    def _best_car(self, detections, image_shape, path_x):
        hazard_limit = (
            float(image_shape[1]) * self.config.hazard_lateral_ratio)
        best = None
        best_rank = -1.0
        for det in detections:
            if self._normalized_label(det) != "car":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_car_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if (
                geom is None
                or geom["bottom_ratio"] < self.config.hazard_bottom_ratio
                or abs(float(geom["cx"]) - float(path_x))
                > max(hazard_limit, float(geom["box_w"]) * 0.75)
            ):
                continue
            rank = (
                float(score)
                + float(geom["bottom_ratio"])
                - abs(float(geom["cx"]) - float(path_x))
                / float(max(1, image_shape[1]))
            )
            if rank > best_rank:
                best = geom
                best_rank = rank
        return best

    def _best_car_context_human(self, detections, image_shape):
        best = None
        best_rank = (-1.0, -1.0)
        for det in detections:
            if self._normalized_label(det) != "human":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_human_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None:
                continue
            # Vertical progress is primary: a lower-confidence person already
            # at the line must beat a higher-confidence person farther away.
            rank = (float(geom["bottom_ratio"]), float(score))
            if rank > best_rank:
                best = {"geom": geom, "score": score}
                best_rank = rank
        return best

    def _latched_car_target_x(
            self, path_x, image_shape, now, full_hold=False):
        scale = float(max(1, image_shape[1])) / 640.0
        hold_scale = 1.0
        if not full_hold and self.config.car_avoid_hold_s > 0.0:
            hold_scale = _clamp(
                (float(self.car_avoid_hold_until) - float(now)) /
                float(self.config.car_avoid_hold_s),
                0.0, 1.0)
        offset = (
            float(self.car_avoid_side)
            * float(self.car_avoid_offset_px_640)
            * float(hold_scale)
            * scale)
        return _clamp(
            float(path_x) + offset,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )

    def _clear_car_avoidance_state(self):
        self.car_avoid_side = 0
        self.car_avoid_offset_px_640 = 0.0
        self.car_avoid_hold_until = 0.0
        self.car_human_active = False
        self.car_human_waiting_cross = False
        self.car_human_seen_avoid_side = False
        self.car_human_last_seen_ts = 0.0
        self.car_human_pass_until = 0.0

    def _human_action(
            self, detections, selected, image_shape, lookahead_y,
            lookahead_path_x, now):
        humans = []
        for det in detections:
            if self._normalized_label(det) != "human":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_human_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None:
                continue
            path_x = _interp_path_x(selected["points_xy"], geom["cy"])
            if path_x is None:
                path_x = lookahead_path_x
            distance = float(geom["cx"]) - float(path_x)
            side = self._sign(distance)
            humans.append({
                "geom": geom,
                "score": score,
                "path_x": path_x,
                "distance": distance,
                "side": side,
            })
        if not humans:
            if self.human_pass_active:
                self._clear_human_state()
                return None
            if self.human_detected_latched:
                absence_age = max(
                    0.0, float(now) - self.human_last_seen_ts)
                if absence_age < self.config.human_absence_confirm_s:
                    # A person leaving the image is ambiguous. Require one
                    # continuous 1.5 s interval with no Human detections before
                    # treating the scene as clear.
                    return (
                        STATE_SAFE_STOP,
                        0.0,
                        "human_absence_check",
                        None,
                    )
            elif self._human_preline_missing_waiting(now):
                return (
                    STATE_SAFE_STOP,
                    0.0,
                    "human_preline_absence_check",
                    None,
                )
            self._clear_human_state()
            return None

        human = max(
            humans,
            key=lambda item: (
                float(item["geom"]["bottom_ratio"]),
                float(item["score"]),
            ),
        )
        geom = human["geom"]
        side = human["side"] or self.human_last_side or 1

        if self.human_pass_active:
            return self._human_pass_command(lookahead_path_x, image_shape, side)

        crossed = (
            self.human_waiting_cross
            and human["side"] != 0
            and self.human_last_side is not None
            and human["side"] != self.human_last_side
        )
        if crossed:
            self.human_waiting_cross = False
            self.human_pass_active = True
            self.human_detected_latched = False
            self.human_last_seen_ts = 0.0
            return self._human_pass_command(lookahead_path_x, image_shape, side)

        if self._human_on_stop_line(geom, image_shape, lookahead_y):
            self._clear_human_preline_state()
            self.human_detected_latched = True
            self.human_last_seen_ts = float(now)
            self.human_waiting_cross = True
            if human["side"] != 0:
                self.human_last_side = human["side"]
            return STATE_SAFE_STOP, 0.0, "human_half_lookahead_stop", None

        if self.human_waiting_cross:
            self.human_last_seen_ts = float(now)
            return STATE_SAFE_STOP, 0.0, "human_wait_cross", None

        self._record_human_preline_gap(geom, image_shape, lookahead_y)
        return None

    def _human_pass_command(self, path_x, image_shape, human_side):
        self.human_path_return_guard_until = 0.0
        self.human_path_return_guard_start_x = None
        scale = float(max(1, image_shape[1])) / 640.0
        offset = self.config.human_pass_offset_px_640 * scale
        target_x = _clamp(
            float(path_x) - float(human_side or 1) * offset,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        self.human_pass_offset_x = float(target_x) - float(path_x)
        self.human_path_return_pending = True
        self.human_last_avoid_target_x = float(target_x)
        return STATE_AVOID_HUMAN, self.config.human_pass_speed_mps, "human_cross_pass", target_x

    def _human_on_stop_line(self, geom, image_shape, lookahead_y):
        stop_y = self._human_stop_line_y(image_shape, lookahead_y)
        line_margin = float(image_shape[0]) * self.config.human_stop_line_margin_ratio
        # People approach this horizontal line from the top of the image. Once
        # the bottom edge reaches or passes it, the crossing must be handled
        # even when the detector skipped the exact contact frame.
        return float(geom["bottom"]) + line_margin >= stop_y

    def _human_stop_line_y(self, image_shape, lookahead_y):
        return float(image_shape[0]) - (
            float(image_shape[0]) - float(lookahead_y)
        ) * self.config.human_stop_progress_ratio

    def _record_human_preline_gap(self, geom, image_shape, lookahead_y):
        stop_y = self._human_stop_line_y(image_shape, lookahead_y)
        gap = max(0.0, stop_y - float(geom["bottom"]))
        self.human_preline_last_gap_px_480 = (
            gap * 480.0 / float(max(1, image_shape[0])))
        # Reappearance before the line immediately releases a dropout wait.
        self.human_preline_wait_until = 0.0

    def _human_preline_missing_waiting(self, now):
        gap = self.human_preline_last_gap_px_480
        if gap is None or gap > self.config.human_preline_missing_px_480:
            self._clear_human_preline_state()
            return False
        if self.human_preline_wait_until <= 0.0:
            self.human_preline_wait_until = (
                float(now) + self.config.human_absence_confirm_s)
        if float(now) < self.human_preline_wait_until:
            return True
        self._clear_human_preline_state()
        return False

    def _clear_human_preline_state(self):
        self.human_preline_last_gap_px_480 = None
        self.human_preline_wait_until = 0.0

    def _clear_human_state(self, clear_detection=True):
        self.human_waiting_cross = False
        self.human_last_side = None
        self.human_pass_active = False
        if clear_detection:
            self.human_detected_latched = False
            self.human_last_seen_ts = 0.0
            self._clear_human_preline_state()

    def _sign_should_stop(self, geom, image_shape):
        area_ratio = geom.get("area_ratio")
        if area_ratio is None:
            area_ratio = (float(geom["box_w"]) * float(geom["box_h"])) / float(max(1, image_shape[0] * image_shape[1]))
        # TurnSign approach distance is represented by its apparent size. Once
        # it reaches the calibrated Preview size, stop immediately and wait for
        # a current left/right API result; do not also require it to intersect
        # the path lookahead row. Keep this area gate identical to the OCR
        # snapshot gate so stopping and OCR submission happen on the same frame.
        return float(area_ratio) >= self.config.sign_stop_area_ratio

    def _best_coin(self, detections, image_shape, path_x):
        best = None
        best_rank = -1.0
        for det in detections:
            label = str(det.get("label") or det.get("category") or "").lower()
            if label != "coin":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_coin_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None or geom["bottom_ratio"] < self.config.coin_bottom_ratio:
                continue
            lateral = abs(geom["cx"] - path_x) / float(max(1, image_shape[1]))
            if lateral > self.config.coin_lateral_ratio:
                continue
            rank = score + geom["bottom_ratio"] - lateral
            if rank > best_rank:
                best = geom
                best_rank = rank
        return best

    def _avoid_target_x(self, label, geom, path_x, image_shape):
        scale = float(max(1, image_shape[1])) / 640.0
        base_offset = (
            self.config.human_avoid_offset_px_640
            if label == "human"
            else self.config.car_avoid_offset_px_640
        )
        offset = max(
            float(base_offset) * scale,
            float(geom["box_w"]) * self.config.avoid_box_width_gain)
        if label == "car":
            offset *= float(self.config.car_avoid_steer_gain)
            offset = min(
                offset,
                float(self.config.car_avoid_max_offset_px_640) * scale)
        side = 1.0 if geom["cx"] <= path_x else -1.0
        return _clamp(float(path_x) + side * offset, 0.0, float(max(0, image_shape[1] - 1)))

    def _coin_target_x(self, geom, path_x, image_shape):
        scale = float(max(1, image_shape[1])) / 640.0
        max_bias = self.config.gold_max_bias_px_640 * scale
        bias = _clamp((float(geom["cx"]) - float(path_x)) * self.config.gold_bias_gain, -max_bias, max_bias)
        return _clamp(float(path_x) + bias, 0.0, float(max(0, image_shape[1] - 1)))

    @staticmethod
    def _detection_geom(det, image_shape):
        bbox = det.get("bbox") or det.get("box_xyxy")
        center = det.get("center")
        size = det.get("size")
        if bbox and len(bbox) >= 4:
            try:
                left, top, right, bottom = [float(value) for value in bbox[:4]]
            except (TypeError, ValueError):
                return None
        elif center and size and len(center) >= 2 and len(size) >= 2:
            try:
                cx, cy = [float(value) for value in center[:2]]
                box_w, box_h = [float(value) for value in size[:2]]
            except (TypeError, ValueError):
                return None
            left = cx - box_w * 0.5
            right = cx + box_w * 0.5
            top = cy - box_h * 0.5
            bottom = cy + box_h * 0.5
        else:
            return None
        if right <= left or bottom <= top:
            return None
        return {
            "left": left,
            "top": top,
            "right": right,
            "bottom": bottom,
            "cx": (left + right) * 0.5,
            "cy": (top + bottom) * 0.5,
            "box_w": right - left,
            "box_h": bottom - top,
            "bottom_ratio": bottom / float(max(1, image_shape[0])),
            "area_ratio": _finite_float(det.get("area_ratio")),
        }

    @staticmethod
    def _normalized_label(det):
        label = str(det.get("label") or det.get("category") or det.get("class_name") or "")
        return label.strip().lower().replace("_", "").replace("-", "").replace(" ", "")

    @staticmethod
    def _sign(value):
        value = float(value)
        if value > 0.0:
            return 1
        if value < 0.0:
            return -1
        return 0

    def _is_turnsign_detection(self, det):
        return self._normalized_label(det) == "turnsign"

    def _reset_track_error_response(self, reason="fixed_step"):
        self.track_error_trend_sign = 0
        self.track_error_trend_frames = 0
        self.track_error_response = str(reason)

    def _shape_track_error(self, error, curve_mode=False):
        """Use quiet straight response and full confirmed-curve response."""
        limit = float(self.config.max_track_error_640)
        error = _clamp(float(error), -limit, limit)
        magnitude = abs(error)
        if curve_mode:
            return float(error)

        deadband = float(self.config.track_deadband_px_640)
        small_limit = max(
            deadband + 1.0, float(self.config.track_small_error_px_640))
        fast_limit = max(small_limit + 1.0,
                         float(self.config.track_fast_error_px_640))
        if magnitude <= deadband:
            return 0.0
        # Ramp the gain from the straight-line noise gain to unity.  This
        # keeps a 5-15 px straight-line fluctuation small while retaining
        # nearly all of a substantial curve error.
        normalized = _clamp(
            (magnitude - deadband) / (fast_limit - deadband), 0.0, 1.0)
        gain = (
            float(self.config.track_small_error_gain) * (1.0 - normalized)
            + normalized)
        shaped_magnitude = min(limit, (magnitude - deadband) * gain)
        return float(math.copysign(shaped_magnitude, error))

    def _limit_error(self, error, adaptive=False, curve_mode=False):
        """Apply a stable straight response and a fast coherent bend response."""
        clamped = _clamp(
            float(error), -self.config.max_track_error_640,
            self.config.max_track_error_640)
        if not adaptive:
            self._reset_track_error_response()
            delta = _clamp(
                clamped - self.last_error,
                -self.config.max_error_step_640,
                self.config.max_error_step_640)
            return float(self.last_error + delta)

        shaped = self._shape_track_error(
            clamped, curve_mode=bool(curve_mode))
        sign = self._sign(shaped)
        previous_sign = int(self.track_error_trend_sign)
        same_direction = sign != 0 and sign == previous_sign
        if same_direction:
            self.track_error_trend_frames += 1
        else:
            self.track_error_trend_frames = 1 if sign else 0
        self.track_error_trend_sign = sign
        base_step = float(self.config.max_error_step_640)
        magnitude = abs(shaped)
        last_magnitude = abs(float(self.last_error))
        is_reversal = (
            sign != 0 and self._sign(self.last_error) != 0 and
            sign != self._sign(self.last_error))
        is_unwinding = (
            magnitude < last_magnitude and
            self._sign(self.last_error) in {0, sign})
        if is_unwinding and last_magnitude > self.config.track_small_error_px_640:
            step = base_step * 1.5
            mode = "fast_unwind"
        elif is_reversal:
            step = base_step * self.config.track_reverse_step_scale
            mode = "reverse_guard"
        elif curve_mode:
            step = base_step * self.config.curve_track_step_scale
            mode = "curve_track"
        elif (same_direction and self.track_error_trend_frames >= 2 and
              magnitude >= self.config.track_fast_error_px_640):
            step = base_step * self.config.track_fast_step_scale
            mode = "fast_curve"
        elif magnitude <= self.config.track_small_error_px_640:
            step = base_step * 0.35
            mode = "straight_damping"
        else:
            step = base_step
            mode = "normal_track"
        self.track_error_response = mode
        # A sign change after one noisy frame is deliberately restrained.  A
        # persistent opposite-side error gets its normal response on the
        # following frame through the trend counter.
        delta = _clamp(shaped - self.last_error, -step, step)
        return float(self.last_error + delta)

    @staticmethod
    def _command(error, speed, state, flags=CONTROL_FLAG_USE_TARGET_SPEED):
        if int(state) in {STATE_SAFE_STOP, STATE_LINE_LOSS_SAFE_STOP} and float(speed) == 0.0:
            flags = 0
        return {
            "track_error": float(error),
            "target_speed": float(speed),
            "state_cmd": int(state),
            "flags": int(flags),
            "safe_stop": int(state) in {STATE_SAFE_STOP, STATE_LINE_LOSS_SAFE_STOP} and float(speed) == 0.0,
        }

    @staticmethod
    def _extract_ocr_direction(response):
        response = response if isinstance(response, dict) else {}
        instruction = response.get("instruction") or response.get("latest_instruction") or {}
        direction = str(instruction.get("direction") or instruction.get("preferred_branch") or "").lower()
        if bool(response.get("instruction_current")) and direction in {"left", "right"}:
            return direction, True
        return None, False

    def _ocr_has_current_direction(self):
        return bool(self.ocr_confirmed_current)

    def _confirm_ocr_direction(self, direction, is_current):
        if not is_current or direction not in {"left", "right"}:
            self.ocr_pending_direction = None
            self.ocr_pending_frames = 0
            self.ocr_confirmed_current = False
            return None, False
        if direction == self.ocr_pending_direction:
            self.ocr_pending_frames += 1
        else:
            self.ocr_pending_direction = direction
            self.ocr_pending_frames = 1
        self.ocr_confirmed_current = (
            self.ocr_pending_frames >= self.config.ocr_confirm_frames)
        if not self.ocr_confirmed_current:
            return None, False
        return direction, True

    def _apply_ocr_direction_lock(self, direction):
        if direction not in {"left", "right"}:
            return
        self.branch_lock = direction
        self.branch_lock_source = "ocr"
        self.selected_slot_lock = 0 if direction == "left" else 1

    def _expire_ocr_lock(self, now):
        if self.branch_lock_source != "ocr" or self.last_valid_ocr_ts <= 0.0:
            return False
        if now - self.last_valid_ocr_ts < self.config.ocr_lock_lifetime_s:
            return False
        self._set_default_curve_branch()
        return True

    def _set_default_curve_branch(self):
        self.branch_lock = "left"
        self.branch_lock_source = "default"
        self.selected_slot_lock = 0

    def _ocr_lock_age(self, now):
        if self.branch_lock_source != "ocr" or self.last_valid_ocr_ts <= 0.0:
            return None
        return max(0.0, now - self.last_valid_ocr_ts)

    def _ocr_lock_remaining(self, now):
        age = self._ocr_lock_age(now)
        if age is None:
            return None
        return max(0.0, self.config.ocr_lock_lifetime_s - age)


    @staticmethod
    def _image_shape(result):
        frame = result.get("_source_frame")
        if frame is None:
            frame = result.get("frame")
        if frame is not None and hasattr(frame, "shape") and len(frame.shape) >= 2:
            return frame.shape
        shape = result.get("image_shape") or result.get("frame_shape")
        if isinstance(shape, (list, tuple)) and len(shape) >= 2:
            return int(shape[0]), int(shape[1]), 3
        return 480, 640, 3

    def _path_pair_stats(self, first_points, second_points, image_shape):
        first = np.asarray(first_points, dtype=np.float32)
        second = np.asarray(second_points, dtype=np.float32)
        if len(first) == 0 or len(second) == 0:
            return {"mean_distance_640": 1e9, "separated_rows": 0}
        low = max(float(np.min(first[:, 1])), float(np.min(second[:, 1])))
        high = min(float(np.max(first[:, 1])), float(np.max(second[:, 1])))
        if low > high:
            return {"mean_distance_640": 1e9, "separated_rows": 0}
        rows = low + (high - low) * _PAIR_FRACTIONS
        first_x = _interp_path_x_many(first, rows)
        second_x = _interp_path_x_many(second, rows)
        valid = np.isfinite(first_x) & np.isfinite(second_x)
        if not np.any(valid):
            return {"mean_distance_640": 1e9, "separated_rows": 0}
        distances = (
            np.abs(first_x[valid] - second_x[valid]) * 640.0 /
            float(max(1, image_shape[1])))
        return {
            "mean_distance_640": float(np.mean(distances)),
            "separated_rows": int(np.count_nonzero(distances >= self.config.branch_separation_px_640)),
        }

    @staticmethod
    def _summarize_candidate(candidate, image_shape):
        points = np.asarray(candidate.get("points_xy"), dtype=np.float32)
        road_support = _finite_float(candidate.get("road_support"))
        return {
            "slot": int(candidate.get("slot", -1)),
            "role": str(candidate.get("role") or ""),
            "source": str(candidate.get("source") or "unknown"),
            "points": int(len(points)),
            "score": float(candidate.get("score", 0.0)),
            "coverage": float(candidate.get("coverage", 0.0)),
            "road_support": road_support,
            "exit_type": str(candidate.get("exit_type") or ""),
            "occlusion_bridge_rows": int(candidate.get(
                "occlusion_bridge_rows", 0)),
            "curve_fit_rmse": float(candidate.get(
                "curve_fit_rmse", 0.0)),
            "human_occlusion_hold": bool(candidate.get(
                "human_occlusion_hold", False)),
            "near_x": None if len(points) == 0 else float(points[np.argmax(points[:, 1]), 0]),
            "lookahead_x": _interp_path_x(points, image_shape[0] * 0.62),
        }

    @staticmethod
    def _debug_path_points(candidate):
        points = np.asarray(candidate.get("points_xy"), dtype=np.float32)
        return VisionControlPlanner._debug_points(points)

    @staticmethod
    def _debug_points(points):
        points = np.asarray(points, dtype=np.float32)
        if len(points) == 0:
            return []
        stride = max(1, int(math.ceil(len(points) / 48.0)))
        return [[float(x), float(y)] for x, y in points[::stride]]


def render_vision_control_debug(frame, result):
    if frame is None or result is None:
        return frame
    debug = (result.get("vision_control") if isinstance(result, dict) else None) or {}
    if not debug:
        return frame
    h, w = frame.shape[:2]
    selected_slot = debug.get("selected_slot")
    center_x = int(round(w * _clamp(_env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8)))
    cv2.line(frame, (center_x, int(h * 0.45)), (center_x, h - 1), (210, 210, 210), 1, cv2.LINE_AA)

    target = debug.get("control_target") or {}
    path_target_x = _finite_float(target.get("path_target_x"))
    target_x = _finite_float(target.get("target_x"), path_target_x)
    lookahead_y = _finite_float(
        target.get("path_target_y"),
        _finite_float(target.get("lookahead_y"), h * 0.62))
    confidence_split_y = _clamp(
        float(lookahead_y) - 10.0, 0.0, float(max(0, h - 1)))
    lower_confidence_boost = _clamp(_env_float(
        "VISION_CONTROL_POINT_LOWER_CONFIDENCE_BOOST", 0.35), 0.0, 1.0)
    upper_confidence_decay = _clamp(_env_float(
        "VISION_CONTROL_POINT_UPPER_CONFIDENCE_DECAY", 0.55), 0.0, 1.0)

    road_mask = (result.get("road") or {}).get("mask")
    if road_mask is None:
        road_mask = result.get("road_mask")
    for line in _extract_curve_preview_lines(result):
        points = np.rint(line["points_xy"]).astype(np.int32)
        if len(points) < 2:
            continue
        points[:, 0] = np.clip(points[:, 0], 0, w - 1)
        points[:, 1] = np.clip(points[:, 1], 0, h - 1)
        probabilities = _adjust_point_display_confidences(
            points, line.get("probabilities"), confidence_split_y,
            lower_confidence_boost, upper_confidence_decay)
        if len(points) < 2:
            continue
        visible = _semantic_road_point_mask(
            points, road_mask, frame.shape)
        _draw_identity_probability_curve(
            frame, points, probabilities,
            int(line.get("slot", 0)),
            thickness=3 if int(line.get("slot", -1)) == selected_slot else 2,
            visible_mask=visible)

    split_y = int(round(confidence_split_y))
    cv2.line(
        frame, (0, split_y), (w - 1, split_y),
        (0, 165, 255), 1, cv2.LINE_AA)
    cv2.putText(
        frame, "POINT CONF", (4, max(14, split_y - 4)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1,
        cv2.LINE_AA)
    human_stop_line_y = _finite_float(target.get("human_stop_line_y"))
    if human_stop_line_y is not None:
        stop_y = int(round(_clamp(human_stop_line_y, 0, h - 1)))
        cv2.line(
            frame, (0, stop_y), (w - 1, stop_y),
            (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(
            frame, "HUMAN STOP", (max(4, w - 145), max(14, stop_y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1,
            cv2.LINE_AA)
    if target_x is not None:
        x = int(round(_clamp(target_x, 0, w - 1)))
        y = int(round(_clamp(lookahead_y, 0, h - 1)))
        if path_target_x is not None and abs(float(target_x) - path_target_x) >= 0.5:
            base_x = int(round(_clamp(path_target_x, 0, w - 1)))
            cv2.circle(
                frame, (base_x, y), 5, (255, 255, 255), 1,
                cv2.LINE_AA)
            cv2.line(
                frame, (base_x, y), (x, y), (0, 165, 255), 2,
                cv2.LINE_AA)
        cv2.circle(frame, (x, y), 7, (255, 0, 255), -1, cv2.LINE_AA)
        cv2.line(frame, (0, y), (w - 1, y), (255, 0, 255), 1, cv2.LINE_AA)

    command = debug.get("command") or {}
    merge_tag = " MERGE->G" if debug.get("curve_merge_override") else ""
    text = "{} raw={} lock={}/{} slot={}{} err={:.1f}".format(
        debug.get("route_state", "-"),
        debug.get("raw_route_state", "-"),
        debug.get("branch_lock") or "-",
        "-" if debug.get("selected_slot_lock") is None else debug.get("selected_slot_lock"),
        "-" if selected_slot is None else selected_slot,
        merge_tag,
        float(command.get("track_error", 0.0)),
    )
    cv2.putText(frame, text, (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2, cv2.LINE_AA)
    detected_count = int(debug.get(
        "detected_path_count", 1))
    line_loss_reason = str(
        debug.get("line_loss_reason")
        or debug.get("line_connection_reason")
        or "")
    line_status = {
        "no_candidate": "NO_LINE",
        "short_near_support": "SHORT",
        "detached_near_anchor": "DETACHED",
        "invalid_path": "INVALID",
        "no_selected_route": "NO_ROUTE",
    }.get(line_loss_reason, "OK")
    line_color = (
        (0, 0, 255) if debug.get("line_loss_active")
        else (30, 230, 255))
    road_shape = str(
        debug.get("road_shape_state") or "straight").upper()
    cv2.putText(
        frame, "PATH: {}  LINE: {}  ROAD: {}".format(
            detected_count, line_status, road_shape),
        (10, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
        line_color, 2, cv2.LINE_AA)
    return frame


def _identity_probability_color(slot, probability):
    intensity = int(round(80.0 + 175.0 * _clamp(probability, 0.0, 1.0)))
    # OpenCV uses BGR: left is always blue, right is always green.
    return (intensity, 0, 0) if int(slot) == 0 else (0, intensity, 0)


def _draw_identity_probability_curve(
        frame, points, probabilities, slot, thickness=2,
        visible_mask=None):
    points = np.rint(np.asarray(points, dtype=np.float32)).astype(np.int32)
    probabilities = np.asarray(probabilities, dtype=np.float32)
    if len(probabilities) != len(points):
        probabilities = np.ones(len(points), dtype=np.float32)
    if visible_mask is None:
        visible = np.ones(len(points), dtype=bool)
    else:
        visible = np.asarray(visible_mask, dtype=bool)
        if len(visible) != len(points):
            visible = np.ones(len(points), dtype=bool)
    line_thickness = max(1, int(thickness))
    for index in range(len(points) - 1):
        if not (visible[index] and visible[index + 1]):
            continue
        start = tuple(int(value) for value in points[index])
        end = tuple(int(value) for value in points[index + 1])
        probability = 0.5 * float(
            probabilities[index] + probabilities[index + 1])
        color = _identity_probability_color(slot, probability)
        cv2.line(
            frame, start, end, (0, 0, 0), line_thickness + 2,
            cv2.LINE_AA)
        cv2.line(
            frame, start, end, color, line_thickness, cv2.LINE_AA)


def _birdseye_debug_default_source_quad():
    top_y = _clamp(
        _env_float("AR_BIRDSEYE_SRC_TOP_Y", 0.52),
        0.05, 0.85)
    bottom_y = _clamp(
        _env_float("AR_BIRDSEYE_SRC_BOTTOM_Y", 0.95),
        top_y + 0.05, 1.0)
    top_left_x = _clamp(
        _env_float("AR_BIRDSEYE_SRC_TOP_LEFT_X", 0.44), 0.0, 0.90)
    top_right_x = _clamp(
        _env_float("AR_BIRDSEYE_SRC_TOP_RIGHT_X", 0.56),
        top_left_x + 0.02, 1.0)
    bottom_left_x = _clamp(
        _env_float("AR_BIRDSEYE_SRC_BOTTOM_LEFT_X", 0.22), 0.0, 0.90)
    bottom_right_x = _clamp(
        _env_float("AR_BIRDSEYE_SRC_BOTTOM_RIGHT_X", 0.78),
        bottom_left_x + 0.02, 1.0)
    return np.asarray([
        [top_left_x, top_y],
        [top_right_x, top_y],
        [bottom_right_x, bottom_y],
        [bottom_left_x, bottom_y],
    ], dtype=np.float32)


def _birdseye_debug_homography(
        image_shape, source_quad_normalized=None):
    """Build the display-only image-to-ground homography for the preview."""
    height, width = image_shape[:2]
    width_scale = float(max(1, width - 1))
    height_scale = float(max(1, height - 1))

    source_normalized = np.asarray(
        (_birdseye_debug_default_source_quad()
         if source_quad_normalized is None else source_quad_normalized),
        dtype=np.float32)
    if source_normalized.shape != (4, 2):
        source_normalized = _birdseye_debug_default_source_quad()
    source_normalized = np.clip(source_normalized, 0.0, 1.0)
    destination_left_x = _clamp(
        _env_float("AR_BIRDSEYE_DST_LEFT_X", 0.18), 0.0, 0.45)
    destination_right_x = _clamp(
        _env_float("AR_BIRDSEYE_DST_RIGHT_X", 0.82),
        destination_left_x + 0.10, 1.0)
    destination_top_y = _clamp(
        _env_float("AR_BIRDSEYE_DST_TOP_Y", 0.02), 0.0, 0.30)
    destination_bottom_y = _clamp(
        _env_float("AR_BIRDSEYE_DST_BOTTOM_Y", 0.98),
        destination_top_y + 0.20, 1.0)

    source = source_normalized * np.asarray(
        [width_scale, height_scale], dtype=np.float32)
    destination = np.asarray([
        [destination_left_x * width_scale,
         destination_top_y * height_scale],
        [destination_right_x * width_scale,
         destination_top_y * height_scale],
        [destination_right_x * width_scale,
         destination_bottom_y * height_scale],
        [destination_left_x * width_scale,
         destination_bottom_y * height_scale],
    ], dtype=np.float32)
    return cv2.getPerspectiveTransform(source, destination), source, destination


def _birdseye_debug_result_road_mask(result):
    if not isinstance(result, dict):
        return None
    road = result.get("road") or {}
    road_mask = road.get("mask") if isinstance(road, dict) else None
    if road_mask is None:
        road_mask = result.get("road_mask")
    road_mask = np.asarray(
        road_mask) if road_mask is not None else np.empty((0, 0))
    if road_mask.ndim != 2 or road_mask.size == 0:
        return None
    return (road_mask != 0).astype(np.uint8)


def _birdseye_debug_path_reference_x(result, image_y, image_width):
    debug = (
        (result.get("vision_control") if isinstance(result, dict) else None)
        or {})
    selected_path = debug.get("selected_path")
    if selected_path is not None:
        points = np.asarray(selected_path, dtype=np.float32)
        if points.ndim == 2 and points.shape[1:] == (2,) and len(points) >= 2:
            path_x = _interp_path_x(points, float(image_y))
            if path_x is not None:
                return _clamp(path_x, 0.0, float(max(0, image_width - 1)))
    target = debug.get("control_target") or {}
    target_x = _finite_float(
        target.get("path_target_x"), target.get("target_x"))
    if target_x is not None:
        return _clamp(target_x, 0.0, float(max(0, image_width - 1)))
    return 0.5 * float(max(0, image_width - 1))


def _birdseye_debug_mask_run(
        road_mask, row, reference_x, band_rows=2, search_rows=8):
    height, width = road_mask.shape
    offsets = [0]
    for distance in range(1, max(0, int(search_rows)) + 1):
        offsets.extend((-distance, distance))
    for offset in offsets:
        center_row = int(np.clip(int(row) + offset, 0, height - 1))
        row_start = max(0, center_row - max(0, int(band_rows)))
        row_end = min(height, center_row + max(0, int(band_rows)) + 1)
        occupied = np.any(road_mask[row_start:row_end] != 0, axis=0)
        padded = np.pad(occupied.astype(np.int8), (1, 1))
        transitions = np.diff(padded)
        starts = np.flatnonzero(transitions == 1)
        ends = np.flatnonzero(transitions == -1) - 1
        runs = [
            (int(start), int(end))
            for start, end in zip(starts, ends)
            if end - start + 1 >= 2
        ]
        if not runs:
            continue

        def run_distance(run):
            start, end = run
            if start <= reference_x <= end:
                return 0.0
            return min(abs(reference_x - start), abs(reference_x - end))

        return min(
            runs,
            key=lambda run: (
                run_distance(run),
                abs(0.5 * (run[0] + run[1]) - reference_x),
                -(run[1] - run[0])),
        )
    return None


def _birdseye_debug_mask_vertical_span(road_mask):
    """Return the stable occupied-row span of the current road component."""
    road_mask = (np.asarray(road_mask) != 0).astype(np.uint8)
    if road_mask.ndim != 2 or not road_mask.size:
        return None
    height, width = road_mask.shape
    row_counts = np.count_nonzero(road_mask, axis=1)
    minimum_count = max(2, int(round(width * 0.02)))
    valid = row_counts >= minimum_count
    if not np.any(valid):
        return None
    # Bridge one- or two-row segmentation holes before selecting a span.
    valid = np.convolve(valid.astype(np.int8), [1, 1, 1], mode="same") >= 2
    padded = np.pad(valid.astype(np.int8), (1, 1))
    transitions = np.diff(padded)
    starts = np.flatnonzero(transitions == 1)
    ends = np.flatnonzero(transitions == -1) - 1
    spans = [
        (int(start), int(end))
        for start, end in zip(starts, ends)
        if end - start + 1 >= 4
    ]
    if not spans:
        return None
    # Prefer the longest coherent component; ties go to the lower span,
    # which is the incoming road rather than a small far-field speck.
    start, end = max(spans, key=lambda item: (
        item[1] - item[0] + 1, item[1]))
    if end - start < max(8, int(round(height * 0.12))):
        return None
    return (
        float(start) / float(max(1, height - 1)),
        float(end) / float(max(1, height - 1)),
    )


def _birdseye_debug_source_quad_from_road(result, image_shape):
    """Fit the display ROI to the current semantic-road geometry."""
    if not _env_bool("AR_BIRDSEYE_FOLLOW_ROAD_MASK", True):
        return None
    road_mask = _birdseye_debug_result_road_mask(result)
    if road_mask is None or not np.any(road_mask):
        return None
    image_height, image_width = image_shape[:2]
    mask_height, mask_width = road_mask.shape
    vertical_span = _birdseye_debug_mask_vertical_span(road_mask)
    if vertical_span is None:
        return None
    top_y, bottom_y = vertical_span
    band_rows = max(0, _env_int("AR_BIRDSEYE_MASK_BAND_ROWS", 2))
    search_rows = max(0, _env_int("AR_BIRDSEYE_MASK_SEARCH_ROWS", 8))

    def find_run(y_normalized):
        image_y = y_normalized * float(max(1, image_height - 1))
        reference_image_x = _birdseye_debug_path_reference_x(
            result, image_y, image_width)
        reference_mask_x = (
            reference_image_x * float(max(1, mask_width - 1)) /
            float(max(1, image_width - 1)))
        mask_y = int(round(
            y_normalized * float(max(1, mask_height - 1))))
        return _birdseye_debug_mask_run(
            road_mask, mask_y, reference_mask_x,
            band_rows=band_rows, search_rows=search_rows)

    top_run = find_run(top_y)
    bottom_run = find_run(bottom_y)
    if top_run is None or bottom_run is None:
        return None
    margin = _clamp(
        _env_float("AR_BIRDSEYE_MASK_MARGIN_X", 0.006), 0.0, 0.08)
    top_left = _clamp(
        float(top_run[0]) / float(max(1, mask_width - 1)) - margin,
        0.0, 0.98)
    top_right = _clamp(
        float(top_run[1]) / float(max(1, mask_width - 1)) + margin,
        top_left + 0.01, 1.0)
    bottom_left = _clamp(
        float(bottom_run[0]) / float(max(1, mask_width - 1)) - margin,
        0.0, 0.98)
    bottom_right = _clamp(
        float(bottom_run[1]) / float(max(1, mask_width - 1)) + margin,
        bottom_left + 0.01, 1.0)
    top_width = top_right - top_left
    bottom_width = bottom_right - bottom_left
    if not (0.025 <= top_width <= 0.60):
        return None
    if not (0.08 <= bottom_width <= 0.98):
        return None
    # Camera height changes can make the visible road nearly parallel.  Do
    # not reject that valid geometry merely because the bottom is not 12%
    # wider than the top; temporal gating below handles noisy candidates.
    if bottom_width < top_width:
        return None
    return np.asarray([
        [top_left, top_y],
        [top_right, top_y],
        [bottom_right, bottom_y],
        [bottom_left, bottom_y],
    ], dtype=np.float32)


def _project_birdseye_debug_points(points, homography, source_quad,
                                    output_shape):
    """Project only points inside the configured ground-plane trapezoid."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        return np.empty((0, 2), dtype=np.float32), np.zeros(0, dtype=bool)

    finite = np.isfinite(points).all(axis=1)
    top_y = 0.5 * float(source_quad[0, 1] + source_quad[1, 1])
    bottom_y = 0.5 * float(source_quad[2, 1] + source_quad[3, 1])
    source_visible = (
        finite
        & (points[:, 1] >= top_y)
        & (points[:, 1] <= bottom_y)
    )
    projected = np.full(points.shape, np.nan, dtype=np.float32)
    if np.any(source_visible):
        projected[source_visible] = cv2.perspectiveTransform(
            points[source_visible].reshape(-1, 1, 2), homography
        ).reshape(-1, 2)
    height, width = output_shape[:2]
    output_visible = (
        source_visible
        & np.isfinite(projected).all(axis=1)
        & (projected[:, 0] >= 0.0)
        & (projected[:, 0] <= max(0, width - 1))
        & (projected[:, 1] >= 0.0)
        & (projected[:, 1] <= max(0, height - 1))
    )
    return projected, output_visible


def _draw_birdseye_debug_curve(
        frame, points, probabilities, slot, homography, source_quad,
        thickness=2):
    projected, visible = _project_birdseye_debug_points(
        points, homography, source_quad, frame.shape)
    if len(projected) < 2:
        return
    projected = np.where(np.isfinite(projected), projected, 0.0)
    _draw_identity_probability_curve(
        frame, projected, probabilities, slot, thickness=thickness,
        visible_mask=visible)


def _birdseye_debug_road_geometry(
        result, source_quad, output_shape):
    road_mask = _birdseye_debug_result_road_mask(result)
    if road_mask is None:
        return None

    output_height, output_width = output_shape[:2]
    mask_height, mask_width = road_mask.shape
    source_to_mask = np.asarray([
        [float(max(1, mask_width - 1)) /
         float(max(1, output_width - 1)), 0.0, 0.0],
        [0.0, float(max(1, mask_height - 1)) /
         float(max(1, output_height - 1)), 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    ground_mask = road_mask.copy()
    ground_roi = np.zeros_like(ground_mask, dtype=np.uint8)
    mask_quad = cv2.perspectiveTransform(
        source_quad.reshape(-1, 1, 2), source_to_mask
    ).reshape(-1, 2)
    cv2.fillConvexPoly(
        ground_roi, np.rint(mask_quad).astype(np.int32), 1,
        lineType=cv2.LINE_8)
    ground_mask &= ground_roi
    mask_to_source = np.asarray([
        [float(max(1, output_width - 1)) /
         float(max(1, mask_width - 1)), 0.0, 0.0],
        [0.0, float(max(1, output_height - 1)) /
         float(max(1, mask_height - 1)), 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return ground_mask, mask_to_source


def _draw_birdseye_debug_road(
        frame, result, homography, source_quad):
    geometry = _birdseye_debug_road_geometry(
        result, source_quad, frame.shape)
    if geometry is None:
        return
    ground_mask, mask_to_source = geometry
    if not np.any(ground_mask):
        return
    mask_to_birdseye = (
        np.asarray(homography, dtype=np.float64) @ mask_to_source)
    # The previous implementation transformed every contour and then drew
    # thousands of individual road dots.  On the RK3588 that made the writer
    # thread fall behind the camera.  One nearest-neighbour mask warp keeps
    # the same display-only information while remaining bounded and cheap.
    warped_mask = cv2.warpPerspective(
        (ground_mask.astype(np.uint8) * 255), mask_to_birdseye,
        (frame.shape[1], frame.shape[0]), flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    visible = warped_mask != 0
    if not np.any(visible):
        return
    road_color = np.asarray((55, 55, 220), dtype=np.float32)
    alpha = _clamp(
        _env_float("AR_BIRDSEYE_ROAD_ALPHA", 0.60), 0.05, 1.0)
    frame_pixels = frame[visible].astype(np.float32)
    frame[visible] = np.clip(
        frame_pixels * (1.0 - alpha) + road_color * alpha,
        0.0, 255.0).astype(np.uint8)


def render_vision_control_birdseye(
        frame, result, source_quad_normalized=None, homography_bundle=None):
    """Render an IPM debug panel without feeding anything back to control."""
    if frame is None or frame.size == 0:
        return frame
    if source_quad_normalized is None:
        source_quad_normalized = _birdseye_debug_default_source_quad()
        if _env_bool("AR_BIRDSEYE_FOLLOW_ROAD_MASK", True):
            candidate = _birdseye_debug_source_quad_from_road(
                result, frame.shape)
            if candidate is not None:
                source_quad_normalized = candidate
    height, width = frame.shape[:2]
    if homography_bundle is None:
        homography, source_quad, destination_quad = (
            _birdseye_debug_homography(
                frame.shape, source_quad_normalized=source_quad_normalized))
    else:
        homography, source_quad, destination_quad = homography_bundle
    panel = cv2.warpPerspective(
        frame, homography, (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(12, 12, 12))
    if _env_bool("AR_BIRDSEYE_ROAD_OVERLAY", True):
        _draw_birdseye_debug_road(
            panel, result, homography, source_quad)

    # A lightweight visual grid helps judge geometry; it is deliberately not
    # labelled in metres because this preview has no camera calibration yet.
    grid_color = (72, 72, 72)
    destination_left = int(round(destination_quad[0, 0]))
    destination_right = int(round(destination_quad[1, 0]))
    destination_top = int(round(destination_quad[0, 1]))
    destination_bottom = int(round(destination_quad[2, 1]))
    for fraction in (0.20, 0.40, 0.60, 0.80):
        grid_y = int(round(
            destination_top + fraction *
            (destination_bottom - destination_top)))
        cv2.line(
            panel, (destination_left, grid_y),
            (destination_right, grid_y), grid_color, 1, cv2.LINE_AA)
    for fraction in (0.25, 0.50, 0.75):
        grid_x = int(round(
            destination_left + fraction *
            (destination_right - destination_left)))
        cv2.line(
            panel, (grid_x, destination_top),
            (grid_x, destination_bottom), grid_color, 1, cv2.LINE_AA)

    debug = (
        (result.get("vision_control") if isinstance(result, dict) else None)
        or {})
    selected_slot = debug.get("selected_slot")
    if isinstance(result, dict):
        for line in _extract_curve_preview_lines(result):
            _draw_birdseye_debug_curve(
                panel, line["points_xy"], line.get("probabilities"),
                int(line.get("slot", 0)), homography, source_quad,
                thickness=(
                    4 if int(line.get("slot", -1)) == selected_slot else 2))

    selected_path_value = debug.get("selected_path")
    selected_path = np.asarray(
        [] if selected_path_value is None else selected_path_value,
        dtype=np.float32)
    if (selected_path.ndim == 2 and selected_path.shape[1:] == (2,)
            and len(selected_path) >= 2):
        selected_projected, selected_visible = (
            _project_birdseye_debug_points(
                selected_path, homography, source_quad, panel.shape))
        selected_pixels = np.rint(np.where(
            np.isfinite(selected_projected), selected_projected, 0.0)
        ).astype(np.int32)
        for index in range(len(selected_pixels) - 1):
            if not (selected_visible[index] and selected_visible[index + 1]):
                continue
            cv2.line(
                panel, tuple(selected_pixels[index]),
                tuple(selected_pixels[index + 1]),
                (0, 0, 0), 6, cv2.LINE_AA)
            cv2.line(
                panel, tuple(selected_pixels[index]),
                tuple(selected_pixels[index + 1]),
                (0, 255, 255), 3, cv2.LINE_AA)

    center_x = float(width) * _clamp(
        _env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8)
    car_source = np.asarray([
        [center_x, source_quad[2, 1]],
    ], dtype=np.float32)
    car_projected = cv2.perspectiveTransform(
        car_source.reshape(-1, 1, 2), homography).reshape(-1, 2)[0]
    car_x = int(round(_clamp(car_projected[0], 0, width - 1)))
    car_y = int(round(_clamp(car_projected[1], 0, height - 1)))
    cv2.drawMarker(
        panel, (car_x, car_y), (255, 255, 255), cv2.MARKER_TRIANGLE_UP,
        max(12, int(round(width * 0.035))), 2, cv2.LINE_AA)

    target = debug.get("control_target") or {}
    path_target_x = _finite_float(target.get("path_target_x"))
    target_x = _finite_float(target.get("target_x"), path_target_x)
    lookahead_y = _finite_float(
        target.get("path_target_y"),
        _finite_float(target.get("lookahead_y"), height * 0.62))

    def project_target(x_value):
        if x_value is None or lookahead_y is None:
            return None
        source_point = np.asarray(
            [[float(x_value), float(lookahead_y)]], dtype=np.float32)
        projected, visible = _project_birdseye_debug_points(
            source_point, homography, source_quad, panel.shape)
        if len(visible) != 1 or not visible[0]:
            return None
        return tuple(np.rint(projected[0]).astype(np.int32))

    path_target_point = project_target(path_target_x)
    control_target_point = project_target(target_x)
    if path_target_point is not None:
        cv2.circle(
            panel, path_target_point, 6, (255, 255, 255), 2,
            cv2.LINE_AA)
    if control_target_point is not None:
        cv2.line(
            panel, (car_x, car_y), control_target_point,
            (255, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(
            panel, control_target_point, 8, (255, 0, 255), -1,
            cv2.LINE_AA)
        cv2.circle(
            panel, control_target_point, 10, (255, 255, 255), 1,
            cv2.LINE_AA)

    overlay_height = max(52, int(round(height * 0.12)))
    cv2.rectangle(panel, (0, 0), (width - 1, overlay_height),
                  (0, 0, 0), -1)
    cv2.putText(
        panel, "BIRD'S-EYE / IPM DEBUG", (10, 23),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2,
        cv2.LINE_AA)
    cv2.putText(
        panel, "DISPLAY ONLY - CONTROL UNCHANGED", (10, 47),
        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (190, 190, 190), 1,
        cv2.LINE_AA)
    if not debug:
        cv2.putText(
            panel, "WAITING FOR PATH DEBUG", (10, overlay_height + 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 165, 255), 2,
            cv2.LINE_AA)
    return panel


class VisionControlBirdseyeRenderer:
    """Low-latency, stable display renderer for the local preview only.

    The source trapezoid follows the current semantic-road component.  A
    conservative fallback is used only while the mask is unavailable.
    """

    def __init__(self):
        self.source_quad_normalized = None
        self.source_status = "MASK FALLBACK"
        self.missing_mask_frames = 0
        self._pending_quad = None
        self._pending_frames = 0
        self._homography_bundle = None
        self._homography_shape = None
        self._homography_source_key = None

    def _update_source_quad(self, result, image_shape):
        fixed = _birdseye_debug_default_source_quad()
        if not _env_bool("AR_BIRDSEYE_FOLLOW_ROAD_MASK", True):
            self.source_quad_normalized = fixed
            self.source_status = "MASK FALLBACK"
            self.missing_mask_frames = 0
            self._pending_quad = None
            self._pending_frames = 0
            return self.source_quad_normalized

        candidate = _birdseye_debug_source_quad_from_road(
            result, image_shape)
        hold_frames = max(
            0, _env_int("AR_BIRDSEYE_MASK_HOLD_FRAMES", 4))
        if candidate is None:
            self.missing_mask_frames += 1
            if (self.source_quad_normalized is None
                    or self.missing_mask_frames > hold_frames):
                self.source_quad_normalized = fixed
                self.source_status = "MASK FALLBACK"
            else:
                self.source_status = "ROAD MASK HOLD"
            return self.source_quad_normalized

        self.missing_mask_frames = 0
        if self.source_quad_normalized is None:
            self.source_quad_normalized = candidate.copy()
            self.source_status = "ROAD MASK"
        else:
            jump_threshold = _clamp(
                _env_float("AR_BIRDSEYE_MASK_JUMP_THRESHOLD_X", 0.12),
                0.02, 0.50)
            jump_threshold_y = _clamp(
                _env_float("AR_BIRDSEYE_MASK_JUMP_THRESHOLD_Y", 0.14),
                0.02, 0.40)
            candidate_delta = float(np.max(np.abs(
                candidate[:, 0] - self.source_quad_normalized[:, 0])))
            candidate_delta_y = float(np.max(np.abs(
                candidate[:, 1] - self.source_quad_normalized[:, 1])))
            if (candidate_delta > jump_threshold
                    or candidate_delta_y > jump_threshold_y):
                confirm_frames = max(
                    1, _env_int("AR_BIRDSEYE_MASK_CONFIRM_FRAMES", 3))
                if (self._pending_quad is None
                        or float(np.max(np.abs(
                            candidate[:, 0] - self._pending_quad[:, 0])))
                        > jump_threshold * 0.5
                        or float(np.max(np.abs(
                            candidate[:, 1] - self._pending_quad[:, 1])))
                        > jump_threshold_y * 0.5):
                    self._pending_quad = candidate.copy()
                    self._pending_frames = 1
                else:
                    self._pending_frames += 1
                if self._pending_frames < confirm_frames:
                    self.source_status = "ROAD MASK HOLD"
                    return self.source_quad_normalized
                candidate = self._pending_quad
                self._pending_quad = None
                self._pending_frames = 0
            else:
                self._pending_quad = None
                self._pending_frames = 0
            alpha = _clamp(
                _env_float("AR_BIRDSEYE_MASK_SMOOTH_ALPHA", 0.45),
                0.02, 1.0)
            max_step = _clamp(
                _env_float("AR_BIRDSEYE_MASK_MAX_STEP_X", 0.06),
                0.002, 0.20)
            max_step_y = _clamp(
                _env_float("AR_BIRDSEYE_MASK_MAX_STEP_Y", 0.045),
                0.002, 0.20)
            delta = alpha * (
                candidate - self.source_quad_normalized)
            self.source_quad_normalized += np.clip(
                delta,
                np.asarray([-max_step, -max_step_y], dtype=np.float32),
                np.asarray([max_step, max_step_y], dtype=np.float32))
        self.source_status = "ROAD MASK"
        return self.source_quad_normalized

    def _homography_for(self, frame_shape, source_quad):
        source_key = tuple(np.round(
            np.asarray(source_quad, dtype=np.float32).reshape(-1), 5))
        shape_key = tuple(frame_shape[:2])
        if (self._homography_bundle is None
                or self._homography_shape != shape_key
                or self._homography_source_key != source_key):
            self._homography_bundle = _birdseye_debug_homography(
                frame_shape, source_quad_normalized=source_quad)
            self._homography_shape = shape_key
            self._homography_source_key = source_key
        return self._homography_bundle

    def __call__(self, frame, result):
        source_quad = self._update_source_quad(result, frame.shape)
        homography_bundle = self._homography_for(frame.shape, source_quad)
        return render_vision_control_birdseye(
            frame, result, source_quad_normalized=source_quad,
            homography_bundle=homography_bundle)

    def draw_source_roi(self, frame):
        if (frame is None or frame.size == 0
                or not _env_bool("AR_BIRDSEYE_SHOW_SOURCE_ROI", True)):
            return frame
        source_quad = (
            self.source_quad_normalized
            if self.source_quad_normalized is not None
            else _birdseye_debug_default_source_quad())
        _homography, source, _destination = self._homography_for(
            frame.shape, source_quad)
        source_pixels = np.rint(source).astype(np.int32)
        cv2.polylines(
            frame, [source_pixels], True, (0, 0, 255), 2, cv2.LINE_AA)
        label_y = max(18, int(np.min(source_pixels[:, 1])) - 7)
        cv2.putText(
            frame, "IPM ROI: {}".format(self.source_status),
            (max(4, int(source_pixels[0, 0])), label_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1,
            cv2.LINE_AA)
        return frame


def _fit_smooth_majority_curve(
        points, probabilities, image_width, inlier_px_640=10.0,
        sample_step_px=4.0, extend_to_y=None,
        max_extension_y_px_480=64.0,
        max_extension_deviation_px_640=18.0,
        max_extension_lateral_px_640=48.0):
    """Fit a low-order curve, favoring maximum support then smoothness."""
    points = np.asarray(points, dtype=np.float64)
    probabilities = np.asarray(probabilities, dtype=np.float64)
    empty_points = np.empty((0, 2), dtype=np.float32)
    empty_probabilities = np.empty((0,), dtype=np.float32)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 3):
        return empty_points, empty_probabilities, np.zeros(len(points), bool)
    if len(probabilities) != len(points):
        probabilities = np.ones(len(points), dtype=np.float64)

    order = np.argsort(points[:, 1], kind="stable")
    sorted_points = points[order]
    sorted_probabilities = probabilities[order]
    center_y = float(np.mean(sorted_points[:, 1]))
    scale_y = max(1.0, 0.5 * float(np.ptp(sorted_points[:, 1])))
    normalized_y = (sorted_points[:, 1] - center_y) / scale_y
    threshold = max(
        2.0, float(inlier_px_640) * float(max(1, image_width)) / 640.0)

    # Six evenly spread anchors provide 35 deterministic line/quadratic
    # hypotheses, enough for minority-outlier rejection without burdening
    # every preview frame with an exhaustive fit.
    sample_count = min(6, len(sorted_points))
    sample_indices = np.unique(np.rint(np.linspace(
        0, len(sorted_points) - 1, sample_count)).astype(np.int32))
    candidates = []
    try:
        candidates.append((1, np.polyfit(
            normalized_y, sorted_points[:, 0], 1)))
    except (ValueError, np.linalg.LinAlgError):
        pass
    line_pairs = np.asarray([
        (sample_indices[first], sample_indices[second])
        for first in range(len(sample_indices) - 1)
        for second in range(first + 1, len(sample_indices))
    ], dtype=np.int32)
    if len(line_pairs):
        first_y = normalized_y[line_pairs[:, 0]]
        second_y = normalized_y[line_pairs[:, 1]]
        denominator = second_y - first_y
        valid = np.abs(denominator) > 1e-6
        slopes = (
            sorted_points[line_pairs[valid, 1], 0] -
            sorted_points[line_pairs[valid, 0], 0]) / denominator[valid]
        intercepts = (
            sorted_points[line_pairs[valid, 0], 0] -
            slopes * first_y[valid])
        candidates.extend(
            (1, np.asarray([slope, intercept], dtype=np.float64))
            for slope, intercept in zip(slopes, intercepts))

    try:
        candidates.append((2, np.polyfit(
            normalized_y, sorted_points[:, 0], 2)))
    except (ValueError, np.linalg.LinAlgError):
        pass
    quadratic_groups = np.asarray([
        (sample_indices[first], sample_indices[second],
         sample_indices[third])
        for first in range(len(sample_indices) - 2)
        for second in range(first + 1, len(sample_indices) - 1)
        for third in range(second + 1, len(sample_indices))
    ], dtype=np.int32)
    if len(quadratic_groups):
        group_y = normalized_y[quadratic_groups]
        matrices = np.stack((
            group_y * group_y, group_y,
            np.ones_like(group_y)), axis=2)
        determinants = np.linalg.det(matrices)
        valid = np.abs(determinants) > 1e-9
        if np.any(valid):
            coefficients = np.linalg.solve(
                matrices[valid],
                sorted_points[quadratic_groups[valid], 0])
            candidates.extend(
                (2, coefficient) for coefficient in coefficients)

    evaluated = []
    if candidates:
        degrees = np.asarray(
            [item[0] for item in candidates], dtype=np.int32)
        coefficient_matrix = np.zeros(
            (len(candidates), 3), dtype=np.float64)
        for index, (degree, coefficients) in enumerate(candidates):
            coefficient_matrix[index, -len(coefficients):] = coefficients
        design = np.stack((
            normalized_y * normalized_y,
            normalized_y,
            np.ones_like(normalized_y)), axis=1)
        predictions = design @ coefficient_matrix.T
        all_residuals = np.abs(
            sorted_points[:, 0, None] - predictions)
        all_inliers = all_residuals <= threshold
        counts = np.count_nonzero(all_inliers, axis=0)
        for index, (degree, coefficients) in enumerate(candidates):
            count = int(counts[index])
            if count < max(3, degree + 2):
                continue
            inliers = all_inliers[:, index]
            median_error = float(np.median(
                all_residuals[inliers, index]))
            curvature = (
                2.0 * abs(float(coefficient_matrix[index, 0]))
                if degree == 2 else 0.0)
            evaluated.append({
                "degree": degree,
                "coefficients": coefficients,
                "inliers": inliers,
                "count": count,
                "smooth_cost": (
                    median_error + 0.04 * curvature + 0.15 * degree),
            })
    if not evaluated:
        return empty_points, empty_probabilities, np.zeros(len(points), bool)

    maximum_support = max(item["count"] for item in evaluated)
    near_maximum = [
        item for item in evaluated
        if item["count"] >= maximum_support - 1]
    best = min(near_maximum, key=lambda item: item["smooth_cost"])
    degree = int(best["degree"])
    inliers = np.asarray(best["inliers"], dtype=bool)
    coefficients = np.asarray(best["coefficients"], dtype=np.float64)
    for _iteration in range(2):
        if int(np.count_nonzero(inliers)) < degree + 2:
            break
        coefficients = np.polyfit(
            normalized_y[inliers], sorted_points[inliers, 0], degree)
        residuals = np.abs(
            sorted_points[:, 0] - np.polyval(coefficients, normalized_y))
        refined = residuals <= threshold
        if np.array_equal(refined, inliers):
            break
        inliers = refined

    if int(np.count_nonzero(inliers)) < 3:
        return empty_points, empty_probabilities, np.zeros(len(points), bool)
    inlier_points = sorted_points[inliers]
    inlier_probabilities = sorted_probabilities[inliers]
    inlier_order = np.argsort(inlier_points[:, 1], kind="stable")
    inlier_points = inlier_points[inlier_order]
    inlier_probabilities = inlier_probabilities[inlier_order]
    start_y = float(inlier_points[0, 1])
    end_y = float(inlier_points[-1, 1])
    extend_to_y = _finite_float(extend_to_y)
    include_extension_y = False
    if extend_to_y is not None:
        if start_y <= extend_to_y <= end_y:
            include_extension_y = True
        else:
            edge_y = start_y if extend_to_y < start_y else end_y
            vertical_gap = abs(float(extend_to_y) - edge_y)
            edge_count = min(6, len(inlier_points))
            edge_points = (
                inlier_points[:edge_count]
                if extend_to_y < start_y else inlier_points[-edge_count:])
            polynomial_x = float(np.polyval(
                coefficients, (float(extend_to_y) - center_y) / scale_y))
            tangent_coefficients = np.polyfit(
                edge_points[:, 1], edge_points[:, 0], 1)
            tangent_x = float(np.polyval(
                tangent_coefficients, float(extend_to_y)))
            edge_x = float(edge_points[0, 0] if extend_to_y < start_y
                           else edge_points[-1, 0])
            width_scale = float(max(1, image_width)) / 640.0
            extension_safe = (
                vertical_gap <= float(max_extension_y_px_480) and
                abs(polynomial_x - tangent_x) <=
                float(max_extension_deviation_px_640) * width_scale and
                abs(polynomial_x - edge_x) <=
                float(max_extension_lateral_px_640) * width_scale)
            if extension_safe:
                start_y = min(start_y, float(extend_to_y))
                end_y = max(end_y, float(extend_to_y))
                include_extension_y = True
    sample_count = max(
        2, int(math.ceil((end_y - start_y) /
                         max(1.0, float(sample_step_px)))) + 1)
    fitted_y = np.linspace(start_y, end_y, sample_count, dtype=np.float64)
    if include_extension_y:
        fitted_y = np.unique(np.append(fitted_y, float(extend_to_y)))
    fitted_x = np.polyval(
        coefficients, (fitted_y - center_y) / scale_y)
    fitted_x = np.clip(fitted_x, 0.0, float(max(0, image_width - 1)))
    fitted_probabilities = np.interp(
        fitted_y, inlier_points[:, 1], inlier_probabilities)
    fitted_points = np.stack((fitted_x, fitted_y), axis=1).astype(np.float32)

    original_inliers = np.zeros(len(points), dtype=bool)
    original_inliers[order] = inliers
    return (fitted_points,
            np.clip(fitted_probabilities, 0.0, 1.0).astype(np.float32),
            original_inliers)


class _FittedControlPathTracker:
    """Adaptive per-slot tracker for fitted control/preview curves.

    Small frame-to-frame changes are blended to suppress straight-line model
    noise.  A coherent same-direction change is blended aggressively so the
    tracker does not add a visible delay at a curve entrance.  Very large
    discontinuities still require a second-frame confirmation to protect
    against a slot switch or an isolated bad fit.
    """

    def __init__(self, jump_threshold_px_640=48.0,
                 confirm_tolerance_px_640=16.0,
                 hold_frames=0, timeout_s=0.5, hold_decay=0.72):
        self.jump_threshold_px_640 = max(
            0.0, float(jump_threshold_px_640))
        self.confirm_tolerance_px_640 = max(
            0.0, float(confirm_tolerance_px_640))
        self.hold_frames = max(0, int(hold_frames))
        self.timeout_s = max(0.0, float(timeout_s))
        self.hold_decay = _clamp(float(hold_decay), 0.0, 1.0)
        self._slots = {}

    @staticmethod
    def _empty():
        return (np.empty((0, 2), dtype=np.float32),
                np.empty((0,), dtype=np.float32))

    @staticmethod
    def _median_curve_distance(first, second):
        first = np.asarray(first, dtype=np.float32)
        second = np.asarray(second, dtype=np.float32)
        if len(first) < 2 or len(second) < 2:
            return None
        y = second[:, 1]
        overlap = (
            (y >= float(first[0, 1])) &
            (y <= float(first[-1, 1])))
        if int(np.count_nonzero(overlap)) < 2:
            return None
        first_x = np.interp(
            y[overlap], first[:, 1], first[:, 0]).astype(np.float32)
        return float(np.median(np.abs(second[overlap, 0] - first_x)))

    @staticmethod
    def _median_signed_curve_displacement(first, second):
        first = np.asarray(first, dtype=np.float32)
        second = np.asarray(second, dtype=np.float32)
        if len(first) < 2 or len(second) < 2:
            return None
        y = second[:, 1]
        overlap = (
            (y >= float(first[0, 1])) &
            (y <= float(first[-1, 1])))
        if int(np.count_nonzero(overlap)) < 2:
            return None
        first_x = np.interp(
            y[overlap], first[:, 1], first[:, 0]).astype(np.float32)
        return float(np.median(second[overlap, 0] - first_x))

    @staticmethod
    def _blend_with_previous(previous_points, current_points, alpha):
        previous_points = np.asarray(previous_points, dtype=np.float32)
        current_points = np.asarray(current_points, dtype=np.float32).copy()
        if len(previous_points) < 2 or len(current_points) == 0:
            return current_points
        y = current_points[:, 1]
        overlap = (
            (y >= float(previous_points[0, 1])) &
            (y <= float(previous_points[-1, 1])))
        if not np.any(overlap):
            return current_points
        previous_x = np.interp(
            y[overlap], previous_points[:, 1], previous_points[:, 0])
        current_points[overlap, 0] = (
            previous_x + float(alpha) *
            (current_points[overlap, 0] - previous_x))
        return current_points

    def update(self, slot, points, probabilities, image_shape, now=None):
        slot = int(slot)
        now = time.monotonic() if now is None else float(now)
        points = np.asarray(points, dtype=np.float32)
        probabilities = np.asarray(probabilities, dtype=np.float32)
        if len(probabilities) != len(points):
            probabilities = np.ones(len(points), dtype=np.float32)
        shape = tuple(int(value) for value in image_shape[:2])
        previous = self._slots.get(slot)
        if previous is not None and (
                previous["shape"] != shape or
                now - float(previous["seen_at"]) > self.timeout_s):
            self._slots.pop(slot, None)
            previous = None

        valid_current = (
            points.ndim == 2 and points.shape[1] == 2 and len(points) >= 2)
        if not valid_current:
            if previous is None:
                return self._empty()
            misses = int(previous["misses"]) + 1
            if misses > self.hold_frames:
                self._slots.pop(slot, None)
                return self._empty()
            previous["misses"] = misses
            held_probabilities = (
                previous["probabilities"] * self.hold_decay ** misses)
            return (previous["points"].copy(),
                    held_probabilities.astype(np.float32))

        order = np.argsort(points[:, 1], kind="stable")
        tracked_points = points[order].copy()
        tracked_probabilities = np.clip(
            probabilities[order], 0.0, 1.0).astype(np.float32)
        confirmed_jump = False
        if previous is not None:
            width_scale = float(max(1, shape[1])) / 640.0
            displacement = self._median_curve_distance(
                previous["points"], tracked_points)
            if (displacement is not None and
                    displacement > self.jump_threshold_px_640 * width_scale):
                pending = previous.get("pending")
                pending_distance = (
                    self._median_curve_distance(
                        pending["points"], tracked_points)
                    if pending is not None else None)
                confirmed = (
                    pending_distance is not None and
                    pending_distance <=
                    self.confirm_tolerance_px_640 * width_scale)
                if not confirmed:
                    previous["pending"] = {
                        "points": tracked_points.copy(),
                        "probabilities": tracked_probabilities.copy(),
                        "seen_at": now,
                    }
                    return (previous["points"].copy(),
                            previous["probabilities"].copy())
                confirmed_jump = True

            signed_displacement = self._median_signed_curve_displacement(
                previous["points"], tracked_points)
            if signed_displacement is not None and not confirmed_jump:
                noise_floor = 3.0 * width_scale
                if abs(signed_displacement) <= noise_floor:
                    previous["motion_sign"] = 0
                    previous["motion_frames"] = 0
                else:
                    motion_sign = 1 if signed_displacement > 0.0 else -1
                    if motion_sign == int(previous.get("motion_sign", 0)):
                        previous["motion_frames"] = (
                            int(previous.get("motion_frames", 0)) + 1)
                    else:
                        previous["motion_sign"] = motion_sign
                        previous["motion_frames"] = 1
                motion_frames = int(previous.get("motion_frames", 0))
                absolute_displacement = abs(float(signed_displacement))
                if (
                    absolute_displacement <= 10.0 * width_scale
                    and motion_frames < 2
                ):
                    alpha = 0.20
                elif motion_frames >= 2:
                    # Like the OCR parking correction, a direction must first
                    # be observed consistently. Once confirmed, even a small
                    # per-frame movement is treated as a real curve entrance
                    # instead of permanent straight-line jitter.
                    alpha = 0.82
                else:
                    alpha = 0.55
                # The pending branch above handles only discontinuities large
                # enough to look like a route/slot switch.  Normal fitted
                # movement is blended here and therefore keeps the target
                # responsive without passing pixel noise straight through.
                tracked_points = self._blend_with_previous(
                    previous["points"], tracked_points, alpha)

        self._slots[slot] = {
            "points": tracked_points.copy(),
            "probabilities": tracked_probabilities.copy(),
            "shape": shape,
            "seen_at": now,
            "misses": 0,
            "pending": None,
            "motion_sign": (
                int(previous.get("motion_sign", 0))
                if previous is not None else 0),
            "motion_frames": (
                int(previous.get("motion_frames", 0))
                if previous is not None else 0),
        }
        return tracked_points, tracked_probabilities


def _adjust_point_display_confidences(
        points, probabilities, split_y, lower_boost, upper_decay):
    points = np.asarray(points, dtype=np.float32)
    probabilities = np.asarray(probabilities, dtype=np.float32)
    if len(probabilities) != len(points):
        probabilities = np.ones(len(points), dtype=np.float32)
    adjusted = np.clip(probabilities, 0.0, 1.0).copy()
    lower = points[:, 1] >= float(split_y)
    boost = _clamp(float(lower_boost), 0.0, 1.0)
    decay = _clamp(float(upper_decay), 0.0, 1.0)
    adjusted[lower] += (1.0 - adjusted[lower]) * boost
    adjusted[~lower] *= decay
    return np.clip(adjusted, 0.0, 1.0)


def _associated_point_mask(
        points, image_width, max_anchor_gap=1,
        jump_per_anchor_px_640=36.0, min_component_size=3,
        eligible_mask=None, max_line_error_px_640=6.0,
        min_direction_cosine=0.95):
    """Keep points supported by a strongly linear three-anchor group."""
    points = np.asarray(points, dtype=np.float32)
    count = len(points)
    required_points = max(3, int(min_component_size))
    if count < required_points:
        return np.zeros(count, dtype=bool)
    if eligible_mask is None:
        eligible = np.ones(count, dtype=bool)
    else:
        eligible = np.asarray(eligible_mask, dtype=bool)
        if len(eligible) != count:
            eligible = np.ones(count, dtype=bool)
    width_scale = float(max(1, image_width)) / 640.0
    keep = np.zeros(count, dtype=bool)
    anchor_step = max(1, int(max_anchor_gap))
    maximum_dx = float(jump_per_anchor_px_640) * width_scale * anchor_step
    maximum_line_error = float(max_line_error_px_640) * width_scale
    minimum_cosine = _clamp(float(min_direction_cosine), -1.0, 1.0)
    for first in range(count - 2 * anchor_step):
        indices = np.asarray([
            first, first + anchor_step, first + 2 * anchor_step],
            dtype=np.int32)
        if not np.all(eligible[indices]):
            continue
        first_vector = points[indices[1]] - points[indices[0]]
        second_vector = points[indices[2]] - points[indices[1]]
        if (abs(float(first_vector[0])) > maximum_dx or
                abs(float(second_vector[0])) > maximum_dx):
            continue
        first_length = float(np.linalg.norm(first_vector))
        second_length = float(np.linalg.norm(second_vector))
        if first_length <= 1e-6 or second_length <= 1e-6:
            continue
        direction_cosine = float(np.dot(first_vector, second_vector)) / (
            first_length * second_length)
        chord = points[indices[2]] - points[indices[0]]
        chord_length = float(np.linalg.norm(chord))
        if chord_length <= 1e-6:
            continue
        middle_offset = points[indices[1]] - points[indices[0]]
        line_error = abs(float(
            chord[0] * middle_offset[1] -
            chord[1] * middle_offset[0])) / chord_length
        if (direction_cosine >= minimum_cosine and
                line_error <= maximum_line_error):
            keep[indices] = True
    return keep


def _densify_associated_lower_points(
        points, probabilities, associated_mask, image_width, image_height,
        sparse_gap_px_480=18.0, target_spacing_px_480=12.0,
        jump_per_anchor_px_640=36.0, max_insertions=3,
        lower_boundary_y=None):
    """Fill sparse connected display anchors, only in the lower half."""
    points = np.asarray(points, dtype=np.float32)
    probabilities = np.asarray(probabilities, dtype=np.float32)
    associated_mask = np.asarray(associated_mask, dtype=bool)
    if len(probabilities) != len(points):
        probabilities = np.ones(len(points), dtype=np.float32)
    if len(associated_mask) != len(points):
        associated_mask = np.ones(len(points), dtype=bool)

    height_scale = float(max(1, image_height)) / 480.0
    width_scale = float(max(1, image_width)) / 640.0
    lower_half_y = (
        0.5 * float(max(0, image_height - 1))
        if lower_boundary_y is None else float(lower_boundary_y))
    sparse_gap = max(1.0, float(sparse_gap_px_480) * height_scale)
    target_spacing = max(1.0, float(target_spacing_px_480) * height_scale)
    maximum_dx = float(jump_per_anchor_px_640) * width_scale
    insertion_limit = max(0, int(max_insertions))

    display_points = []
    display_probabilities = []
    for index, point in enumerate(points):
        if not associated_mask[index]:
            continue
        display_points.append(point)
        display_probabilities.append(probabilities[index])
        next_index = index + 1
        if next_index >= len(points) or not associated_mask[next_index]:
            continue
        next_point = points[next_index]
        if (float(point[1]) < lower_half_y or
                float(next_point[1]) < lower_half_y or
                abs(float(next_point[0] - point[0])) > maximum_dx):
            continue
        distance = float(np.linalg.norm(next_point - point))
        if distance <= sparse_gap:
            continue
        insertion_count = min(
            insertion_limit,
            max(1, int(math.ceil(distance / target_spacing)) - 1),
        )
        for insertion in range(1, insertion_count + 1):
            ratio = float(insertion) / float(insertion_count + 1)
            display_points.append(point + (next_point - point) * ratio)
            display_probabilities.append(
                probabilities[index] +
                (probabilities[next_index] - probabilities[index]) * ratio)

    if not display_points:
        return (np.empty((0, 2), dtype=np.float32),
                np.empty((0,), dtype=np.float32))
    return (np.asarray(display_points, dtype=np.float32),
            np.asarray(display_probabilities, dtype=np.float32))


def _semantic_road_point_mask(points, road_mask, image_shape):
    """Return points that land inside the semantic road segmentation."""
    points = np.asarray(points, dtype=np.float32)
    road = np.asarray(road_mask) if road_mask is not None else np.empty((0, 0))
    if road.ndim != 2 or not road.size:
        # Some synthetic/debug frames do not carry segmentation. Keep their
        # points visible rather than silently changing the existing preview.
        return np.ones(len(points), dtype=bool)
    height, width = image_shape[:2]
    road_height, road_width = road.shape
    x = np.clip(
        np.rint(points[:, 0] * float(road_width - 1) /
               float(max(width - 1, 1))), 0, road_width - 1).astype(np.int32)
    y = np.clip(
        np.rint(points[:, 1] * float(road_height - 1) /
               float(max(height - 1, 1))), 0, road_height - 1).astype(np.int32)
    return np.asarray(road[y, x]) != 0


def _select_control_curve_segment(
        points, probabilities, visible_mask, lookahead_y):
    """Choose one semantic-road segment, preferring lookahead coverage."""
    points = np.asarray(points, dtype=np.float32)
    probabilities = np.asarray(probabilities, dtype=np.float32)
    visible = np.asarray(visible_mask, dtype=bool)
    if len(probabilities) != len(points):
        probabilities = np.ones(len(points), dtype=np.float32)
    if len(visible) != len(points):
        visible = np.ones(len(points), dtype=bool)
    segments = []
    start = None
    for index, is_visible in enumerate(visible.tolist() + [False]):
        if is_visible and start is None:
            start = index
        elif not is_visible and start is not None:
            if index - start >= 3:
                segment_points = points[start:index]
                distance = (
                    0.0 if float(segment_points[0, 1]) <= lookahead_y <=
                    float(segment_points[-1, 1]) else
                    float(np.min(np.abs(
                        segment_points[:, 1] - float(lookahead_y)))))
                segments.append((distance, -(index - start), start, index))
            start = None
    if not segments:
        return (np.empty((0, 2), dtype=np.float32),
                np.empty((0,), dtype=np.float32))
    _distance, _negative_length, start, end = min(segments)
    return points[start:end].copy(), probabilities[start:end].copy()


def _extract_curve_preview_lines(result, max_lines=2):
    if not isinstance(result, dict):
        return []
    paths = result.get("paths")
    if paths is None:
        paths = (result.get("centerline") or {}).get("paths") or []
    lines = []
    for path in list(paths)[:max(0, min(2, int(max_lines)))]:
        points = np.asarray(path.get("points_xy"), dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
            continue
        probabilities = np.asarray(
            path.get("point_confidences", []), dtype=np.float32)
        if len(probabilities) != len(points):
            probabilities = np.full(
                len(points), float(path.get("score", 1.0)),
                dtype=np.float32)
        lines.append({
            "slot": int(path.get("slot", len(lines))),
            "identity": str(path.get("identity") or path.get("role") or ""),
            "points_xy": points,
            "probabilities": probabilities,
            "score": float(np.mean(probabilities)),
        })
    return lines[:2]
