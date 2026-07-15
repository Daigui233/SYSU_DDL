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


def _env_flag(name, default):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _path_source_from_env():
    value = os.environ.get(
        "VISION_CONTROL_PATH_SOURCE",
        os.environ.get("MULTITASK_PATH_SOURCE", "curve"),
    ).strip().lower()
    return value if value in {"curve", "heatmap"} else "curve"


def _finite_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


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


def _heatmap_polyline_mask(points, image_shape, heatmap_shape):
    """Rasterize an image-space path on the original heatmap grid."""
    points = np.asarray(points, dtype=np.float32)
    heat_h, heat_w = heatmap_shape
    mask = np.zeros((heat_h, heat_w), dtype=np.uint8)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        return mask
    img_h, img_w = image_shape[:2]
    heat_points = np.empty_like(points)
    heat_points[:, 0] = (
        points[:, 0] * float(max(heat_w - 1, 1)) /
        float(max(img_w - 1, 1)))
    heat_points[:, 1] = (
        points[:, 1] * float(max(heat_h - 1, 1)) /
        float(max(img_h - 1, 1)))
    heat_points = np.rint(heat_points).astype(np.int32)
    heat_points[:, 0] = np.clip(heat_points[:, 0], 0, heat_w - 1)
    heat_points[:, 1] = np.clip(heat_points[:, 1], 0, heat_h - 1)
    cv2.polylines(
        mask, [heat_points.reshape((-1, 1, 2))], False,
        1, 1, cv2.LINE_8)
    return mask


def _path_heat_support_metrics(
        points, image_shape, support_maps, source_slots,
        minimum_probability, require_all_channels=False):
    """Measure continuous heat support for every raster cell of a path."""
    support_maps = np.asarray(support_maps, dtype=np.float32)
    if support_maps.ndim != 3 or support_maps.shape[0] < 1:
        return {
            "valid": False, "support_ratio": 0.0,
            "min_probability": 0.0, "mean_probability": 0.0,
            "low_run_pixels": 0, "sample_count": 0,
        }
    slots = sorted({
        int(slot) for slot in source_slots
        if 0 <= int(slot) < support_maps.shape[0]
    })
    if not slots:
        return {
            "valid": False, "support_ratio": 0.0,
            "min_probability": 0.0, "mean_probability": 0.0,
            "low_run_pixels": 0, "sample_count": 0,
        }
    selected = support_maps[slots]
    combined = (
        np.min(selected, axis=0) if require_all_channels
        else np.max(selected, axis=0))
    mask = _heatmap_polyline_mask(
        points, image_shape, combined.shape)
    path_pixels = mask != 0
    values = combined[path_pixels]
    if not values.size:
        return {
            "valid": False, "support_ratio": 0.0,
            "min_probability": 0.0, "mean_probability": 0.0,
            "low_run_pixels": 0, "sample_count": 0,
        }
    supported = values >= float(minimum_probability)
    low_mask = (path_pixels & (combined < float(minimum_probability))).astype(
        np.uint8)
    low_run = 0
    if np.any(low_mask):
        count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(
            low_mask, connectivity=8)
        if count > 1:
            low_run = int(np.max(stats[1:, cv2.CC_STAT_AREA]))
    return {
        # A 3x3 maximum filter is already applied to support_maps. After that
        # tolerance, even one unsupported raster cell is a real blank crossing.
        "valid": bool(np.all(supported)),
        "support_ratio": float(np.mean(supported)),
        "min_probability": float(np.min(values)),
        "mean_probability": float(np.mean(values)),
        "low_run_pixels": low_run,
        "sample_count": int(values.size),
    }


def _path_point_probabilities(points, image_shape, support_maps, source_slots):
    points = np.asarray(points, dtype=np.float32)
    support_maps = np.asarray(support_maps, dtype=np.float32)
    if (points.ndim != 2 or points.shape[1] != 2 or not len(points) or
            support_maps.ndim != 3 or support_maps.shape[0] < 1):
        return np.empty(0, dtype=np.float32)
    slots = sorted({
        int(slot) for slot in source_slots
        if 0 <= int(slot) < support_maps.shape[0]
    })
    if not slots:
        return np.empty(0, dtype=np.float32)
    combined = np.max(support_maps[slots], axis=0)
    heat_h, heat_w = combined.shape
    img_h, img_w = image_shape[:2]
    xs = np.rint(
        points[:, 0] * float(max(heat_w - 1, 1)) /
        float(max(img_w - 1, 1))).astype(np.int32)
    ys = np.rint(
        points[:, 1] * float(max(heat_h - 1, 1)) /
        float(max(img_h - 1, 1))).astype(np.int32)
    xs = np.clip(xs, 0, heat_w - 1)
    ys = np.clip(ys, 0, heat_h - 1)
    return combined[ys, xs].astype(np.float32)


@dataclass
class VisionControlConfig:
    visual_center_x: float = 0.50
    lookahead_y_ratio: float = 0.625
    max_track_error_640: float = 160.0
    max_error_step_640: float = 32.0
    error_trend_window: int = 5
    error_trend_min_frames: int = 3
    error_trend_kd: float = 0.15
    error_trend_deadband_640: float = 5.0
    error_trend_max_adjust_640: float = 20.0
    default_track_left_max_error_640: float = 210.0
    default_track_left_error_step_640: float = 32.0
    default_track_right_max_error_640: float = 210.0
    default_track_right_error_step_640: float = 36.0
    default_track_left_error_gain: float = 0.75
    default_track_right_error_gain: float = 0.87
    normal_speed_mps: float = 0.15
    recover_speed_mps: float = 0.15
    obstacle_speed_mps: float = 0.15
    human_speed_mps: float = 0.40
    human_pass_speed_mps: float = 0.42
    collect_speed_mps: float = 0.15
    turnsign_slow_speed_mps: float = 0.10
    heat_threshold: float = 0.22
    blank_probability: float = 0.05
    peak_scan_top_ratio: float = 0.45
    peak_scan_bottom_ratio: float = 0.55
    min_peak_component_area: int = 20
    peak_min_distance_px: int = 8
    greedy_search_radius_px: int = 8
    bottom_reach_ratio: float = 0.90
    side_exit_min_y_ratio: float = 2.0 / 3.0
    side_exit_margin_ratio: float = 0.05
    recovery_max_gap_rows: int = 12
    recovery_max_radius_px: int = 24
    recovery_min_probability: float = 0.35
    recovery_min_continuation_rows: int = 8
    recovery_ambiguity_margin: float = 0.08
    curve_fit_blend: float = 0.25
    road_mask_threshold: float = 0.20
    heat_peak_top_k: int = 6
    row_step: int = 2
    min_path_points: int = 12
    min_path_coverage: float = 0.28
    min_mean_heat: float = 0.28
    min_path_support_probability: float = 0.12
    max_link_jump_px: float = 16.0
    road_penalty_weight: float = 0.75
    history_weight: float = 0.035
    jump_weight: float = 0.018
    path_ema_alpha: float = 0.32
    path_smooth_window: int = 5
    path_max_step_px_640: float = 40.0
    path_state_hold_frames: int = 8
    route_confirm_frames: int = 6
    branch_release_frames: int = 20
    branch_separation_px_640: float = 70.0
    branch_separation_rows: int = 8
    overlap_px_640: float = 28.0
    fragment_search_radius_px_640: float = 72.0
    fragment_max_tangent_delta_deg: float = 55.0
    heatmap_component_budget: int = 2
    default_outer_after_s: float = 15.0
    ocr_lock_lifetime_s: float = 10.0
    ocr_confirm_frames: int = 1
    outer_slot: int = 0
    curve_merge_support_ratio: float = 0.70
    curve_merge_near_px_640: float = 36.0
    curve_merge_enter_evidence: int = 4
    curve_merge_release_frames: int = 30
    no_path_stop_s: float = 0.8
    recover_hold_s: float = 0.5
    hazard_bottom_ratio: float = 250.0 / 480.0
    hazard_lateral_ratio: float = 0.18
    coin_bottom_ratio: float = 0.55
    coin_lateral_ratio: float = 0.24
    sign_stop_height_ratio: float = 0.24
    # Measured from the live 640x480 AR Preview: the desired stop-size
    # TurnSign box is about 140x69 px (9660 px^2, or 3.14% of the frame).
    sign_stop_area_ratio: float = 0.031
    sign_stop_line_margin_ratio: float = 0.08
    sign_slow_min_score: float = 0.40
    sign_latch_frames: int = 3
    sign_ocr_timeout_s: float = 8.0
    sign_ocr_pulse_speed_mps: float = 0.25
    sign_ocr_pulse_duration_s: float = 0.30
    turnsign_detection_line_ratio: float = 185.0 / 480.0
    turnsign_edge_margin_ratio: float = 0.15
    turnsign_steer_gain: float = 1.5
    turnsign_reverse_speed_mps: float = -0.08
    turnsign_reverse_duration_s: float = 2.0
    turnsign_initial_brake_s: float = 0.5
    human_stop_line_margin_ratio: float = 0.0
    human_stop_progress_ratio: float = 8.0 / 9.0
    human_preline_missing_px_480: float = 20.0
    human_brake_reverse_speed_mps: float = -0.05
    human_brake_reverse_duration_s: float = 0.2
    human_return_duration_s: float = 1.0
    human_return_error_640: float = 40.0
    human_cross_release_px_640: float = 45.0
    human_pass_offset_px_640: float = 38.0
    human_speed_hold_s: float = 1.0
    human_absence_confirm_s: float = 1.5
    car_avoid_offset_px_640: float = 56.0
    car_avoid_speed_mps: float = 0.10
    car_avoid_ramp_s: float = 1.5
    car_avoid_hold_s: float = 1.0
    car_human_pass_speed_mps: float = 0.42
    car_human_pass_hold_s: float = 1.0
    human_avoid_offset_px_640: float = 75.0
    avoid_box_width_gain: float = 0.35
    gold_bias_gain: float = 0.45
    gold_max_bias_px_640: float = 75.0
    min_human_score: float = 0.35
    min_car_score: float = 0.35
    min_coin_score: float = 0.35
    path_source: str = "curve"

    @classmethod
    def from_env(cls):
        return cls(
            visual_center_x=_clamp(_env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8),
            lookahead_y_ratio=_clamp(_env_float("VISION_CONTROL_LOOKAHEAD_Y_RATIO", 0.625), 0.25, 0.95),
            max_track_error_640=max(1.0, _env_float("VISION_CONTROL_MAX_ERROR_640", 160.0)),
            max_error_step_640=max(1.0, _env_float("VISION_CONTROL_MAX_STEP_640", 32.0)),
            error_trend_window=max(
                3, _env_int("VISION_CONTROL_ERROR_TREND_WINDOW", 5)),
            error_trend_min_frames=max(
                3, _env_int("VISION_CONTROL_ERROR_TREND_MIN_FRAMES", 3)),
            error_trend_kd=max(
                0.0, _env_float("VISION_CONTROL_ERROR_TREND_KD", 0.15)),
            error_trend_deadband_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_ERROR_TREND_DEADBAND_640", 5.0)),
            error_trend_max_adjust_640=max(
                0.0, _env_float(
                    "VISION_CONTROL_ERROR_TREND_MAX_ADJUST_640", 20.0)),
            default_track_left_max_error_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_DEFAULT_LEFT_MAX_ERROR_640", 210.0)),
            default_track_left_error_step_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_DEFAULT_LEFT_MAX_STEP_640", 32.0)),
            default_track_right_max_error_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_DEFAULT_RIGHT_MAX_ERROR_640", 210.0)),
            default_track_right_error_step_640=max(
                1.0, _env_float(
                    "VISION_CONTROL_DEFAULT_RIGHT_MAX_STEP_640", 36.0)),
            default_track_left_error_gain=_clamp(
                _env_float("VISION_CONTROL_DEFAULT_LEFT_ERROR_GAIN", 0.75),
                0.0, 1.0),
            default_track_right_error_gain=max(
                0.0, _env_float(
                    "VISION_CONTROL_DEFAULT_RIGHT_ERROR_GAIN", 0.87)),
            normal_speed_mps=max(0.0, _env_float("VISION_CONTROL_NORMAL_SPEED", 0.15)),
            recover_speed_mps=max(0.0, _env_float("VISION_CONTROL_RECOVER_SPEED", 0.15)),
            obstacle_speed_mps=max(0.0, _env_float("VISION_CONTROL_OBSTACLE_SPEED", 0.15)),
            human_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED", 0.40)),
            human_pass_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_SPEED", 0.42)),
            collect_speed_mps=max(0.0, _env_float("VISION_CONTROL_COLLECT_SPEED", 0.15)),
            turnsign_slow_speed_mps=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_SLOW_SPEED", 0.10)),
            heat_threshold=_clamp(_env_float("VISION_CONTROL_HEAT_THRESHOLD", 0.22), 0.01, 0.99),
            blank_probability=_clamp(
                _env_float("VISION_CONTROL_BLANK_PROBABILITY", 0.05),
                0.0, 0.99),
            peak_scan_top_ratio=_clamp(
                _env_float("VISION_CONTROL_PEAK_SCAN_TOP_RATIO", 0.45),
                0.05, 0.90),
            peak_scan_bottom_ratio=_clamp(
                _env_float("VISION_CONTROL_PEAK_SCAN_BOTTOM_RATIO", 0.55),
                0.10, 0.95),
            min_peak_component_area=max(
                1, _env_int("VISION_CONTROL_MIN_PEAK_COMPONENT_AREA", 20)),
            peak_min_distance_px=max(
                1, _env_int("VISION_CONTROL_PEAK_MIN_DISTANCE", 8)),
            greedy_search_radius_px=max(
                1, _env_int("VISION_CONTROL_GREEDY_SEARCH_RADIUS", 8)),
            bottom_reach_ratio=_clamp(
                _env_float("VISION_CONTROL_BOTTOM_REACH_RATIO", 0.90),
                0.60, 1.0),
            side_exit_min_y_ratio=_clamp(
                _env_float("VISION_CONTROL_SIDE_EXIT_MIN_Y_RATIO", 2.0 / 3.0),
                0.40, 0.95),
            side_exit_margin_ratio=_clamp(
                _env_float("VISION_CONTROL_SIDE_EXIT_MARGIN_RATIO", 0.05),
                0.01, 0.25),
            recovery_max_gap_rows=max(
                0, _env_int("VISION_CONTROL_RECOVERY_MAX_GAP_ROWS", 12)),
            recovery_max_radius_px=max(
                1, _env_int("VISION_CONTROL_RECOVERY_MAX_RADIUS", 24)),
            recovery_min_probability=_clamp(
                _env_float("VISION_CONTROL_RECOVERY_MIN_PROBABILITY", 0.35),
                0.05, 0.99),
            recovery_min_continuation_rows=max(
                2, _env_int(
                    "VISION_CONTROL_RECOVERY_MIN_CONTINUATION_ROWS", 8)),
            recovery_ambiguity_margin=max(
                0.0, _env_float(
                    "VISION_CONTROL_RECOVERY_AMBIGUITY_MARGIN", 0.08)),
            curve_fit_blend=_clamp(
                _env_float("VISION_CONTROL_CURVE_FIT_BLEND", 0.25),
                0.0, 0.75),
            road_mask_threshold=_clamp(_env_float("VISION_CONTROL_ROAD_MASK_THRESHOLD", 0.20), 0.0, 1.0),
            heat_peak_top_k=max(1, _env_int("VISION_CONTROL_HEAT_TOP_K", 6)),
            row_step=max(1, _env_int("VISION_CONTROL_ROW_STEP", 2)),
            min_path_points=max(3, _env_int("VISION_CONTROL_MIN_PATH_POINTS", 12)),
            min_path_coverage=_clamp(_env_float("VISION_CONTROL_MIN_PATH_COVERAGE", 0.28), 0.01, 1.0),
            min_mean_heat=_clamp(_env_float("VISION_CONTROL_MIN_MEAN_HEAT", 0.28), 0.01, 0.99),
            min_path_support_probability=_clamp(
                _env_float(
                    "VISION_CONTROL_MIN_PATH_SUPPORT_PROBABILITY",
                    _env_float("VISION_CONTROL_MIN_PATH_SUPPORT", 0.12)),
                0.01, 0.99),
            max_link_jump_px=max(2.0, _env_float("VISION_CONTROL_MAX_LINK_JUMP", 16.0)),
            road_penalty_weight=max(0.0, _env_float("VISION_CONTROL_ROAD_WEIGHT", 0.75)),
            history_weight=max(0.0, _env_float("VISION_CONTROL_HISTORY_WEIGHT", 0.035)),
            jump_weight=max(0.0, _env_float("VISION_CONTROL_JUMP_WEIGHT", 0.018)),
            path_ema_alpha=_clamp(_env_float("VISION_CONTROL_PATH_EMA_ALPHA", 0.32), 0.0, 1.0),
            path_smooth_window=max(1, _env_int("VISION_CONTROL_PATH_SMOOTH_WINDOW", 5)),
            path_max_step_px_640=max(1.0, _env_float("VISION_CONTROL_PATH_MAX_STEP_640", 40.0)),
            path_state_hold_frames=max(1, _env_int("VISION_CONTROL_PATH_HOLD_FRAMES", 8)),
            route_confirm_frames=max(1, _env_int("VISION_CONTROL_ROUTE_CONFIRM_FRAMES", 6)),
            branch_release_frames=max(1, _env_int("VISION_CONTROL_BRANCH_RELEASE_FRAMES", 20)),
            branch_separation_px_640=max(1.0, _env_float("VISION_CONTROL_BRANCH_SEP_640", 70.0)),
            branch_separation_rows=max(1, _env_int("VISION_CONTROL_BRANCH_SEP_ROWS", 8)),
            overlap_px_640=max(1.0, _env_float("VISION_CONTROL_OVERLAP_640", 28.0)),
            fragment_search_radius_px_640=max(
                2.0, _env_float("VISION_CONTROL_FRAGMENT_RADIUS_640", 72.0)),
            fragment_max_tangent_delta_deg=_clamp(
                _env_float(
                    "VISION_CONTROL_FRAGMENT_MAX_TANGENT_DELTA_DEG", 55.0),
                5.0, 85.0),
            heatmap_component_budget=max(
                2, _env_int("VISION_CONTROL_HEATMAP_COMPONENT_BUDGET", 2)),
            default_outer_after_s=max(0.0, _env_float("VISION_CONTROL_DEFAULT_OUTER_AFTER", 15.0)),
            ocr_lock_lifetime_s=max(
                0.1, _env_float("VISION_CONTROL_OCR_LOCK_LIFETIME_S", 10.0)),
            ocr_confirm_frames=max(1, _env_int("VISION_CONTROL_OCR_CONFIRM_FRAMES", 1)),
            outer_slot=max(0, min(1, _env_int("VISION_CONTROL_OUTER_SLOT", 0))),
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
            no_path_stop_s=max(0.1, _env_float("VISION_CONTROL_NO_PATH_STOP_S", 0.8)),
            recover_hold_s=max(0.0, _env_float("VISION_CONTROL_RECOVER_HOLD_S", 0.5)),
            hazard_bottom_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_BOTTOM_RATIO", 250.0 / 480.0), 0.0, 1.0),
            hazard_lateral_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_LATERAL_RATIO", 0.18), 0.01, 0.5),
            coin_bottom_ratio=_clamp(_env_float("VISION_CONTROL_COIN_BOTTOM_RATIO", 0.55), 0.0, 1.0),
            coin_lateral_ratio=_clamp(_env_float("VISION_CONTROL_COIN_LATERAL_RATIO", 0.24), 0.01, 0.5),
            sign_stop_height_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_HEIGHT_RATIO", 0.24), 0.01, 1.0),
            sign_stop_area_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_AREA_RATIO", 0.031), 0.001, 1.0),
            sign_stop_line_margin_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_LINE_MARGIN_RATIO", 0.08), 0.0, 0.5),
            sign_slow_min_score=_clamp(_env_float("VISION_CONTROL_SIGN_SLOW_MIN_SCORE", 0.40), 0.0, 1.0),
            sign_latch_frames=max(1, _env_int("VISION_CONTROL_SIGN_LATCH_FRAMES", 3)),
            sign_ocr_timeout_s=max(0.1, _env_float("VISION_CONTROL_SIGN_OCR_TIMEOUT_S", 8.0)),
            sign_ocr_pulse_speed_mps=max(0.0, _env_float("VISION_CONTROL_SIGN_OCR_PULSE_SPEED", 0.25)),
            sign_ocr_pulse_duration_s=max(0.0, _env_float("VISION_CONTROL_SIGN_OCR_PULSE_DURATION_S", 0.30)),
            turnsign_detection_line_ratio=_clamp(_env_float("VISION_CONTROL_TURNSIGN_LINE_RATIO", 185.0 / 480.0), 0.05, 0.95),
            turnsign_edge_margin_ratio=_clamp(_env_float("VISION_CONTROL_TURNSIGN_EDGE_MARGIN_RATIO", 0.15), 0.0, 0.45),
            turnsign_steer_gain=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_STEER_GAIN", 1.5)),
            turnsign_reverse_speed_mps=-abs(_env_float("VISION_CONTROL_TURNSIGN_REVERSE_SPEED", -0.08)),
            turnsign_reverse_duration_s=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_REVERSE_DURATION_S", 2.0)),
            turnsign_initial_brake_s=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_INITIAL_BRAKE_S", 0.5)),
            human_stop_line_margin_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_LINE_MARGIN_RATIO", 0.0), 0.0, 0.5),
            human_stop_progress_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_PROGRESS_RATIO", 8.0 / 9.0), 0.0, 1.0),
            human_preline_missing_px_480=max(0.0, _env_float("VISION_CONTROL_HUMAN_PRELINE_MISSING_PX_480", 20.0)),
            human_brake_reverse_speed_mps=-abs(_env_float("VISION_CONTROL_HUMAN_BRAKE_REVERSE_SPEED", -0.05)),
            human_brake_reverse_duration_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_BRAKE_REVERSE_DURATION_S", 0.2)),
            human_return_duration_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_RETURN_DURATION_S", 1.0)),
            human_return_error_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_RETURN_ERROR_640", 40.0)),
            human_cross_release_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_CROSS_RELEASE_640", 45.0)),
            human_pass_offset_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_OFFSET_640", 38.0)),
            human_speed_hold_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED_HOLD_S", 1.0)),
            human_absence_confirm_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_ABSENCE_CONFIRM_S", 1.5)),
            car_avoid_offset_px_640=max(0.0, _env_float("VISION_CONTROL_CAR_AVOID_OFFSET_640", 56.0)),
            car_avoid_speed_mps=max(0.0, _env_float("VISION_CONTROL_CAR_AVOID_SPEED", 0.10)),
            car_avoid_ramp_s=max(0.01, _env_float("VISION_CONTROL_CAR_AVOID_RAMP_S", 1.5)),
            car_avoid_hold_s=max(0.0, _env_float("VISION_CONTROL_CAR_AVOID_HOLD_S", 1.0)),
            car_human_pass_speed_mps=max(0.0, _env_float("VISION_CONTROL_CAR_HUMAN_PASS_SPEED", 0.42)),
            car_human_pass_hold_s=max(0.0, _env_float("VISION_CONTROL_CAR_HUMAN_PASS_HOLD_S", 1.0)),
            human_avoid_offset_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_AVOID_OFFSET_640", 75.0)),
            avoid_box_width_gain=max(0.0, _env_float("VISION_CONTROL_AVOID_BOX_WIDTH_GAIN", 0.35)),
            gold_bias_gain=max(0.0, _env_float("VISION_CONTROL_GOLD_BIAS_GAIN", 0.45)),
            gold_max_bias_px_640=max(0.0, _env_float("VISION_CONTROL_GOLD_MAX_BIAS_640", 75.0)),
            min_human_score=_clamp(_env_float("VISION_CONTROL_HUMAN_MIN_SCORE", 0.35), 0.0, 1.0),
            min_car_score=_clamp(_env_float("VISION_CONTROL_CAR_MIN_SCORE", 0.35), 0.0, 1.0),
            min_coin_score=_clamp(_env_float("VISION_CONTROL_COIN_MIN_SCORE", 0.35), 0.0, 1.0),
            path_source=_path_source_from_env(),
        )


class HeatmapPathSearch:
    """Small Viterbi-style path search over one 120x160 heatmap channel."""

    def __init__(self, config=None):
        self.config = config or VisionControlConfig.from_env()

    def search(self, heatmap, road_mask=None, image_shape=(480, 640, 3), slot=0, history_points=None):
        if heatmap is None:
            return None
        heatmap = np.asarray(heatmap, dtype=np.float32)
        if heatmap.ndim != 2 or heatmap.size == 0:
            return None
        heatmap = np.clip(heatmap, 1e-4, 1.0)
        h, w = heatmap.shape[:2]
        road = self._prepare_road(road_mask, (h, w))
        road_threshold = float(self.config.road_mask_threshold)
        if road is not None and np.any(road >= road_threshold):
            heatmap = np.where(
                road >= road_threshold, heatmap, 1e-4).astype(np.float32)
        else:
            road = None
        support_map = cv2.dilate(
            heatmap, np.ones((3, 3), dtype=np.uint8))
        rows = list(range(h - 2, max(int(h * 0.18), 1), -self.config.row_step))
        layers = []
        expected_rows = len(rows)
        for row in rows:
            peaks = self._row_peaks(heatmap[row], self.config.heat_threshold, self.config.heat_peak_top_k)
            if not peaks:
                continue
            nodes = []
            for x, probability in peaks:
                road_support = float(road[row, x]) if road is not None else 1.0
                if road is not None and road_support < road_threshold:
                    continue
                nodes.append({
                    "x": int(x),
                    "y": int(row),
                    "p": float(probability),
                    "road": road_support,
                    "cost": 0.0,
                    "prev": None,
                })
            if nodes:
                layers.append(nodes)
        if not layers:
            return None

        anchor_x = self._history_anchor_x(history_points, image_shape, w)
        center_anchor = self.config.visual_center_x * float(w - 1)
        if anchor_x is None:
            anchor_x = center_anchor
        max_jump = self.config.max_link_jump_px
        for layer_index, nodes in enumerate(layers):
            if layer_index == 0:
                for node in nodes:
                    node["cost"] = self._node_cost(node) + abs(node["x"] - anchor_x) * self.config.history_weight
                    node["length"] = 1
                continue
            previous_nodes = layers[layer_index - 1]
            row_gap = max(1.0, abs(float(nodes[0]["y"] - previous_nodes[0]["y"])) / float(self.config.row_step))
            allowed_jump = max_jump * (1.0 + 0.35 * (row_gap - 1.0))
            for node in nodes:
                best_cost = None
                best_previous = None
                for previous in previous_nodes:
                    jump = abs(float(node["x"] - previous["x"]))
                    if jump > allowed_jump:
                        continue
                    if not self._segment_has_heat_support(
                            previous, node, support_map,
                            self.config.min_path_support_probability):
                        continue
                    cost = previous["cost"] + self._node_cost(node) + jump * self.config.jump_weight
                    if best_cost is None or cost < best_cost:
                        best_cost = cost
                        best_previous = previous
                if best_previous is None:
                    node["cost"] = self._node_cost(node) + abs(node["x"] - anchor_x) * self.config.history_weight
                    node["length"] = 1
                else:
                    node["cost"] = best_cost
                    node["prev"] = best_previous
                    node["length"] = int(best_previous.get("length", 1)) + 1

        possible_end_nodes = [
            node
            for layer in layers
            for node in layer
            if int(node.get("length", 1)) >= self.config.min_path_points
        ]
        if not possible_end_nodes:
            return None
        # Continuous coverage is decisive. Heat magnitude only breaks ties
        # between equally long locally supported chains.
        end_node = min(
            possible_end_nodes,
            key=lambda item: (
                -int(item.get("length", 1)),
                float(item["cost"]) /
                float(max(1, int(item.get("length", 1)))),
            ),
        )
        chain = []
        node = end_node
        while node is not None:
            chain.append(node)
            node = node.get("prev")
        chain.reverse()
        if len(chain) < self.config.min_path_points:
            return None
        coverage = len(chain) / float(max(1, expected_rows))
        mean_heat = float(np.mean([node["p"] for node in chain]))
        if coverage < self.config.min_path_coverage or mean_heat < self.config.min_mean_heat:
            return None

        img_h, img_w = image_shape[:2]
        points = np.zeros((len(chain), 2), dtype=np.float32)
        points[:, 0] = [node["x"] * float(max(img_w - 1, 1)) / float(max(w - 1, 1)) for node in chain]
        points[:, 1] = [node["y"] * float(max(img_h - 1, 1)) / float(max(h - 1, 1)) for node in chain]
        support = _path_heat_support_metrics(
            points, image_shape, support_map[np.newaxis, ...], (0,),
            self.config.min_path_support_probability)
        if not support["valid"]:
            return None
        return {
            "slot": int(slot),
            "source_slot": int(slot),
            "source_slots": {int(slot)},
            "role": "left" if int(slot) == 0 else "right",
            "source": "heatmap_viterbi",
            "points_xy": points,
            "score": mean_heat,
            "heatmap_score": mean_heat,
            "coverage": coverage,
            "road_support": float(np.mean([node["road"] for node in chain])),
            "point_confidences": np.asarray([node["p"] for node in chain], dtype=np.float32),
            "heat_support_ratio": support["support_ratio"],
            "heat_support_min": support["min_probability"],
            "heat_support_mean": support["mean_probability"],
            "heat_support_low_run": support["low_run_pixels"],
        }

    def _node_cost(self, node):
        heat_cost = -math.log(max(1e-4, float(node["p"])))
        road_cost = (1.0 - float(node["road"])) * self.config.road_penalty_weight
        return heat_cost + road_cost

    @staticmethod
    def _segment_has_heat_support(
            first, second, support_map, minimum_probability):
        x0, y0 = int(first["x"]), int(first["y"])
        x1, y1 = int(second["x"]), int(second["y"])
        count = max(abs(x1 - x0), abs(y1 - y0)) + 1
        if count <= 1:
            return float(support_map[y0, x0]) >= float(minimum_probability)
        xs = np.rint(np.linspace(x0, x1, count)).astype(np.int32)
        ys = np.rint(np.linspace(y0, y1, count)).astype(np.int32)
        return bool(np.all(
            support_map[ys, xs] >= float(minimum_probability)))

    @staticmethod
    def _prepare_road(road_mask, shape):
        if road_mask is None:
            return None
        road = np.asarray(road_mask, dtype=np.float32)
        if road.ndim != 2 or road.size == 0:
            return None
        if road.shape[:2] != shape:
            road = cv2.resize(road, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
        return np.clip(road, 0.0, 1.0)

    @staticmethod
    def _row_peaks(values, threshold, top_k):
        values = np.asarray(values, dtype=np.float32)
        if values.size < 3:
            return []
        mask = (
            (values[1:-1] >= values[:-2])
            & (values[1:-1] >= values[2:])
            & (values[1:-1] >= float(threshold))
        )
        xs = np.flatnonzero(mask) + 1
        if xs.size == 0:
            return []
        order = sorted(xs.tolist(), key=lambda x: float(values[x]), reverse=True)
        selected = []
        for x in order:
            if all(abs(int(x) - previous_x) >= 3 for previous_x, _ in selected):
                selected.append((int(x), float(values[x])))
            if len(selected) >= int(top_k):
                break
        return selected

    @staticmethod
    def _history_anchor_x(history_points, image_shape, heatmap_width):
        if history_points is None or len(history_points) == 0:
            return None
        points = np.asarray(history_points, dtype=np.float32)
        bottom = points[np.argmax(points[:, 1])]
        img_w = max(1.0, float(image_shape[1] - 1))
        return float(bottom[0]) * float(max(heatmap_width - 1, 1)) / img_w


class HeatmapPeakPathDetector:
    """Seed paths in a middle band and trace them to a safe screen exit."""

    def __init__(self, config=None):
        self.config = config or VisionControlConfig.from_env()

    def extract(self, heatmaps, image_shape=(480, 640, 3)):
        heatmaps = np.clip(np.asarray(heatmaps, dtype=np.float32), 0.0, 1.0)
        if heatmaps.ndim != 3 or heatmaps.shape[0] < 1:
            return [], self._empty_debug()
        heatmaps = heatmaps[:2]
        _channels, height, width = heatmaps.shape
        if height < 2 or width < 3:
            return [], self._empty_debug()

        top, bottom = self._scan_rows(height)
        channel_profiles = np.mean(
            heatmaps[:, top:bottom + 1, :], axis=1)
        for slot in range(len(channel_profiles)):
            channel_profiles[slot] = cv2.GaussianBlur(
                channel_profiles[slot].reshape((1, -1)),
                (5, 1), 0.0).reshape((-1,))
        profile = np.max(channel_profiles, axis=0)
        peaks = self._profile_peaks(profile)
        component_data = [
            self._components(channel) for channel in heatmaps
        ]

        accepted = []
        examined = []
        for peak_x in peaks:
            peak_probability = float(profile[peak_x])
            record = {
                "x": int(peak_x),
                "probability": peak_probability,
                "accepted": False,
            }
            if peak_probability < float(self.config.heat_threshold):
                record["reason"] = "peak_probability_too_low"
                examined.append(record)
                continue

            candidate = None
            slot_order = np.argsort(channel_profiles[:, peak_x])[::-1]
            slot_rejections = []
            for slot_value in slot_order:
                slot = int(slot_value)
                slot_probability = float(channel_profiles[slot, peak_x])
                if slot_probability < float(self.config.heat_threshold):
                    slot_rejections.append("slot_probability_too_low")
                    continue
                labels, stats = component_data[slot]
                seed = self._seed_in_band(
                    heatmaps[slot], labels, peak_x, top, bottom)
                if seed is None:
                    slot_rejections.append("blank_seed")
                    continue
                seed_x, seed_y, label = seed
                component_area = int(stats[label, cv2.CC_STAT_AREA])
                if component_area < int(self.config.min_peak_component_area):
                    slot_rejections.append("component_too_small")
                    continue
                trace_result = self._trace_component_to_exit(
                    heatmaps[slot], labels, stats,
                    label, seed_x, seed_y)
                if trace_result is None:
                    slot_rejections.append("does_not_reach_allowed_exit")
                    continue
                traced, trace_meta = trace_result
                points, probabilities = self._to_image_points(
                    traced, image_shape, (height, width))
                supported_probabilities = probabilities[
                    probabilities >= float(self.config.blank_probability)]
                mean_probability = float(np.mean(supported_probabilities))
                support_ratio = float(
                    len(supported_probabilities)) / float(len(probabilities))
                candidate = {
                    "source": "heatmap_hysteresis_greedy",
                    "source_slot": slot,
                    "source_slots": {slot},
                    "slot": slot,
                    "points_xy": points,
                    "point_confidences": probabilities,
                    "score": mean_probability,
                    "heatmap_score": mean_probability,
                    "peak_probability": peak_probability,
                    "peak_x": int(peak_x),
                    "peak_y": int(seed_y),
                    "component_area": component_area,
                    "component_count": 1 + int(trace_meta["bridge_count"]),
                    "coverage": float(len(traced)) / float(height),
                    "road_support": 1.0,
                    "valid_exit": True,
                    "exit_type": str(trace_meta["exit_type"]),
                    "reaches_bottom_region": (
                        trace_meta["exit_type"] == "bottom"),
                    "bottom_reach_ratio": float(
                        self.config.bottom_reach_ratio),
                    "side_exit_min_y_ratio": float(
                        self.config.side_exit_min_y_ratio),
                    "occlusion_bridge_count": int(
                        trace_meta["bridge_count"]),
                    "occlusion_bridge_rows": int(
                        trace_meta["bridge_rows"]),
                    "curve_fit_degree": int(
                        trace_meta["curve_fit_degree"]),
                    "curve_fit_rmse": float(
                        trace_meta["curve_fit_rmse"]),
                    "hysteresis_high_threshold": float(
                        self.config.heat_threshold),
                    "hysteresis_low_threshold": float(
                        self.config.blank_probability),
                    "heat_supported": True,
                    "heat_support_ratio": support_ratio,
                    "heat_support_min": float(np.min(probabilities)),
                    "heat_support_mean": mean_probability,
                    "heat_support_low_run": int(trace_meta["bridge_rows"]),
                    "spatial_prefiltered": True,
                }
                record.update({
                    "accepted": True,
                    "source_slot": slot,
                    "component_area": component_area,
                    "exit_type": str(trace_meta["exit_type"]),
                    "bridge_rows": int(trace_meta["bridge_rows"]),
                })
                break

            if candidate is None:
                record["reason"] = (
                    slot_rejections[0] if slot_rejections
                    else "no_supporting_channel")
            else:
                accepted.append(candidate)
            examined.append(record)
            if len(accepted) == 2:
                break

        debug = {
            "scan_rows": [int(top), int(bottom)],
            "scan_ratios": [
                float(top) / float(max(height - 1, 1)),
                float(bottom) / float(max(height - 1, 1)),
            ],
            "blank_probability": float(self.config.blank_probability),
            "peak_probability_threshold": float(self.config.heat_threshold),
            "min_component_area": int(self.config.min_peak_component_area),
            "bottom_reach_ratio": float(self.config.bottom_reach_ratio),
            "side_exit_min_y_ratio": float(
                self.config.side_exit_min_y_ratio),
            "side_exit_margin_ratio": float(
                self.config.side_exit_margin_ratio),
            "peaks_examined": examined,
            "valid_path_count": len(accepted),
            # The requested route decision is binary: two valid paths, or one.
            "detected_path_count": 2 if len(accepted) >= 2 else 1,
        }
        return accepted, debug

    def _scan_rows(self, height):
        top_ratio = float(self.config.peak_scan_top_ratio)
        bottom_ratio = float(self.config.peak_scan_bottom_ratio)
        if bottom_ratio < top_ratio:
            top_ratio, bottom_ratio = bottom_ratio, top_ratio
        top = int(round(top_ratio * float(height - 1)))
        bottom = int(round(bottom_ratio * float(height - 1)))
        top = max(0, min(height - 2, top))
        bottom = max(top + 1, min(height - 1, bottom))
        return top, bottom

    def _profile_peaks(self, profile):
        profile = np.asarray(profile, dtype=np.float32)
        if profile.size < 3:
            return []
        candidates = []
        if profile[0] >= profile[1]:
            candidates.append(0)
        candidates.extend((np.flatnonzero(
            (profile[1:-1] >= profile[:-2]) &
            (profile[1:-1] >= profile[2:])) + 1).tolist())
        if profile[-1] >= profile[-2]:
            candidates.append(len(profile) - 1)
        candidates = sorted(
            set(candidates), key=lambda x: float(profile[x]), reverse=True)
        selected = []
        min_distance = max(1, int(self.config.peak_min_distance_px))
        for x in candidates:
            if float(profile[x]) < float(self.config.blank_probability):
                continue
            if all(abs(int(x) - kept) >= min_distance for kept in selected):
                selected.append(int(x))
        return selected

    def _components(self, heatmap):
        foreground = (
            np.asarray(heatmap, dtype=np.float32) >=
            float(self.config.blank_probability))
        _count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            foreground.astype(np.uint8), connectivity=8)
        return labels, stats

    def _seed_in_band(self, heatmap, labels, peak_x, top, bottom):
        radius = max(1, int(self.config.peak_min_distance_px) // 3)
        left = max(0, int(peak_x) - radius)
        right = min(heatmap.shape[1] - 1, int(peak_x) + radius)
        region = heatmap[top:bottom + 1, left:right + 1]
        if region.size == 0:
            return None
        flat_order = np.argsort(region.reshape(-1))[::-1]
        for flat_index in flat_order:
            local_y, local_x = np.unravel_index(int(flat_index), region.shape)
            seed_x = left + int(local_x)
            seed_y = top + int(local_y)
            label = int(labels[seed_y, seed_x])
            if (label > 0 and
                    float(heatmap[seed_y, seed_x]) >=
                    float(self.config.blank_probability)):
                return seed_x, seed_y, label
        return None

    def _trace_component_to_exit(
            self, heatmap, labels, stats, label, seed_x, seed_y):
        downward = self._trace_direction(
            heatmap, labels, label, seed_x, seed_y, 1, include_seed=True)
        bridge_rows = 0
        bridge_count = 0
        exit_type = self._allowed_exit_type(
            downward[-1] if downward else None, heatmap.shape)
        if exit_type is None:
            recovery = self._recover_below_gap(
                heatmap, labels, stats, downward)
            if recovery is None:
                return None
            bridge, continuation, exit_type = recovery
            bridge_rows = len(bridge)
            bridge_count = 1
            downward.extend(bridge)
            downward.extend(continuation)

        upward = self._trace_direction(
            heatmap, labels, label, seed_x, seed_y, -1,
            include_seed=False)
        traced = list(reversed(downward)) + upward
        if len(traced) < int(self.config.min_path_points):
            return None
        traced, fit_meta = self._fit_and_filter_trace(
            traced, heatmap.shape[1])
        return traced, {
            "exit_type": str(exit_type),
            "bridge_count": bridge_count,
            "bridge_rows": bridge_rows,
            "curve_fit_degree": int(fit_meta["degree"]),
            "curve_fit_rmse": float(fit_meta["rmse"]),
        }

    def _recover_below_gap(self, heatmap, labels, stats, downward):
        """Conservatively reconnect one short occlusion gap below a track."""
        if (not downward or
                int(self.config.recovery_max_gap_rows) <= 0):
            return None
        last_x = float(downward[-1][0])
        last_y = int(downward[-1][1])
        maximum_gap = min(
            int(self.config.recovery_max_gap_rows),
            heatmap.shape[0] - 1 - last_y)
        if maximum_gap <= 0:
            return None

        recent = downward[-min(18, len(downward)):]
        minimum_probability = max(
            float(self.config.heat_threshold),
            float(self.config.recovery_min_probability))
        minimum_area = max(
            int(self.config.min_peak_component_area),
            int(self.config.recovery_min_continuation_rows))
        seen_labels = set()
        candidates = []
        for offset in range(1, maximum_gap + 1):
            row = last_y + offset
            predicted_x = float(self._predict_curve_x(recent, row))
            predicted_x = _clamp(
                predicted_x, 0.0, heatmap.shape[1] - 1.0)
            radius = min(
                int(self.config.recovery_max_radius_px),
                int(self.config.greedy_search_radius_px) + offset)
            left = max(0, int(math.floor(predicted_x - radius)))
            right = min(
                heatmap.shape[1] - 1,
                int(math.ceil(predicted_x + radius)))
            xs = np.arange(left, right + 1, dtype=np.int32)
            eligible = (
                (labels[row, xs] > 0) &
                (heatmap[row, xs] >= minimum_probability))
            xs = xs[eligible]
            for candidate_label in np.unique(labels[row, xs]).tolist():
                candidate_label = int(candidate_label)
                if candidate_label <= 0 or candidate_label in seen_labels:
                    continue
                seen_labels.add(candidate_label)
                if int(stats[candidate_label, cv2.CC_STAT_AREA]) < minimum_area:
                    continue
                label_xs = xs[labels[row, xs] == candidate_label]
                if not len(label_xs):
                    continue
                values = heatmap[row, label_xs].astype(np.float32)
                distances = np.abs(
                    label_xs.astype(np.float32) - predicted_x)
                local_scores = values - distances * float(
                    self.config.jump_weight)
                best_index = int(np.argmax(local_scores))
                candidate_x = int(label_xs[best_index])
                continuation = self._trace_direction(
                    heatmap, labels, candidate_label,
                    candidate_x, row, 1, include_seed=True)
                exit_type = self._allowed_exit_type(
                    continuation[-1] if continuation else None,
                    heatmap.shape)
                if exit_type is None:
                    continue
                minimum_rows = min(
                    int(self.config.recovery_min_continuation_rows),
                    max(2, heatmap.shape[0] - row))
                if len(continuation) < minimum_rows:
                    continue
                distance_ratio = (
                    abs(float(candidate_x) - predicted_x) /
                    float(max(1, radius)))
                continuation_probability = float(np.mean([
                    point[2] for point in continuation[
                        :min(len(continuation), 12)]
                ]))
                score = (
                    float(heatmap[row, candidate_x]) +
                    0.10 * continuation_probability -
                    0.45 * distance_ratio -
                    0.015 * float(offset))
                candidates.append({
                    "score": score,
                    "seed_x": candidate_x,
                    "seed_y": row,
                    "predicted_x": predicted_x,
                    "continuation": continuation,
                    "exit_type": exit_type,
                })

        if not candidates:
            return None
        candidates.sort(key=lambda item: float(item["score"]), reverse=True)
        if (len(candidates) > 1 and
                float(candidates[0]["score"]) -
                float(candidates[1]["score"]) <
                float(self.config.recovery_ambiguity_margin)):
            return None

        selected = candidates[0]
        bridge = []
        for row in range(last_y + 1, int(selected["seed_y"])):
            x = self._predict_curve_x(recent, row)
            x = _clamp(x, 0.0, heatmap.shape[1] - 1.0)
            bridge.append((float(x), float(row), 0.0))
        return (
            bridge,
            list(selected["continuation"]),
            str(selected["exit_type"]),
        )

    def _allowed_exit_type(self, point, heatmap_shape):
        if point is None:
            return None
        height, width = heatmap_shape[:2]
        x, y = float(point[0]), float(point[1])
        bottom_start = int(math.ceil(
            float(self.config.bottom_reach_ratio) *
            float(height - 1)))
        if y >= bottom_start:
            return "bottom"
        side_y_start = int(math.ceil(
            float(self.config.side_exit_min_y_ratio) *
            float(height - 1)))
        side_margin = max(
            1.0,
            float(self.config.side_exit_margin_ratio) *
            float(width - 1))
        if y >= side_y_start and x <= side_margin:
            return "left_side"
        if y >= side_y_start and x >= float(width - 1) - side_margin:
            return "right_side"
        return None

    def _fit_and_filter_trace(self, traced, heatmap_width):
        """Robust quadratic fit with conservative correction of observed x."""
        values = np.asarray(traced, dtype=np.float64)
        if len(values) < 3:
            return traced, {"degree": 0, "rmse": 0.0}
        supported = values[:, 2] >= float(self.config.blank_probability)
        if int(np.count_nonzero(supported)) < 3:
            return traced, {"degree": 0, "rmse": 0.0}
        observed = values[supported]
        degree = 2 if len(observed) >= 7 and np.ptp(observed[:, 1]) >= 6 else 1
        center = float(np.mean(observed[:, 1]))
        scale = max(1.0, float(np.ptp(observed[:, 1])) * 0.5)
        normalized_y = (observed[:, 1] - center) / scale
        weights = np.sqrt(np.clip(
            observed[:, 2], float(self.config.blank_probability), 1.0))
        coefficients = np.polyfit(
            normalized_y, observed[:, 0], degree, w=weights)
        predicted_observed = np.polyval(coefficients, normalized_y)
        residuals = observed[:, 0] - predicted_observed
        median = float(np.median(residuals))
        mad = float(np.median(np.abs(residuals - median)))
        inlier_limit = max(2.0, 3.5 * 1.4826 * mad)
        inliers = np.abs(residuals - median) <= inlier_limit
        if int(np.count_nonzero(inliers)) >= degree + 2:
            coefficients = np.polyfit(
                normalized_y[inliers], observed[inliers, 0], degree,
                w=weights[inliers])
            predicted_observed = np.polyval(coefficients, normalized_y)
            residuals = observed[:, 0] - predicted_observed

        predicted = np.polyval(
            coefficients, (values[:, 1] - center) / scale)
        blend = float(self.config.curve_fit_blend)
        corrections = np.clip(predicted - values[:, 0], -3.0, 3.0)
        values[supported, 0] += blend * corrections[supported]
        values[~supported, 0] = predicted[~supported]
        values[:, 0] = np.clip(values[:, 0], 0.0, heatmap_width - 1.0)
        rmse = float(np.sqrt(np.mean(residuals * residuals)))
        return [tuple(row) for row in values.tolist()], {
            "degree": degree,
            "rmse": rmse,
        }

    def _predict_curve_x(self, points, target_y):
        points = np.asarray(points, dtype=np.float64)
        if len(points) < 2:
            return float(points[-1, 0]) if len(points) else 0.0
        degree = 2 if len(points) >= 7 and np.ptp(points[:, 1]) >= 6 else 1
        center = float(points[-1, 1])
        scale = max(1.0, float(np.ptp(points[:, 1])))
        normalized_y = (points[:, 1] - center) / scale
        weights = np.sqrt(np.clip(
            points[:, 2], float(self.config.blank_probability), 1.0))
        coefficients = np.polyfit(
            normalized_y, points[:, 0], degree, w=weights)
        predicted = float(np.polyval(
            coefficients, (float(target_y) - center) / scale))
        # Keep short extrapolations close to the recent tangent so a noisy
        # quadratic cannot jump to an unrelated lower component.
        tail = points[-min(6, len(points)):]
        linear = np.polyfit(tail[:, 1], tail[:, 0], 1)
        tangent = float(np.polyval(linear, float(target_y)))
        allowance = max(
            2.0, 0.5 * abs(float(target_y) - center))
        return _clamp(predicted, tangent - allowance, tangent + allowance)

    def _trace_direction(
            self, heatmap, labels, label, seed_x, seed_y, step,
            include_seed):
        points = []
        current_x = int(seed_x)
        previous_x = None
        if include_seed:
            points.append((
                float(current_x), float(seed_y),
                float(heatmap[seed_y, current_x])))
        row = int(seed_y) + int(step)
        radius = max(1, int(self.config.greedy_search_radius_px))
        low = float(self.config.blank_probability)
        while 0 <= row < heatmap.shape[0]:
            predicted_x = current_x
            if previous_x is not None:
                predicted_x += current_x - previous_x
            left = max(0, min(current_x, predicted_x) - radius)
            right = min(
                heatmap.shape[1] - 1,
                max(current_x, predicted_x) + radius)
            xs = np.arange(left, right + 1, dtype=np.int32)
            valid = (
                (labels[row, xs] == int(label)) &
                (heatmap[row, xs] >= low))
            xs = xs[valid]
            if not len(xs):
                break
            values = heatmap[row, xs].astype(np.float32)
            jump = np.abs(xs.astype(np.float32) - float(predicted_x))
            scores = values - jump * float(self.config.jump_weight)
            best = int(np.argmax(scores))
            next_x = int(xs[best])
            points.append((
                float(next_x), float(row), float(heatmap[row, next_x])))
            previous_x, current_x = current_x, next_x
            row += int(step)
        return points

    @staticmethod
    def _to_image_points(traced, image_shape, heatmap_shape):
        traced = np.asarray(traced, dtype=np.float32)
        heat_h, heat_w = heatmap_shape
        img_h, img_w = image_shape[:2]
        points = np.empty((len(traced), 2), dtype=np.float32)
        points[:, 0] = (
            traced[:, 0] * float(max(img_w - 1, 1)) /
            float(max(heat_w - 1, 1)))
        points[:, 1] = (
            traced[:, 1] * float(max(img_h - 1, 1)) /
            float(max(heat_h - 1, 1)))
        return points, traced[:, 2].copy()

    @staticmethod
    def _empty_debug():
        return {
            "scan_rows": [],
            "scan_ratios": [],
            "blank_probability": 0.05,
            "peaks_examined": [],
            "valid_path_count": 0,
            "detected_path_count": 1,
        }


class VisionControlPlanner:
    """Convert pure visual perception into one TC264D command and debug packet."""

    def __init__(self, config=None, log_func=None):
        self.config = config or VisionControlConfig.from_env()
        self.path_search = HeatmapPathSearch(self.config)
        self.peak_path_detector = HeatmapPeakPathDetector(self.config)
        self.log_func = log_func
        self.last_selected_points = None
        self.last_path_target_x = None
        self.last_path_target_y = None
        self.last_path_target_slot = None
        self.last_path_target_ts = 0.0
        self.last_slot_points = {}
        self.last_valid_ts = 0.0
        self.last_error = 0.0
        self.error_trend_history = []
        self.error_trend_sign = 0
        self.error_trend_mode = None
        self.route_state = ROUTE_NONE
        self.route_reason = "initial"
        self.route_initialized = False
        self.pending_route_state = None
        self.pending_route_frames = 0
        curve_defaults_blue = self.config.path_source == "curve"
        self.branch_lock = "left" if curve_defaults_blue else None
        self.branch_lock_source = "default" if curve_defaults_blue else None
        self.selected_slot_lock = 0 if curve_defaults_blue else None
        self.selected_slot_missing_frames = 0
        self.curve_merge_override = False
        self.curve_merge_bad_evidence = 0
        self.curve_merge_blue_stable_frames = 0
        self.curve_merge_reason = "inactive"
        self.curve_merge_metrics = {}
        self.selection_reason = "initial"
        self.path_missing_frames = {}
        self.fork_seen_since = None
        self.single_seen_frames = 0
        self.last_valid_ocr_ts = 0.0
        self.last_ocr_direction = None
        self.ocr_pending_direction = None
        self.ocr_pending_frames = 0
        self.ocr_confirmed_current = False
        self.human_waiting_cross = False
        self.human_last_side = None
        self.human_pass_active = False
        self.human_pass_side = None
        self.human_pass_offset_x = 0.0
        self.human_speed_hold_until = 0.0
        self.human_detected_latched = False
        self.human_last_seen_ts = 0.0
        self.human_preline_last_gap_px_480 = None
        self.human_preline_wait_until = 0.0
        self.human_brake_reverse_until = 0.0
        self.human_return_start_ts = 0.0
        self.human_return_until = 0.0
        self.human_return_error_sign = 0
        self.human_consumed_active = False
        self.human_consumed_clear_since = 0.0
        self.human_new_pending = False
        self.human_new_last_geom = None
        self.human_new_last_seen_ts = 0.0
        self.human_current_prefer_upper = False
        self.car_avoid_side = 0
        self.car_avoid_offset_px_640 = 0.0
        self.car_avoid_target_offset_px_640 = 0.0
        self.car_avoid_phase = "idle"
        self.car_avoid_parallel_elapsed_s = 0.0
        self.car_avoid_return_start_offset_px_640 = 0.0
        self.car_avoid_last_update_ts = 0.0
        self.car_avoid_last_motion_active = False
        self.car_avoid_hold_until = 0.0
        self.car_human_active = False
        self.car_human_waiting_cross = False
        self.car_human_seen_avoid_side = False
        self.car_human_last_seen_ts = 0.0
        self.car_human_pass_until = 0.0
        self.sign_ocr_active_since = None
        self.sign_ocr_pulse_until = 0.0
        self.sign_ocr_pulse_sent = False
        self.sign_seen_frames = 0
        self.sign_latched_since = None
        self.turnsign_reverse_until = 0.0
        self.turnsign_post_reverse_stop = False
        self.turnsign_control_session_id = None
        self.turnsign_brake_until = 0.0

    def update(self, perception_result, ocr_response=None, now=None):
        now = float(time.monotonic() if now is None else now)
        self._advance_car_avoidance_state(now)
        start = time.perf_counter()
        perception_result = perception_result if isinstance(perception_result, dict) else {}
        image_shape = self._image_shape(perception_result)
        candidates, search_ms = self._extract_candidates(perception_result, image_shape)
        raw_route_state, raw_route_reason = self._classify_routes(
            candidates, image_shape)
        route_state, route_reason = self._stabilize_route_state(
            raw_route_state, raw_route_reason)
        raw_ocr_direction, raw_ocr_current = self._extract_ocr_direction(ocr_response)
        ocr_direction, ocr_current = self._confirm_ocr_direction(
            raw_ocr_direction, raw_ocr_current)
        ocr_lock_expired = False
        if ocr_current:
            self.last_valid_ocr_ts = now
            self.last_ocr_direction = ocr_direction
            self.branch_lock = ocr_direction
            self.branch_lock_source = "ocr"
            self.selected_slot_lock = 0 if ocr_direction == "left" else 1
            self.selected_slot_missing_frames = 0
        else:
            ocr_lock_expired = self._expire_ocr_lock(now)
        self._update_default_outer(route_state, now, ocr_current)
        self._update_curve_merge_continuity(candidates, image_shape)
        selected = self._select_candidate(candidates, route_state, image_shape)
        command, control_target = self._build_command(
            selected, route_state, route_reason, perception_result,
            image_shape, now, ocr_response)
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
            "curve_merge_override": bool(self.curve_merge_override),
            "curve_merge_reason": self.curve_merge_reason,
            "curve_merge_bad_evidence": self.curve_merge_bad_evidence,
            "curve_merge_blue_stable_frames": (
                self.curve_merge_blue_stable_frames),
            "curve_merge_metrics": dict(self.curve_merge_metrics),
            "raw_ocr_direction": raw_ocr_direction if raw_ocr_current else None,
            "raw_ocr_current": bool(raw_ocr_current),
            "ocr_pending_direction": self.ocr_pending_direction,
            "ocr_pending_frames": self.ocr_pending_frames,
            "ocr_confirm_frames": self.config.ocr_confirm_frames,
            "ocr_direction": ocr_direction if ocr_current else None,
            "ocr_current": bool(ocr_current),
            "ocr_lock_expired": bool(ocr_lock_expired),
            "ocr_lock_age_s": self._ocr_lock_age(now),
            "ocr_lock_remaining_s": self._ocr_lock_remaining(now),
            "default_outer_elapsed": self._default_outer_elapsed(now),
            "selected_slot": None if selected is None else int(selected.get("slot", -1)),
            "candidate_count": len(candidates),
            "detected_path_count": int(perception_result.get(
                "detected_path_count", 2 if len(candidates) >= 2 else 1)),
            "valid_path_count": int((perception_result.get(
                "heatmap_peak_detection") or {}).get(
                    "valid_path_count", len(candidates))),
            "heatmap_peak_detection": dict(perception_result.get(
                "heatmap_peak_detection") or {}),
            "candidates": [self._summarize_candidate(item, image_shape) for item in candidates],
            "heatmap_lines": [
                self._debug_path_points(item)
                for item in perception_result.get("heatmap_debug_lines", [])
            ],
            "candidate_paths": [self._debug_path_points(item) for item in candidates],
            "selected_path": None if selected is None else self._debug_path_points(selected),
            "control_target": control_target,
            "command": dict(command) if command else None,
            "timings_ms": {
                "path_search": float(search_ms),
                "control_total": float(elapsed_ms),
            },
        }
        perception_result["vision_control"] = debug
        self.car_avoid_last_motion_active = bool(
            self.car_avoid_side != 0
            and command is not None
            and float(command.get("target_speed", 0.0)) > 0.0
            and int(command.get("state_cmd", STATE_TRACK)) in {
                STATE_AVOID_CAR, STATE_AVOID_HUMAN
            }
        )
        return command, debug

    def _merge_close_display_paths(self, candidates, image_shape):
        display = []
        for candidate in candidates[:2]:
            item = dict(candidate)
            item["points_xy"] = np.asarray(candidate.get("points_xy"), dtype=np.float32).copy()
            item["display_segments_xy"] = [item["points_xy"].copy()]
            display.append(item)
        if len(display) != 2:
            return display
        first = display[0]["points_xy"]
        second = display[1]["points_xy"]
        if len(first) < 2 or len(second) < 2:
            return display
        overlap_threshold = (
            self.config.overlap_px_640 *
            float(max(1, image_shape[1])) / 640.0)
        merged_first = first.copy()
        merged_second = second.copy()
        first_other_x = _interp_path_x_many(second, first[:, 1])
        second_other_x = _interp_path_x_many(first, second[:, 1])
        first_overlap = (
            np.isfinite(first_other_x) &
            (np.abs(first[:, 0] - first_other_x) <= overlap_threshold)
        )
        second_overlap = (
            np.isfinite(second_other_x) &
            (np.abs(second[:, 0] - second_other_x) <= overlap_threshold)
        )
        merged_first[first_overlap, 0] = 0.5 * (
            first[first_overlap, 0] + first_other_x[first_overlap])
        merged_second[second_overlap, 0] = 0.5 * (
            second[second_overlap, 0] + second_other_x[second_overlap])
        second_is_separate = ~second_overlap
        display[0]["points_xy"] = merged_first
        display[1]["points_xy"] = merged_second
        display[0]["display_segments_xy"] = [merged_first]
        display[1]["display_segments_xy"] = self._split_visible_segments(
            merged_second, second_is_separate)
        return display

    @staticmethod
    def _split_visible_segments(points, visible):
        points = np.asarray(points, dtype=np.float32)
        visible = np.asarray(visible, dtype=bool)
        segments = []
        start = None
        for index, is_visible in enumerate(visible.tolist() + [False]):
            if is_visible and start is None:
                start = index
            elif not is_visible and start is not None:
                segment = points[start:index]
                if len(segment) >= 2:
                    segments.append(segment.copy())
                start = None
        return segments

    def _extract_candidates(self, result, image_shape):
        started = time.perf_counter()
        centerline = result.get("centerline") or {}
        support_maps = None
        road = (result.get("road") or {}).get("mask")
        if road is None:
            road = result.get("road_mask")
        if self.config.path_source == "curve":
            paths = (centerline.get("curve_paths") or
                     result.get("curve_paths") or [])
            candidates = []
            for path in paths:
                points = np.asarray(path.get("points_xy"), dtype=np.float32)
                if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
                    continue
                candidate = dict(path)
                candidate["points_xy"] = points
                candidate["source"] = "row_classifier"
                candidate["source_slot"] = int(path.get("slot", len(candidates)))
                candidate["source_slots"] = {candidate["source_slot"]}
                candidate["coverage"] = 1.0
                candidate["road_support"] = path.get("road_overlap")
                # This keeps only the existing small moving-average filter;
                # no B-spline fit or unsupported-point extrapolation is used.
                candidates.append(candidate)
            candidates = candidates[:2]
        else:
            heatmaps = centerline.get("heatmaps")
            if heatmaps is None:
                heatmaps = result.get("path_heatmaps")
            if heatmaps is None:
                return [], 0.0
            heatmaps = np.asarray(heatmaps, dtype=np.float32)
            if heatmaps.ndim != 3 or heatmaps.shape[0] < 1:
                return [], 0.0
            effective_maps, support_maps, _has_road = (
                self._prepare_heatmap_support_maps(heatmaps, road))
            candidates, peak_debug = self.peak_path_detector.extract(
                effective_maps, image_shape=image_shape)
            result["heatmap_peak_detection"] = peak_debug
            result["detected_path_count"] = int(
                peak_debug["detected_path_count"])
            centerline["detected_path_count"] = int(
                peak_debug["detected_path_count"])
            centerline["valid_path_count"] = int(
                peak_debug["valid_path_count"])
            result["heatmap_debug_lines"] = list(candidates)

        if self.config.path_source != "curve":
            candidates = self._assign_heatmap_slots(candidates[:2], image_shape)
        candidates.sort(key=lambda item: int(item.get("slot", 99)))
        candidates = self._smooth_candidates(
            candidates, image_shape, support_maps=support_maps)
        candidates = self._constrain_candidates_to_road(
            candidates, road, image_shape)
        self._publish_filtered_paths(result, candidates)
        elapsed = (time.perf_counter() - started) * 1000.0
        return candidates, elapsed

    def _prepare_heatmap_support_maps(self, heatmaps, road_mask):
        heatmaps = np.clip(
            np.asarray(heatmaps, dtype=np.float32)[:2], 0.0, 1.0)
        heat_h, heat_w = heatmaps.shape[1:3]
        road = HeatmapPathSearch._prepare_road(
            road_mask, (heat_h, heat_w))
        has_road = (
            road is not None and
            np.any(road >= float(self.config.road_mask_threshold)))
        if has_road:
            effective = np.where(
                road[np.newaxis, ...] >=
                float(self.config.road_mask_threshold),
                heatmaps, 0.0).astype(np.float32)
        else:
            effective = heatmaps.copy()
        support = np.empty_like(effective)
        kernel = np.ones((3, 3), dtype=np.uint8)
        for slot in range(len(effective)):
            support[slot] = cv2.dilate(effective[slot], kernel)
        return effective, support, has_road

    def _annotate_heat_support(
            self, line, image_shape, support_maps,
            require_all_channels=False):
        source_slots = line.get("source_slots")
        if not source_slots:
            source_slots = {int(line.get(
                "source_slot", line.get("slot", -1)))}
        metrics = _path_heat_support_metrics(
            line.get("points_xy"), image_shape, support_maps,
            source_slots, float(line.get(
                "hysteresis_low_threshold",
                self.config.min_path_support_probability)),
            require_all_channels=require_all_channels)
        line["heat_support_ratio"] = metrics["support_ratio"]
        line["heat_support_min"] = metrics["min_probability"]
        line["heat_support_mean"] = metrics["mean_probability"]
        line["heat_support_low_run"] = metrics["low_run_pixels"]
        line["heat_support_samples"] = metrics["sample_count"]
        line["heat_supported"] = bool(metrics["valid"])
        return bool(metrics["valid"])

    def _extract_heatmap_debug_lines(
            self, heatmaps, road_mask, image_shape, support_maps=None,
            return_invalid=False):
        heatmaps = np.asarray(heatmaps, dtype=np.float32)
        if heatmaps.ndim != 3:
            return []
        h, w = heatmaps.shape[1:3]
        road = HeatmapPathSearch._prepare_road(road_mask, (h, w))
        road_threshold = float(self.config.road_mask_threshold)
        has_road = road is not None and np.any(road >= road_threshold)
        if support_maps is None:
            effective_maps, support_maps, _has_road = (
                self._prepare_heatmap_support_maps(heatmaps, road_mask))
        else:
            effective_maps = heatmaps
        lines = []
        invalid_slots = set()
        component_groups = []
        for slot in range(min(2, heatmaps.shape[0])):
            heatmap = np.clip(effective_maps[slot], 0.0, 1.0)
            mask = heatmap >= float(self.config.heat_threshold)
            if has_road:
                mask &= road >= road_threshold
            components, labels, stats, _centroids = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), connectivity=8)
            minimum = max(4, self.config.min_path_points // 2)
            valid_labels = np.flatnonzero(
                (stats[1:, cv2.CC_STAT_AREA] >= minimum) &
                (stats[1:, cv2.CC_STAT_HEIGHT] >= minimum)) + 1
            label_heat_sums = np.bincount(
                labels.reshape(-1),
                weights=heatmap.reshape(-1),
                minlength=components)
            if not len(valid_labels):
                component_groups.append([])
                continue
            mean_heat = (
                label_heat_sums[valid_labels] /
                np.maximum(
                    1, stats[valid_labels, cv2.CC_STAT_AREA]))
            priorities = (
                stats[valid_labels, cv2.CC_STAT_HEIGHT].astype(
                    np.float32) * 4.0 +
                np.sqrt(stats[valid_labels, cv2.CC_STAT_AREA].astype(
                    np.float32)) + mean_heat.astype(np.float32) * 0.01)
            order = np.argsort(priorities)[::-1]
            component_groups.append([
                (float(priorities[index]), int(slot),
                 int(valid_labels[index]), heatmap, labels, stats,
                 label_heat_sums)
                for index in order
            ])

        selected_components = []
        budget = max(2, int(self.config.heatmap_component_budget))
        # First preserve one globally strong region per populated channel;
        # then spend any remaining budget on the strongest leftover regions.
        for group in component_groups:
            if group and len(selected_components) < budget:
                selected_components.append(group.pop(0))
        while len(selected_components) < budget:
            available = [group[0] for group in component_groups if group]
            if not available:
                break
            best = max(available, key=lambda item: item[0])
            selected_components.append(best)
            component_groups[int(best[1])].pop(0)

        for (_priority, slot, label, heatmap, labels, stats,
             label_heat_sums) in selected_components:
                points = self._component_centerline_points(
                    labels, label, heatmap, image_shape,
                    component_stats=stats[label])
                if len(points) < max(3, self.config.min_path_points // 2):
                    continue
                item = {
                    "slot": int(slot),
                    "source_slot": int(slot),
                    "source_slots": {int(slot)},
                    "points_xy": points,
                    "score": float(label_heat_sums[label]) /
                    float(max(1, stats[label, cv2.CC_STAT_AREA])),
                    "heat_mass": float(label_heat_sums[label]),
                    "component_count": 1,
                    "component_rows": int(stats[label, cv2.CC_STAT_HEIGHT]),
                    "component_area": int(stats[label, cv2.CC_STAT_AREA]),
                    "spatial_prefiltered": True,
                }
                self._refresh_fragment_geometry(item)
                if self._annotate_heat_support(
                        item, image_shape, support_maps):
                    lines.append(item)
                else:
                    invalid_slots.add(int(slot))
        joined = self._join_heatmap_fragments(
            lines, image_shape, support_maps=support_maps)
        if return_invalid:
            return joined, invalid_slots
        return joined

    def _find_heatmap_intersection(self, heatmaps, road, image_shape):
        if heatmaps.shape[0] < 2:
            return None
        threshold = float(self.config.heat_threshold)
        support = heatmaps[:2] >= threshold
        if not np.any(support[0]) or not np.any(support[1]):
            return None

        if _skeletonize is not None:
            skeletons = np.stack([
                _skeletonize(channel).astype(np.uint8)
                for channel in support
            ])
            method = "skimage_skeleton"
        else:
            skeletons = support.astype(np.uint8)
            method = "binary_support"

        h, w = support.shape[1:3]
        radius = max(
            1,
            int(math.ceil(
                self.config.overlap_px_640 * float(w) / 640.0)),
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1))
        near_first = skeletons[0].astype(bool) & (
            cv2.dilate(skeletons[1], kernel) != 0)
        near_second = skeletons[1].astype(bool) & (
            cv2.dilate(skeletons[0], kernel) != 0)
        seeds = (near_first | near_second).astype(np.uint8)
        if not np.any(seeds):
            return None

        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            seeds, connectivity=8)
        kept = np.zeros_like(seeds)
        min_rows = max(1, int(self.config.intersection_min_rows))
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            if max(area, height) >= min_rows:
                kept[labels == label] = 1
        if not np.any(kept):
            return None

        row_flags = np.any(kept != 0, axis=1)
        gap_rows = max(0, int(self.config.intersection_gap_rows))
        if gap_rows:
            row_flags = cv2.morphologyEx(
                row_flags.astype(np.uint8).reshape((-1, 1)),
                cv2.MORPH_CLOSE,
                np.ones((gap_rows * 2 + 1, 1), dtype=np.uint8),
            ).reshape((-1,)).astype(bool)
        regions = self._true_runs(row_flags, min_rows)
        if not regions:
            return None

        kept_rows = np.zeros(h, dtype=bool)
        for start, end in regions:
            kept_rows[start:end + 1] = True
        kept &= kept_rows[:, np.newaxis].astype(np.uint8)

        expanded = cv2.dilate(kept, kernel) != 0
        overlap_probability = np.minimum(
            cv2.dilate(heatmaps[0], kernel),
            cv2.dilate(heatmaps[1], kernel),
        )
        intersection_mask = (
            expanded & np.any(support, axis=0) &
            (road >= float(self.config.road_mask_threshold)) &
            kept_rows[:, np.newaxis]
        )

        raw_centers = np.full(h, np.nan, dtype=np.float32)
        for y in np.flatnonzero(np.any(kept != 0, axis=1)):
            xs = np.flatnonzero(kept[y] != 0)
            weights = overlap_probability[y, xs].astype(np.float32)
            if float(np.sum(weights)) > 0.0:
                raw_centers[y] = float(np.average(xs, weights=weights))
            else:
                raw_centers[y] = float(np.mean(xs))

        known_rows = np.flatnonzero(np.isfinite(raw_centers))
        if not known_rows.size:
            return None
        centers = raw_centers.copy()
        requested_rows = np.flatnonzero(kept_rows)
        centers[requested_rows] = np.interp(
            requested_rows, known_rows, raw_centers[known_rows])

        img_h, img_w = image_shape[:2]
        points = np.asarray([
            [
                float(centers[y]) * float(max(img_w - 1, 1)) /
                float(max(w - 1, 1)),
                float(y) * float(max(img_h - 1, 1)) /
                float(max(h - 1, 1)),
            ]
            for y in requested_rows[::-1]
        ], dtype=np.float32)
        image_regions = [
            [
                float(start) * float(max(img_h - 1, 1)) /
                float(max(h - 1, 1)),
                float(end) * float(max(img_h - 1, 1)) /
                float(max(h - 1, 1)),
            ]
            for start, end in regions
        ]
        return {
            "mask": intersection_mask,
            "points_xy": points,
            "regions_y": image_regions,
            "heatmap_regions_y": [list(item) for item in regions],
            "score": float(np.mean(overlap_probability[kept != 0])),
            "method": method,
        }

    @staticmethod
    def _true_runs(flags, min_length):
        flags = np.asarray(flags, dtype=bool)
        runs = []
        start = None
        for index, value in enumerate(flags.tolist() + [False]):
            if value and start is None:
                start = index
            elif not value and start is not None:
                if index - start >= int(min_length):
                    runs.append((start, index - 1))
                start = None
        return runs

    def _attach_intersection_paths(self, lines, intersection, image_shape):
        if intersection is None:
            self.intersection_missing_frames += 1
            if self.intersection_missing_frames > self.config.path_state_hold_frames:
                self.last_intersection_points = None
            return lines

        shared_points = np.asarray(
            intersection.get("points_xy"), dtype=np.float32)
        if len(shared_points) < 2:
            return lines
        shared_points = self._smooth_path_points(
            shared_points, self.last_intersection_points, image_shape[1])
        self.last_intersection_points = shared_points.copy()
        self.intersection_missing_frames = 0
        intersection["points_xy"] = shared_points

        common = {
            "points_xy": shared_points,
            "score": float(intersection.get("score", 0.0)),
            "component_count": 1,
        }
        if not lines:
            return [{
                "slot": 0,
                "source_slot": 0,
                "role": "left",
                "points_xy": shared_points.copy(),
                "branch_points_xy": np.empty((0, 2), dtype=np.float32),
                "score": float(intersection.get("score", 0.0)),
                "component_count": 1,
                "component_rows": len(shared_points),
                "vertical_span_px": self._vertical_span(shared_points),
                "intersection_points_xy": shared_points.copy(),
                "intersection_y_ranges": list(intersection.get("regions_y") or []),
                "intersection_method": intersection.get("method"),
                "shared_only": True,
            }]

        attached = []
        for line in lines:
            branch_points = np.asarray(
                line.get("points_xy"), dtype=np.float32).copy()
            merged = self._blend_heatmap_evidence(line, common)
            merged["branch_points_xy"] = branch_points
            merged["intersection_points_xy"] = shared_points.copy()
            merged["intersection_y_ranges"] = list(
                intersection.get("regions_y") or [])
            merged["intersection_method"] = intersection.get("method")
            attached.append(merged)
        return attached

    def _lock_intersection_points(self, candidates):
        locked = []
        for candidate in candidates:
            shared_value = candidate.get("intersection_points_xy")
            ranges = candidate.get("intersection_y_ranges") or []
            if shared_value is None or not ranges:
                locked.append(candidate)
                continue
            shared = np.asarray(shared_value, dtype=np.float32)
            if len(shared) < 2 or not ranges:
                locked.append(candidate)
                continue
            item = dict(candidate)
            points = np.asarray(
                candidate.get("points_xy"), dtype=np.float32).copy()
            for index, point in enumerate(points):
                if not VisionControlPlanner._y_in_ranges(point[1], ranges):
                    continue
                shared_x = _interp_path_x(shared, point[1])
                if shared_x is not None:
                    points[index, 0] = shared_x
            item["points_xy"] = points
            self.last_slot_points[int(item.get("slot", -1))] = points.copy()
            locked.append(item)
        return locked

    @staticmethod
    def _y_in_ranges(y, ranges):
        return any(
            min(float(start), float(end)) <= float(y) <=
            max(float(start), float(end))
            for start, end in ranges
        )

    @staticmethod
    def _intersection_debug(intersection):
        if intersection is None:
            return {"active": False, "regions_y": [], "points_xy": []}
        points = np.asarray(intersection.get("points_xy"), dtype=np.float32)
        return {
            "active": True,
            "method": str(intersection.get("method") or ""),
            "score": float(intersection.get("score", 0.0)),
            "regions_y": [list(item) for item in intersection.get("regions_y", [])],
            "points_xy": VisionControlPlanner._debug_points(points),
        }

    @staticmethod
    def _vertical_span(points):
        points = np.asarray(points, dtype=np.float32)
        if len(points) < 2:
            return 0.0
        return float(np.max(points[:, 1]) - np.min(points[:, 1]))

    def _join_heatmap_fragments(
            self, lines, image_shape, support_maps=None):
        if (support_maps is not None and
                self.config.heat_threshold <=
                self.config.min_path_support_probability):
            # Separate threshold-components cannot have a connector that is
            # simultaneously above an equal-or-higher support threshold.
            return [dict(item) for item in lines]
        remaining = sorted(
            (dict(item) for item in lines),
            key=self._heatmap_line_rank,
            reverse=True)
        tracks = []
        while remaining:
            track = remaining.pop(0)
            track["points_xy"] = np.asarray(
                track.get("points_xy"), dtype=np.float32).copy()
            self._refresh_fragment_geometry(track)
            while remaining:
                best_index = None
                best_connection = None
                best_key = None
                for index, fragment in enumerate(remaining):
                    if not self._fragment_endpoints_maybe_close(
                            track, fragment, image_shape):
                        continue
                    connection = self._fragment_join_cost(
                        track, fragment, image_shape,
                        support_maps=support_maps)
                    if connection is None:
                        continue
                    key = (
                        -float(connection["support_ratio"]),
                        float(connection["distance"]),
                        float(connection["tangent_penalty"]),
                        -float(fragment.get("heat_mass", 0.0)),
                    )
                    if best_key is None or key < best_key:
                        best_index = index
                        best_connection = connection
                        best_key = key
                if best_index is None:
                    break
                fragment = remaining.pop(best_index)
                track = self._merge_heatmap_fragments(
                    track, fragment,
                    points_xy=best_connection["points_xy"])
            tracks.append(track)
        return tracks

    @staticmethod
    def _refresh_fragment_geometry(line):
        points = np.asarray(line.get("points_xy"), dtype=np.float32)
        if len(points) < 2:
            line["fragment_y_min"] = 0.0
            line["fragment_y_max"] = 0.0
            return
        if float(points[0, 1]) < float(points[-1, 1]):
            points = points[::-1].copy()
            line["points_xy"] = points
        line["fragment_y_max"] = float(points[0, 1])
        line["fragment_y_min"] = float(points[-1, 1])

    def _fragment_endpoints_maybe_close(
            self, first, second, image_shape):
        first_points = np.asarray(first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(second.get("points_xy"), dtype=np.float32)
        if len(first_points) < 2 or len(second_points) < 2:
            return False
        first_min = float(first.get(
            "fragment_y_min", first_points[-1, 1]))
        first_max = float(first.get(
            "fragment_y_max", first_points[0, 1]))
        second_min = float(second.get(
            "fragment_y_min", second_points[-1, 1]))
        second_max = float(second.get(
            "fragment_y_max", second_points[0, 1]))
        if first_min > second_max:
            first_end = first_points[-1]
            second_end = second_points[0]
        elif second_min > first_max:
            first_end = first_points[0]
            second_end = second_points[-1]
        else:
            return False
        radius = (
            self.config.fragment_search_radius_px_640 *
            float(max(1, image_shape[1])) / 640.0)
        delta = first_end - second_end
        return float(np.dot(delta, delta)) <= radius * radius

    def _fragment_join_cost(
            self, first, second, image_shape, support_maps=None):
        first_points = np.asarray(
            first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(
            second.get("points_xy"), dtype=np.float32)
        if len(first_points) < 2 or len(second_points) < 2:
            return None
        first_min = float(first.get(
            "fragment_y_min", first_points[-1, 1]))
        first_max = float(first.get(
            "fragment_y_max", first_points[0, 1]))
        second_min = float(second.get(
            "fragment_y_min", second_points[-1, 1]))
        second_max = float(second.get(
            "fragment_y_max", second_points[0, 1]))
        if first_min > second_max:
            lower, upper = first_points, second_points
            gap = first_min - second_max
        elif second_min > first_max:
            lower, upper = second_points, first_points
            gap = second_min - first_max
        else:
            return None

        radius = (
            self.config.fragment_search_radius_px_640 *
            float(max(1, image_shape[1])) / 640.0)
        lower_end = lower[-1]
        upper_start = upper[0]
        connector = upper_start - lower_end
        distance = math.hypot(
            float(connector[0]), float(connector[1]))
        if distance > radius or distance <= 1e-6:
            return None
        direction = connector / distance
        lower_tangent = self._endpoint_tangent(lower, at_end=True)
        upper_tangent = self._endpoint_tangent(upper, at_end=False)
        lower_alignment = float(np.dot(lower_tangent, direction))
        upper_alignment = float(np.dot(upper_tangent, direction))
        tangent_similarity = float(np.dot(
            lower_tangent, upper_tangent))
        minimum_cosine = math.cos(math.radians(
            self.config.fragment_max_tangent_delta_deg))
        if min(
                lower_alignment, upper_alignment,
                tangent_similarity) < minimum_cosine:
            return None
        source_slots = (
            set(first.get("source_slots") or {
                int(first.get("source_slot", -1))}) |
            set(second.get("source_slots") or {
                int(second.get("source_slot", -1))}))
        support_ratio = 1.0
        if support_maps is not None:
            first_slots = set(first.get("source_slots") or {
                int(first.get("source_slot", -1))})
            second_slots = set(second.get("source_slots") or {
                int(second.get("source_slot", -1))})
            support = _path_heat_support_metrics(
                np.asarray([lower_end, upper_start], dtype=np.float32),
                image_shape, support_maps, source_slots,
                self.config.min_path_support_probability,
                require_all_channels=first_slots != second_slots)
            if not support["valid"]:
                return None
            support_ratio = float(support["support_ratio"])
        return {
            "support_ratio": support_ratio,
            "distance": float(distance),
            "vertical_gap": float(gap),
            "tangent_penalty": float(
                3.0 - lower_alignment - upper_alignment -
                tangent_similarity),
            "points_xy": np.concatenate((lower, upper), axis=0),
        }

    @staticmethod
    def _endpoint_tangent(points, at_end):
        points = np.asarray(points, dtype=np.float32)
        reach = min(4, len(points) - 1)
        vector = (
            points[-1] - points[-1 - reach] if at_end
            else points[reach] - points[0])
        norm = math.hypot(float(vector[0]), float(vector[1]))
        if norm <= 1e-6:
            return np.zeros(2, dtype=np.float32)
        return vector / norm

    def _merge_heatmap_fragments(
            self, first, second, points_xy=None):
        first_points = np.asarray(first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(second.get("points_xy"), dtype=np.float32)
        points = np.asarray(
            points_xy if points_xy is not None else
            np.concatenate((first_points, second_points), axis=0),
            dtype=np.float32)
        first_count = max(1, len(first_points))
        second_count = max(1, len(second_points))
        merged = dict(first)
        merged["points_xy"] = points
        merged["score"] = (
            float(first.get("score", 0.0)) * first_count +
            float(second.get("score", 0.0)) * second_count
        ) / float(first_count + second_count)
        merged["component_rows"] = (
            int(first.get("component_rows", first_count)) +
            int(second.get("component_rows", second_count))
        )
        merged["component_count"] = (
            int(first.get("component_count", 1)) +
            int(second.get("component_count", 1))
        )
        merged["vertical_span_px"] = self._vertical_span(points)
        merged["heat_mass"] = (
            float(first.get("heat_mass", 0.0)) +
            float(second.get("heat_mass", 0.0)))
        merged["component_area"] = (
            int(first.get("component_area", 0)) +
            int(second.get("component_area", 0)))
        merged["source_slots"] = (
            set(first.get("source_slots") or {
                int(first.get("source_slot", -1))}) |
            set(second.get("source_slots") or {
                int(second.get("source_slot", -1))}))
        self._refresh_fragment_geometry(merged)
        return merged

    def _select_heatmap_lines(self, lines, image_shape):
        ordered = sorted(
            lines, key=self._heatmap_line_rank, reverse=True)
        selected = []
        for line in ordered:
            if any(self._same_overall_path(
                    line, kept, image_shape) for kept in selected):
                continue
            selected.append(line)
            if len(selected) == 2:
                break
        return selected

    def _heatmap_line_rank(self, line):
        points = np.asarray(line.get("points_xy"), dtype=np.float32)
        span = self._vertical_span(points)
        arc = self._path_arc_length(points)
        return (
            span,
            float(line.get("heat_support_ratio", 0.0)),
            arc,
            float(line.get("heat_support_min", 0.0)),
            float(line.get("score", 0.0)),
        )

    def _same_overall_path(self, first, second, image_shape):
        first_points = np.asarray(first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(second.get("points_xy"), dtype=np.float32)
        if len(first_points) < 2 or len(second_points) < 2:
            return False
        low = max(
            float(np.min(first_points[:, 1])),
            float(np.min(second_points[:, 1])))
        high = min(
            float(np.max(first_points[:, 1])),
            float(np.max(second_points[:, 1])))
        if low <= high:
            rows = low + (high - low) * _PAIR_FRACTIONS
            distances = np.abs(
                _interp_path_x_many(first_points, rows) -
                _interp_path_x_many(second_points, rows))
            distances *= 640.0 / float(max(1, image_shape[1]))
            separated = int(np.count_nonzero(
                distances >= self.config.branch_separation_px_640))
            return (
                separated < self.config.branch_separation_rows and
                float(np.mean(distances)) <= self.config.overlap_px_640)

        # Disjoint high-heat regions remain independent candidates unless a
        # separately validated connector joined them earlier. Deduplicating
        # them here would silently discard global evidence.
        return False

    def _blend_heatmap_evidence(self, first, second):
        first_points = np.asarray(first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(second.get("points_xy"), dtype=np.float32)
        rows = {}
        for line, points in ((first, first_points), (second, second_points)):
            weight = max(1e-4, float(line.get("score", 0.0)))
            for x, y in points:
                key = round(float(y), 3)
                rows.setdefault(key, []).append((float(x), weight))
        merged_points = []
        for y in sorted(rows, reverse=True):
            observations = rows[y]
            weights = np.asarray([item[1] for item in observations], dtype=np.float32)
            xs = np.asarray([item[0] for item in observations], dtype=np.float32)
            merged_points.append([float(np.average(xs, weights=weights)), float(y)])
        merged = dict(first)
        merged["points_xy"] = np.asarray(merged_points, dtype=np.float32)
        first_count = max(1, len(first_points))
        second_count = max(1, len(second_points))
        merged["score"] = (
            float(first.get("score", 0.0)) * first_count +
            float(second.get("score", 0.0)) * second_count
        ) / float(first_count + second_count)
        merged["component_count"] = (
            int(first.get("component_count", 1)) +
            int(second.get("component_count", 1))
        )
        merged["vertical_span_px"] = self._vertical_span(merged_points)
        return merged

    def _heatmap_line_quality(self, line):
        span = max(1.0, self._vertical_span(line.get("points_xy", [])))
        return span * (0.5 + float(line.get("score", 0.0)))

    @staticmethod
    def _path_arc_length(points):
        points = np.asarray(points, dtype=np.float32)
        if len(points) < 2:
            return 0.0
        deltas = points[1:] - points[:-1]
        return float(np.sum(np.hypot(deltas[:, 0], deltas[:, 1])))

    def _history_match_bonus(self, line, quality, image_shape):
        if not self.last_slot_points:
            return 0.0
        distance = min(
            self._path_distance_640(
                line.get("points_xy"), previous, image_shape)
            for previous in self.last_slot_points.values()
        )
        gate = max(
            self.config.branch_separation_px_640 * 2.0,
            self.config.path_max_step_px_640 * 3.0,
        )
        return quality * 0.45 * max(0.0, 1.0 - distance / gate)

    @staticmethod
    def _path_distance_640(first_points, second_points, image_shape):
        first = np.asarray(first_points, dtype=np.float32)
        second = np.asarray(second_points, dtype=np.float32)
        if len(first) < 2 or len(second) < 2:
            return 1e9
        low = max(float(np.min(first[:, 1])), float(np.min(second[:, 1])))
        high = min(float(np.max(first[:, 1])), float(np.max(second[:, 1])))
        if low <= high:
            distances = []
            for y in low + (high - low) * _PAIR_FRACTIONS:
                first_x = _interp_path_x(first, y)
                second_x = _interp_path_x(second, y)
                if first_x is not None and second_x is not None:
                    distances.append(abs(first_x - second_x))
            if distances:
                return float(np.mean(distances)) * 640.0 / float(max(1, image_shape[1]))

        endpoint_distance = min(
            abs(float(first_point[0]) - float(second_point[0]))
            for first_point in (first[0], first[-1])
            for second_point in (second[0], second[-1])
        )
        return endpoint_distance * 640.0 / float(max(1, image_shape[1]))

    def _assign_heatmap_slots(self, lines, image_shape):
        if not lines:
            return []
        assigned = []
        if len(lines) == 1:
            item = dict(lines[0])
            scan_x = self._row_scan_path_position(
                item.get("points_xy"), image_shape)
            center_x = (
                float(image_shape[1]) * self.config.visual_center_x)
            item["slot"] = 0 if scan_x is None or scan_x <= center_x else 1
            assigned.append(item)
        else:
            first = dict(lines[0])
            second = dict(lines[1])
            first_x = self._row_scan_path_position(
                first.get("points_xy"), image_shape)
            second_x = self._row_scan_path_position(
                second.get("points_xy"), image_shape)
            if first_x is not None and second_x is not None and abs(
                    first_x - second_x) >= max(4.0, image_shape[1] * 0.008):
                ordered = (
                    [first, second] if first_x <= second_x
                    else [second, first])
            elif 0 in self.last_slot_points and 1 in self.last_slot_points:
                direct = (
                    self._path_distance_640(first.get("points_xy"), self.last_slot_points[0], image_shape) +
                    self._path_distance_640(second.get("points_xy"), self.last_slot_points[1], image_shape)
                )
                swapped = (
                    self._path_distance_640(first.get("points_xy"), self.last_slot_points[1], image_shape) +
                    self._path_distance_640(second.get("points_xy"), self.last_slot_points[0], image_shape)
                )
                ordered = [first, second] if direct <= swapped else [second, first]
            else:
                ordered = sorted(
                    (first, second),
                    key=lambda item: (
                        self._row_scan_path_position(
                            item.get("points_xy"), image_shape)
                        if self._row_scan_path_position(
                            item.get("points_xy"), image_shape) is not None
                        else image_shape[1] * 0.5))
            for slot, item in enumerate(ordered):
                item["slot"] = slot
                assigned.append(item)
        for item in assigned:
            item["points_xy"] = np.asarray(item.get("points_xy"), dtype=np.float32)
            item["role"] = "left" if int(item["slot"]) == 0 else "right"
            item["identity"] = item["role"]
            item["identity_source"] = "row_scan"
        return assigned

    @staticmethod
    def _row_scan_path_position(points, image_shape):
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
            return None
        row_mask = (
            (points[:, 1] <= image_shape[0] * 0.88) &
            (points[:, 1] >= image_shape[0] * 0.25))
        scan_points = points[row_mask]
        if not len(scan_points):
            scan_points = points
        return float(np.median(scan_points[:, 0]))

    @staticmethod
    def _component_centerline_points(
            labels, label, heatmap, image_shape, component_stats=None):
        if component_stats is None:
            top = 0
            left = 0
            region = labels
        else:
            left = int(component_stats[cv2.CC_STAT_LEFT])
            top = int(component_stats[cv2.CC_STAT_TOP])
            width = int(component_stats[cv2.CC_STAT_WIDTH])
            height = int(component_stats[cv2.CC_STAT_HEIGHT])
            region = labels[top:top + height, left:left + width]
        ys, local_xs = np.nonzero(region == label)
        xs = local_xs + left
        if len(xs) == 0:
            return []
        absolute_ys = ys + top
        row_count = int(region.shape[0])
        weights = heatmap[absolute_ys, xs].astype(np.float64)
        weight_sums = np.bincount(
            ys, weights=weights, minlength=row_count)
        weighted_x_sums = np.bincount(
            ys, weights=weights * xs, minlength=row_count)
        pixel_counts = np.bincount(ys, minlength=row_count)
        pixel_x_sums = np.bincount(
            ys, weights=xs.astype(np.float64), minlength=row_count)
        rows = np.flatnonzero(pixel_counts)[::-1]
        centers = np.divide(
            weighted_x_sums[rows], weight_sums[rows],
            out=np.divide(
                pixel_x_sums[rows], pixel_counts[rows],
                dtype=np.float64),
            where=weight_sums[rows] > 0.0)
        img_h, img_w = image_shape[:2]
        points = np.stack((
            centers * float(max(img_w - 1, 1)) /
            float(max(labels.shape[1] - 1, 1)),
            (rows + top).astype(np.float64) *
            float(max(img_h - 1, 1)) /
            float(max(labels.shape[0] - 1, 1)),
        ), axis=1).astype(np.float32)
        if len(points) >= 5:
            kernel = np.asarray([1.0, 2.0, 3.0, 2.0, 1.0], dtype=np.float32)
            kernel /= float(np.sum(kernel))
            padded = np.pad(points[:, 0], (2, 2), mode="edge")
            points[:, 0] = np.convolve(padded, kernel, mode="valid")
        return points

    def _smooth_candidates(
            self, candidates, image_shape, support_maps=None):
        present_slots = set()
        smoothed_candidates = []
        for candidate in candidates:
            slot = int(candidate.get("slot", -1))
            points = np.asarray(candidate.get("points_xy"), dtype=np.float32)
            if slot < 0 or points.ndim != 2 or points.shape[1] != 2:
                continue
            present_slots.add(slot)
            filtered = dict(candidate)
            filtered["raw_points_xy"] = points.copy()
            previous = self.last_slot_points.get(slot)
            filtered_points = self._smooth_path_points(
                points, previous, image_shape[1],
                apply_spatial=not bool(candidate.get(
                    "spatial_prefiltered", False)))
            filtered["points_xy"] = filtered_points
            filtered["spatial_smoothed"] = self.config.path_smooth_window > 1
            filtered["temporal_smoothed"] = previous is not None
            filtered["smoothing_rejected_low_heat"] = False
            smoothing_changed = not np.allclose(
                filtered_points, points, rtol=0.0, atol=0.05)
            if (support_maps is not None and smoothing_changed and
                    not self._annotate_heat_support(
                        filtered, image_shape, support_maps)):
                # The current-frame candidate was already validated. Never
                # let spatial/temporal filtering drag it through empty heat.
                filtered["points_xy"] = points.copy()
                filtered["spatial_smoothed"] = False
                filtered["temporal_smoothed"] = False
                filtered["smoothing_rejected_low_heat"] = True
                self._annotate_heat_support(
                    filtered, image_shape, support_maps)
            if support_maps is not None:
                source_slots = filtered.get("source_slots") or {
                    int(filtered.get("source_slot", -1))}
                filtered["point_confidences"] = _path_point_probabilities(
                    filtered["points_xy"], image_shape,
                    support_maps, source_slots)
            self.last_slot_points[slot] = filtered_points.copy()
            if filtered["smoothing_rejected_low_heat"]:
                self.last_slot_points[slot] = filtered["points_xy"].copy()
            self.path_missing_frames[slot] = 0
            smoothed_candidates.append(filtered)

        for slot in list(self.last_slot_points):
            if slot in present_slots:
                continue
            missing = self.path_missing_frames.get(slot, 0) + 1
            self.path_missing_frames[slot] = missing
            if missing > self.config.path_state_hold_frames:
                self.last_slot_points.pop(slot, None)
                self.path_missing_frames.pop(slot, None)
        return smoothed_candidates

    def _constrain_candidates_to_road(self, candidates, road_mask, image_shape):
        """Project post-smoothed paths back into the semantic-road support."""
        if road_mask is None:
            return candidates
        road = np.asarray(road_mask, dtype=np.uint8)
        if road.ndim != 2 or not np.any(road):
            return []
        road_y, road_x = np.nonzero(road)
        height, width = image_shape[:2]
        max_distance = 10.0 * float(max(width, height)) / 160.0
        max_distance_sq = max_distance * max_distance
        constrained_candidates = []
        for candidate in candidates:
            points = np.asarray(candidate.get("points_xy"), dtype=np.float32)
            if points.ndim != 2 or points.shape[1] != 2 or not len(points):
                continue
            normalized_x = np.clip(
                np.rint(points[:, 0] * (road.shape[1] - 1) /
                        max(width - 1, 1)), 0, road.shape[1] - 1).astype(np.int32)
            normalized_y = np.clip(
                np.rint(points[:, 1] * (road.shape[0] - 1) /
                        max(height - 1, 1)), 0, road.shape[0] - 1).astype(np.int32)
            kept = []
            projected = []
            for index, (x, y) in enumerate(zip(normalized_x, normalized_y)):
                if road[y, x]:
                    nearest_x, nearest_y = x, y
                else:
                    distances = ((road_x.astype(np.float32) - float(x)) ** 2 +
                                 (road_y.astype(np.float32) - float(y)) ** 2)
                    nearest_index = int(np.argmin(distances))
                    if distances[nearest_index] > max_distance_sq:
                        continue
                    nearest_x = int(road_x[nearest_index])
                    nearest_y = int(road_y[nearest_index])
                kept.append(index)
                projected.append((
                    nearest_x * float(max(width - 1, 0)) /
                    max(road.shape[1] - 1, 1),
                    nearest_y * float(max(height - 1, 0)) /
                    max(road.shape[0] - 1, 1),
                ))
            if len(projected) < 2:
                continue
            item = dict(candidate)
            item["points_xy"] = np.asarray(projected, dtype=np.float32)
            item["road_constrained"] = True
            item["display_segments_xy"] = [item["points_xy"].copy()]
            if "row_indices" in item:
                item["row_indices"] = np.asarray(item["row_indices"])[kept]
            constrained_candidates.append(item)
        return constrained_candidates

    def _smooth_path_points(
            self, points, previous, image_width, apply_spatial=True):
        points = np.asarray(points, dtype=np.float32).copy()
        window = max(1, int(self.config.path_smooth_window))
        if window % 2 == 0:
            window += 1
        if apply_spatial and window > 1 and len(points) >= 3:
            window = min(window, len(points) if len(points) % 2 else len(points) - 1)
            half = window // 2
            weights = np.concatenate((
                np.arange(1, half + 2, dtype=np.float32),
                np.arange(half, 0, -1, dtype=np.float32),
            ))
            padded_x = np.pad(points[:, 0], (half, half), mode="edge")
            points[:, 0] = np.convolve(
                padded_x, weights / np.sum(weights), mode="valid")

        if previous is None or len(previous) < 2 or not len(points):
            return points
        previous = np.asarray(previous, dtype=np.float32)
        order = np.argsort(previous[:, 1])
        previous_y = previous[order, 1]
        previous_x = previous[order, 0]
        previous_y, unique_indices = np.unique(previous_y, return_index=True)
        previous_x = previous_x[unique_indices]
        if len(previous_y) < 2:
            return points
        overlap = (
            (points[:, 1] >= previous_y[0]) &
            (points[:, 1] <= previous_y[-1]))
        if not np.any(overlap):
            return points
        projected_x = np.interp(
            points[overlap, 1], previous_y, previous_x).astype(np.float32)
        max_step = (
            self.config.path_max_step_px_640 *
            float(max(1, image_width)) / 640.0)
        bounded_x = projected_x + np.clip(
            points[overlap, 0] - projected_x, -max_step, max_step)
        alpha = float(self.config.path_ema_alpha)
        points[overlap, 0] = (
            bounded_x * alpha + projected_x * (1.0 - alpha))
        return points

    def _publish_filtered_paths(self, result, candidates):
        raw_paths = list(result.get("paths") or [])
        result["raw_paths"] = raw_paths
        result["paths"] = candidates
        result["display_paths"] = list(candidates[:2])
        # ARPreview lets the vision-control overlay draw these once with
        # probability-segmented identity colors.
        result["vision_control_path_overlay"] = True
        result["temporal"] = {
            "enabled": True,
            "status": "tracking" if candidates else "path_unavailable",
            "alpha": float(self.config.path_ema_alpha),
            "source": "vision_control_slot_filter",
        }
        centerline = result.get("centerline")
        if isinstance(centerline, dict):
            centerline["raw_paths"] = raw_paths
            centerline["paths"] = candidates
            centerline["display_paths"] = list(candidates[:2])
            centerline["temporal"] = result["temporal"]

    @staticmethod
    def _publish_display_paths(result, display_candidates):
        result["display_paths"] = display_candidates
        centerline = result.get("centerline")
        if isinstance(centerline, dict):
            centerline["display_paths"] = display_candidates

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

    def _update_default_outer(self, route_state, now, ocr_current):
        if self.config.path_source == "curve":
            # Row-head slot identity is fixed: slot 0 is the blue/default
            # route and slot 1 is the green/right route. Never let proximity
            # to the vehicle or merge geometry override this policy.
            if self.branch_lock_source != "ocr":
                self._set_default_curve_branch()
            if route_state == ROUTE_MULTI_FORK:
                self.single_seen_frames = 0
                if self.fork_seen_since is None:
                    self.fork_seen_since = now
            elif route_state == ROUTE_SINGLE:
                self.fork_seen_since = None
                self.single_seen_frames += 1
            else:
                self.single_seen_frames = 0
            return
        if route_state == ROUTE_MULTI_FORK:
            self.single_seen_frames = 0
            if self.fork_seen_since is None:
                self.fork_seen_since = now
            if (
                not ocr_current
                and self.branch_lock is None
                and now - self.fork_seen_since >= self.config.default_outer_after_s
            ):
                self.branch_lock = "left" if self.config.outer_slot == 0 else "right"
                self.branch_lock_source = "default"
            return
        if route_state == ROUTE_SINGLE:
            self.fork_seen_since = None
            self.single_seen_frames += 1
            if self.single_seen_frames == self.config.branch_release_frames:
                self._clear_branch_lock()
            return
        if route_state in {ROUTE_NONE, ROUTE_AMBIGUOUS}:
            self.single_seen_frames = 0
            return

    def _update_curve_merge_continuity(self, candidates, image_shape):
        """Temporarily follow stable slot 1 when slot 0 collapses at a merge.

        Curve-head slot identity is normally authoritative.  In real merge
        frames, however, slot 0 alternates between a full route and a short
        piece of the shared trunk while slot 1 remains continuous.  Requiring
        both paths to meet near the vehicle keeps this exception out of
        unrelated missing-line and ordinary two-lane cases.
        """
        self.curve_merge_metrics = {}
        if self.config.path_source != "curve":
            return
        if self.branch_lock != "left":
            self.curve_merge_override = False
            self.curve_merge_bad_evidence = 0
            self.curve_merge_blue_stable_frames = 0
            self.curve_merge_reason = (
                "ocr_right" if self.branch_lock == "right" else "inactive")
            return

        by_slot = {
            int(candidate.get("slot", -1)): candidate
            for candidate in candidates
        }
        blue = by_slot.get(0)
        green = by_slot.get(1)
        if blue is None or green is None:
            # Do not turn an ordinary missing-blue frame into an implicit
            # green fallback.  An already-active merge takeover stays latched
            # until blue has genuinely recovered for several frames.
            if not self.curve_merge_override:
                self.curve_merge_bad_evidence = max(
                    0, self.curve_merge_bad_evidence - 1)
            self.curve_merge_blue_stable_frames = 0
            self.curve_merge_reason = "waiting_for_both_paths"
            if self.curve_merge_override:
                self.selected_slot_lock = 1
            return

        blue_points = np.asarray(blue.get("points_xy"), dtype=np.float32)
        green_points = np.asarray(green.get("points_xy"), dtype=np.float32)
        if not len(blue_points) or not len(green_points):
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
        green_good = green_covers_lookahead and green_span > 0.0
        blue_bad = (
            not blue_covers_lookahead or
            blue_span < (
                green_span * self.config.curve_merge_support_ratio))
        bad_merge_frame = shared_near and green_good and blue_bad
        blue_recovered = (
            blue_covers_lookahead and
            blue_span >= green_span * max(
                0.85, self.config.curve_merge_support_ratio))
        self.curve_merge_metrics = {
            "shared_near": bool(shared_near),
            "near_distance_640": float(near_distance_640),
            "blue_span": blue_span,
            "green_span": green_span,
            "blue_covers_lookahead": bool(blue_covers_lookahead),
            "green_covers_lookahead": bool(green_covers_lookahead),
            "blue_bad": bool(blue_bad),
        }

        if self.curve_merge_override:
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

    def _select_candidate(self, candidates, route_state, image_shape):
        if not candidates:
            self.selection_reason = "no_candidate"
            if self.selected_slot_lock is not None:
                self.selected_slot_missing_frames += 1
                if (self.branch_lock is None and
                        self.selected_slot_missing_frames >
                        self.config.path_state_hold_frames):
                    self.selected_slot_lock = None
                    self.selected_slot_missing_frames = 0
                    self.last_selected_points = None
            return None
        if self.branch_lock in {"left", "right"}:
            wanted_slot = (
                1 if (self.branch_lock == "right" or
                      (self.branch_lock == "left" and
                       self.curve_merge_override)) else 0)
            self.selection_reason = (
                "merge_continuity_green" if
                self.branch_lock == "left" and self.curve_merge_override
                else "locked_{}".format(self.branch_lock))
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == wanted_slot:
                    return self._remember_selection(candidate)
            self.selected_slot_lock = wanted_slot
            self.selected_slot_missing_frames += 1
            return None
        if self.selected_slot_lock is not None:
            self.selection_reason = "held_slot"
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == self.selected_slot_lock:
                    return self._remember_selection(candidate)
            self.selected_slot_missing_frames += 1
            if (self.selected_slot_missing_frames <=
                    self.config.path_state_hold_frames):
                return None
            self.selected_slot_lock = None
            self.selected_slot_missing_frames = 0
        if self.last_selected_points is not None:
            previous_x = _interp_path_x(self.last_selected_points, image_shape[0] * self.config.lookahead_y_ratio)
            if previous_x is not None:
                self.selection_reason = "path_continuity"
                return self._remember_selection(min(
                    candidates,
                    key=lambda item: abs(
                        (_interp_path_x(item["points_xy"], image_shape[0] * self.config.lookahead_y_ratio) or previous_x)
                        - previous_x
                    ),
                ))
        if route_state == ROUTE_MULTI_FORK:
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == self.config.outer_slot:
                    self.selection_reason = "configured_outer"
                    return self._remember_selection(candidate)
        center_x = image_shape[1] * self.config.visual_center_x
        self.selection_reason = "nearest_visual_center"
        return self._remember_selection(min(
            candidates,
            key=lambda item: abs((_interp_path_x(item["points_xy"], image_shape[0] * self.config.lookahead_y_ratio) or center_x) - center_x),
        ))

    def _remember_selection(self, candidate):
        self.selected_slot_lock = int(candidate.get("slot", -1))
        self.selected_slot_missing_frames = 0
        return candidate

    def _build_command(self, selected, route_state, route_reason, result, image_shape, now, ocr_response=None):
        lookahead_y = image_shape[0] * self.config.lookahead_y_ratio
        ocr_route_locked = (
            self.branch_lock_source == "ocr" and
            self.branch_lock in {"left", "right"} and
            self.last_valid_ocr_ts > 0.0 and
            float(now) - self.last_valid_ocr_ts <
            self.config.ocr_lock_lifetime_s)
        selected_covers_lookahead = (
            selected is not None and self._path_covers_y(
                selected.get("points_xy"), lookahead_y))
        if ocr_route_locked and not selected_covers_lookahead:
            self._reset_error_trend()
            if isinstance(ocr_response, dict):
                ocr_response["control_phase"] = "ocr_wait_route"
            return self._command(
                0.0, 0.0, STATE_SAFE_STOP, flags=0), {
                    "target_x": None,
                    "path_target_x": None,
                    "path_target_y": float(lookahead_y),
                    "track_error_640": 0.0,
                    "reason": "ocr_selected_route_unavailable",
                    "task_reason": "ocr_wait_route",
                }
        if ocr_route_locked and isinstance(ocr_response, dict):
            ocr_response["control_phase"] = "ocr_route_ready"
        path_target = (
            self._path_target_on_selected(selected, lookahead_y)
            if selected is not None else None)
        task_path_target_x = None if path_target is None else path_target[0]
        task_state, speed, task_reason, target_override_x = self._task_from_detections(
            result, selected, image_shape, lookahead_y, route_state, now,
            ocr_response=ocr_response,
            path_target_x=task_path_target_x)
        if selected is None:
            self._reset_error_trend()
            age = now - self.last_valid_ts if self.last_valid_ts else 1e9
            held_target = self._held_path_target(now)
            if self.last_valid_ts and age <= self.config.recover_hold_s:
                error = self.last_error
                return self._command(error, self.config.recover_speed_mps, STATE_RECOVER_LINE), {
                    "target_x": None if held_target is None else held_target[0],
                    "path_target_x": None if held_target is None else held_target[0],
                    "path_target_y": None if held_target is None else held_target[1],
                    "path_target_held": held_target is not None,
                    "track_error_640": error,
                    "reason": "recover_hold",
                    "task_reason": task_reason,
                }
            state = STATE_LINE_LOSS_SAFE_STOP if age >= self.config.no_path_stop_s else STATE_RECOVER_LINE
            return self._command(0.0, 0.0 if state == STATE_LINE_LOSS_SAFE_STOP else self.config.recover_speed_mps, state, flags=0 if state == STATE_LINE_LOSS_SAFE_STOP else CONTROL_FLAG_USE_TARGET_SPEED), {
                "target_x": None,
                "track_error_640": 0.0,
                "reason": route_reason,
                "task_reason": task_reason,
            }
        curve_slot_is_locked = (
            self.config.path_source == "curve" and
            self.branch_lock in {"left", "right"})
        if route_state == ROUTE_AMBIGUOUS and not curve_slot_is_locked:
            self._reset_error_trend()
            return self._command(0.0, 0.0, STATE_SAFE_STOP, flags=0), {
                "target_x": None,
                "track_error_640": 0.0,
                "reason": route_reason,
                "task_reason": "ambiguous_stop",
            }

        if path_target is None:
            self._reset_error_trend()
            return self._command(0.0, 0.0, STATE_SAFE_STOP, flags=0), {
                "target_x": None,
                "path_target_x": None,
                "path_target_y": None,
                "track_error_640": 0.0,
                "reason": "missing_target_x",
                "task_reason": task_reason,
            }
        path_target_x, path_target_y, path_target_adaptive = path_target
        target_x = _clamp(
            float(target_override_x) if target_override_x is not None
            else float(path_target_x),
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        task_offset_x = float(target_x) - float(path_target_x)
        raw_error = (float(target_x) / float(max(1, image_shape[1])) - self.config.visual_center_x) * 640.0
        human_return_extra_error = (
            self._human_return_extra_error(now)
            if task_reason == "human_return" else 0.0)
        steering_error = raw_error + human_return_extra_error
        default_track_error_gain = 1.0
        if task_reason == "track":
            if steering_error < 0.0:
                default_track_error_gain = (
                    self.config.default_track_left_error_gain)
            elif steering_error > 0.0:
                default_track_error_gain = (
                    self.config.default_track_right_error_gain)
            steering_error *= default_track_error_gain
        steering_error_before_trend = steering_error
        trend_mode = self._error_trend_mode_for(
            task_state, task_reason, speed)
        (
            steering_error,
            error_trend_adjustment,
            error_trend_slope,
            error_trend_frames,
        ) = self._apply_error_trend(steering_error, trend_mode)
        error_step_limit = self.config.max_error_step_640
        error_limit = self.config.max_track_error_640
        if task_reason == "track":
            if steering_error < 0.0:
                error_limit = (
                    self.config.default_track_left_max_error_640)
                error_step_limit = (
                    self.config.default_track_left_error_step_640)
            elif steering_error > 0.0:
                error_limit = (
                    self.config.default_track_right_max_error_640)
                error_step_limit = (
                    self.config.default_track_right_error_step_640)
        if task_reason in {"turnsign_reverse", "human_brake_reverse"}:
            self._reset_error_trend()
            error_trend_adjustment = 0.0
            error_trend_slope = 0.0
            error_trend_frames = 0
            # Reverse must start with a straight steering target. Do not let
            # the normal error slew retain a previous aggressive edge turn.
            error = 0.0
        else:
            error = self._limit_error(
                steering_error,
                max_error=error_limit,
                max_step=error_step_limit,
            )
        self.last_error = error
        self.last_valid_ts = now
        self.last_selected_points = np.asarray(selected["points_xy"], dtype=np.float32).copy()
        self.last_slot_points[int(selected["slot"])] = self.last_selected_points
        self.last_path_target_x = float(path_target_x)
        self.last_path_target_y = float(path_target_y)
        self.last_path_target_slot = int(selected["slot"])
        self.last_path_target_ts = float(now)
        return self._command(error, speed, task_state), {
            "target_x": float(target_x),
            "path_target_x": float(path_target_x),
            "path_target_y": float(path_target_y),
            "path_target_held": False,
            "path_target_adaptive_y": bool(path_target_adaptive),
            "lookahead_y": float(path_target_y),
            "human_stop_line_y": self._human_stop_line_y(
                image_shape, path_target_y),
            "task_offset_x": float(task_offset_x),
            "task_target_applied": target_override_x is not None,
            "track_error_640": float(error),
            "raw_track_error_640": float(raw_error),
            "error_before_trend_640": float(
                steering_error_before_trend),
            "default_track_error_gain": float(
                default_track_error_gain),
            "human_return_extra_error_640": float(
                human_return_extra_error),
            "car_avoid_phase": str(self.car_avoid_phase),
            "car_avoid_offset_640": float(
                self.car_avoid_offset_px_640),
            "car_avoid_target_offset_640": float(
                self.car_avoid_target_offset_px_640),
            "car_avoid_parallel_elapsed_s": float(
                self.car_avoid_parallel_elapsed_s),
            "steering_input_error_640": float(steering_error),
            "error_trend_adjustment_640": float(
                error_trend_adjustment),
            "error_trend_slope_640_per_frame": float(
                error_trend_slope),
            "error_trend_frames": int(error_trend_frames),
            "error_trend_mode": trend_mode,
            "error_limit_640": float(error_limit),
            "error_step_limit_640": float(error_step_limit),
            "reason": route_reason,
            "task_reason": task_reason,
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
        # Keep the displayed/control lookahead row fixed. Only borrow the x
        # coordinate from the nearest supported endpoint when the curve is
        # temporarily too short to cross that row.
        return float(points[nearest, 0]), float(preferred_y), True

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
            self, result, selected, image_shape, lookahead_y, route_state,
            now, ocr_response=None, path_target_x=None):
        detections = result.get("detections") or []
        if selected is None:
            return STATE_RECOVER_LINE, self.config.recover_speed_mps, "no_path", None
        # Route selection owns the baseline. All task targets must be derived
        # from the exact lookahead point on that locked route, not from another
        # row on the curve.
        target_path_x = _finite_float(path_target_x)
        if target_path_x is None:
            target_path_x = _interp_path_x(selected["points_xy"], lookahead_y)
        if target_path_x is None:
            target_path_x = image_shape[1] * self.config.visual_center_x
        lookahead_path_x = float(target_path_x)
        human_brake_action = self._human_brake_action(now)
        if human_brake_action is not None:
            return human_brake_action
        human_safety_priority = self._human_safety_has_priority(
            detections, image_shape, lookahead_y)
        if human_safety_priority:
            car_action = self._car_avoidance_action(
                detections, image_shape, lookahead_y,
                lookahead_path_x, now)
            if car_action is not None:
                return car_action
            human_action = self._human_action(
                detections, selected, image_shape, lookahead_y,
                lookahead_path_x, now)
            if human_action is not None:
                return human_action
        sign_action = self._turnsign_action(
            detections, image_shape, lookahead_y, ocr_response, now,
            lookahead_path_x)
        if sign_action is not None:
            return sign_action
        if not human_safety_priority:
            car_action = self._car_avoidance_action(
                detections, image_shape, lookahead_y,
                lookahead_path_x, now)
            if car_action is not None:
                return car_action
            human_action = self._human_action(
                detections, selected, image_shape, lookahead_y,
                lookahead_path_x, now)
            if human_action is not None:
                return human_action
        if now < self.human_speed_hold_until:
            held_target_x = _clamp(
                float(target_path_x) + float(self.human_pass_offset_x),
                0.0,
                float(max(0, image_shape[1] - 1)),
            )
            return (
                STATE_AVOID_HUMAN,
                self.config.human_pass_speed_mps,
                "human_speed_hold",
                held_target_x,
            )
        if self._human_return_active(now):
            return (
                STATE_TRACK,
                self.config.normal_speed_mps,
                "human_return",
                None,
            )
        self.human_pass_offset_x = 0.0
        coin = self._best_coin(detections, image_shape, target_path_x)
        if coin is not None:
            return STATE_COLLECT_GOLD, self.config.collect_speed_mps, "coin_bias", self._coin_target_x(coin, target_path_x, image_shape)
        return STATE_TRACK, self.config.normal_speed_mps, "track", None

    def _turnsign_action(
            self, detections, image_shape, lookahead_y, ocr_response, now,
            path_x):
        response = ocr_response if isinstance(ocr_response, dict) else {}
        phase = str(response.get("control_phase") or "")

        session_id = response.get("session_id")
        if (
            bool(response.get("session_active"))
            and session_id is not None
            and session_id != self.turnsign_control_session_id
        ):
            self.turnsign_control_session_id = session_id
            self.turnsign_brake_until = (
                float(now) + self.config.turnsign_initial_brake_s)
            self.turnsign_reverse_until = 0.0
            self.turnsign_post_reverse_stop = False

        # The initial brake is a fixed safety interval. Even a very fast OCR
        # result must not release the vehicle before the full 0.5 s expires.
        if self.turnsign_brake_until > 0.0:
            if float(now) < self.turnsign_brake_until:
                response["control_phase"] = "turnsign_initial_brake"
                return (
                    STATE_SAFE_STOP, 0.0,
                    "turnsign_initial_brake", None)
            self.turnsign_brake_until = 0.0

        if (
            bool(response.get("turnsign_resolved")) or
            self._ocr_has_current_direction(response)
        ):
            # Once a valid OCR result is locked, the consumed physical sign no
            # longer changes speed or steering. Route availability is handled
            # separately by _build_command.
            self.turnsign_reverse_until = 0.0
            self.turnsign_post_reverse_stop = False
            self.turnsign_brake_until = 0.0
            self._clear_turnsign_state()
            if phase != "ocr_route_ready":
                response["control_phase"] = "turnsign_consumed"
            return None

        if phase:
            if self.turnsign_reverse_until > 0.0:
                if float(now) < self.turnsign_reverse_until:
                    response["control_phase"] = "turnsign_reverse"
                    center_x = (
                        float(image_shape[1]) *
                        self.config.visual_center_x)
                    return (
                        STATE_TRACK,
                        self.config.turnsign_reverse_speed_mps,
                        "turnsign_reverse",
                        center_x,
                    )
                self.turnsign_reverse_until = 0.0
                self.turnsign_post_reverse_stop = True

            if self.turnsign_post_reverse_stop:
                self.turnsign_post_reverse_stop = False
                response["control_phase"] = "turnsign_post_reverse_stop"
                return (
                    STATE_SAFE_STOP, 0.0,
                    "turnsign_post_reverse_stop", None)

            if phase == "turnsign_edge_over_line":
                self.turnsign_reverse_until = (
                    float(now) +
                    self.config.turnsign_reverse_duration_s)
                response["control_phase"] = "turnsign_reverse"
                center_x = (
                    float(image_shape[1]) * self.config.visual_center_x)
                return (
                    STATE_TRACK,
                    self.config.turnsign_reverse_speed_mps,
                    "turnsign_reverse",
                    center_x,
                )

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
            detections, image_shape, lookahead_y, response, now)

    def _legacy_turnsign_action(
            self, detections, image_shape, lookahead_y, response, now):
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
                    geom, image_shape, lookahead_y):
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

    def _clear_sign_ocr_state(self):
        self.sign_ocr_active_since = None
        self.sign_ocr_pulse_until = 0.0
        self.sign_ocr_pulse_sent = False

    def _clear_turnsign_state(self):
        self._clear_sign_ocr_state()
        self.sign_seen_frames = 0
        self.sign_latched_since = None

    def _advance_car_avoidance_state(self, now):
        """Advance the lateral ramp using only time spent driving."""
        now = float(now)
        if self.car_avoid_side == 0 or self.car_avoid_phase == "idle":
            self.car_avoid_last_update_ts = now
            self.car_avoid_last_motion_active = False
            return
        if self.car_avoid_last_update_ts <= 0.0:
            self.car_avoid_last_update_ts = now
            return

        dt = max(0.0, now - self.car_avoid_last_update_ts)
        self.car_avoid_last_update_ts = now
        if dt <= 0.0 or not self.car_avoid_last_motion_active:
            return

        ramp_s = max(0.01, float(self.config.car_avoid_ramp_s))
        if self.car_avoid_phase == "ramp_in":
            target = max(
                float(self.config.car_avoid_offset_px_640),
                float(self.car_avoid_target_offset_px_640),
            )
            rate = target / ramp_s
            remaining = max(
                0.0, target - float(self.car_avoid_offset_px_640))
            ramp_dt = min(dt, remaining / max(rate, 1e-6))
            self.car_avoid_offset_px_640 = min(
                target,
                float(self.car_avoid_offset_px_640) + rate * ramp_dt,
            )
            if self.car_avoid_offset_px_640 >= target - 1e-6:
                self.car_avoid_offset_px_640 = target
                self.car_avoid_phase = "hold"
                self.car_avoid_parallel_elapsed_s += max(0.0, dt - ramp_dt)
        elif self.car_avoid_phase == "hold":
            self.car_avoid_parallel_elapsed_s += dt
        elif self.car_avoid_phase == "ramp_out":
            start_offset = max(
                float(self.config.car_avoid_offset_px_640),
                float(self.car_avoid_return_start_offset_px_640),
            )
            rate = start_offset / ramp_s
            self.car_avoid_offset_px_640 = max(
                0.0,
                float(self.car_avoid_offset_px_640) - rate * dt,
            )
            if self.car_avoid_offset_px_640 <= 1e-6:
                self._clear_car_avoidance_state()
                self.car_avoid_last_update_ts = now

    def _car_avoidance_action(
            self, detections, image_shape, lookahead_y, path_x, now):
        """Ramp onto one locked avoidance side across car/human phases."""
        if (
            self.car_human_pass_until > 0.0
            and float(now) >= self.car_human_pass_until
        ):
            self.car_human_pass_until = 0.0
            self.car_human_active = False
            self.car_human_waiting_cross = False
            self.car_human_seen_avoid_side = False
            self.car_human_last_seen_ts = 0.0
        car = self._best_car(detections, image_shape, path_x)
        pass_holding = float(now) < self.car_human_pass_until

        if car is not None:
            desired_target_x = self._avoid_target_x(
                "car", car, path_x, image_shape)
            desired_offset = float(desired_target_x) - float(path_x)
            desired_side = self._sign(desired_offset)
            if self.car_avoid_side == 0:
                self._clear_car_avoidance_state()
                self.car_avoid_side = desired_side or 1
                self.car_avoid_phase = "ramp_in"
                self.car_avoid_last_update_ts = float(now)
                self._clear_human_state()
                self.human_pass_offset_x = 0.0
                self.human_speed_hold_until = 0.0

            # Never cross the baseline while one avoidance sequence is active.
            # A growing box may increase the final magnitude, but the current
            # offset approaches it through the same linear ramp without jumps.
            scale = 640.0 / float(max(1, image_shape[1]))
            desired_magnitude_640 = abs(desired_offset) * scale
            updated_target = max(
                self.car_avoid_target_offset_px_640,
                self.config.car_avoid_offset_px_640,
                desired_magnitude_640,
            )
            if (
                updated_target >
                self.car_avoid_target_offset_px_640 + 1e-6
            ):
                self.car_avoid_target_offset_px_640 = updated_target
                if self.car_avoid_offset_px_640 < updated_target - 1e-6:
                    self.car_avoid_phase = "ramp_in"
                    self.car_avoid_parallel_elapsed_s = 0.0
            if self.car_avoid_phase == "ramp_out":
                self.car_avoid_phase = "ramp_in"
                self.car_avoid_parallel_elapsed_s = 0.0

        if self.car_avoid_side == 0:
            return None

        avoid_target_x = self._latched_car_target_x(path_x, image_shape)
        human_reference_x = self._latched_car_final_target_x(
            path_x, image_shape)
        new_human_status, new_human = self._observe_new_human_while_consumed(
            detections, image_shape, lookahead_y, now)
        if new_human_status == "line" and new_human is not None:
            self._cancel_human_return()
            self._clear_consumed_human_state()
            self.car_human_pass_until = 0.0
            self.car_human_active = True
            self.car_human_waiting_cross = True
            self.human_current_prefer_upper = True
            self.car_human_last_seen_ts = float(now)
            human_side = self._sign(
                float(new_human["geom"]["cx"]) - float(human_reference_x))
            self.car_human_seen_avoid_side = (
                human_side == self.car_avoid_side)
            return self._start_human_brake(now, avoid_target_x)
        if new_human_status == "missing_near":
            self._cancel_human_return()
            return (
                STATE_SAFE_STOP, 0.0,
                "car_human_new_person_absence_check", avoid_target_x)
        human = self._best_car_context_human(
            detections, image_shape, avoid_target_x)

        human_context_active = bool(
            human is not None
            or self.car_human_active
            or pass_holding
            or self.human_consumed_active
            or self.human_new_pending
            or self._human_preline_missing_waiting(now)
        )
        if (
            car is None
            and self.car_avoid_phase == "hold"
            and self.car_avoid_parallel_elapsed_s >=
                self.config.car_avoid_hold_s
            and not human_context_active
        ):
            self.car_avoid_phase = "ramp_out"
            self.car_avoid_return_start_offset_px_640 = max(
                self.car_avoid_offset_px_640,
                self.config.car_avoid_offset_px_640,
            )

        if pass_holding:
            return (
                STATE_AVOID_HUMAN,
                self.config.car_human_pass_speed_mps,
                "car_human_same_side_pass_hold",
                avoid_target_x,
            )

        if self.human_consumed_active:
            # The person that released the car has already been handled. Keep
            # the car-avoidance route, but do not let that same lower-screen
            # Human detection stop the vehicle again.
            return (
                STATE_AVOID_CAR,
                self.config.car_avoid_speed_mps,
                "car_human_consumed_ignore",
                avoid_target_x,
            )

        if human is not None:
            geom = human["geom"]
            reached_line = self._human_on_stop_line(
                geom, image_shape, lookahead_y)
            if not self.car_human_active and not reached_line:
                preline_side = self._sign(
                    float(geom["cx"]) - float(human_reference_x))
                if preline_side == self.car_avoid_side:
                    self.car_human_seen_avoid_side = True
                # A visible person before the stop line does not take control
                # away from the car route. Remember only the vertical gap so a
                # near-line detector dropout can trigger the 1.5 s safety wait.
                self._record_human_preline_gap(
                    geom, image_shape, lookahead_y)
                return (
                    STATE_AVOID_CAR,
                    self.config.car_avoid_speed_mps,
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
                float(geom["cx"]) - float(human_reference_x))
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
                self._schedule_human_return(
                    self.car_human_pass_until,
                    -self.car_avoid_side)
                self.car_human_waiting_cross = False
                self._begin_consumed_human(now)
                return (
                    STATE_AVOID_HUMAN,
                    self.config.car_human_pass_speed_mps,
                    "car_human_same_side_pass",
                    avoid_target_x,
                )

            first_line_contact = (
                reached_line and not self.car_human_waiting_cross)
            if reached_line:
                self.car_human_waiting_cross = True
            if first_line_contact:
                return self._start_human_brake(now, avoid_target_x)
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
            # Release only the pedestrian part of the state. The car route
            # remains locked until its full parallel-driving hold completes.
            self.car_human_active = False
            self.car_human_waiting_cross = False
            self.car_human_seen_avoid_side = False
            self.car_human_last_seen_ts = 0.0

        elif self._human_preline_missing_waiting(now):
            return (
                STATE_SAFE_STOP,
                0.0,
                "car_human_preline_absence_check",
                avoid_target_x,
            )

        if car is not None:
            reason = "car_in_path_bias"
        elif self.car_avoid_phase == "ramp_out":
            reason = "car_avoid_return"
        else:
            reason = "car_avoid_hold"
        return (
            STATE_AVOID_CAR,
            self.config.car_avoid_speed_mps,
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

    def _best_car_context_human(
            self, detections, image_shape, avoid_target_x):
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
            vertical_rank = (
                -float(geom["bottom_ratio"])
                if self.human_current_prefer_upper
                else float(geom["bottom_ratio"]))
            rank = (vertical_rank, float(score))
            if rank > best_rank:
                best = {"geom": geom, "score": score}
                best_rank = rank
        return best

    def _latched_car_target_x(self, path_x, image_shape):
        scale = float(max(1, image_shape[1])) / 640.0
        offset = (
            float(self.car_avoid_side)
            * float(self.car_avoid_offset_px_640)
            * scale)
        return _clamp(
            float(path_x) + offset,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )

    def _latched_car_final_target_x(self, path_x, image_shape):
        scale = float(max(1, image_shape[1])) / 640.0
        offset = (
            float(self.car_avoid_side)
            * max(
                float(self.config.car_avoid_offset_px_640),
                float(self.car_avoid_target_offset_px_640),
            )
            * scale)
        return _clamp(
            float(path_x) + offset,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )

    def _clear_car_avoidance_state(self):
        self.car_avoid_side = 0
        self.car_avoid_offset_px_640 = 0.0
        self.car_avoid_target_offset_px_640 = 0.0
        self.car_avoid_phase = "idle"
        self.car_avoid_parallel_elapsed_s = 0.0
        self.car_avoid_return_start_offset_px_640 = 0.0
        self.car_avoid_last_update_ts = 0.0
        self.car_avoid_last_motion_active = False
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

        new_human_status, new_human = self._observe_new_human_while_consumed(
            detections, image_shape, lookahead_y, now)
        if new_human_status == "line" and new_human is not None:
            self._cancel_human_return()
            self._clear_consumed_human_state()
            self.human_pass_active = False
            self.human_speed_hold_until = 0.0
            self.human_pass_offset_x = 0.0
            self.human_waiting_cross = True
            self.human_current_prefer_upper = True
            self.human_detected_latched = True
            self.human_last_seen_ts = float(now)
            side = self._sign(
                float(new_human["geom"]["cx"]) -
                float(lookahead_path_x))
            if side:
                self.human_last_side = side
            return self._start_human_brake(now)
        if new_human_status == "missing_near":
            self._cancel_human_return()
            return (
                STATE_SAFE_STOP, 0.0,
                "human_new_person_absence_check", None)

        if self.human_consumed_active:
            if (
                self.human_pass_active
                and float(now) < self.human_speed_hold_until
            ):
                return self._human_pass_command(
                    lookahead_path_x, image_shape,
                    self.human_pass_side or 1)
            self.human_pass_active = False
            self.human_pass_offset_x = 0.0
            # After the one-second launch, resume ordinary line following but
            # keep ignoring the already-consumed lower-screen person.
            return None

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

        if self.human_current_prefer_upper:
            human = min(
                humans,
                key=lambda item: (
                    float(item["geom"]["bottom_ratio"]),
                    -float(item["score"]),
                ),
            )
        else:
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
            return self._human_pass_command(
                lookahead_path_x, image_shape,
                self.human_pass_side or side)

        crossed = (
            self.human_waiting_cross
            and human["side"] != 0
            and self.human_last_side is not None
            and human["side"] != self.human_last_side
        )
        if crossed:
            self.human_waiting_cross = False
            self.human_pass_active = True
            self.human_pass_side = side
            self.human_speed_hold_until = (
                float(now) + self.config.human_speed_hold_s)
            self._schedule_human_return(
                self.human_speed_hold_until, side)
            self._begin_consumed_human(now)
            self.human_detected_latched = False
            self.human_last_seen_ts = 0.0
            return self._human_pass_command(lookahead_path_x, image_shape, side)

        if self._human_on_stop_line(geom, image_shape, lookahead_y):
            first_line_contact = not self.human_waiting_cross
            self._clear_human_preline_state()
            self.human_detected_latched = True
            self.human_last_seen_ts = float(now)
            self.human_waiting_cross = True
            if human["side"] != 0:
                self.human_last_side = human["side"]
            if first_line_contact:
                return self._start_human_brake(now)
            return STATE_SAFE_STOP, 0.0, "human_half_lookahead_stop", None

        if self.human_waiting_cross:
            self.human_last_seen_ts = float(now)
            return STATE_SAFE_STOP, 0.0, "human_wait_cross", None

        self._record_human_preline_gap(geom, image_shape, lookahead_y)
        return None

    def _human_pass_command(self, path_x, image_shape, human_side):
        scale = float(max(1, image_shape[1])) / 640.0
        offset = self.config.human_pass_offset_px_640 * scale
        target_x = _clamp(
            float(path_x) - float(human_side or 1) * offset,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        self.human_pass_offset_x = float(target_x) - float(path_x)
        return STATE_AVOID_HUMAN, self.config.human_pass_speed_mps, "human_cross_pass", target_x

    def _schedule_human_return(self, start_ts, error_sign):
        sign = self._sign(error_sign)
        if sign == 0 or self.config.human_return_duration_s <= 0.0:
            self._cancel_human_return()
            return
        self.human_return_start_ts = float(start_ts)
        self.human_return_until = (
            self.human_return_start_ts +
            self.config.human_return_duration_s)
        self.human_return_error_sign = sign

    def _cancel_human_return(self):
        self.human_return_start_ts = 0.0
        self.human_return_until = 0.0
        self.human_return_error_sign = 0

    def _human_return_active(self, now):
        if self.human_return_until <= 0.0:
            return False
        if float(now) < self.human_return_start_ts:
            return False
        if float(now) >= self.human_return_until:
            self._cancel_human_return()
            return False
        return self.human_return_error_sign != 0

    def _human_return_extra_error(self, now):
        if not self._human_return_active(now):
            return 0.0
        duration = max(
            1e-6,
            self.human_return_until - self.human_return_start_ts)
        progress = _clamp(
            (float(now) - self.human_return_start_ts) / duration,
            0.0,
            1.0,
        )
        return (
            float(self.human_return_error_sign) *
            self.config.human_return_error_640 *
            (1.0 - progress))

    def _start_human_brake(self, now, target_x=None):
        self._cancel_human_return()
        self.human_brake_reverse_until = (
            float(now) + self.config.human_brake_reverse_duration_s)
        return (
            STATE_AVOID_HUMAN,
            self.config.human_brake_reverse_speed_mps,
            "human_brake_reverse",
            target_x,
        )

    def _human_brake_action(self, now):
        if self.human_brake_reverse_until <= 0.0:
            return None
        if float(now) < self.human_brake_reverse_until:
            return (
                STATE_AVOID_HUMAN,
                self.config.human_brake_reverse_speed_mps,
                "human_brake_reverse",
                None,
            )
        self.human_brake_reverse_until = 0.0
        return None

    def _human_safety_has_priority(
            self, detections, image_shape, lookahead_y):
        if (
            self.human_waiting_cross
            or self.human_detected_latched
            or self.car_human_waiting_cross
            or self.car_human_active
        ):
            return True
        if (
            self.human_preline_last_gap_px_480 is not None
            and self.human_preline_last_gap_px_480 <=
            self.config.human_preline_missing_px_480
        ):
            return True
        if self.human_new_pending and self.human_new_last_geom is not None:
            stop_y = self._human_stop_line_y(image_shape, lookahead_y)
            gap_px_480 = max(
                0.0,
                float(stop_y) -
                float(self.human_new_last_geom["bottom"]),
            ) * 480.0 / float(max(1, image_shape[0]))
            if gap_px_480 <= self.config.human_preline_missing_px_480:
                return True
        for det in detections or []:
            if self._normalized_label(det) != "human":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_human_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if (
                geom is not None
                and self._human_on_stop_line(
                    geom, image_shape, lookahead_y)
            ):
                return True
        return False

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

    def _begin_consumed_human(self, now):
        """Ignore the released lower-screen person, but arm for a new one."""
        self.human_consumed_active = True
        self.human_consumed_clear_since = 0.0
        self.human_new_pending = False
        self.human_new_last_geom = None
        self.human_new_last_seen_ts = float(now)
        self.human_current_prefer_upper = False

    def _clear_consumed_human_state(self):
        self.human_consumed_active = False
        self.human_consumed_clear_since = 0.0
        self.human_new_pending = False
        self.human_new_last_geom = None
        self.human_new_last_seen_ts = 0.0

    def _observe_new_human_while_consumed(
            self, detections, image_shape, lookahead_y, now):
        """Recognize a new person by first seeing it above the lookahead row."""
        if not self.human_consumed_active:
            return None, None

        people = []
        for det in detections or []:
            if self._normalized_label(det) != "human":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_human_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is not None:
                people.append({"geom": geom, "score": float(score)})

        if not people:
            if self.human_consumed_clear_since <= 0.0:
                self.human_consumed_clear_since = float(now)
            if (
                float(now) - self.human_consumed_clear_since >=
                self.config.human_absence_confirm_s
            ):
                self._clear_consumed_human_state()
                return None, None
        else:
            self.human_consumed_clear_since = 0.0

        above_lookahead = [
            item for item in people
            if float(item["geom"]["bottom"]) < float(lookahead_y)
        ]
        candidate = None
        if above_lookahead:
            # When several new people are above the lookahead row, track the
            # one closest to reaching the stop line first.
            candidate = max(
                above_lookahead,
                key=lambda item: (
                    float(item["geom"]["bottom"]), float(item["score"])))
            self.human_new_pending = True
        elif self.human_new_pending and people:
            # Once armed above the lookahead row, follow the uppermost person
            # as it moves downward. The already-consumed person remains lower
            # in the image and is therefore not selected.
            stop_y = self._human_stop_line_y(image_shape, lookahead_y)
            followable_people = [
                item for item in people
                if float(item["geom"]["bottom"]) <= (
                    float(stop_y) +
                    self.config.human_preline_missing_px_480 *
                    float(max(1, image_shape[0])) / 480.0)
            ]
            # A lower-screen detection has already crossed the monitoring
            # line, so it belongs to the consumed person and must not be
            # mistaken for a newly armed person that just disappeared.
            previous = self.human_new_last_geom
            if previous is not None and followable_people:
                topmost = min(
                    followable_people,
                    key=lambda item: (
                        abs(float(item["geom"]["bottom"]) -
                            float(previous["bottom"])),
                        abs(float(item["geom"]["cx"]) -
                            float(previous["cx"])),
                    ),
                )
                max_vertical_jump = (
                    60.0 * float(max(1, image_shape[0])) / 480.0)
                max_horizontal_jump = max(
                    40.0 * float(max(1, image_shape[1])) / 640.0,
                    float(previous["box_w"]),
                )
                vertical_jump = abs(
                    float(topmost["geom"]["bottom"]) -
                    float(previous["bottom"]))
                horizontal_jump = abs(
                    float(topmost["geom"]["cx"]) -
                    float(previous["cx"]))
                if (
                    vertical_jump <= max_vertical_jump
                    and horizontal_jump <= max_horizontal_jump
                ):
                    candidate = topmost
            elif followable_people:
                candidate = min(
                    followable_people,
                    key=lambda item: (
                        float(item["geom"]["bottom"]),
                        -float(item["score"])))

        if candidate is not None:
            self.human_new_last_geom = dict(candidate["geom"])
            self.human_new_last_seen_ts = float(now)
            if self._human_on_stop_line(
                    candidate["geom"], image_shape, lookahead_y):
                return "line", candidate
            return None, None

        if self.human_new_pending and self.human_new_last_geom is not None:
            stop_y = self._human_stop_line_y(image_shape, lookahead_y)
            gap_px_480 = max(
                0.0,
                stop_y - float(self.human_new_last_geom["bottom"]),
            ) * 480.0 / float(max(1, image_shape[0]))
            missing_age = max(
                0.0, float(now) - self.human_new_last_seen_ts)
            if (
                gap_px_480 <= self.config.human_preline_missing_px_480
                and missing_age < self.config.human_absence_confirm_s
            ):
                return "missing_near", None
            if (
                gap_px_480 > self.config.human_preline_missing_px_480
                or missing_age >= self.config.human_absence_confirm_s
            ):
                self.human_new_pending = False
                self.human_new_last_geom = None
                self.human_new_last_seen_ts = 0.0
        return None, None

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
        self.human_pass_side = None
        self.human_current_prefer_upper = False
        if clear_detection:
            self.human_detected_latched = False
            self.human_last_seen_ts = 0.0
            self._clear_human_preline_state()

    def _sign_should_stop(self, geom, image_shape, lookahead_y):
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
        offset = max(float(base_offset) * scale, float(geom["box_w"]) * self.config.avoid_box_width_gain)
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
        return self._normalized_label(det) in {"turnsign", "roadsign", "sign"}

    def _error_trend_mode_for(self, state, task_reason, speed):
        if float(speed) <= 0.0:
            return None
        reason = str(task_reason or "")
        if reason == "track":
            return "track"
        if reason == "human_return":
            return "human_return"
        if int(state) == STATE_AVOID_CAR:
            return "avoid_car"
        if (
            int(state) == STATE_AVOID_HUMAN
            and reason != "human_brake_reverse"
        ):
            return "avoid_human"
        return None

    def _reset_error_trend(self):
        self.error_trend_history = []
        self.error_trend_sign = 0
        self.error_trend_mode = None

    def _apply_error_trend(self, error, mode):
        if mode is None or self.config.error_trend_kd <= 0.0:
            self._reset_error_trend()
            return float(error), 0.0, 0.0, 0

        value = float(error)
        sign = self._sign(value)
        if sign == 0:
            self._reset_error_trend()
            return value, 0.0, 0.0, 0
        if self.error_trend_mode != mode or self.error_trend_sign != sign:
            self.error_trend_history = []
            self.error_trend_mode = mode
            self.error_trend_sign = sign

        self.error_trend_history.append(abs(value))
        window = max(3, int(self.config.error_trend_window))
        if len(self.error_trend_history) > window:
            self.error_trend_history = self.error_trend_history[-window:]
        frames = len(self.error_trend_history)
        min_frames = min(
            window, max(3, int(self.config.error_trend_min_frames)))
        if frames < min_frames:
            return value, 0.0, 0.0, frames

        magnitudes = np.asarray(
            self.error_trend_history, dtype=np.float64)
        x = np.arange(frames, dtype=np.float64)
        x -= float(np.mean(x))
        denominator = float(np.dot(x, x))
        slope = (
            float(np.dot(x, magnitudes - float(np.mean(magnitudes)))) /
            denominator
            if denominator > 0.0 else 0.0)
        if abs(slope) < self.config.error_trend_deadband_640:
            return value, 0.0, slope, frames

        window_delta = slope * float(max(1, frames - 1))
        magnitude_adjustment = _clamp(
            self.config.error_trend_kd * window_delta,
            -self.config.error_trend_max_adjust_640,
            self.config.error_trend_max_adjust_640,
        )
        original_magnitude = abs(value)
        adjusted_magnitude = max(
            0.0, original_magnitude + magnitude_adjustment)
        adjusted = float(sign) * adjusted_magnitude
        return (
            adjusted,
            adjusted - value,
            slope,
            frames,
        )

    def _limit_error(self, error, max_error=None, max_step=None):
        limit = (
            self.config.max_track_error_640
            if max_error is None else max(1.0, float(max_error)))
        error = _clamp(error, -limit, limit)
        step = (
            self.config.max_error_step_640
            if max_step is None else max(1.0, float(max_step)))
        delta = _clamp(error - self.last_error, -step, step)
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

    def _ocr_has_current_direction(self, response):
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

    def _expire_ocr_lock(self, now):
        if self.branch_lock_source != "ocr" or self.last_valid_ocr_ts <= 0.0:
            return False
        if now - self.last_valid_ocr_ts < self.config.ocr_lock_lifetime_s:
            return False
        if self.config.path_source == "curve":
            self._set_default_curve_branch()
        else:
            self._clear_branch_lock()
        return True

    def _set_default_curve_branch(self):
        self.branch_lock = "left"
        self.branch_lock_source = "default"
        self.selected_slot_lock = 0
        self.selected_slot_missing_frames = 0

    def _clear_branch_lock(self):
        self.branch_lock = None
        self.branch_lock_source = None
        self.selected_slot_lock = None
        self.selected_slot_missing_frames = 0
        self.last_selected_points = None

    def _ocr_lock_age(self, now):
        if self.branch_lock_source != "ocr" or self.last_valid_ocr_ts <= 0.0:
            return None
        return max(0.0, now - self.last_valid_ocr_ts)

    def _ocr_lock_remaining(self, now):
        age = self._ocr_lock_age(now)
        if age is None:
            return None
        return max(0.0, self.config.ocr_lock_lifetime_s - age)

    def _default_outer_elapsed(self, now):
        if self.fork_seen_since is None:
            return 0.0
        return max(0.0, now - self.fork_seen_since)

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

    peak_detection = result.get("heatmap_peak_detection") or {}
    scan_ratios = peak_detection.get("scan_ratios") or []
    if len(scan_ratios) == 2:
        scan_top = int(round(_clamp(scan_ratios[0], 0.0, 1.0) * (h - 1)))
        scan_bottom = int(round(_clamp(scan_ratios[1], 0.0, 1.0) * (h - 1)))
        cv2.rectangle(
            frame, (0, min(scan_top, scan_bottom)),
            (w - 1, max(scan_top, scan_bottom)),
            (30, 230, 255), 1, cv2.LINE_AA)

    for line in _extract_heatmap_preview_lines(result, frame.shape):
        points = np.rint(line["points_xy"]).astype(np.int32)
        if len(points) < 2:
            continue
        points[:, 0] = np.clip(points[:, 0], 0, w - 1)
        points[:, 1] = np.clip(points[:, 1], 0, h - 1)
        _draw_identity_probability_path(
            frame, points, line.get("probabilities"),
            int(line.get("slot", 0)),
            thickness=3 if int(line.get("slot", -1)) == selected_slot else 2)

    target = debug.get("control_target") or {}
    path_target_x = _finite_float(target.get("path_target_x"))
    target_x = _finite_float(target.get("target_x"), path_target_x)
    lookahead_y = _finite_float(
        target.get("path_target_y"),
        _finite_float(target.get("lookahead_y"), h * 0.62))
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
        "detected_path_count",
        peak_detection.get("detected_path_count", 1)))
    cv2.putText(
        frame, "PATH COUNT: {}".format(detected_count),
        (10, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
        (30, 230, 255), 2, cv2.LINE_AA)
    return frame


def _identity_probability_color(slot, probability):
    intensity = int(round(80.0 + 175.0 * _clamp(probability, 0.0, 1.0)))
    # OpenCV uses BGR: left is always blue, right is always green.
    return (intensity, 0, 0) if int(slot) == 0 else (0, intensity, 0)


def _draw_identity_probability_path(
        frame, points, probabilities, slot, thickness=2):
    probabilities = np.asarray(probabilities, dtype=np.float32)
    if len(probabilities) != len(points):
        probabilities = np.ones(len(points), dtype=np.float32)
    for index in range(len(points) - 1):
        probability = 0.5 * (
            float(probabilities[index]) +
            float(probabilities[index + 1]))
        cv2.line(
            frame, tuple(points[index]), tuple(points[index + 1]),
            _identity_probability_color(slot, probability),
            int(thickness), cv2.LINE_AA)


def _extract_heatmap_preview_lines(result, image_shape, max_lines=2):
    if not isinstance(result, dict):
        return []
    paths = result.get("display_paths")
    if paths is None:
        paths = result.get("paths") or []
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
