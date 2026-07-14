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
_TEMPORAL_PATH_SAMPLES = 24
_SOLID_RED_PREVIEW = None


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
        os.environ.get("MULTITASK_PATH_SOURCE", "skeleton"),
    ).strip().lower()
    return value if value in {"curve", "heatmap", "skeleton"} else "skeleton"


def _finite_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


def _smooth_binary_mask_edges(mask, kernel_size=9):
    """Round jagged binary-mask edges without returning grayscale pixels."""
    mask = (np.asarray(mask) != 0).astype(np.uint8)
    kernel_size = max(1, int(kernel_size))
    if kernel_size <= 1 or not np.any(mask):
        return mask
    if kernel_size % 2 == 0:
        kernel_size += 1
    blurred = cv2.GaussianBlur(
        mask * 255, (kernel_size, kernel_size), 0.0,
        borderType=cv2.BORDER_REPLICATE)
    return (blurred >= 127).astype(np.uint8)


def _fill_binary_mask(mask, max_hole_area=32):
    """Fill only small enclosed holes without filling large fork interiors."""
    mask = (np.asarray(mask) != 0).astype(np.uint8)
    max_hole_area = max(0, int(max_hole_area))
    if mask.ndim != 2 or not mask.size or max_hole_area <= 0:
        return mask
    background = (mask == 0).astype(np.uint8)
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
        background, connectivity=8)
    if count <= 1:
        return mask
    border_labels = np.unique(np.concatenate((
        labels[0], labels[-1], labels[:, 0], labels[:, -1],
    )))
    candidates = np.arange(1, count, dtype=np.int32)
    holes = candidates[
        (~np.isin(candidates, border_labels)) &
        (stats[candidates, cv2.CC_STAT_AREA] <= max_hole_area)]
    filled = mask.copy()
    if len(holes):
        filled[np.isin(labels, holes)] = 1
    return filled


def _spatial_hysteresis_binary_mask(
        heatmap, high_threshold=0.35, low_threshold=0.27):
    """Keep weak heat only when it belongs to a current strong component."""
    heatmap = np.asarray(heatmap, dtype=np.float32)
    if heatmap.ndim != 2 or not heatmap.size:
        return np.zeros(heatmap.shape, dtype=np.uint8)
    high_threshold = _clamp(high_threshold, 0.0, 1.0)
    low_threshold = _clamp(low_threshold, 0.0, high_threshold)
    if high_threshold <= 0.0:
        return (heatmap > 0.0).astype(np.uint8)
    strong = heatmap >= high_threshold
    if not np.any(strong):
        return np.zeros(heatmap.shape, dtype=np.uint8)
    weak = heatmap >= low_threshold
    count, labels = cv2.connectedComponents(
        weak.astype(np.uint8), connectivity=8)
    if count <= 1:
        return np.zeros(heatmap.shape, dtype=np.uint8)
    seeded_labels = np.unique(labels[strong])
    seeded_labels = seeded_labels[seeded_labels != 0]
    return np.isin(labels, seeded_labels).astype(np.uint8)


def _connect_binary_mask_samples(
        mask, max_gap=6, minimum_group_area=15, bridge_thickness=3):
    """Connect nearby mask fragments as discrete samples.

    Components whose dilated neighborhoods touch form one proximity group.
    A minimum spanning forest then draws only the shortest bridges needed to
    make each sufficiently large group continuous.  The work is performed on
    component contours, never with Python loops over image pixels.
    """
    mask = (np.asarray(mask) != 0).astype(np.uint8)
    if mask.ndim != 2 or not np.any(mask):
        return mask

    max_gap = max(0, int(max_gap))
    minimum_group_area = max(1, int(minimum_group_area))
    bridge_thickness = max(1, int(bridge_thickness))
    component_count, labels, stats, _centroids = (
        cv2.connectedComponentsWithStats(mask, connectivity=8))
    if component_count <= 1:
        return np.zeros_like(mask)

    component_labels = np.arange(1, component_count, dtype=np.int32)
    if max_gap > 0 and component_count > 2:
        radius = max(1, int(math.ceil(float(max_gap) * 0.5)))
        proximity_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1))
        proximity_mask = cv2.dilate(mask, proximity_kernel, iterations=1)
        proximity_count, proximity_labels = cv2.connectedComponents(
            proximity_mask, connectivity=8)
        component_groups = np.zeros(component_count, dtype=np.int32)
        foreground = labels != 0
        np.maximum.at(
            component_groups, labels[foreground],
            proximity_labels[foreground])
    else:
        proximity_count = component_count
        component_groups = np.arange(component_count, dtype=np.int32)

    group_areas = np.bincount(
        component_groups[1:],
        weights=stats[1:, cv2.CC_STAT_AREA].astype(np.float64),
        minlength=proximity_count)
    valid_groups = np.flatnonzero(group_areas >= minimum_group_area)
    valid_components = component_labels[np.isin(
        component_groups[1:], valid_groups)]
    if not len(valid_components):
        return np.zeros_like(mask)

    connected = np.isin(labels, valid_components).astype(np.uint8)
    if max_gap <= 0 or len(valid_components) <= 1:
        return connected

    valid_component_set = set(valid_components.tolist())
    boundary_parts = {}
    contours, _hierarchy = cv2.findContours(
        connected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        points = contour.reshape((-1, 2))
        if not len(points):
            continue
        component = int(labels[points[0, 1], points[0, 0]])
        if component in valid_component_set:
            boundary_parts.setdefault(component, []).append(points)
    boundaries = {
        component: np.concatenate(parts).astype(np.float32)
        for component, parts in boundary_parts.items()
    }

    for group in valid_groups:
        members = valid_components[
            component_groups[valid_components] == int(group)]
        if len(members) <= 1:
            continue

        edges = []
        for first_index in range(len(members) - 1):
            first_label = int(members[first_index])
            first_points = boundaries.get(first_label)
            if first_points is None or not len(first_points):
                continue
            for second_index in range(first_index + 1, len(members)):
                second_label = int(members[second_index])
                second_points = boundaries.get(second_label)
                if second_points is None or not len(second_points):
                    continue
                first_left = int(stats[first_label, cv2.CC_STAT_LEFT])
                first_top = int(stats[first_label, cv2.CC_STAT_TOP])
                first_right = first_left + int(
                    stats[first_label, cv2.CC_STAT_WIDTH]) - 1
                first_bottom = first_top + int(
                    stats[first_label, cv2.CC_STAT_HEIGHT]) - 1
                second_left = int(stats[second_label, cv2.CC_STAT_LEFT])
                second_top = int(stats[second_label, cv2.CC_STAT_TOP])
                second_right = second_left + int(
                    stats[second_label, cv2.CC_STAT_WIDTH]) - 1
                second_bottom = second_top + int(
                    stats[second_label, cv2.CC_STAT_HEIGHT]) - 1
                gap_x = max(
                    0, second_left - first_right,
                    first_left - second_right)
                gap_y = max(
                    0, second_top - first_bottom,
                    first_top - second_bottom)
                if gap_x * gap_x + gap_y * gap_y > max_gap * max_gap:
                    # Exact contour distance cannot be smaller than the
                    # bounding-box distance.  Reject far component pairs
                    # before the much more expensive batchDistance call.
                    continue
                distances, nearest = cv2.batchDistance(
                    first_points, second_points, cv2.CV_32F,
                    normType=cv2.NORM_L2, K=1)
                nearest_first = int(np.argmin(distances[:, 0]))
                distance = float(distances[nearest_first, 0])
                if distance > float(max_gap):
                    continue
                nearest_second = int(nearest[nearest_first, 0])
                edges.append((
                    distance, first_index, second_index,
                    first_points[nearest_first],
                    second_points[nearest_second],
                ))

        parents = list(range(len(members)))

        def find_root(index):
            while parents[index] != index:
                parents[index] = parents[parents[index]]
                index = parents[index]
            return index

        for _distance, first_index, second_index, first, second in sorted(
                edges, key=lambda item: item[0]):
            first_root = find_root(first_index)
            second_root = find_root(second_index)
            if first_root == second_root:
                continue
            parents[second_root] = first_root
            cv2.line(
                connected,
                tuple(np.rint(first).astype(np.int32)),
                tuple(np.rint(second).astype(np.int32)),
                1, bridge_thickness, cv2.LINE_8)

    return connected


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


def _resample_path_by_count(points, sample_count=48):
    """Resample a 2-D path by arc length, including horizontal paths."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        return points.copy()
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.concatenate((
        np.asarray([True]), segment_lengths > 1e-5))
    points = points[keep]
    if len(points) < 2:
        return points.copy()
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.concatenate((
        np.asarray([0.0], dtype=np.float32),
        np.cumsum(segment_lengths, dtype=np.float32)))
    total = float(arc[-1])
    if total <= 1e-5:
        return points.copy()
    sample_count = max(2, int(sample_count))
    samples = np.linspace(0.0, total, sample_count, dtype=np.float32)
    return np.column_stack((
        np.interp(samples, arc, points[:, 0]),
        np.interp(samples, arc, points[:, 1]),
    )).astype(np.float32)


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
    max_track_error_640: float = 210.0
    max_error_step_640: float = 36.0
    normal_speed_mps: float = 0.15
    recover_speed_mps: float = 0.15
    obstacle_speed_mps: float = 0.15
    human_speed_mps: float = 0.15
    human_pass_speed_mps: float = 0.35
    collect_speed_mps: float = 0.15
    turnsign_slow_speed_mps: float = 0.08
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
    path_ema_alpha: float = 0.50
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
    skeleton_threshold: float = 0.35
    skeleton_low_threshold: float = 0.27
    skeleton_min_area: int = 15
    skeleton_min_length: int = 8
    skeleton_close_iterations: int = 1
    skeleton_edge_smooth_kernel: int = 9
    skeleton_max_hole_area: int = 32
    skeleton_connect_max_gap: int = 6
    skeleton_bridge_thickness: int = 3
    skeleton_min_branch_length: int = 10
    skeleton_max_branches: int = 2
    default_outer_after_s: float = 15.0
    ocr_lock_lifetime_s: float = 10.0
    ocr_confirm_frames: int = 1
    outer_slot: int = 0
    no_path_stop_s: float = 0.8
    recover_hold_s: float = 0.5
    hazard_bottom_ratio: float = 0.58
    hazard_lateral_ratio: float = 0.18
    coin_bottom_ratio: float = 0.55
    coin_lateral_ratio: float = 0.24
    sign_stop_height_ratio: float = 0.24
    sign_stop_area_ratio: float = 0.035
    sign_stop_line_margin_ratio: float = 0.08
    sign_slow_min_score: float = 0.35
    sign_ocr_timeout_s: float = 8.0
    sign_ocr_pulse_speed_mps: float = 0.25
    sign_ocr_pulse_duration_s: float = 0.30
    human_stop_line_margin_ratio: float = 0.08
    human_stop_progress_ratio: float = 0.75
    human_cross_release_px_640: float = 45.0
    human_pass_offset_px_640: float = 38.0
    human_speed_hold_s: float = 0.5
    car_avoid_offset_px_640: float = 55.0
    human_avoid_offset_px_640: float = 75.0
    avoid_box_width_gain: float = 0.35
    gold_bias_gain: float = 0.45
    gold_max_bias_px_640: float = 75.0
    min_human_score: float = 0.35
    min_car_score: float = 0.35
    min_coin_score: float = 0.35
    path_source: str = "skeleton"

    @classmethod
    def from_env(cls):
        return cls(
            visual_center_x=_clamp(_env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8),
            lookahead_y_ratio=_clamp(_env_float("VISION_CONTROL_LOOKAHEAD_Y_RATIO", 0.625), 0.25, 0.95),
            max_track_error_640=max(1.0, _env_float("VISION_CONTROL_MAX_ERROR_640", 210.0)),
            max_error_step_640=max(1.0, _env_float("VISION_CONTROL_MAX_STEP_640", 36.0)),
            normal_speed_mps=max(0.0, _env_float("VISION_CONTROL_NORMAL_SPEED", 0.15)),
            recover_speed_mps=max(0.0, _env_float("VISION_CONTROL_RECOVER_SPEED", 0.15)),
            obstacle_speed_mps=max(0.0, _env_float("VISION_CONTROL_OBSTACLE_SPEED", 0.15)),
            human_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED", 0.15)),
            human_pass_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_SPEED", 0.35)),
            collect_speed_mps=max(0.0, _env_float("VISION_CONTROL_COLLECT_SPEED", 0.15)),
            turnsign_slow_speed_mps=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_SLOW_SPEED", 0.08)),
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
            path_ema_alpha=_clamp(_env_float("VISION_CONTROL_PATH_EMA_ALPHA", 0.50), 0.0, 1.0),
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
            skeleton_threshold=_clamp(
                _env_float("VISION_CONTROL_SKELETON_THRESHOLD", 0.35),
                0.0, 0.99),
            skeleton_low_threshold=_clamp(
                _env_float(
                    "VISION_CONTROL_SKELETON_LOW_THRESHOLD", 0.27),
                0.0, 0.99),
            skeleton_min_area=max(
                1, _env_int("VISION_CONTROL_SKELETON_MIN_AREA", 15)),
            skeleton_min_length=max(
                2, _env_int("VISION_CONTROL_SKELETON_MIN_LENGTH", 8)),
            skeleton_close_iterations=max(
                0, _env_int("VISION_CONTROL_SKELETON_CLOSE_ITERATIONS", 1)),
            skeleton_edge_smooth_kernel=max(
                1, _env_int("VISION_CONTROL_SKELETON_EDGE_KERNEL", 9)),
            skeleton_max_hole_area=max(
                0, _env_int(
                    "VISION_CONTROL_SKELETON_MAX_HOLE_AREA", 32)),
            skeleton_connect_max_gap=max(
                0, _env_int(
                    "VISION_CONTROL_SKELETON_MAX_CONNECT_GAP", 6)),
            skeleton_bridge_thickness=max(
                1, _env_int(
                    "VISION_CONTROL_SKELETON_BRIDGE_THICKNESS", 3)),
            skeleton_min_branch_length=max(
                1, _env_int(
                    "VISION_CONTROL_SKELETON_MIN_BRANCH_LENGTH", 10)),
            skeleton_max_branches=max(
                1, _env_int(
                    "VISION_CONTROL_SKELETON_MAX_BRANCHES", 2)),
            default_outer_after_s=max(0.0, _env_float("VISION_CONTROL_DEFAULT_OUTER_AFTER", 15.0)),
            ocr_lock_lifetime_s=max(
                0.1, _env_float("VISION_CONTROL_OCR_LOCK_LIFETIME_S", 10.0)),
            ocr_confirm_frames=max(1, _env_int("VISION_CONTROL_OCR_CONFIRM_FRAMES", 1)),
            outer_slot=max(0, min(1, _env_int("VISION_CONTROL_OUTER_SLOT", 0))),
            no_path_stop_s=max(0.1, _env_float("VISION_CONTROL_NO_PATH_STOP_S", 0.8)),
            recover_hold_s=max(0.0, _env_float("VISION_CONTROL_RECOVER_HOLD_S", 0.5)),
            hazard_bottom_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_BOTTOM_RATIO", 0.58), 0.0, 1.0),
            hazard_lateral_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_LATERAL_RATIO", 0.18), 0.01, 0.5),
            coin_bottom_ratio=_clamp(_env_float("VISION_CONTROL_COIN_BOTTOM_RATIO", 0.55), 0.0, 1.0),
            coin_lateral_ratio=_clamp(_env_float("VISION_CONTROL_COIN_LATERAL_RATIO", 0.24), 0.01, 0.5),
            sign_stop_height_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_HEIGHT_RATIO", 0.24), 0.01, 1.0),
            sign_stop_area_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_AREA_RATIO", 0.035), 0.001, 1.0),
            sign_stop_line_margin_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_LINE_MARGIN_RATIO", 0.08), 0.0, 0.5),
            sign_slow_min_score=_clamp(_env_float("VISION_CONTROL_SIGN_SLOW_MIN_SCORE", 0.35), 0.0, 1.0),
            sign_ocr_timeout_s=max(0.1, _env_float("VISION_CONTROL_SIGN_OCR_TIMEOUT_S", 8.0)),
            sign_ocr_pulse_speed_mps=max(0.0, _env_float("VISION_CONTROL_SIGN_OCR_PULSE_SPEED", 0.25)),
            sign_ocr_pulse_duration_s=max(0.0, _env_float("VISION_CONTROL_SIGN_OCR_PULSE_DURATION_S", 0.30)),
            human_stop_line_margin_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_LINE_MARGIN_RATIO", 0.08), 0.0, 0.5),
            human_stop_progress_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_PROGRESS_RATIO", 0.75), 0.0, 1.0),
            human_cross_release_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_CROSS_RELEASE_640", 45.0)),
            human_pass_offset_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_OFFSET_640", 38.0)),
            human_speed_hold_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED_HOLD_S", 0.5)),
            car_avoid_offset_px_640=max(0.0, _env_float("VISION_CONTROL_CAR_AVOID_OFFSET_640", 55.0)),
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
        self.last_slot_points = {}
        self.last_slot_branch_tails = {}
        self.last_valid_ts = 0.0
        self.last_error = 0.0
        self.route_state = ROUTE_NONE
        self.route_reason = "initial"
        self.route_initialized = False
        self.pending_route_state = None
        self.pending_route_frames = 0
        self.branch_lock = None
        self.branch_lock_source = None
        self.selected_slot_lock = None
        self.selected_branch_signature = None
        self.selected_slot_missing_frames = 0
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
        self.human_speed_hold_until = 0.0
        self.sign_ocr_active_since = None
        self.sign_ocr_pulse_until = 0.0
        self.sign_ocr_pulse_sent = False

    def update(self, perception_result, ocr_response=None, now=None):
        now = float(time.monotonic() if now is None else now)
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
            wanted_slot = 1 if ocr_direction == "right" else 0
            if self.selected_slot_lock != wanted_slot:
                self.selected_branch_signature = None
            self.branch_lock = "right" if ocr_direction == "right" else "left"
            self.branch_lock_source = "ocr"
            self.selected_slot_lock = wanted_slot
            self.selected_slot_missing_frames = 0
        else:
            ocr_lock_expired = self._expire_ocr_lock(now)
        self._update_default_outer(route_state, now, ocr_current)
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
            "selected_branch_temporal_lock": bool(
                self.selected_branch_signature is not None),
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
                candidate["source"] = "direct_curve"
                candidate["source_slot"] = int(path.get("slot", len(candidates)))
                candidate["source_slots"] = {candidate["source_slot"]}
                candidate["coverage"] = 1.0
                candidate["road_support"] = None
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
            road = (result.get("road") or {}).get("mask")
            if road is None:
                road = result.get("road_mask")
            effective_maps, support_maps, _has_road = (
                self._prepare_heatmap_support_maps(heatmaps, road))
            if self.config.path_source == "skeleton":
                (candidates, invalid_slots, skeleton_masks,
                 distribution_masks) = (
                    self._extract_heatmap_debug_lines(
                    effective_maps, road, image_shape,
                    support_maps=support_maps, return_invalid=True,
                    return_skeleton_masks=True))
                result["semantic_skeleton_masks"] = skeleton_masks
                result["semantic_heat_distribution_masks"] = (
                    distribution_masks)
                peak_debug = {
                    "method": "semantic_skeleton",
                    "binary_threshold": float(
                        self.config.skeleton_threshold),
                    "spatial_low_threshold": float(min(
                        self.config.skeleton_threshold,
                        self.config.skeleton_low_threshold)),
                    "min_component_area": int(
                        self.config.skeleton_min_area),
                    "min_skeleton_length": int(
                        self.config.skeleton_min_length),
                    "max_connect_gap": int(
                        self.config.skeleton_connect_max_gap),
                    "max_hole_area": int(
                        self.config.skeleton_max_hole_area),
                    "min_branch_length": int(
                        self.config.skeleton_min_branch_length),
                    "max_branches": int(
                        self.config.skeleton_max_branches),
                    "detected_path_count": int(len(candidates)),
                    "valid_path_count": int(len(candidates)),
                    "invalid_slots": sorted(int(slot) for slot in invalid_slots),
                }
            else:
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

        candidates = self._assign_heatmap_slots(candidates[:2], image_shape)
        candidates.sort(key=lambda item: int(item.get("slot", 99)))
        candidates = self._smooth_candidates(
            candidates, image_shape, support_maps=support_maps)
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
            return_invalid=False, return_skeleton_masks=False):
        heatmaps = np.asarray(heatmaps, dtype=np.float32)
        if heatmaps.ndim != 3:
            empty_masks = np.empty((0, 0, 0), dtype=np.uint8)
            if return_invalid and return_skeleton_masks:
                return [], set(), empty_masks, empty_masks
            if return_invalid:
                return [], set()
            if return_skeleton_masks:
                return [], empty_masks
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
        skeleton_masks = []
        distribution_masks = []
        component_groups = []
        binary_threshold = float(self.config.skeleton_threshold)
        low_threshold = min(
            binary_threshold,
            float(self.config.skeleton_low_threshold))
        close_iterations = max(
            0, int(self.config.skeleton_close_iterations))
        edge_smooth_kernel = max(
            1, int(self.config.skeleton_edge_smooth_kernel))
        reference_area = 120 * 160
        heatmap_scale = math.sqrt(float(h * w) / float(reference_area))
        connect_max_gap = max(
            0, int(round(
                float(self.config.skeleton_connect_max_gap) *
                heatmap_scale)))
        bridge_thickness = max(
            1, int(round(
                float(self.config.skeleton_bridge_thickness) *
                heatmap_scale)))
        max_hole_area = max(
            0, int(round(
                float(self.config.skeleton_max_hole_area) *
                heatmap_scale ** 2)))
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (3, 3))
        minimum_area = max(
            1,
            int(round(
                float(self.config.skeleton_min_area) * float(h * w) /
                float(reference_area))),
        )
        for slot in range(min(2, heatmaps.shape[0])):
            heatmap = np.clip(effective_maps[slot], 0.0, 1.0)
            mask = _spatial_hysteresis_binary_mask(
                heatmap, binary_threshold, low_threshold)
            if has_road:
                mask &= (road >= road_threshold).astype(np.uint8)
            mask = _connect_binary_mask_samples(
                mask, max_gap=connect_max_gap,
                minimum_group_area=minimum_area,
                bridge_thickness=bridge_thickness)
            if has_road:
                mask &= (road >= road_threshold).astype(np.uint8)
            if close_iterations:
                mask = cv2.morphologyEx(
                    mask, cv2.MORPH_CLOSE, close_kernel,
                    iterations=close_iterations)
                if has_road:
                    mask &= (road >= road_threshold).astype(np.uint8)
            mask = _smooth_binary_mask_edges(mask, edge_smooth_kernel)
            if has_road:
                mask &= (road >= road_threshold).astype(np.uint8)
            mask = _fill_binary_mask(
                mask, max_hole_area=max_hole_area)
            if has_road:
                mask &= (road >= road_threshold).astype(np.uint8)
            components, labels, stats, _centroids = cv2.connectedComponentsWithStats(
                mask, connectivity=8)
            valid_labels = np.flatnonzero(
                stats[1:, cv2.CC_STAT_AREA] >= minimum_area) + 1
            cleaned_mask = np.isin(labels, valid_labels).astype(np.uint8) * 255
            # Publish the exact cleaned mask used for thinning so AR Preview
            # and the blue/green paths share one geometric source of truth.
            distribution_masks.append(
                (cleaned_mask != 0).astype(np.uint8))
            ximgproc = getattr(cv2, "ximgproc", None)
            thinning = getattr(ximgproc, "thinning", None)
            if callable(thinning) and np.any(cleaned_mask):
                padded_mask = cv2.copyMakeBorder(
                    cleaned_mask, 1, 1, 1, 1,
                    cv2.BORDER_CONSTANT, value=0)
                skeleton_mask = thinning(
                    padded_mask,
                    thinningType=getattr(
                        ximgproc, "THINNING_ZHANGSUEN", 0))[1:-1, 1:-1]
                skeleton_count, skeleton_labels, skeleton_stats, _ = (
                    cv2.connectedComponentsWithStats(
                        (skeleton_mask != 0).astype(np.uint8),
                        connectivity=8))
                valid_skeleton_labels = np.flatnonzero(
                    skeleton_stats[1:, cv2.CC_STAT_AREA] >=
                    int(self.config.skeleton_min_length)) + 1
                skeleton_mask = np.isin(
                    skeleton_labels, valid_skeleton_labels).astype(np.uint8)
            else:
                skeleton_mask = np.zeros_like(mask)
            skeleton_masks.append(skeleton_mask)
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
                np.maximum(
                    stats[valid_labels, cv2.CC_STAT_HEIGHT],
                    stats[valid_labels, cv2.CC_STAT_WIDTH],
                ).astype(np.float32) * 4.0 +
                np.sqrt(stats[valid_labels, cv2.CC_STAT_AREA].astype(
                    np.float32)) + mean_heat.astype(np.float32) * 0.01)
            order = np.argsort(priorities)[::-1]
            component_groups.append([
                (float(priorities[index]), int(slot),
                 int(valid_labels[index]), heatmap, labels, stats,
                 label_heat_sums, skeleton_mask)
                for index in order
            ])

        selected_components = [
            component
            for group in component_groups
            for component in group
        ]
        selected_components.sort(key=lambda item: item[0], reverse=True)

        for (_priority, slot, label, heatmap, labels, stats,
             label_heat_sums, skeleton_mask) in selected_components:
            component_paths = self._component_centerline_paths(
                labels, label, heatmap, image_shape,
                component_stats=stats[label],
                history_points=self.last_slot_points,
                skeleton_mask=skeleton_mask,
                max_paths=self.config.skeleton_max_branches,
                min_branch_length=(
                    self.config.skeleton_min_branch_length))
            branch_tail_start = 0
            if len(component_paths) >= 2:
                first_path = np.asarray(component_paths[0], dtype=np.float32)
                second_path = np.asarray(component_paths[1], dtype=np.float32)
                common_limit = min(len(first_path), len(second_path))
                same_prefix = np.all(np.isclose(
                    first_path[:common_limit], second_path[:common_limit],
                    rtol=0.0, atol=0.05), axis=1)
                differences = np.flatnonzero(~same_prefix)
                common_length = (
                    int(differences[0]) if len(differences)
                    else common_limit)
                branch_tail_start = max(0, common_length - 1)
            for branch_index, points in enumerate(component_paths):
                if len(points) < int(self.config.skeleton_min_length):
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
                    "hysteresis_low_threshold": float(low_threshold),
                    "source": "semantic_skeleton_root_branch",
                    "skeleton_component_key": (int(slot), int(label)),
                    "branch_index": int(branch_index),
                    "branch_count": int(len(component_paths)),
                    "spatial_prefiltered": True,
                }
                if len(component_paths) >= 2:
                    item["branch_tail_points_xy"] = np.asarray(
                        points[branch_tail_start:],
                        dtype=np.float32).copy()
                self._refresh_fragment_geometry(item)
                self._annotate_heat_support(
                    item, image_shape, support_maps)
                lines.append(item)
        joined = self._join_heatmap_fragments(
            lines, image_shape, support_maps=support_maps)
        joined = self._select_heatmap_lines(joined, image_shape)
        skeleton_masks = np.asarray(skeleton_masks, dtype=np.uint8)
        distribution_masks = np.asarray(
            distribution_masks, dtype=np.uint8)
        if return_invalid and return_skeleton_masks:
            return (joined, invalid_slots, skeleton_masks,
                    distribution_masks)
        if return_invalid:
            return joined, invalid_slots
        if return_skeleton_masks:
            return joined, skeleton_masks
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
        first_component = first.get("skeleton_component_key")
        second_component = second.get("skeleton_component_key")
        if (first_component is not None and
                first_component == second_component and
                int(first.get("branch_index", -1)) !=
                int(second.get("branch_index", -1)) and
                min(int(first.get("branch_count", 1)),
                    int(second.get("branch_count", 1))) >= 2):
            # Both paths were already pruned by their distinct root-to-leaf
            # tail length. Keep their shared root trunk instead of treating
            # the overlap as a duplicate line.
            return False
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
            rows = low + (high - low) * _PAIR_FRACTIONS
            first_x = _interp_path_x_many(first, rows)
            second_x = _interp_path_x_many(second, rows)
            valid = np.isfinite(first_x) & np.isfinite(second_x)
            if np.any(valid):
                return (
                    float(np.mean(np.abs(first_x[valid] - second_x[valid]))) *
                    640.0 / float(max(1, image_shape[1])))

        endpoint_distance = min(
            abs(float(first_point[0]) - float(second_point[0]))
            for first_point in (first[0], first[-1])
            for second_point in (second[0], second[-1])
        )
        return endpoint_distance * 640.0 / float(max(1, image_shape[1]))

    @staticmethod
    def _path_shape_distance_640(
            first_points, second_points, image_shape, sample_count=16):
        """Compare branch geometry in 2-D with one vectorized resampling."""
        first = _resample_path_by_count(first_points, sample_count)
        second = _resample_path_by_count(second_points, sample_count)
        return VisionControlPlanner._sampled_shape_distance_640(
            first, second, image_shape, sample_count)

    @staticmethod
    def _sampled_shape_distance_640(
            first, second, image_shape, sample_count=16):
        first = np.asarray(first, dtype=np.float32)
        second = np.asarray(second, dtype=np.float32)
        if len(first) != sample_count or len(second) != sample_count:
            return 1e9
        direct = (
            np.linalg.norm(first[0] - second[0]) +
            np.linalg.norm(first[-1] - second[-1]))
        reverse = (
            np.linalg.norm(first[0] - second[-1]) +
            np.linalg.norm(first[-1] - second[0]))
        if reverse < direct:
            second = second[::-1]
        distance = np.linalg.norm(first - second, axis=1)
        return (
            float(np.mean(distance)) * 640.0 /
            float(max(1, image_shape[1])))

    @staticmethod
    def _identity_path_points(line):
        tail = line.get("branch_tail_points_xy")
        if tail is not None:
            tail = np.asarray(tail, dtype=np.float32)
            if tail.ndim == 2 and tail.shape[1] == 2 and len(tail) >= 2:
                return tail
        return np.asarray(line.get("points_xy"), dtype=np.float32)

    def _assign_heatmap_slots(self, lines, image_shape):
        if not lines:
            return []
        assigned = []
        identity_source = "row_scan"
        if len(lines) == 1:
            item = dict(lines[0])
            item_samples = _resample_path_by_count(
                item.get("points_xy"), 16)
            history_distances = {
                int(slot): self._sampled_shape_distance_640(
                    item_samples, _resample_path_by_count(previous, 16),
                    image_shape, 16)
                for slot, previous in self.last_slot_points.items()
            }
            history_gate = max(
                self.config.branch_separation_px_640 * 2.0,
                self.config.path_max_step_px_640 * 3.0)
            if (history_distances and
                    min(history_distances.values()) <= history_gate):
                item["slot"] = min(
                    history_distances, key=history_distances.get)
                identity_source = "temporal_match"
            else:
                scan_x = self._row_scan_path_position(
                    item.get("points_xy"), image_shape)
                center_x = (
                    float(image_shape[1]) * self.config.visual_center_x)
                item["slot"] = (
                    0 if scan_x is None or scan_x <= center_x else 1)
            assigned.append(item)
        else:
            first = dict(lines[0])
            second = dict(lines[1])
            first_identity = self._identity_path_points(first)
            second_identity = self._identity_path_points(second)
            first_x = self._row_scan_path_position(
                first_identity, image_shape)
            second_x = self._row_scan_path_position(
                second_identity, image_shape)
            if 0 in self.last_slot_points and 1 in self.last_slot_points:
                first_history = self.last_slot_branch_tails.get(
                    0, self.last_slot_points[0])
                second_history = self.last_slot_branch_tails.get(
                    1, self.last_slot_points[1])
                first_samples = _resample_path_by_count(first_identity, 16)
                second_samples = _resample_path_by_count(second_identity, 16)
                first_history_samples = _resample_path_by_count(
                    first_history, 16)
                second_history_samples = _resample_path_by_count(
                    second_history, 16)
                direct = (
                    self._sampled_shape_distance_640(
                        first_samples, first_history_samples,
                        image_shape, 16) +
                    self._sampled_shape_distance_640(
                        second_samples, second_history_samples,
                        image_shape, 16)
                )
                swapped = (
                    self._sampled_shape_distance_640(
                        first_samples, second_history_samples,
                        image_shape, 16) +
                    self._sampled_shape_distance_640(
                        second_samples, first_history_samples,
                        image_shape, 16)
                )
                history_gate = 2.0 * max(
                    self.config.branch_separation_px_640 * 2.0,
                    self.config.path_max_step_px_640 * 3.0)
                if min(direct, swapped) <= history_gate:
                    ordered = (
                        [first, second]
                        if direct <= swapped else [second, first])
                    identity_source = "temporal_match"
                elif first_x is not None and second_x is not None:
                    ordered = (
                        [first, second] if first_x <= second_x
                        else [second, first])
                else:
                    ordered = [first, second]
            elif first_x is not None and second_x is not None and abs(
                    first_x - second_x) >= max(
                        4.0, image_shape[1] * 0.008):
                ordered = (
                    [first, second] if first_x <= second_x
                    else [second, first])
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
            item["identity_source"] = identity_source
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
            labels, label, heatmap, image_shape, component_stats=None,
            history_points=None, skeleton_mask=None):
        paths = VisionControlPlanner._component_centerline_paths(
            labels, label, heatmap, image_shape,
            component_stats=component_stats,
            history_points=history_points,
            skeleton_mask=skeleton_mask,
            max_paths=1, min_branch_length=1)
        if not paths:
            return np.empty((0, 2), dtype=np.float32)
        return paths[0]

    @staticmethod
    def _component_centerline_paths(
            labels, label, heatmap, image_shape, component_stats=None,
            history_points=None, skeleton_mask=None,
            max_paths=2, min_branch_length=10):
        """Return root-to-leaf paths that share the same ego-side trunk."""
        labels = np.asarray(labels)
        heatmap = np.asarray(heatmap, dtype=np.float32)
        if (labels.ndim != 2 or heatmap.shape != labels.shape or
                labels.size == 0):
            return []

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

        component_mask = np.where(
            region == label, 255, 0).astype(np.uint8)
        if not np.any(component_mask):
            return []

        published_skeleton = np.asarray(skeleton_mask)
        if (published_skeleton.ndim == 2 and
                published_skeleton.shape == labels.shape):
            skeleton = published_skeleton[
                top:top + region.shape[0],
                left:left + region.shape[1]].astype(np.uint8)
            skeleton &= (region == label).astype(np.uint8)
            coordinate_left = left
            coordinate_top = top
        else:
            ximgproc = getattr(cv2, "ximgproc", None)
            thinning = getattr(ximgproc, "thinning", None)
            if not callable(thinning):
                fallback = VisionControlPlanner._weighted_component_centerline_points(
                    labels, label, heatmap, image_shape,
                    component_stats=component_stats)
                return [] if not len(fallback) else [fallback]
            padding = 1
            padded = cv2.copyMakeBorder(
                component_mask, padding, padding, padding, padding,
                cv2.BORDER_CONSTANT, value=0)
            skeleton = thinning(
                padded,
                thinningType=getattr(
                    ximgproc, "THINNING_ZHANGSUEN", 0))
            coordinate_left = left - padding
            coordinate_top = top - padding
        nonzero = cv2.findNonZero(skeleton)
        if nonzero is None:
            return []

        local_coordinates = nonzero.reshape((-1, 2)).astype(np.int32)
        coordinates = local_coordinates.astype(np.float32)
        coordinates[:, 0] += float(coordinate_left)
        coordinates[:, 1] += float(coordinate_top)
        if not len(coordinates):
            return []

        # Build an 8-neighbor graph with vectorized coordinate lookups. Short
        # skeleton spurs then remain side branches instead of redirecting a
        # greedy nearest-neighbor trace from frame to frame.
        index_map = np.full(skeleton.shape, -1, dtype=np.int32)
        index_map[
            local_coordinates[:, 1], local_coordinates[:, 0]
        ] = np.arange(len(coordinates), dtype=np.int32)
        neighbor_offsets = np.asarray([
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),            (1, 0),
            (-1, 1),  (0, 1),  (1, 1),
        ], dtype=np.int32)
        neighbor_x = (
            local_coordinates[:, 0, np.newaxis] +
            neighbor_offsets[np.newaxis, :, 0])
        neighbor_y = (
            local_coordinates[:, 1, np.newaxis] +
            neighbor_offsets[np.newaxis, :, 1])
        in_bounds = (
            (neighbor_x >= 0) & (neighbor_x < skeleton.shape[1]) &
            (neighbor_y >= 0) & (neighbor_y < skeleton.shape[0]))
        neighbors = np.full(neighbor_x.shape, -1, dtype=np.int32)
        neighbors[in_bounds] = index_map[
            neighbor_y[in_bounds], neighbor_x[in_bounds]]
        degree = np.count_nonzero(neighbors >= 0, axis=1)

        endpoints = np.flatnonzero(degree == 1)
        root_candidates = (
            endpoints if len(endpoints)
            else np.arange(len(coordinates), dtype=np.int32))
        center_x = float(max(labels.shape[1] - 1, 0)) * 0.5
        root_order = np.lexsort((
            np.abs(coordinates[root_candidates, 0] - center_x),
            -coordinates[root_candidates, 1],
        ))
        root = int(root_candidates[int(root_order[0])])

        distance = np.full(len(coordinates), -1, dtype=np.int32)
        parent = np.full(len(coordinates), -1, dtype=np.int32)
        queue = np.empty(len(coordinates), dtype=np.int32)
        queue[0] = root
        distance[root] = 0
        head = 0
        tail = 1
        while head < tail:
            current = int(queue[head])
            head += 1
            adjacent = neighbors[current]
            adjacent = adjacent[adjacent >= 0]
            unvisited = adjacent[distance[adjacent] < 0]
            if not len(unvisited):
                continue
            distance[unvisited] = distance[current] + 1
            parent[unvisited] = current
            queue[tail:tail + len(unvisited)] = unvisited
            tail += len(unvisited)

        targets = endpoints[
            (endpoints != root) & (distance[endpoints] >= 0)]
        if not len(targets):
            reachable = np.flatnonzero(distance > 0)
            if len(reachable):
                targets = np.asarray([
                    reachable[int(np.argmax(distance[reachable]))]
                ], dtype=np.int32)
        if not len(targets):
            return []
        img_h, img_w = image_shape[:2]
        ridge = cv2.distanceTransform(
            (component_mask != 0).astype(np.uint8),
            cv2.DIST_L2, 3)
        maximum_distance = max(1, int(np.max(distance[targets])))
        maximum_ridge = max(1e-6, float(np.max(ridge)))
        if isinstance(history_points, dict):
            raw_history_paths = list(history_points.values())
        elif isinstance(history_points, np.ndarray):
            raw_history_paths = [history_points]
        elif history_points is None:
            raw_history_paths = []
        else:
            raw_history_paths = list(history_points)
        history_paths = []
        for history_value in raw_history_paths:
            history = np.asarray(history_value, dtype=np.float32)
            if (history.ndim == 2 and history.shape[1] == 2 and
                    len(history) >= 2):
                history_paths.append(history)

        path_candidates = []
        for target_value in targets:
            current = int(target_value)
            reverse_path = []
            while current >= 0:
                reverse_path.append(current)
                if current == root:
                    break
                current = int(parent[current])
            if not reverse_path or reverse_path[-1] != root:
                continue
            path_indices = np.asarray(
                reverse_path[::-1], dtype=np.int32)
            heat_points = coordinates[path_indices]
            heat_x = np.clip(
                np.rint(heat_points[:, 0]).astype(np.int32),
                0, labels.shape[1] - 1)
            heat_y = np.clip(
                np.rint(heat_points[:, 1]).astype(np.int32),
                0, labels.shape[0] - 1)
            ridge_x = np.clip(heat_x - left, 0, ridge.shape[1] - 1)
            ridge_y = np.clip(heat_y - top, 0, ridge.shape[0] - 1)
            length_score = (
                float(distance[int(target_value)]) /
                float(maximum_distance))
            heat_score = float(np.mean(heatmap[heat_y, heat_x]))
            ridge_score = (
                float(np.mean(ridge[ridge_y, ridge_x])) /
                maximum_ridge)
            image_points = np.empty_like(
                heat_points, dtype=np.float32)
            image_points[:, 0] = (
                heat_points[:, 0] * float(max(img_w - 1, 1)) /
                float(max(labels.shape[1] - 1, 1)))
            image_points[:, 1] = (
                heat_points[:, 1] * float(max(img_h - 1, 1)) /
                float(max(labels.shape[0] - 1, 1)))
            history_score = 0.0
            if history_paths:
                history_distance = min(
                    VisionControlPlanner._path_distance_640(
                        image_points, history, image_shape)
                    for history in history_paths)
                history_score = max(
                    0.0, 1.0 - history_distance / 120.0)
            score = (
                0.50 * length_score +
                0.25 * heat_score +
                0.10 * ridge_score +
                0.15 * history_score)
            path_candidates.append({
                "score": float(score),
                "indices": path_indices,
                "points": image_points,
                "endpoint_x": float(heat_points[-1, 0]),
            })

        path_candidates.sort(
            key=lambda item: (
                item["score"], len(item["indices"]),
                -item["endpoint_x"]),
            reverse=True)
        selected = []
        minimum_tail = max(1, int(min_branch_length))
        for candidate in path_candidates:
            distinct = True
            for kept in selected:
                first_indices = candidate["indices"]
                second_indices = kept["indices"]
                common_limit = min(
                    len(first_indices), len(second_indices))
                differences = np.flatnonzero(
                    first_indices[:common_limit] !=
                    second_indices[:common_limit])
                common_length = (
                    int(differences[0]) if len(differences)
                    else common_limit)
                distinct_tail = min(
                    len(first_indices) - common_length,
                    len(second_indices) - common_length)
                if distinct_tail < minimum_tail:
                    distinct = False
                    break
            if not distinct:
                continue
            selected.append(candidate)
            if len(selected) >= max(1, int(max_paths)):
                break
        return [
            np.asarray(item["points"], dtype=np.float32)
            for item in selected
        ]

    @staticmethod
    def _weighted_component_centerline_points(
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
            return np.empty((0, 2), dtype=np.float32)
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
            smoothing_changed = (
                filtered_points.shape != points.shape or
                not np.allclose(
                    filtered_points, points, rtol=0.0, atol=0.05))
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
            branch_tail = filtered.get("branch_tail_points_xy")
            if branch_tail is not None:
                branch_tail = np.asarray(branch_tail, dtype=np.float32)
                if (branch_tail.ndim == 2 and branch_tail.shape[1] == 2 and
                        len(branch_tail) >= 2):
                    self.last_slot_branch_tails[slot] = branch_tail.copy()
            self.path_missing_frames[slot] = 0
            smoothed_candidates.append(filtered)

        for slot in list(self.last_slot_points):
            if slot in present_slots:
                continue
            missing = self.path_missing_frames.get(slot, 0) + 1
            self.path_missing_frames[slot] = missing
            if missing > self.config.path_state_hold_frames:
                self.last_slot_points.pop(slot, None)
                self.last_slot_branch_tails.pop(slot, None)
                self.path_missing_frames.pop(slot, None)
        return smoothed_candidates

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
        if previous.ndim != 2 or previous.shape[1] != 2:
            return points
        current_samples = _resample_path_by_count(
            points, _TEMPORAL_PATH_SAMPLES)
        previous_samples = _resample_path_by_count(
            previous, _TEMPORAL_PATH_SAMPLES)
        if (len(current_samples) != _TEMPORAL_PATH_SAMPLES or
                len(previous_samples) != _TEMPORAL_PATH_SAMPLES):
            return points
        direct_endpoint_cost = float(
            np.linalg.norm(current_samples[0] - previous_samples[0]) +
            np.linalg.norm(current_samples[-1] - previous_samples[-1]))
        reversed_endpoint_cost = float(
            np.linalg.norm(current_samples[0] - previous_samples[-1]) +
            np.linalg.norm(current_samples[-1] - previous_samples[0]))
        if reversed_endpoint_cost < direct_endpoint_cost:
            previous_samples = previous_samples[::-1].copy()
        max_step = (
            self.config.path_max_step_px_640 *
            float(max(1, image_width)) / 640.0)
        displacement = current_samples - previous_samples
        displacement_length = np.linalg.norm(displacement, axis=1)
        step_scale = np.minimum(
            1.0,
            max_step / np.maximum(displacement_length, 1e-6))
        bounded = (
            previous_samples +
            displacement * step_scale[:, np.newaxis])
        alpha = float(self.config.path_ema_alpha)
        blended = (
            bounded * alpha +
            previous_samples * (1.0 - alpha))
        # Preserve current topology endpoints; only the path interior is
        # temporally filtered. Support validation in _smooth_candidates keeps
        # every accepted point on the current low-threshold heat region.
        blended[0] = current_samples[0]
        blended[-1] = current_samples[-1]
        return blended.astype(np.float32)

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
        if route_state == ROUTE_MULTI_FORK:
            self.single_seen_frames = 0
            if self.fork_seen_since is None:
                self.fork_seen_since = now
            if (
                not ocr_current
                and self.branch_lock is None
            ):
                # Straight-through behavior is the physical left branch in
                # this course. Only a current, confirmed OCR "right" result
                # is allowed to override this default.
                self.branch_lock = "left"
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

    def _select_candidate(self, candidates, route_state, image_shape):
        if not candidates:
            if self.selected_slot_lock is not None:
                self.selected_slot_missing_frames += 1
                if (self.branch_lock_source != "ocr" and
                        self.selected_slot_missing_frames >
                        self.config.path_state_hold_frames):
                    self._clear_branch_lock()
            return None
        if self.branch_lock in {"left", "right"}:
            wanted_slot = 0 if self.branch_lock == "left" else 1
            if self.selected_slot_lock != wanted_slot:
                self.selected_branch_signature = None
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == wanted_slot:
                    return self._remember_selection(candidate)
            self.selected_slot_lock = wanted_slot
            self.selected_slot_missing_frames += 1
            return None
        if (route_state == ROUTE_MULTI_FORK and
                self.selected_branch_signature is not None):
            matched = []
            for candidate in candidates:
                identity_points = self._identity_path_points(candidate)
                samples = _resample_path_by_count(identity_points, 16)
                distance = self._sampled_shape_distance_640(
                    samples, self.selected_branch_signature,
                    image_shape, 16)
                matched.append((distance, candidate))
            if matched:
                distance, candidate = min(matched, key=lambda item: item[0])
                lock_gate = max(
                    self.config.branch_separation_px_640 * 1.5,
                    self.config.path_max_step_px_640 * 3.0)
                if distance <= lock_gate:
                    return self._remember_selection(candidate)
        if self.selected_slot_lock is not None:
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == self.selected_slot_lock:
                    return self._remember_selection(candidate)
            self.selected_slot_missing_frames += 1
            if (self.selected_slot_missing_frames <=
                    self.config.path_state_hold_frames):
                return None
            self.selected_slot_lock = None
            self.selected_slot_missing_frames = 0
            self.selected_branch_signature = None
        if self.last_selected_points is not None:
            previous_x = _interp_path_x(self.last_selected_points, image_shape[0] * self.config.lookahead_y_ratio)
            if previous_x is not None:
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
                    return self._remember_selection(candidate)
        center_x = image_shape[1] * self.config.visual_center_x
        return self._remember_selection(min(
            candidates,
            key=lambda item: abs((_interp_path_x(item["points_xy"], image_shape[0] * self.config.lookahead_y_ratio) or center_x) - center_x),
        ))

    def _remember_selection(self, candidate):
        self.selected_slot_lock = int(candidate.get("slot", -1))
        self.selected_slot_missing_frames = 0
        branch_tail = candidate.get("branch_tail_points_xy")
        if branch_tail is not None:
            signature = _resample_path_by_count(branch_tail, 16)
            if len(signature) == 16:
                previous = self.selected_branch_signature
                if previous is None or len(previous) != 16:
                    self.selected_branch_signature = signature.copy()
                else:
                    direct = (
                        np.linalg.norm(signature[0] - previous[0]) +
                        np.linalg.norm(signature[-1] - previous[-1]))
                    reverse = (
                        np.linalg.norm(signature[0] - previous[-1]) +
                        np.linalg.norm(signature[-1] - previous[0]))
                    if reverse < direct:
                        signature = signature[::-1].copy()
                    # Keep a slow branch-identity signature independent from
                    # the lighter geometry EMA. One noisy heatmap frame then
                    # cannot move the selected lookahead to the other tail.
                    self.selected_branch_signature = (
                        previous * 0.80 + signature * 0.20
                    ).astype(np.float32)
        return candidate

    def _build_command(self, selected, route_state, route_reason, result, image_shape, now, ocr_response=None):
        lookahead_y = image_shape[0] * self.config.lookahead_y_ratio
        task_state, speed, task_reason, target_override_x = self._task_from_detections(
            result, selected, image_shape, lookahead_y, route_state, now, ocr_response=ocr_response)
        if selected is None:
            age = now - self.last_valid_ts if self.last_valid_ts else 1e9
            if self.last_valid_ts and age <= self.config.recover_hold_s:
                error = self.last_error
                return self._command(error, self.config.recover_speed_mps, STATE_RECOVER_LINE), {
                    "target_x": None,
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
        if route_state == ROUTE_AMBIGUOUS:
            return self._command(0.0, 0.0, STATE_SAFE_STOP, flags=0), {
                "target_x": None,
                "track_error_640": 0.0,
                "reason": route_reason,
                "task_reason": "ambiguous_stop",
            }

        target_x = target_override_x
        if target_x is None:
            target_x = _interp_path_x(selected["points_xy"], lookahead_y)
        if target_x is None:
            return self._command(0.0, 0.0, STATE_SAFE_STOP, flags=0), {
                "target_x": None,
                "track_error_640": 0.0,
                "reason": "missing_target_x",
                "task_reason": task_reason,
            }
        raw_error = (float(target_x) / float(max(1, image_shape[1])) - self.config.visual_center_x) * 640.0
        error = self._limit_error(raw_error)
        self.last_error = error
        self.last_valid_ts = now
        self.last_selected_points = np.asarray(selected["points_xy"], dtype=np.float32).copy()
        self.last_slot_points[int(selected["slot"])] = self.last_selected_points
        return self._command(error, speed, task_state), {
            "target_x": float(target_x),
            "lookahead_y": float(lookahead_y),
            "track_error_640": float(error),
            "raw_track_error_640": float(raw_error),
            "reason": route_reason,
            "task_reason": task_reason,
        }

    def _task_from_detections(self, result, selected, image_shape, lookahead_y, route_state, now, ocr_response=None):
        detections = result.get("detections") or []
        if selected is None:
            return STATE_RECOVER_LINE, self.config.recover_speed_mps, "no_path", None
        sign_action = self._turnsign_action(detections, image_shape, lookahead_y, ocr_response, now)
        sign_mode = None
        sign_target_x = None
        if isinstance(sign_action, tuple):
            sign_mode, sign_target_x = sign_action
        else:
            sign_mode = sign_action
        if sign_mode == "stop":
            return STATE_SAFE_STOP, 0.0, "turnsign_stop", None
        if sign_mode == "pulse":
            return STATE_TRACK, self.config.sign_ocr_pulse_speed_mps, "turnsign_ocr_timeout_pulse", None
        target_path_x = _interp_path_x(selected["points_xy"], image_shape[0] * 0.68)
        if target_path_x is None:
            target_path_x = image_shape[1] * self.config.visual_center_x
        lookahead_path_x = _interp_path_x(selected["points_xy"], lookahead_y)
        if lookahead_path_x is None:
            lookahead_path_x = target_path_x
        human_action = self._human_action(
            detections, selected, image_shape, lookahead_y, lookahead_path_x)
        if human_action is not None:
            if int(human_action[0]) == STATE_AVOID_HUMAN:
                self.human_speed_hold_until = float(now) + self.config.human_speed_hold_s
            return human_action
        if now < self.human_speed_hold_until:
            return STATE_AVOID_HUMAN, self.config.human_pass_speed_mps, "human_speed_hold", None
        hazard_limit = image_shape[1] * self.config.hazard_lateral_ratio
        for det in detections:
            label = self._normalized_label(det)
            if label != "car":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_car_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None or geom["bottom_ratio"] < self.config.hazard_bottom_ratio:
                continue
            if abs(geom["cx"] - target_path_x) > max(hazard_limit, geom["box_w"] * 0.75):
                continue
            avoid_target_x = self._avoid_target_x(label, geom, target_path_x, image_shape)
            return STATE_AVOID_CAR, self.config.obstacle_speed_mps, "car_in_path_bias", avoid_target_x
        coin = self._best_coin(detections, image_shape, target_path_x)
        if coin is not None:
            return STATE_COLLECT_GOLD, self.config.collect_speed_mps, "coin_bias", self._coin_target_x(coin, target_path_x, image_shape)
        if sign_mode == "slow":
            return (
                STATE_TRACK,
                min(self.config.normal_speed_mps, self.config.turnsign_slow_speed_mps),
                "turnsign_slow",
                sign_target_x,
            )
        return STATE_TRACK, self.config.normal_speed_mps, "track", None

    def _turnsign_action(self, detections, image_shape, lookahead_y, ocr_response, now):
        has_active_ocr = bool((ocr_response or {}).get("active"))
        if self._ocr_has_current_direction(ocr_response):
            self._clear_sign_ocr_state()
            return None
        has_sign = False
        should_stop = False
        sign_target_x = None
        best_rank = -1.0
        for det in detections:
            if not self._is_turnsign_detection(det):
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.sign_slow_min_score:
                continue
            has_sign = True
            geom = self._detection_geom(det, image_shape)
            if geom is not None:
                rank = float(score) + float(geom.get("area_ratio") or 0.0) * 8.0
                if rank > best_rank:
                    best_rank = rank
                    sign_target_x = float(geom["cx"])
                if self._sign_should_stop(geom, image_shape, lookahead_y):
                    should_stop = True
        if has_active_ocr:
            if self.sign_ocr_active_since is None:
                self.sign_ocr_active_since = float(now)
                self.sign_ocr_pulse_until = 0.0
                self.sign_ocr_pulse_sent = False
            if not has_sign:
                return "stop", None
            return ("stop", None) if should_stop else ("slow", sign_target_x)
        self._clear_sign_ocr_state()
        if should_stop:
            return "stop", None
        return ("slow", sign_target_x) if has_sign or has_active_ocr else None

    def _clear_sign_ocr_state(self):
        self.sign_ocr_active_since = None
        self.sign_ocr_pulse_until = 0.0
        self.sign_ocr_pulse_sent = False

    def _human_action(self, detections, selected, image_shape, lookahead_y, lookahead_path_x):
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
            path_margin = max(
                float(image_shape[1]) * self.config.hazard_lateral_ratio,
                float(geom["box_w"]) * 0.75,
            )
            humans.append({
                "geom": geom,
                "score": score,
                "path_x": path_x,
                "distance": distance,
                "side": side,
                "on_path": abs(distance) <= path_margin,
            })
        if not humans:
            self._clear_human_state()
            return None

        human = max(
            humans,
            key=lambda item: (
                bool(item["on_path"]),
                float(item["geom"]["bottom_ratio"]),
                float(item["score"]),
            ),
        )
        geom = human["geom"]
        side = human["side"] or self.human_last_side or 1
        if human["side"] != 0:
            if self.human_last_side is None:
                self.human_last_side = human["side"]

        if self.human_pass_active:
            self.human_pass_side = side
            return self._human_pass_command(lookahead_path_x, image_shape, side)

        crossed = (
            self.human_waiting_cross
            and human["side"] != 0
            and self.human_last_side is not None
            and human["side"] != self.human_last_side
        )
        release_distance = (
            self.config.human_cross_release_px_640 *
            float(max(1, image_shape[1])) / 640.0)
        released_to_side = (
            self.human_waiting_cross
            and not human["on_path"]
            and abs(float(human["distance"])) >= release_distance
        )
        if crossed or released_to_side:
            self.human_pass_active = True
            self.human_pass_side = side
            return self._human_pass_command(lookahead_path_x, image_shape, side)

        if human["on_path"] and self._human_on_stop_line(geom, image_shape, lookahead_y):
            self.human_waiting_cross = True
            if human["side"] != 0:
                self.human_last_side = human["side"]
            return STATE_SAFE_STOP, 0.0, "human_half_lookahead_stop", None

        if not human["on_path"] and not self.human_waiting_cross:
            self._clear_human_state()
        return None

    def _human_pass_command(self, path_x, image_shape, human_side):
        scale = float(max(1, image_shape[1])) / 640.0
        offset = self.config.human_pass_offset_px_640 * scale
        target_x = _clamp(
            float(path_x) - float(human_side or 1) * offset,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        return STATE_AVOID_HUMAN, self.config.human_pass_speed_mps, "human_cross_pass", target_x

    def _human_on_stop_line(self, geom, image_shape, lookahead_y):
        stop_y = float(image_shape[0]) - (
            float(image_shape[0]) - float(lookahead_y)
        ) * self.config.human_stop_progress_ratio
        line_margin = float(image_shape[0]) * self.config.human_stop_line_margin_ratio
        return (
            float(geom["top"]) - line_margin <= stop_y <= float(geom["bottom"]) + line_margin
            or abs(float(geom["cy"]) - stop_y) <= line_margin
        )

    def _clear_human_state(self):
        self.human_waiting_cross = False
        self.human_last_side = None
        self.human_pass_active = False
        self.human_pass_side = None

    def _sign_should_stop(self, geom, image_shape, lookahead_y):
        height_ratio = float(geom["box_h"]) / float(max(1, image_shape[0]))
        area_ratio = geom.get("area_ratio")
        if area_ratio is None:
            area_ratio = (float(geom["box_w"]) * float(geom["box_h"])) / float(max(1, image_shape[0] * image_shape[1]))
        large_enough = (
            height_ratio >= self.config.sign_stop_height_ratio
            or float(area_ratio) >= self.config.sign_stop_area_ratio
        )
        if not large_enough:
            return False
        line_margin = float(image_shape[0]) * self.config.sign_stop_line_margin_ratio
        return (
            float(geom["top"]) - line_margin <= float(lookahead_y) <= float(geom["bottom"]) + line_margin
            or abs(float(geom["bottom"]) - float(lookahead_y)) <= line_margin
        )

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

    def _limit_error(self, error):
        error = _clamp(error, -self.config.max_track_error_640, self.config.max_track_error_640)
        delta = _clamp(error - self.last_error, -self.config.max_error_step_640, self.config.max_error_step_640)
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
        self._clear_branch_lock()
        return True

    def _clear_branch_lock(self):
        self.branch_lock = None
        self.branch_lock_source = None
        self.selected_slot_lock = None
        self.selected_branch_signature = None
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
    _draw_semantic_heat_distribution(frame, result)
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
    target_x = _finite_float(target.get("target_x"))
    lookahead_y = _finite_float(target.get("lookahead_y"), h * 0.62)
    if target_x is not None:
        x = int(round(_clamp(target_x, 0, w - 1)))
        y = int(round(_clamp(lookahead_y, 0, h - 1)))
        cv2.circle(frame, (x, y), 7, (255, 0, 255), -1, cv2.LINE_AA)
        cv2.line(frame, (0, y), (w - 1, y), (255, 0, 255), 1, cv2.LINE_AA)

    command = debug.get("command") or {}
    text = "{} raw={} lock={}/{} slot={} err={:.1f}".format(
        debug.get("route_state", "-"),
        debug.get("raw_route_state", "-"),
        debug.get("branch_lock") or "-",
        "-" if debug.get("selected_slot_lock") is None else debug.get("selected_slot_lock"),
        "-" if selected_slot is None else selected_slot,
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


def _draw_semantic_heat_distribution(frame, result):
    global _SOLID_RED_PREVIEW
    published_masks = result.get("semantic_heat_distribution_masks")
    use_processed_masks = published_masks is not None
    if use_processed_masks:
        masks = np.asarray(published_masks, dtype=np.uint8)
    else:
        centerline = result.get("centerline") or {}
        heatmaps = centerline.get("heatmaps")
        if heatmaps is None:
            heatmaps = result.get("path_heatmaps")
        if heatmaps is None:
            return
        heatmaps = np.asarray(heatmaps, dtype=np.float32)
        threshold = _clamp(_env_float(
            "VISION_CONTROL_SKELETON_THRESHOLD", 0.35), 0.0, 0.99)
        low_threshold = min(threshold, _clamp(_env_float(
            "VISION_CONTROL_SKELETON_LOW_THRESHOLD", 0.27),
            0.0, 0.99))
        masks = heatmaps
    if masks.ndim == 2:
        masks = masks[np.newaxis, ...]
    if masks.ndim != 3 or not len(masks):
        return

    height, width = frame.shape[:2]
    close_iterations = max(0, _env_int(
        "VISION_CONTROL_SKELETON_CLOSE_ITERATIONS", 1))
    edge_smooth_kernel = max(1, _env_int(
        "VISION_CONTROL_SKELETON_EDGE_KERNEL", 9))
    max_hole_area = max(0, _env_int(
        "VISION_CONTROL_SKELETON_MAX_HOLE_AREA", 32))
    connect_max_gap = max(0, _env_int(
        "VISION_CONTROL_SKELETON_MAX_CONNECT_GAP", 6))
    bridge_thickness = max(1, _env_int(
        "VISION_CONTROL_SKELETON_BRIDGE_THICKNESS", 3))
    minimum_area = max(1, _env_int(
        "VISION_CONTROL_SKELETON_MIN_AREA", 15))
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (3, 3))
    coverage = np.zeros((height, width), dtype=np.float32)
    for mask in masks[:2]:
        if use_processed_masks:
            mask = (mask != 0).astype(np.uint8)
        else:
            mask = _spatial_hysteresis_binary_mask(
                mask, threshold, low_threshold)
            heat_h, heat_w = mask.shape
            heatmap_scale = math.sqrt(
                float(heat_h * heat_w) / float(120 * 160))
            mask = _connect_binary_mask_samples(
                mask,
                max_gap=max(0, int(round(
                    connect_max_gap * heatmap_scale))),
                minimum_group_area=max(
                    1, int(round(minimum_area * heatmap_scale ** 2))),
                bridge_thickness=max(
                    1, int(round(bridge_thickness * heatmap_scale))))
            if close_iterations:
                mask = cv2.morphologyEx(
                    mask, cv2.MORPH_CLOSE, close_kernel,
                    iterations=close_iterations)
            mask = _smooth_binary_mask_edges(mask, edge_smooth_kernel)
            mask = _fill_binary_mask(
                mask, max_hole_area=max(
                    0, int(round(
                        max_hole_area * heatmap_scale ** 2))))
        resized = cv2.resize(
            mask.astype(np.float32), (width, height),
            interpolation=cv2.INTER_LINEAR)
        coverage = np.maximum(coverage, np.clip(resized, 0.0, 1.0))

    active = coverage > 0.0
    if not np.any(active):
        return
    # Keep the mask interior opaque red while using only the interpolated
    # boundary coverage as alpha, eliminating 4x nearest-neighbor stair steps.
    interior = coverage >= 1.0
    if np.any(interior):
        if (_SOLID_RED_PREVIEW is None or
                _SOLID_RED_PREVIEW.shape != frame.shape or
                _SOLID_RED_PREVIEW.dtype != frame.dtype):
            _SOLID_RED_PREVIEW = np.empty_like(frame)
            _SOLID_RED_PREVIEW[:] = (0, 0, 255)
        cv2.copyTo(
            _SOLID_RED_PREVIEW,
            interior.astype(np.uint8) * 255, frame)
    edge = active & ~interior
    if np.any(edge):
        alpha = coverage[edge, np.newaxis]
        red = np.asarray((0.0, 0.0, 255.0), dtype=np.float32)
        frame[edge] = np.rint(
            frame[edge].astype(np.float32) * (1.0 - alpha) + red * alpha
        ).astype(np.uint8)


def _identity_probability_color(slot, probability):
    intensity = int(round(80.0 + 175.0 * _clamp(probability, 0.0, 1.0)))
    # OpenCV uses BGR: left is always blue, right is always green.
    return (intensity, 0, 0) if int(slot) == 0 else (0, intensity, 0)


def _draw_identity_probability_path(
        frame, points, probabilities, slot, thickness=2):
    probabilities = np.asarray(probabilities, dtype=np.float32)
    if len(probabilities) != len(points):
        probabilities = np.ones(len(points), dtype=np.float32)
    if _env_flag("VISION_CONTROL_FAST_RENDER", True):
        probability = (
            float(np.mean(probabilities)) if len(probabilities) else 1.0)
        cv2.polylines(
            frame, [points], False,
            _identity_probability_color(slot, probability),
            int(thickness), cv2.LINE_AA)
        return
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
