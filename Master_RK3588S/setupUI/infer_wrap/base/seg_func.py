import os
import cv2
import numpy as np


IMG_SIZE = (256, 256)
FORK_CLS_SIZE = (128, 128)
MAX_CLASS_CHANNELS = 16
ROAD_CLASS_ID = int(os.environ.get("ROAD_CLASS_ID", "1"))
ROAD_ALPHA = 0.32
ROAD_COLOR = np.array([0, 255, 80], dtype=np.uint8)

# Camera center is only the image reference used to compute track_error.
DRAW_CAMERA_CENTER_REFERENCE = False

TOP_CROP_RATIO = 0.34
MIN_ROAD_RATIO = 0.001
MAX_ROAD_RATIO = 0.60
MAX_RAW_ROAD_RATIO = 0.86
MIN_COMPONENT_AREA_RATIO = 0.0015
MAX_COMPONENTS = 3
MIN_SEGMENT_WIDTH = 12
SCAN_STEP = 8
SCAN_ROW_WINDOW = 5
MAX_CENTER_JUMP_RATIO = 0.22
MIN_MID_POINTS = 4
SEGMENTATION_LOOKAHEAD_Y = 300

FORK_CLASS_CONF = 0.48
FORK_EARLY_CLASS_CONF = 0.32
FORK_MISS_CONF = 0.58
ENABLE_GEOMETRY_ONLY_FORK = os.environ.get("ENABLE_GEOMETRY_ONLY_FORK", "0") == "1"
FORK_MIN_SPLIT_ROWS = 2
FORK_MIN_BRANCH_POINTS = 4
FORK_MIN_BRANCH_SEPARATION_RATIO = 0.10
FORK_MIN_BRANCH_PIXELS = 160
FORK_MIN_TOTAL_PIXELS = 420
FORK_EARLY_MIN_SPLIT_ROWS = 1
FORK_EARLY_MIN_BRANCH_POINTS = 3
FORK_EARLY_MIN_BRANCH_SEPARATION_RATIO = 0.045
FORK_EARLY_MIN_BRANCH_PIXELS = 72
FORK_EARLY_MIN_TOTAL_PIXELS = 180
FORK_EARLY_CONFIRM_FRAMES = 3
FORK_CONFIRM_FRAMES = 2
FORK_RELEASE_FRAMES = 5

# Connected-mask fork partitioning. Normal tracking keeps SCAN_STEP=8; a
# classifier-confirmed fork uses denser rows only while branch geometry is needed.
FORK_DENSE_SCAN_STEP = 3
FORK_DENSE_SCAN_ROW_WINDOW = 3
FORK_PARTITION_MIN_DIVIDER_POINTS = 3
FORK_PARTITION_POLY_DEGREE = 2
FORK_PARTITION_MIN_SAMPLE_SPAN_PX = 18
FORK_PARTITION_MAX_DIVIDER_RMSE_RATIO = 0.035
FORK_PARTITION_DEFAULT_EXTENSION_PX = 12
FORK_PARTITION_MAX_EXTENSION_PX = 48
FORK_PARTITION_DRAW_STEP = 8

CHANNEL_EXPANSION_RATIO = 1.32
CHANNEL_EXPANSION_MIN_ROWS = 3
CHANNEL_PATH_DEVIATION_RATIO = 0.035
CHANNEL_PATH_DEVIATION_MIN_ROWS = 3
CHANNEL_HOLD_FRAMES = 7
CHANNEL_HOLD_CORRECTION_RATIO = 0.018
CHANNEL_NORMAL_BLEND = 0.72
CHANNEL_RECOVERY_BLEND = 0.45
CHANNEL_PREDICTION_GAIN = 0.55
CHANNEL_RESET_LOST_FRAMES = 8

FORK_CLASS_NAMES = ["normal", "fork", "miss"]

# Fork-strength ramp: late trigger, fast climb.
# Strength 0.0 = OFF (NORMAL, no postproc). 1.0 = LOCK (strict confirmed).
FORK_STRENGTH_ACTIVATE_EARLY_COUNT = 2   # need ≥ this many early votes to activate
FORK_STRENGTH_HOLD_MIN = 0.6             # hold only when strength ≥ this
FORK_STRENGTH_EARLY_TABLE = {            # early_count → strength (only ≥ ACTIVATE_EARLY_COUNT)
    2: 0.60,
    3: 0.85,
    4: 0.95,
}

_prev_branch_count = 0
_prev_path = []
_prev_prev_path = []
_prev_row_widths = {}
_channel_hold = 0
_fork_votes = []
_fork_hold = 0
_prev_fork_candidates = {}
_lost_frames = 0


def reset_centerline_state():
    """Reset temporal path state for tests and pipeline restarts."""
    global _prev_branch_count, _prev_path, _prev_prev_path, _prev_row_widths
    global _channel_hold, _fork_votes, _fork_hold, _prev_fork_candidates, _lost_frames
    _prev_branch_count = 0
    _prev_path = []
    _prev_prev_path = []
    _prev_row_widths = {}
    _channel_hold = 0
    _fork_votes = []
    _fork_hold = 0
    _prev_fork_candidates = {}
    _lost_frames = 0


def _update_lost_state(road_valid):
    global _lost_frames, _prev_path, _prev_prev_path, _prev_row_widths
    global _channel_hold, _prev_fork_candidates
    if road_valid:
        _lost_frames = 0
        return
    _lost_frames += 1
    if _lost_frames < CHANNEL_RESET_LOST_FRAMES:
        return
    _prev_path = []
    _prev_prev_path = []
    _prev_row_widths = {}
    _channel_hold = 0
    _prev_fork_candidates = {}


def _to_class_map(seg_map):
    seg = np.asarray(seg_map)
    seg = np.squeeze(seg)

    if seg.ndim == 2:
        return seg.astype(np.uint8)

    if seg.ndim == 3:
        # NCHW after squeeze: C,H,W. NHWC: H,W,C.
        if seg.shape[0] <= MAX_CLASS_CHANNELS and seg.shape[0] < seg.shape[-1]:
            if seg.shape[0] >= 2:
                seg = seg.copy()
                road_ch = ROAD_CLASS_ID
                seg[road_ch] = seg[road_ch] - 0.10
            return np.argmax(seg, axis=0).astype(np.uint8)
        if seg.shape[-1] <= MAX_CLASS_CHANNELS:
            if seg.shape[-1] >= 2:
                seg = seg.copy()
                road_ch = ROAD_CLASS_ID
                seg[..., road_ch] = seg[..., road_ch] - 0.10
            return np.argmax(seg, axis=-1).astype(np.uint8)

    raise ValueError(f"unexpected segmentation shape: {seg.shape}")


def _extract_class_map(outputs, original_size):
    out = outputs[0] if isinstance(outputs, (list, tuple)) else outputs
    class_map = _to_class_map(out)
    h, w = original_size
    if class_map.shape[:2] != (h, w):
        class_map = cv2.resize(class_map, (w, h), interpolation=cv2.INTER_NEAREST)
    return class_map.astype(np.uint8)


def _make_raw_mask(class_map):
    mask = (class_map == ROAD_CLASS_ID).astype(np.uint8)
    h, _ = mask.shape
    mask[: int(h * TOP_CROP_RATIO), :] = 0

    kernel3 = np.ones((3, 3), np.uint8)
    kernel5 = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3, iterations=1)
    return mask


def _component_score(labels, stats, label, h, w):
    x, y, width, height, area = stats[label]
    bottom_y = int(h * 0.72)
    bottom_roi = labels[bottom_y:, x:x + width] == label
    bottom_contact = float(bottom_roi.sum()) / float(max(1, width * max(1, h - bottom_y)))

    xs = np.flatnonzero(bottom_roi.sum(axis=0))
    if xs.size:
        bottom_center = x + (xs[0] + xs[-1]) * 0.5
        center_score = 1.0 - min(1.0, abs(bottom_center - w * 0.5) / max(1.0, w * 0.5))
    else:
        center_score = 0.0

    area_score = float(area) / float(h * w)
    height_score = float(height) / float(max(1, h))
    return area_score * 3.0 + bottom_contact * 1.8 + center_score * 0.6 + height_score * 0.35


def _select_road(mask):
    global _prev_branch_count

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        _prev_branch_count = 0
        return np.zeros_like(mask), False

    h, w = mask.shape
    min_area = max(60, int(h * w * MIN_COMPONENT_AREA_RATIO))
    candidates = []
    for label in range(1, num_labels):
        x, y, width, height, area = stats[label]
        if area < min_area or width < 4 or height < 4:
            continue
        candidates.append((_component_score(labels, stats, label, h, w), label))

    if not candidates:
        _prev_branch_count = 0
        return np.zeros_like(mask), False

    candidates.sort(reverse=True)
    _prev_branch_count = min(len(candidates), MAX_COMPONENTS)
    selected = np.zeros_like(mask)
    for _, label in candidates[:MAX_COMPONENTS]:
        selected[labels == label] = 1

    kernel3 = np.ones((3, 3), np.uint8)
    selected = cv2.morphologyEx(selected, cv2.MORPH_CLOSE, kernel3, iterations=1)
    return selected, True


def _quality(mask, raw_ratio):
    ratio = float(mask.sum()) / float(mask.size) if mask.size else 0.0
    if raw_ratio > MAX_RAW_ROAD_RATIO:
        return False, ratio, "raw-full"
    if ratio < MIN_ROAD_RATIO:
        return False, ratio, "small"
    if ratio > MAX_ROAD_RATIO:
        return False, ratio, "large"
    return True, ratio, "ok"


def _valid_mask(mask, valid, raw_ratio):
    ok = False
    ratio = 0.0
    reason = "lost"
    if valid:
        ok, ratio, reason = _quality(mask, raw_ratio)

    if ok:
        return mask, True, ratio, reason
    return np.zeros_like(mask), False, ratio, reason


def _overlay_road(frame, mask):
    if not np.any(mask):
        return frame
    out = frame.copy()
    road = mask.astype(bool)
    out[road] = (
        out[road].astype(np.float32) * (1.0 - ROAD_ALPHA)
        + ROAD_COLOR.astype(np.float32) * ROAD_ALPHA
    ).astype(np.uint8)
    return out


def _row_segments(row, min_width=MIN_SEGMENT_WIDTH):
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


def _scan_rows_with_params(mask, step, row_window):
    h, _ = mask.shape
    top = int(h * TOP_CROP_RATIO)
    step = max(1, int(step))
    half = max(0, int(row_window) // 2)
    rows = []
    for y in range(h - 8, top, -step):
        y0 = max(0, y - half)
        y1 = min(h, y + half + 1)
        row = np.max(mask[y0:y1], axis=0)
        segments = _row_segments(row, MIN_SEGMENT_WIDTH)
        if segments:
            rows.append({"y": int(y), "segments": segments})
    return rows


def _scan_rows(mask):
    return _scan_rows_with_params(mask, SCAN_STEP, SCAN_ROW_WINDOW)


def _bottom_reference_x(mask, w):
    h = mask.shape[0]
    for y in range(h - 1, max(0, int(h * 0.78)), -1):
        segments = _row_segments(mask[y], MIN_SEGMENT_WIDTH)
        if segments:
            seg = max(segments, key=lambda item: item["width"])
            return float(seg["center"])
    return float(w * 0.5)


def _path_from_rows(rows, w, start_ref=None, side=None):
    if not rows:
        return [], 0.0

    start_ref = float(w * 0.5 if start_ref is None else start_ref)
    max_jump = max(36.0, w * MAX_CENTER_JUMP_RATIO)
    scores = []
    parents = []

    for i, row in enumerate(rows):
        row_scores = []
        row_parents = []
        for cand in row["segments"]:
            cx = float(cand["center"])
            width = float(cand["width"])
            base_score = min(width, w * 0.45) * 0.055
            base_score -= abs(cx - start_ref) * (0.018 if i < 3 else 0.003)
            if side == "left":
                base_score -= max(0.0, cx - w * 0.52) * 0.012
            elif side == "right":
                base_score -= max(0.0, w * 0.48 - cx) * 0.012

            if i == 0:
                row_scores.append(base_score)
                row_parents.append(None)
                continue

            best_score = base_score - i * 0.35
            best_parent = None
            for k, prev in enumerate(rows[i - 1]["segments"]):
                prev_score = scores[i - 1][k]
                px = float(prev["center"])
                dx = abs(cx - px)
                dy_steps = max(1.0, abs(float(row["y"] - rows[i - 1]["y"])) / float(SCAN_STEP))
                jump_limit = max_jump * dy_steps
                jump_penalty = (dx / max(1.0, jump_limit)) ** 2 * 7.5
                if dx > jump_limit * 1.55:
                    jump_penalty += 12.0
                width_change = abs(width - float(prev["width"])) / max(1.0, max(width, float(prev["width"])))
                candidate_score = prev_score + base_score - jump_penalty - width_change * 0.85
                if candidate_score > best_score or best_parent is None:
                    best_score = candidate_score
                    best_parent = k

            row_scores.append(best_score)
            row_parents.append(best_parent)

        scores.append(row_scores)
        parents.append(row_parents)

    best_i = None
    best_j = None
    best_score = -1e18
    for i, row_scores in enumerate(scores):
        for j, score in enumerate(row_scores):
            length_bonus = i * 2.2
            total = score + length_bonus
            if total > best_score:
                best_score = total
                best_i = i
                best_j = j

    if best_i is None:
        return [], 0.0

    points = []
    i = best_i
    j = best_j
    while i is not None and j is not None and i >= 0:
        cand = rows[i]["segments"][j]
        points.append((int(round(cand["center"])), int(rows[i]["y"])))
        j = parents[i][j]
        i -= 1
    points.reverse()
    points = sorted(points, key=lambda p: p[1], reverse=True)
    return points, float(best_score)


def _smooth_path(points, w):
    if len(points) < 4:
        return points
    ordered = sorted(points, key=lambda p: p[1])
    xs = np.array([p[0] for p in ordered], dtype=np.float32)
    ys = np.array([p[1] for p in ordered], dtype=np.float32)

    smooth = xs.copy()
    for i in range(len(xs)):
        lo = max(0, i - 2)
        hi = min(len(xs), i + 3)
        weights = np.ones(hi - lo, dtype=np.float32)
        weights[np.arange(lo, hi) == i] = 2.2
        smooth[i] = float(np.sum(xs[lo:hi] * weights) / np.sum(weights))

    if len(points) >= 7:
        try:
            coeff = np.polyfit(ys, smooth, 2)
            fit = np.polyval(coeff, ys)
            smooth = 0.72 * smooth + 0.28 * fit
        except Exception:
            pass

    result = [
        (int(np.clip(round(x), 0, w - 1)), int(y))
        for x, y in zip(smooth, ys)
    ]
    return sorted(result, key=lambda p: p[1], reverse=True)


def _build_midline(mask):
    h, w = mask.shape
    rows = _scan_rows(mask)
    if not rows:
        return [], False, rows, 0.0

    start_ref = _bottom_reference_x(mask, w)
    points, score = _path_from_rows(rows, w, start_ref=start_ref)
    if len(points) < MIN_MID_POINTS:
        return [], False, rows, score
    return _smooth_path(points, w), True, rows, score


def _make_side_rows(rows, side):
    side_index = 0 if side == "left" else -1
    forced = []
    for row in rows:
        segments = row.get("segments") or []
        if not segments:
            continue
        seg = segments[side_index] if len(segments) >= 2 else segments[0]
        forced.append({"y": row["y"], "segments": [seg]})
    return forced


def _point_x_at_y(points, target_y, w):
    if not points:
        return None
    ordered = sorted(points, key=lambda p: p[1])
    ys = np.array([p[1] for p in ordered], dtype=np.float32)
    xs = np.array([p[0] for p in ordered], dtype=np.float32)
    if len(points) == 1:
        return float(np.clip(xs[0], 0, w - 1))
    return float(np.clip(np.interp(float(target_y), ys, xs), 0, w - 1))


def _path_error(points, center_x, lookahead_y, w):
    x = _point_x_at_y(points, lookahead_y, w)
    if x is None:
        return None
    return float(x - center_x)


def _extract_fork_geometry(
    rows,
    center_x,
    lookahead_y,
    w,
    min_split_rows=FORK_MIN_SPLIT_ROWS,
    min_branch_points=FORK_MIN_BRANCH_POINTS,
    min_separation_ratio=FORK_MIN_BRANCH_SEPARATION_RATIO,
    min_branch_pixels=FORK_MIN_BRANCH_PIXELS,
    min_total_pixels=FORK_MIN_TOTAL_PIXELS,
    row_step=SCAN_STEP,
):
    min_sep = max(16.0, w * float(min_separation_ratio))
    split_rows = []
    left_pixels = 0
    right_pixels = 0
    area_scale = max(1, int(row_step))
    for row in rows:
        segments = row["segments"]
        if len(segments) < 2:
            continue
        left = segments[0]
        right = segments[-1]
        if float(right["center"]) - float(left["center"]) >= min_sep:
            left_width = int(left.get("width", 0))
            right_width = int(right.get("width", 0))
            left_pixels += max(0, left_width) * area_scale
            right_pixels += max(0, right_width) * area_scale
            split_rows.append({
                "y": row["y"],
                "segments": [left, right],
                "separation": float(right["center"]) - float(left["center"]),
                "left_pixels": max(0, left_width) * area_scale,
                "right_pixels": max(0, right_width) * area_scale,
            })

    branch_pixel_threshold = max(
        int(min_branch_pixels),
        int(w * area_scale * max(1, int(min_split_rows)) * 0.012),
    )
    total_pixel_threshold = max(
        int(min_total_pixels),
        branch_pixel_threshold * 2,
    )
    total_pixels = int(left_pixels + right_pixels)
    pixel_stats = {
        "left_pixels": int(left_pixels),
        "right_pixels": int(right_pixels),
        "total_pixels": int(total_pixels),
        "branch_pixel_threshold": int(branch_pixel_threshold),
        "total_pixel_threshold": int(total_pixel_threshold),
    }

    if len(split_rows) < int(min_split_rows):
        return {
            "valid": False,
            "reason": "few_split_rows",
            "split_rows": len(split_rows),
            "candidates": [],
            **pixel_stats,
        }

    if (
        left_pixels < branch_pixel_threshold
        or right_pixels < branch_pixel_threshold
        or total_pixels < total_pixel_threshold
    ):
        return {
            "valid": False,
            "reason": "few_fork_pixels",
            "split_rows": len(split_rows),
            "candidates": [],
            **pixel_stats,
        }

    left_rows = _make_side_rows(rows, "left")
    right_rows = _make_side_rows(rows, "right")
    left_path, left_score = _path_from_rows(left_rows, w, start_ref=center_x, side="left")
    right_path, right_score = _path_from_rows(right_rows, w, start_ref=center_x, side="right")
    if len(left_path) < int(min_branch_points) or len(right_path) < int(min_branch_points):
        return {
            "valid": False,
            "reason": "short_branch",
            "split_rows": len(split_rows),
            "candidates": [],
            **pixel_stats,
        }

    left_path = _smooth_path(left_path, w)
    right_path = _smooth_path(right_path, w)
    left_err = _path_error(left_path, center_x, lookahead_y, w)
    right_err = _path_error(right_path, center_x, lookahead_y, w)
    if left_err is None or right_err is None:
        return {
            "valid": False,
            "reason": "no_lookahead",
            "split_rows": len(split_rows),
            "candidates": [],
            **pixel_stats,
        }

    candidates = [
        {
            "side": "left",
            "branch": "outer",
            "path": [(int(x), int(y)) for x, y in left_path],
            "error": float(left_err),
            "score": float(left_score),
            "pixels": int(left_pixels),
        },
        {
            "side": "right",
            "branch": "inner",
            "path": [(int(x), int(y)) for x, y in right_path],
            "error": float(right_err),
            "score": float(right_score),
            "pixels": int(right_pixels),
        },
    ]
    return {
        "valid": True,
        "reason": "ok",
        "split_rows": len(split_rows),
        "candidates": candidates,
        **pixel_stats,
    }


def _fork_divider_samples(rows, w, min_separation_ratio):
    min_sep = max(16.0, float(w) * float(min_separation_ratio))
    samples = []
    for row in rows:
        segments = row.get("segments") or []
        if len(segments) < 2:
            continue
        left = segments[0]
        right = segments[-1]
        center_sep = float(right["center"]) - float(left["center"])
        if center_sep < min_sep:
            continue
        inner_left = float(left["right"])
        inner_right = float(right["left"])
        samples.append({
            "y": int(row["y"]),
            "x": (inner_left + inner_right) * 0.5,
            "gap": max(1.0, inner_right - inner_left - 1.0),
        })
    return samples


def _fit_fork_divider(samples, rows, h, w):
    if len(samples) < FORK_PARTITION_MIN_DIVIDER_POINTS:
        return None

    ys = np.asarray([item["y"] for item in samples], dtype=np.float32)
    xs = np.asarray([item["x"] for item in samples], dtype=np.float32)
    if float(np.max(ys) - np.min(ys)) < float(FORK_PARTITION_MIN_SAMPLE_SPAN_PX):
        return None
    all_ys = [int(row.get("y", 0)) for row in rows]
    if not all_ys:
        return None
    split_min_y = int(round(float(np.min(ys))))
    split_max_y = int(round(float(np.max(ys))))
    common_above_span = max(0, split_min_y - min(all_ys))
    common_below_span = max(0, max(all_ys) - split_max_y)
    if max(common_above_span, common_below_span) < max(1, int(FORK_DENSE_SCAN_STEP)):
        return None
    split_side = "upper" if common_below_span >= common_above_span else "lower"

    degree = min(max(1, int(FORK_PARTITION_POLY_DEGREE)), len(samples) - 1)
    try:
        coeff = np.polyfit(ys, xs, degree)
    except Exception:
        return None
    fit_xs = np.polyval(coeff, ys)
    fit_rmse = float(np.sqrt(np.mean((fit_xs - xs) ** 2)))
    if fit_rmse > max(4.0, float(w) * float(FORK_PARTITION_MAX_DIVIDER_RMSE_RATIO)):
        return None

    boundary_split_y = split_max_y if split_side == "upper" else split_min_y
    direction = 1 if split_side == "upper" else -1
    fork_y = boundary_split_y + direction * int(FORK_PARTITION_DEFAULT_EXTENSION_PX)
    if len(samples) >= 2:
        gaps = np.asarray([item["gap"] for item in samples], dtype=np.float32)
        try:
            gap_coeff = np.polyfit(ys, gaps, 1)
            slope = float(gap_coeff[0])
            slope_matches = slope < -1e-3 if split_side == "upper" else slope > 1e-3
            if slope_matches:
                zero_y = -float(gap_coeff[1]) / slope
                low_y = boundary_split_y if split_side == "upper" else boundary_split_y - float(FORK_PARTITION_MAX_EXTENSION_PX)
                high_y = boundary_split_y + float(FORK_PARTITION_MAX_EXTENSION_PX) if split_side == "upper" else boundary_split_y
                if low_y <= zero_y <= high_y:
                    fork_y = int(round(zero_y))
        except Exception:
            pass

    if split_side == "upper":
        fork_y = int(np.clip(
            fork_y,
            boundary_split_y,
            min(h - 1, boundary_split_y + int(FORK_PARTITION_MAX_EXTENSION_PX)),
        ))
    else:
        fork_y = int(np.clip(
            fork_y,
            max(0, boundary_split_y - int(FORK_PARTITION_MAX_EXTENSION_PX)),
            boundary_split_y,
        ))

    def divider_x(y):
        return float(np.clip(np.polyval(coeff, float(y)), 0, w - 1))

    top_y = max(0, int(h * TOP_CROP_RATIO))
    draw_start = top_y if split_side == "upper" else fork_y
    draw_stop = fork_y if split_side == "upper" else h - 1
    divider_points = [
        (int(round(divider_x(y))), int(y))
        for y in range(draw_start, draw_stop + 1, max(1, int(FORK_PARTITION_DRAW_STEP)))
    ]
    end_y = fork_y if split_side == "upper" else h - 1
    if not divider_points or divider_points[-1][1] != end_y:
        divider_points.append((int(round(divider_x(end_y))), end_y))

    return {
        "coeff": coeff,
        "fork_y": fork_y,
        "fork_x": int(round(divider_x(fork_y))),
        "split_side": split_side,
        "fit_rmse": fit_rmse,
        "divider_points": divider_points,
    }


def _partition_connected_fork_mask(mask, divider):
    left_mask = np.asarray(mask, dtype=np.uint8).copy()
    right_mask = np.asarray(mask, dtype=np.uint8).copy()
    h, w = left_mask.shape
    fork_y = int(np.clip(divider["fork_y"], 0, h - 1))
    coeff = divider["coeff"]
    split_side = str(divider.get("split_side") or "upper")

    split_rows = range(0, fork_y + 1) if split_side == "upper" else range(fork_y, h)
    for y in split_rows:
        split_x = int(np.clip(round(float(np.polyval(coeff, float(y)))), 0, w - 1))
        left_mask[y, split_x + 1:] = 0
        right_mask[y, :split_x + 1] = 0
    return left_mask, right_mask


def _partition_fork_geometry(
    mask,
    rows,
    geometry,
    center_x,
    lookahead_y,
    w,
    min_branch_points,
    min_separation_ratio,
):
    result = dict(geometry or {})
    result.setdefault("partition_valid", False)
    result.setdefault("partition_method", "row_segments")
    result.setdefault("partition_side", None)
    result.setdefault("partition_fit_rmse", None)
    result.setdefault("fork_point", None)
    result.setdefault("divider_points", [])
    if not result.get("valid"):
        return result

    samples = _fork_divider_samples(rows, w, min_separation_ratio)
    divider = _fit_fork_divider(samples, rows, mask.shape[0], w)
    if divider is None:
        return result

    left_mask, right_mask = _partition_connected_fork_mask(mask, divider)
    left_path, left_ok, _left_rows, left_score = _build_midline(left_mask)
    right_path, right_ok, _right_rows, right_score = _build_midline(right_mask)
    if not left_ok or not right_ok:
        return result
    if len(left_path) < int(min_branch_points) or len(right_path) < int(min_branch_points):
        return result

    left_err = _path_error(left_path, center_x, lookahead_y, w)
    right_err = _path_error(right_path, center_x, lookahead_y, w)
    if left_err is None or right_err is None:
        return result

    result.update({
        "partition_valid": True,
        "partition_method": "fitted_divider",
        "partition_side": str(divider["split_side"]),
        "partition_fit_rmse": float(divider["fit_rmse"]),
        "fork_point": (int(divider["fork_x"]), int(divider["fork_y"])),
        "divider_points": [(int(x), int(y)) for x, y in divider["divider_points"]],
        "candidates": [
            {
                "side": "left",
                "branch": "outer",
                "path": [(int(x), int(y)) for x, y in left_path],
                "error": float(left_err),
                "score": float(left_score),
                "pixels": int(np.count_nonzero(left_mask)),
            },
            {
                "side": "right",
                "branch": "inner",
                "path": [(int(x), int(y)) for x, y in right_path],
                "error": float(right_err),
                "score": float(right_score),
                "pixels": int(np.count_nonzero(right_mask)),
            },
        ],
    })
    return result


def _softmax(logits):
    arr = np.asarray(logits, dtype=np.float32).reshape(-1)
    if arr.size == 0:
        return arr
    arr = arr - np.max(arr)
    exp = np.exp(arr)
    return exp / max(1e-6, float(np.sum(exp)))


def _parse_fork_result(result):
    if result is None:
        return {
            "available": False,
            "label": None,
            "name": "none",
            "confidence": 0.0,
            "probabilities": None,
            "error": None,
        }

    if isinstance(result, dict):
        if "logits" in result:
            probs = _softmax(result.get("logits"))
            label = int(np.argmax(probs)) if probs.size else int(result.get("label", 0))
            conf = float(probs[label]) if probs.size else float(result.get("confidence", 0.0))
        else:
            label = int(result.get("label", 0))
            conf = float(result.get("confidence", 0.0))
            probs = result.get("probabilities")
            probs = None if probs is None else np.asarray(probs, dtype=np.float32)
        label = max(0, min(len(FORK_CLASS_NAMES) - 1, label))
        return {
            "available": True,
            "label": label,
            "name": FORK_CLASS_NAMES[label],
            "confidence": conf,
            "probabilities": None if probs is None else [float(v) for v in probs.reshape(-1)],
            "error": result.get("error"),
        }

    probs = _softmax(result)
    label = int(np.argmax(probs)) if probs.size else 0
    conf = float(probs[label]) if probs.size else 0.0
    label = max(0, min(len(FORK_CLASS_NAMES) - 1, label))
    return {
        "available": True,
        "label": label,
        "name": FORK_CLASS_NAMES[label],
        "confidence": conf,
        "probabilities": [float(v) for v in probs.reshape(-1)],
        "error": None,
    }


def make_fork_classifier_input(road_mask):
    mask = (np.asarray(road_mask) > 0).astype(np.uint8) * 255
    mask = cv2.resize(mask, FORK_CLS_SIZE, interpolation=cv2.INTER_NEAREST)
    return np.repeat(mask[:, :, None], 3, axis=2)


def _run_fork_classifier(fork_classifier, road_mask):
    if fork_classifier is None:
        return None
    try:
        if hasattr(fork_classifier, "infer_mask"):
            return fork_classifier.infer_mask(road_mask)
        return fork_classifier(road_mask)
    except Exception as exc:
        return {"label": 0, "confidence": 0.0, "error": str(exc)}


def _scaled_dense_row_count(coarse_count):
    coarse_count = max(1, int(coarse_count))
    if coarse_count <= 1:
        return 1
    span = float(coarse_count - 1) * float(SCAN_STEP)
    return max(2, int(np.ceil(span / float(max(1, FORK_DENSE_SCAN_STEP)))) + 1)


def _update_fork_state(fork_cls, geometry, early_geometry, road_valid):
    global _fork_votes, _fork_hold
    if not road_valid:
        _fork_votes = []
        _fork_hold = 0
        return "MISS"

    geometry_valid = bool(geometry.get("valid"))
    cls_available = bool(fork_cls.get("available"))
    cls_label = fork_cls.get("label")
    cls_conf = float(fork_cls.get("confidence") or 0.0)

    if cls_available and cls_label == 2 and cls_conf >= FORK_MISS_CONF:
        _fork_votes = []
        _fork_hold = 0
        return "MISS"

    if cls_available:
        early_vote = bool(early_geometry.get("valid")) and cls_label == 1 and cls_conf >= FORK_EARLY_CLASS_CONF
        strict_vote = geometry_valid and cls_label == 1 and cls_conf >= FORK_CLASS_CONF
        _fork_votes.append((early_vote, strict_vote))
        _fork_votes = _fork_votes[-max(FORK_EARLY_CONFIRM_FRAMES, FORK_CONFIRM_FRAMES, 4):]
        early_confirmed = sum(1 for early, _strict in _fork_votes[-FORK_EARLY_CONFIRM_FRAMES:] if early) >= FORK_EARLY_CONFIRM_FRAMES
        strict_confirmed = sum(1 for _early, strict in _fork_votes[-FORK_CONFIRM_FRAMES:] if strict) >= FORK_CONFIRM_FRAMES
        if strict_confirmed:
            _fork_hold = FORK_RELEASE_FRAMES
            return "FORK_CONFIRMED"
        if strict_vote or early_confirmed:
            _fork_hold = max(_fork_hold, 1)
            return "FORK_CANDIDATE"
        if early_vote or (cls_label == 1 and cls_conf >= FORK_EARLY_CLASS_CONF):
            return "FORK_EARLY"
        if _fork_hold > 0 and bool(early_geometry.get("valid")):
            _fork_hold -= 1
            return "FORK_CANDIDATE"
        _fork_hold = max(0, _fork_hold - 1)
        return "NORMAL"

    if ENABLE_GEOMETRY_ONLY_FORK and geometry_valid:
        return "FORK_CONFIRMED"
    return "NORMAL"


def _compute_fork_strength():
    """Late-trigger, fast-climb fork confidence strength 0.0–1.0.

    0.0  = no postproc (NORMAL road, raw DP output).
    0.60 = first activation (≥2 early votes).
    0.85 = strong (≥3 early votes).
    0.95 = near-lock (≥4 early votes).
    1.0  = lock (≥2 strict votes).
    """
    if not _fork_votes:
        return 0.0
    window = _fork_votes[-4:]
    strict_count = sum(1 for _, s in window if s)
    early_count = sum(1 for e, _ in window if e)

    if strict_count >= 2:
        return 1.0

    if early_count < FORK_STRENGTH_ACTIVATE_EARLY_COUNT:
        return 0.0

    return FORK_STRENGTH_EARLY_TABLE.get(early_count, 0.95)


def _postproc_state_name(fork_strength, channel_hold):
    if channel_hold > 0:
        return "HOLD"
    if fork_strength <= 0.0:
        return "OFF"
    if fork_strength >= 1.0:
        return "LOCK"
    if fork_strength >= 0.7:
        return "ACTIVE"
    return "WARM"


def _row_for_y(rows, y):
    if not rows:
        return None
    return min(rows, key=lambda item: abs(int(item.get("y", 0)) - int(y)))


def _path_row_widths(rows, points):
    widths = {}
    for x, y in points:
        row = _row_for_y(rows, y)
        if row is None:
            continue
        segments = row.get("segments") or []
        if not segments:
            continue
        seg = min(
            segments,
            key=lambda item: 0.0
            if float(item["left"]) <= float(x) <= float(item["right"])
            else abs(float(item["center"]) - float(x)),
        )
        widths[int(y)] = float(seg.get("width", 0.0))
    return widths


def _predict_path_x(y, w):
    previous = _point_x_at_y(_prev_path, y, w)
    if previous is None:
        return None
    older = _point_x_at_y(_prev_prev_path, y, w)
    if older is None:
        return previous
    return float(np.clip(previous + (previous - older) * CHANNEL_PREDICTION_GAIN, 0, w - 1))


def _stabilize_single_path(points, rows, w, positive_y):
    """Fork-aware midline temporal stabilization.  Only called when fork_strength > 0.

    Blend and hold are both driven by fork_strength so protection ramps up smoothly
    as fork evidence accumulates, instead of switching at a single binary threshold.
    """
    global _prev_path, _prev_prev_path, _prev_row_widths, _channel_hold
    # Caller guarantees points and _prev_path are non-empty when fork_strength > 0.
    if not points:
        return []
    if not _prev_path:
        _prev_path = list(points)
        _prev_row_widths = _path_row_widths(rows, points)
        return list(points)

    current_widths = _path_row_widths(rows, points)
    expanded_rows = 0
    deviated_rows = 0
    for _x, y in points:
        current = current_widths.get(int(y))
        previous = _prev_row_widths.get(int(y))
        if current is not None and previous is not None and previous > 1.0:
            expanded_rows += int(current > previous * CHANNEL_EXPANSION_RATIO)
        predicted = _predict_path_x(y, w)
        if predicted is not None:
            deviated_rows += int(abs(float(_x) - predicted) > w * CHANNEL_PATH_DEVIATION_RATIO)

    hold_trigger = (
        expanded_rows >= CHANNEL_EXPANSION_MIN_ROWS
        or deviated_rows >= CHANNEL_PATH_DEVIATION_MIN_ROWS
    )

    # Hold only activates when fork confidence is high enough (late trigger).
    if hold_trigger and not positive_y:
        _channel_hold = CHANNEL_HOLD_FRAMES
    elif _channel_hold > 0:
        _channel_hold -= 1

    # Blend: interpolated by fork_strength (no more binary on/off).
    # No-hold: 1.0 (raw DP) → 0.72   Hold: 0.72 → 0.18
    if _channel_hold > 0 and not positive_y:
        blend = 0.18
    elif not positive_y:
        blend = CHANNEL_RECOVERY_BLEND if expanded_rows else CHANNEL_NORMAL_BLEND
    else:
        blend = CHANNEL_NORMAL_BLEND

    max_correction = max(5.0, w * CHANNEL_HOLD_CORRECTION_RATIO)
    stabilized = []
    for raw_x, y in points:
        predicted = _predict_path_x(y, w)
        if predicted is None:
            stabilized.append((int(raw_x), int(y)))
            continue
        raw_x_f = float(raw_x)
        if _channel_hold > 0 and not positive_y:
            raw_x_f = float(np.clip(raw_x_f, predicted - max_correction, predicted + max_correction))
        x = predicted * (1.0 - blend) + raw_x_f * blend
        stabilized.append((int(np.clip(round(x), 0, w - 1)), int(y)))

    stabilized = _smooth_path(stabilized, w)
    _prev_prev_path = list(_prev_path)
    _prev_path = list(stabilized)
    if not positive_y and expanded_rows == 0:
        _prev_row_widths = current_widths
    return stabilized


def _stabilize_fork_candidates(candidates, w):
    global _prev_fork_candidates
    result = []
    next_paths = {}
    for candidate in candidates:
        item = dict(candidate)
        side = str(item.get("side") or "")
        path = list(item.get("path") or [])
        previous = _prev_fork_candidates.get(side) or []
        if previous and path:
            blended = []
            for x, y in path:
                px = _point_x_at_y(previous, y, w)
                bx = float(x) if px is None else float(x) * 0.68 + float(px) * 0.32
                blended.append((int(np.clip(round(bx), 0, w - 1)), int(y)))
            path = _smooth_path(blended, w)
        item["path"] = path
        next_paths[side] = list(path)
        result.append(item)
    _prev_fork_candidates = next_paths
    return result


def _choose_control_path(main_points, fork_candidates):
    if len(fork_candidates) < 2:
        return main_points, "main"

    for candidate in fork_candidates:
        if candidate.get("side") == "left":
            return candidate.get("path") or main_points, "left"
    candidate = fork_candidates[0]
    return candidate.get("path") or main_points, str(candidate.get("side") or "fork")


def _draw_path(frame, points, color, thickness=4):
    if not points:
        return
    pts = np.array(points, np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], False, color, thickness, cv2.LINE_AA)


def _draw_centerlines(frame, mask, selected_points, fork_candidates, fork_state, selected_side):
    if len(fork_candidates) >= 2:
        for candidate in fork_candidates:
            _draw_path(frame, candidate.get("path") or [], (96, 96, 96), 3)

    _draw_path(frame, selected_points, (0, 0, 255), 5)
    if not selected_points:
        return None

    h, _ = mask.shape
    lookahead_y = int(np.clip(SEGMENTATION_LOOKAHEAD_Y, 0, h - 1))
    tx = _point_x_at_y(selected_points, lookahead_y, mask.shape[1])
    if tx is None:
        return None
    cv2.circle(frame, (int(round(tx)), lookahead_y), 8, (0, 255, 255), -1)
    if len(fork_candidates) >= 2:
        cv2.putText(
            frame,
            f"selected: {selected_side}",
            (int(round(tx)) + 10, max(22, lookahead_y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
    return tx


def _empty_centerline_info(frame, reason="lost"):
    h, w = frame.shape[:2]
    return {
        "line_valid": False,
        "track_error": None,
        "road_valid": False,
        "road_held": False,
        "road_state": "LOST",
        "midline_valid": False,
        "midline_state": "LOST",
        "road_ratio": 0.0,
        "reason": reason,
        "branch_count": 0,
        "center_x": w // 2,
        "target_x": None,
        "segmentation_lookahead_y": int(np.clip(SEGMENTATION_LOOKAHEAD_Y, 0, h - 1)),
        "road_mask": None,
        "mid_points": [],
        "fork_state": "MISS",
        "fork_mode": "single",
        "channel_state": "TRACK",
        "channel_hold": 0,
        "postproc_state": "OFF",
        "fork_strength": 0.0,
        "fork_classifier": {
            "available": False,
            "label": None,
            "name": "none",
            "confidence": 0.0,
            "error": None,
        },
        "fork_geometry_valid": False,
        "fork_strict_geometry_valid": False,
        "fork_split_rows": 0,
        "fork_left_pixels": 0,
        "fork_right_pixels": 0,
        "fork_total_pixels": 0,
        "fork_branch_pixel_threshold": FORK_MIN_BRANCH_PIXELS,
        "fork_total_pixel_threshold": FORK_MIN_TOTAL_PIXELS,
        "fork_partition_valid": False,
        "fork_partition_method": "none",
        "fork_partition_side": None,
        "fork_partition_fit_rmse": None,
        "fork_point": None,
        "fork_divider_points": [],
        "fork_scan_step": int(SCAN_STEP),
        "fork_candidates": [],
        "fork_selected_side": None,
        "fork_reason": reason,
    }


def draw_centerline_debug(frame, info, draw_text=True):
    info = dict(info or {})
    road_mask = info.get("road_mask")
    if road_mask is None:
        road_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    else:
        road_mask = np.asarray(road_mask, dtype=np.uint8)

    out = _overlay_road(frame.copy(), road_mask)
    h, w = out.shape[:2]
    center_x = int(info.get("center_x") or (w // 2))
    lookahead_y = int(np.clip(info.get("segmentation_lookahead_y", SEGMENTATION_LOOKAHEAD_Y), 0, h - 1))
    selected_points = [(int(x), int(y)) for x, y in (info.get("mid_points") or [])]
    fork_candidates = []
    for candidate in info.get("fork_candidates") or []:
        item = dict(candidate)
        item["path"] = [(int(x), int(y)) for x, y in (item.get("path") or item.get("points") or [])]
        fork_candidates.append(item)

    _draw_centerlines(
        out,
        road_mask,
        selected_points,
        fork_candidates,
        info.get("fork_state") or "MISS",
        info.get("fork_selected_side") or "main",
    )

    if info.get("fork_partition_valid"):
        divider_points = [(int(x), int(y)) for x, y in (info.get("fork_divider_points") or [])]
        _draw_path(out, divider_points, (0, 255, 255), 2)
        fork_point = info.get("fork_point")
        if fork_point is not None and len(fork_point) >= 2:
            cv2.circle(out, (int(fork_point[0]), int(fork_point[1])), 7, (0, 165, 255), -1)

    if DRAW_CAMERA_CENTER_REFERENCE:
        cv2.line(out, (center_x, h - 1), (center_x, int(h * 0.55)), (255, 255, 0), 1)

    control_center_x = info.get("control_center_x")
    if control_center_x is not None:
        cv2.circle(out, (int(round(float(control_center_x))), lookahead_y), 6, (0, 255, 0), -1)

    if not draw_text:
        return out

    road_valid = bool(info.get("road_valid"))
    road_state = str(info.get("road_state") or ("OK" if road_valid else "LOST"))
    mid_state = str(info.get("midline_state") or ("OK" if info.get("midline_valid") else "LOST"))
    ratio = float(info.get("road_ratio") or 0.0)
    reason = str(info.get("reason") or "")
    state_color = (0, 255, 0) if road_valid else (0, 165, 255)
    fork_state = str(info.get("fork_state") or "MISS")
    fork_color = (0, 255, 255)
    if fork_state == "FORK_CONFIRMED":
        fork_color = (255, 0, 255)
    elif fork_state == "MISS":
        fork_color = (0, 165, 255)

    fork_cls = info.get("fork_classifier") or {}
    fork_name = str(fork_cls.get("name") or "none")
    try:
        fork_conf = float(fork_cls.get("confidence") or 0.0)
    except Exception:
        fork_conf = 0.0
    split_rows = int(info.get("fork_split_rows") or 0)
    partition_side = str(info.get("fork_partition_side") or "-")
    left_pixels = int(info.get("fork_left_pixels") or 0)
    right_pixels = int(info.get("fork_right_pixels") or 0)
    total_pixels = int(info.get("fork_total_pixels") or 0)
    branch_pixel_threshold = int(info.get("fork_branch_pixel_threshold") or 0)
    total_pixel_threshold = int(info.get("fork_total_pixel_threshold") or 0)
    branch_count = int(info.get("branch_count") or 0)
    track_error = info.get("track_error")
    err_text = f"Track err: {float(track_error):.1f}" if track_error is not None else "Track err: N/A"

    cv2.putText(out, f"Road seg: {road_state} {ratio:.1%} {reason}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
    cv2.putText(out, f"Midline: {mid_state}", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
    postproc_state = str(info.get("postproc_state") or "OFF")
    fork_strength_info = float(info.get("fork_strength") or 0.0)
    channel_hold_info = int(info.get("channel_hold") or 0)
    pp_color = (0, 255, 0)
    if postproc_state == "LOCK":
        pp_color = (255, 0, 255)
    elif postproc_state in ("HOLD", "ACTIVE"):
        pp_color = (0, 255, 255)
    elif postproc_state == "WARM":
        pp_color = (0, 200, 200)

    cv2.putText(out, err_text, (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(
        out,
        f"Fork: {fork_state} cls={fork_name} {fork_conf:.2f} split={split_rows} cut={partition_side}",
        (10, 144),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        fork_color,
        2,
    )
    cv2.putText(
        out,
        f"PostProc: {postproc_state} str={fork_strength_info:.2f} hold={channel_hold_info}",
        (10, 172),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        pp_color,
        2,
    )
    cv2.putText(
        out,
        f"Fork pix L/R/T: {left_pixels}/{right_pixels}/{total_pixels}>{branch_pixel_threshold}/{total_pixel_threshold}",
        (10, 200),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )
    cv2.putText(out, f"Branches: {branch_count}", (10, 228), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    return out


def extract_centerline_info(seg_map, frame, fork_classifier=None, draw_debug=True):
    global _prev_path, _prev_prev_path, _prev_row_widths, _channel_hold
    try:
        class_map = _to_class_map(seg_map)
    except ValueError as exc:
        out = frame
        if draw_debug:
            out = frame.copy()
            cv2.putText(out, "SEG MAP ERROR", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            cv2.putText(out, str(exc), (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        return out, _empty_centerline_info(frame, reason=str(exc))

    raw_mask = _make_raw_mask(class_map)
    raw_ratio = float((class_map == ROAD_CLASS_ID).sum()) / float(class_map.size) if class_map.size else 0.0
    selected, selected_valid = _select_road(raw_mask)
    road_mask, road_valid, ratio, reason = _valid_mask(selected, selected_valid, raw_ratio)
    _update_lost_state(road_valid)

    h, w = road_mask.shape
    center_x = w // 2
    lookahead_y = int(np.clip(SEGMENTATION_LOOKAHEAD_Y, 0, h - 1))

    mid_points, mid_ok, rows, mid_score = _build_midline(road_mask) if road_valid else ([], False, [], 0.0)
    fork_raw = _run_fork_classifier(fork_classifier, road_mask) if road_valid else None
    fork_cls = _parse_fork_result(fork_raw)
    fork_scan_requested = bool(
        road_valid
        and rows
        and (
            (
                fork_cls.get("available")
                and fork_cls.get("label") == 1
                and float(fork_cls.get("confidence") or 0.0) >= FORK_EARLY_CLASS_CONF
            )
            or ENABLE_GEOMETRY_ONLY_FORK
        )
    )
    geometry_rows = (
        _scan_rows_with_params(road_mask, FORK_DENSE_SCAN_STEP, FORK_DENSE_SCAN_ROW_WINDOW)
        if fork_scan_requested
        else rows
    )
    geometry_step = FORK_DENSE_SCAN_STEP if fork_scan_requested else SCAN_STEP
    strict_split_rows = _scaled_dense_row_count(FORK_MIN_SPLIT_ROWS) if fork_scan_requested else FORK_MIN_SPLIT_ROWS
    strict_branch_points = (
        _scaled_dense_row_count(FORK_MIN_BRANCH_POINTS) if fork_scan_requested else FORK_MIN_BRANCH_POINTS
    )
    early_split_rows = (
        _scaled_dense_row_count(FORK_EARLY_MIN_SPLIT_ROWS) if fork_scan_requested else FORK_EARLY_MIN_SPLIT_ROWS
    )
    early_branch_points = (
        _scaled_dense_row_count(FORK_EARLY_MIN_BRANCH_POINTS) if fork_scan_requested else FORK_EARLY_MIN_BRANCH_POINTS
    )

    if road_valid and geometry_rows:
        geometry = _extract_fork_geometry(
            geometry_rows,
            center_x,
            lookahead_y,
            w,
            min_split_rows=strict_split_rows,
            min_branch_points=strict_branch_points,
            row_step=geometry_step,
        )
        geometry = _partition_fork_geometry(
            road_mask,
            geometry_rows,
            geometry,
            center_x,
            lookahead_y,
            w,
            strict_branch_points,
            FORK_MIN_BRANCH_SEPARATION_RATIO,
        )
        early_geometry = _extract_fork_geometry(
            geometry_rows,
            center_x,
            lookahead_y,
            w,
            min_split_rows=early_split_rows,
            min_branch_points=early_branch_points,
            min_separation_ratio=FORK_EARLY_MIN_BRANCH_SEPARATION_RATIO,
            min_branch_pixels=FORK_EARLY_MIN_BRANCH_PIXELS,
            min_total_pixels=FORK_EARLY_MIN_TOTAL_PIXELS,
            row_step=geometry_step,
        )
        early_geometry = _partition_fork_geometry(
            road_mask,
            geometry_rows,
            early_geometry,
            center_x,
            lookahead_y,
            w,
            early_branch_points,
            FORK_EARLY_MIN_BRANCH_SEPARATION_RATIO,
        )
    else:
        geometry = {"valid": False, "reason": "no_midline", "split_rows": 0, "candidates": []}
        early_geometry = {"valid": False, "reason": "no_midline", "split_rows": 0, "candidates": []}

    geometry["scan_step"] = int(geometry_step)
    early_geometry["scan_step"] = int(geometry_step)
    fork_state = _update_fork_state(fork_cls, geometry, early_geometry, road_valid)
    fork_strength = _compute_fork_strength()
    candidate_source = geometry if geometry.get("valid") else early_geometry

    positive_y = fork_state in ("FORK_EARLY", "FORK_CANDIDATE", "FORK_CONFIRMED")
    if mid_ok:
        mid_points = _stabilize_single_path(mid_points, rows, w, positive_y)
        fork_candidates = _stabilize_fork_candidates(candidate_source.get("candidates") or [], w)
    else:
        _channel_hold = 0
        fork_candidates = []

    postproc_state = _postproc_state_name(fork_strength, _channel_hold)
    selected_points, selected_side = _choose_control_path(
        mid_points,
        fork_candidates if fork_state == "FORK_CONFIRMED" else [],
    )

    selected_ok = len(selected_points) >= MIN_MID_POINTS
    draw_points = selected_points if selected_ok else []
    target_x = _point_x_at_y(draw_points, lookahead_y, w) if selected_ok else None
    track_error = None
    control_center_x = None
    raw_error = None
    if selected_ok and len(draw_points) >= 2:
        control_center_x = target_x
        if control_center_x is not None:
            raw_error = float(control_center_x - center_x)
            track_error = raw_error

    road_state = "OK" if road_valid else "LOST"
    mid_state = "OK" if selected_ok else "LOST"

    info = {
        "line_valid": track_error is not None,
        "track_error": float(track_error) if track_error is not None else None,
        "road_valid": bool(road_valid),
        "road_held": False,
        "road_state": road_state,
        "midline_valid": bool(selected_ok),
        "midline_state": mid_state,
        "road_ratio": float(ratio),
        "reason": reason,
        "branch_count": int(_prev_branch_count),
        "center_x": int(center_x),
        "target_x": int(round(control_center_x)) if control_center_x is not None else (int(round(target_x)) if target_x is not None else None),
        "control_center_x": int(round(control_center_x)) if control_center_x is not None else None,
        "segmentation_lookahead_y": int(lookahead_y),
        "raw_track_error": float(raw_error) if raw_error is not None else None,
        "road_mask": road_mask,
        "mid_points": [(int(x), int(y)) for x, y in draw_points],
        "midline_score": float(mid_score),
        "fork_state": fork_state,
        "fork_mode": "dual" if len(fork_candidates) >= 2 else "single",
        "channel_state": "HOLD" if _channel_hold > 0 else "TRACK",
        "channel_hold": int(_channel_hold),
        "postproc_state": postproc_state,
        "fork_strength": float(fork_strength),
        "fork_classifier": fork_cls,
        "fork_geometry_valid": bool(candidate_source.get("valid")),
        "fork_strict_geometry_valid": bool(geometry.get("valid")),
        "fork_split_rows": int(candidate_source.get("split_rows", 0)),
        "fork_left_pixels": int(candidate_source.get("left_pixels", 0)),
        "fork_right_pixels": int(candidate_source.get("right_pixels", 0)),
        "fork_total_pixels": int(candidate_source.get("total_pixels", 0)),
        "fork_branch_pixel_threshold": int(candidate_source.get("branch_pixel_threshold", FORK_MIN_BRANCH_PIXELS)),
        "fork_total_pixel_threshold": int(candidate_source.get("total_pixel_threshold", FORK_MIN_TOTAL_PIXELS)),
        "fork_partition_valid": bool(candidate_source.get("partition_valid")),
        "fork_partition_method": str(candidate_source.get("partition_method") or "row_segments"),
        "fork_partition_side": candidate_source.get("partition_side"),
        "fork_partition_fit_rmse": (
            float(candidate_source["partition_fit_rmse"])
            if candidate_source.get("partition_fit_rmse") is not None
            else None
        ),
        "fork_point": (
            [int(candidate_source["fork_point"][0]), int(candidate_source["fork_point"][1])]
            if candidate_source.get("fork_point") is not None
            else None
        ),
        "fork_divider_points": [
            (int(x), int(y)) for x, y in (candidate_source.get("divider_points") or [])
        ],
        "fork_scan_step": int(candidate_source.get("scan_step") or geometry_step),
        "fork_candidates": [
            {
                "side": str(c.get("side")),
                "branch": str(c.get("branch") or ""),
                "error": float(c.get("error", 0.0)),
                "score": float(c.get("score", 0.0)),
                "pixels": int(c.get("pixels", 0)),
                "points": [(int(x), int(y)) for x, y in (c.get("path") or [])],
            }
            for c in fork_candidates
        ],
        "fork_selected_side": selected_side if selected_side != "main" else None,
        "fork_reason": str(candidate_source.get("reason") or reason),
    }
    out = draw_centerline_debug(frame, info) if draw_debug else frame
    return out, info


def extract_centerline(seg_map, frame):
    out, info = extract_centerline_info(seg_map, frame)
    return out, info.get("track_error")


def myFunc(rknn_lite, img_bgr):
    original_size = img_bgr.shape[:2]
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_in = cv2.resize(img_rgb, IMG_SIZE, interpolation=cv2.INTER_LINEAR)
    outputs = rknn_lite.inference(inputs=[np.expand_dims(img_in, axis=0)])
    return _extract_class_map(outputs, original_size)
