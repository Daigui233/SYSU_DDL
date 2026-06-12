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
FORK_MISS_CONF = 0.58
ENABLE_GEOMETRY_ONLY_FORK = os.environ.get("ENABLE_GEOMETRY_ONLY_FORK", "0") == "1"
FORK_MIN_SPLIT_ROWS = 2
FORK_MIN_BRANCH_POINTS = 4
FORK_MIN_BRANCH_SEPARATION_RATIO = 0.10

FORK_CLASS_NAMES = ["normal", "fork", "miss"]

_prev_branch_count = 0


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


def _scan_rows(mask):
    h, _ = mask.shape
    top = int(h * TOP_CROP_RATIO)
    half = max(1, SCAN_ROW_WINDOW // 2)
    rows = []
    for y in range(h - 8, top, -SCAN_STEP):
        y0 = max(0, y - half)
        y1 = min(h, y + half + 1)
        row = np.max(mask[y0:y1], axis=0)
        segments = _row_segments(row, MIN_SEGMENT_WIDTH)
        if segments:
            rows.append({"y": int(y), "segments": segments})
    return rows


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


def _extract_fork_geometry(rows, center_x, lookahead_y, w):
    min_sep = max(28.0, w * FORK_MIN_BRANCH_SEPARATION_RATIO)
    split_rows = []
    for row in rows:
        segments = row["segments"]
        if len(segments) < 2:
            continue
        left = segments[0]
        right = segments[-1]
        if float(right["center"]) - float(left["center"]) >= min_sep:
            split_rows.append({
                "y": row["y"],
                "segments": [left, right],
                "separation": float(right["center"]) - float(left["center"]),
            })

    if len(split_rows) < FORK_MIN_SPLIT_ROWS:
        return {
            "valid": False,
            "reason": "few_split_rows",
            "split_rows": len(split_rows),
            "candidates": [],
        }

    left_rows = _make_side_rows(rows, "left")
    right_rows = _make_side_rows(rows, "right")
    left_path, left_score = _path_from_rows(left_rows, w, start_ref=center_x, side="left")
    right_path, right_score = _path_from_rows(right_rows, w, start_ref=center_x, side="right")
    if len(left_path) < FORK_MIN_BRANCH_POINTS or len(right_path) < FORK_MIN_BRANCH_POINTS:
        return {
            "valid": False,
            "reason": "short_branch",
            "split_rows": len(split_rows),
            "candidates": [],
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
        }

    candidates = [
        {
            "side": "left",
            "branch": "outer",
            "path": [(int(x), int(y)) for x, y in left_path],
            "error": float(left_err),
            "score": float(left_score),
        },
        {
            "side": "right",
            "branch": "inner",
            "path": [(int(x), int(y)) for x, y in right_path],
            "error": float(right_err),
            "score": float(right_score),
        },
    ]
    return {
        "valid": True,
        "reason": "ok",
        "split_rows": len(split_rows),
        "candidates": candidates,
    }


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


def _update_fork_state(fork_cls, geometry, road_valid):
    if not road_valid:
        return "MISS"

    geometry_valid = bool(geometry.get("valid"))
    cls_available = bool(fork_cls.get("available"))
    cls_label = fork_cls.get("label")
    cls_conf = float(fork_cls.get("confidence") or 0.0)

    if cls_available and cls_label == 2 and cls_conf >= FORK_MISS_CONF:
        return "MISS"

    if cls_available:
        if geometry_valid and cls_label == 1 and cls_conf >= FORK_CLASS_CONF:
            return "FORK_CONFIRMED"
        if cls_label == 1:
            return "FORK_CANDIDATE"
        return "NORMAL"

    if ENABLE_GEOMETRY_ONLY_FORK and geometry_valid:
        return "FORK_CONFIRMED"
    return "NORMAL"


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
        "fork_classifier": {
            "available": False,
            "label": None,
            "name": "none",
            "confidence": 0.0,
            "error": None,
        },
        "fork_geometry_valid": False,
        "fork_split_rows": 0,
        "fork_candidates": [],
        "fork_selected_side": None,
        "fork_reason": reason,
    }


def extract_centerline_info(seg_map, frame, fork_classifier=None):
    try:
        class_map = _to_class_map(seg_map)
    except ValueError as exc:
        out = frame.copy()
        cv2.putText(out, "SEG MAP ERROR", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        cv2.putText(out, str(exc), (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        return out, _empty_centerline_info(frame, reason=str(exc))

    raw_mask = _make_raw_mask(class_map)
    raw_ratio = float((class_map == ROAD_CLASS_ID).sum()) / float(class_map.size) if class_map.size else 0.0
    selected, selected_valid = _select_road(raw_mask)
    road_mask, road_valid, ratio, reason = _valid_mask(selected, selected_valid, raw_ratio)

    out = _overlay_road(frame.copy(), road_mask)
    h, w = road_mask.shape
    center_x = w // 2
    lookahead_y = int(np.clip(SEGMENTATION_LOOKAHEAD_Y, 0, h - 1))

    mid_points, mid_ok, rows, mid_score = _build_midline(road_mask) if road_valid else ([], False, [], 0.0)
    fork_raw = _run_fork_classifier(fork_classifier, road_mask) if road_valid else None
    fork_cls = _parse_fork_result(fork_raw)
    geometry = (
        _extract_fork_geometry(rows, center_x, lookahead_y, w)
        if road_valid and rows
        else {"valid": False, "reason": "no_midline", "split_rows": 0, "candidates": []}
    )
    fork_state = _update_fork_state(fork_cls, geometry, road_valid)
    fork_candidates = geometry.get("candidates") or []
    selected_points, selected_side = _choose_control_path(
        mid_points,
        fork_candidates,
    )

    selected_ok = len(selected_points) >= MIN_MID_POINTS
    draw_points = selected_points if selected_ok else []
    target_x = _draw_centerlines(out, road_mask, draw_points, fork_candidates, fork_state, selected_side)

    if DRAW_CAMERA_CENTER_REFERENCE:
        cv2.line(out, (center_x, h - 1), (center_x, int(h * 0.55)), (255, 255, 0), 1)

    track_error = None
    control_center_x = None
    raw_error = None
    if selected_ok and len(draw_points) >= 2:
        control_center_x = _point_x_at_y(draw_points, lookahead_y, w)
        if control_center_x is not None:
            raw_error = float(control_center_x - center_x)
            track_error = raw_error
            cv2.circle(out, (int(round(control_center_x)), lookahead_y), 6, (0, 255, 0), -1)

    road_state = "OK" if road_valid else "LOST"
    mid_state = "OK" if selected_ok else "LOST"
    state_color = (0, 255, 0) if road_valid else (0, 165, 255)
    fork_color = (0, 255, 255)
    if fork_state == "FORK_CONFIRMED":
        fork_color = (255, 0, 255)
    elif fork_state == "MISS":
        fork_color = (0, 165, 255)

    cv2.putText(out, f"Road seg: {road_state} {ratio:.1%} {reason}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
    cv2.putText(out, f"Midline: {mid_state}", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
    err_text = f"Track err: {track_error:.1f}" if track_error is not None else "Track err: N/A"
    cv2.putText(out, err_text, (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(
        out,
        f"Fork: {fork_state} cls={fork_cls['name']} {fork_cls['confidence']:.2f} split={geometry.get('split_rows', 0)}",
        (10, 144),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        fork_color,
        2,
    )
    cv2.putText(out, f"Branches: {_prev_branch_count}", (10, 172), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

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
        "fork_classifier": fork_cls,
        "fork_geometry_valid": bool(geometry.get("valid")),
        "fork_split_rows": int(geometry.get("split_rows", 0)),
        "fork_candidates": [
            {
                "side": str(c.get("side")),
                "branch": str(c.get("branch") or ""),
                "error": float(c.get("error", 0.0)),
                "score": float(c.get("score", 0.0)),
                "points": [(int(x), int(y)) for x, y in (c.get("path") or [])],
            }
            for c in fork_candidates
        ],
        "fork_selected_side": selected_side if selected_side != "main" else None,
        "fork_reason": str(geometry.get("reason") or reason),
    }
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
