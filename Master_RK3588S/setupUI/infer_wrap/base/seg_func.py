import os
import cv2
import numpy as np

IMG_SIZE = (256, 256)
MAX_CLASS_CHANNELS = 16
ROAD_CLASS_ID = int(os.environ.get("ROAD_CLASS_ID", "1"))
ROAD_ALPHA = 0.32
ROAD_COLOR = np.array([0, 255, 80], dtype=np.uint8)

# Camera center is only the image reference used to compute track_error.
# It is hidden by default because the real driving target is the red/purple path.
DRAW_CAMERA_CENTER_REFERENCE = False

TOP_CROP_RATIO = 0.34
MIN_ROAD_RATIO = 0.001
MAX_ROAD_RATIO = 0.60
MAX_RAW_ROAD_RATIO = 0.86
MIN_COMPONENT_AREA_RATIO = 0.0015
MAX_COMPONENTS = 2
MIN_SEGMENT_WIDTH = 12
SCAN_STEP = 8
MAX_CENTER_JUMP_RATIO = 0.20
MIN_MID_POINTS = 4
SEGMENTATION_LOOKAHEAD_Y = 300

# Pixel error is measured at one fixed lookahead row; no weighted error is applied.
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
    return area_score * 3.0 + bottom_contact * 1.5 + center_score * 0.6


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
    out[road] = (out[road].astype(np.float32) * (1.0 - ROAD_ALPHA) + ROAD_COLOR.astype(np.float32) * ROAD_ALPHA).astype(np.uint8)
    return out


def _best_row_center(row, ref_x, prev_x, max_jump):
    xs = np.flatnonzero(row)
    if xs.size < MIN_SEGMENT_WIDTH:
        return None
    splits = np.where(np.diff(xs) > 1)[0] + 1
    segments = np.split(xs, splits)

    best = None
    best_score = -1e9
    for seg in segments:
        width = int(seg[-1] - seg[0] + 1)
        if width < MIN_SEGMENT_WIDTH:
            continue
        cx = int((int(seg[0]) + int(seg[-1])) * 0.5)
        if prev_x is not None and abs(cx - prev_x) > max_jump:
            continue
        score = width * 2.0 - abs(cx - ref_x) * 2.2
        if prev_x is not None:
            score -= abs(cx - prev_x) * 1.2
        if score > best_score:
            best_score = score
            best = cx
    return best


def _fit_midline(points, w):
    if len(points) < 6:
        return points
    pts = sorted(points, key=lambda p: p[1])
    y = np.array([p[1] for p in pts], dtype=np.float32)
    x = np.array([p[0] for p in pts], dtype=np.float32)
    try:
        degree = 2 if len(points) >= 8 else 1
        coeff = np.polyfit(y, x, degree)
        fit_x = np.polyval(coeff, y)
    except Exception:
        return points

    result = []
    for raw_x, raw_y, fx in zip(x, y, fit_x):
        result.append((int(np.clip(0.55 * raw_x + 0.45 * fx, 0, w - 1)), int(raw_y)))
    return sorted(result, key=lambda p: p[1], reverse=True)


def _build_midline(mask):
    h, w = mask.shape
    top = int(h * TOP_CROP_RATIO)
    image_center = w // 2

    ref_x = image_center
    bot_xs = np.flatnonzero(mask[h - 1])
    if bot_xs.size >= MIN_SEGMENT_WIDTH:
        bot_cx = float(int(bot_xs[0]) + int(bot_xs[-1])) * 0.5
        ref_x = int(bot_cx)
    max_jump = max(35, int(w * MAX_CENTER_JUMP_RATIO))
    points = []
    misses = 0

    for y in range(h - 8, top, -SCAN_STEP):
        prev_x = points[-1][0] if points else ref_x
        cx = _best_row_center(mask[y], ref_x, prev_x, max_jump)
        if cx is None:
            misses += 1
            if points and misses > 4:
                break
            continue
        misses = 0
        ref_x = cx
        points.append((cx, y))

    if len(points) < MIN_MID_POINTS:
        return [], False

    points = _fit_midline(points, w)
    return points, True


def _draw_midline(frame, mask, points):
    if not points:
        return None
    pts = np.array(points, np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], False, (0, 0, 255), 5)
    h, _ = mask.shape
    lookahead_y = int(np.clip(SEGMENTATION_LOOKAHEAD_Y, 0, h - 1))
    idx = min(range(len(points)), key=lambda i: abs(points[i][1] - lookahead_y))
    tx, ty = points[idx]
    cv2.circle(frame, (tx, ty), 8, (0, 255, 255), -1)
    return tx


def _point_x_at_y(points, target_y, w):
    if not points:
        return None

    idx = min(range(len(points)), key=lambda i: abs(points[i][1] - target_y))
    return float(np.clip(points[idx][0], 0, w - 1))


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
    }


def extract_centerline_info(seg_map, frame):
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
    mid_points, mid_ok = _build_midline(road_mask) if road_valid else ([], False)
    draw_points = mid_points if mid_ok else []
    target_x = _draw_midline(out, road_mask, draw_points)

    h, w = road_mask.shape
    center_x = w // 2
    if DRAW_CAMERA_CENTER_REFERENCE:
        cv2.line(out, (center_x, h - 1), (center_x, int(h * 0.55)), (255, 255, 0), 1)

    track_error = None
    control_center_x = None
    raw_error = None
    lookahead_y = int(np.clip(SEGMENTATION_LOOKAHEAD_Y, 0, h - 1))
    if mid_ok and len(draw_points) >= 2:
        control_center_x = _point_x_at_y(draw_points, lookahead_y, w)
        raw_error = float(control_center_x - center_x)
        track_error = raw_error
        cv2.circle(out, (int(control_center_x), lookahead_y), 6, (0, 255, 0), -1)

    road_state = "OK" if road_valid else "LOST"
    mid_state = "OK" if mid_ok else "LOST"
    state_color = (0, 255, 0) if road_valid else (0, 165, 255)
    cv2.putText(out, f"Road seg: {road_state} {ratio:.1%} {reason}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
    cv2.putText(out, f"Midline: {mid_state}", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
    err_text = f"Track err: {track_error:.1f}" if track_error is not None else "Track err: N/A"
    cv2.putText(out, err_text, (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(out, f"Branches: {_prev_branch_count}", (10, 144), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    info = {
        "line_valid": track_error is not None,
        "track_error": float(track_error) if track_error is not None else None,
        "road_valid": bool(road_valid),
        "road_held": False,
        "road_state": road_state,
        "midline_valid": bool(mid_ok),
        "midline_state": mid_state,
        "road_ratio": float(ratio),
        "reason": reason,
        "branch_count": int(_prev_branch_count),
        "center_x": int(center_x),
        "target_x": int(control_center_x) if control_center_x is not None else (int(target_x) if target_x is not None else None),
        "control_center_x": int(control_center_x) if control_center_x is not None else None,
        "segmentation_lookahead_y": int(lookahead_y),
        "raw_track_error": float(raw_error) if raw_error is not None else None,
        "road_mask": road_mask,
        "mid_points": [(int(x), int(y)) for x, y in draw_points],
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
