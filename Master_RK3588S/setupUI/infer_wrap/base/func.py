"""Fast CPU post-processing for the six-output RKNN multi-task model.

The diagnostic centerline is traced from each frame's path heatmaps. Model
B-spline points remain in the result for comparison, but are not substituted
for the raw heatmap trace.
"""

import os
import time
from pathlib import Path

import cv2
import numpy as np


MODEL_WIDTH = 640
MODEL_HEIGHT = 480
PIXEL_STRIDE = 4
DET_CANDIDATES = 6300
MAX_PATHS = 2
PATH_POINTS = 32

try:
    cv2.setNumThreads(max(1, int(os.environ.get(
        "MULTITASK_OPENCV_THREADS", "2"))))
except (TypeError, ValueError):
    cv2.setNumThreads(2)


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


DET_SCORE_THRESHOLD = _env_float("MULTITASK_DET_THRESHOLD", 0.25)
DET_NMS_THRESHOLD = _env_float("MULTITASK_NMS_THRESHOLD", 0.60)
DET_PRE_NMS_TOP_K = max(1, _env_int("MULTITASK_PRE_NMS_TOP_K", 1000))
MAX_DETECTIONS = max(1, _env_int("MULTITASK_MAX_DETECTIONS", 100))
COIN_MIN_SHORT_SIDE = max(
    0.0, _env_float("MULTITASK_COIN_MIN_SHORT_SIDE", 10.0))
ROAD_THRESHOLD = float(np.clip(
    _env_float("MULTITASK_ROAD_THRESHOLD", 0.50), 0.0, 1.0))
ROAD_OVERLAY_ALPHA = float(np.clip(
    _env_float("MULTITASK_ROAD_OVERLAY_ALPHA", 0.28), 0.0, 1.0))
PATH_HEATMAP_ALPHA = float(np.clip(
    _env_float("MULTITASK_PATH_HEATMAP_ALPHA", 0.45), 0.0, 1.0))
PATH_HEATMAP_THRESHOLD = float(np.clip(
    _env_float("MULTITASK_PATH_HEATMAP_THRESHOLD", 0.25), 0.0, 1.0))
RENDER_DET_THRESHOLD = float(np.clip(
    _env_float("MULTITASK_RENDER_DET_THRESHOLD", 0.45), 0.0, 1.0))
RENDER_MAX_DETECTIONS = max(
    0, _env_int("MULTITASK_RENDER_MAX_DETECTIONS", 6))
RENDER_MAX_PER_CLASS = max(
    0, _env_int("MULTITASK_RENDER_MAX_PER_CLASS", 2))
RENDER_FULL_MAX_DETECTIONS = max(
    0, _env_int("MULTITASK_RENDER_FULL_MAX_DETECTIONS", 20))
RENDER_PATH_MIN_SCORE = float(np.clip(
    _env_float("MULTITASK_RENDER_PATH_MIN_SCORE", 0.35), 0.0, 1.0))
RENDER_PATH_MIN_COUNT_CONFIDENCE = float(np.clip(
    _env_float("MULTITASK_RENDER_PATH_MIN_COUNT_CONFIDENCE", 0.40),
    0.0, 1.0))
RENDER_MODE = os.environ.get(
    "MULTITASK_RENDER_MODE", "heatmap").strip().lower()
RENDER_MODE = {"path": "drive", "road": "debug"}.get(
    RENDER_MODE, RENDER_MODE)
if RENDER_MODE not in {"off", "heatmap", "drive", "debug", "full"}:
    RENDER_MODE = "heatmap"

PATH_COLORS = {
    "single": (0, 255, 255),
    "left": (60, 220, 80),
    "right": (255, 220, 40),
}

DETECTION_COLORS = {
    "Door": (80, 200, 255),
    "TurnSign": (0, 255, 120),
    "BeginSign": (0, 220, 255),
    "EndSign": (0, 150, 255),
    "Coin": (0, 220, 255),
    "Human": (60, 60, 255),
    "Car": (255, 160, 60),
    "SpeedSign": (255, 80, 220),
}


def _load_classes():
    names_path = Path(__file__).resolve().parent / "model" / "coco.names"
    try:
        classes = tuple(
            line.strip() for line in names_path.read_text(
                encoding="utf-8").splitlines() if line.strip())
    except OSError as exc:
        raise RuntimeError("failed to read labels: {}".format(names_path)) from exc
    if len(classes) != 8:
        raise RuntimeError("multitask RKNN requires 8 labels, got {} in {}".format(
            len(classes), names_path))
    return classes


CLASSES = _load_classes()


def sigmoid(values):
    values = np.clip(np.asarray(values, dtype=np.float32), -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-values))


def _probability_to_logit(probability):
    if probability <= 0.0:
        return -np.inf
    if probability >= 1.0:
        return np.inf
    return float(np.log(probability / (1.0 - probability)))


ROAD_LOGIT_THRESHOLD = _probability_to_logit(ROAD_THRESHOLD)


def _squeeze_batch(value, name):
    value = np.asarray(value)
    if value.ndim and value.shape[0] == 1:
        value = value[0]
    if not np.all(np.isfinite(value)):
        raise ValueError("{} contains NaN or Inf".format(name))
    return value.astype(np.float32, copy=False)


def _clip_probability(values, name):
    if np.any(values < -0.05) or np.any(values > 1.05):
        raise ValueError("{} must contain probabilities in [0,1]".format(name))
    return np.clip(values, 0.0, 1.0)


def parse_outputs(outputs):
    """Validate and normalize the fixed six-output deployment contract."""
    if not isinstance(outputs, (list, tuple)) or len(outputs) != 6:
        count = len(outputs) if isinstance(outputs, (list, tuple)) else type(outputs)
        raise ValueError(
            "multitask RKNN must return 6 outputs "
            "[boxes, scores, pixel, path_points, path_scores, "
            "path_count_scores]; got {}".format(count))

    boxes = _squeeze_batch(outputs[0], "det_boxes")
    scores = _squeeze_batch(outputs[1], "det_scores")
    pixel = _squeeze_batch(outputs[2], "pixel_logits")
    path_points = _squeeze_batch(outputs[3], "path_points")
    path_scores = _squeeze_batch(outputs[4], "path_scores").reshape(-1)
    path_count_scores = _squeeze_batch(
        outputs[5], "path_count_scores").reshape(-1)

    if boxes.ndim != 2:
        raise ValueError("det_boxes must be [6300,4], got {}".format(boxes.shape))
    if boxes.shape == (4, DET_CANDIDATES):
        boxes = boxes.T
    if boxes.shape != (DET_CANDIDATES, 4):
        raise ValueError("det_boxes expected [6300,4], got {}".format(boxes.shape))

    if scores.ndim != 2:
        raise ValueError("det_scores must be [6300,8], got {}".format(scores.shape))
    if scores.shape == (len(CLASSES), DET_CANDIDATES):
        scores = scores.T
    expected_scores = (DET_CANDIDATES, len(CLASSES))
    if scores.shape != expected_scores:
        raise ValueError("det_scores expected {}, got {}".format(
            expected_scores, scores.shape))

    if pixel.ndim != 3:
        raise ValueError("pixel_logits must be [3,120,160], got {}".format(
            pixel.shape))
    if pixel.shape[-1] == 3 and pixel.shape[0] != 3:
        pixel = pixel.transpose(2, 0, 1)
    expected_pixel = (3, MODEL_HEIGHT // PIXEL_STRIDE,
                      MODEL_WIDTH // PIXEL_STRIDE)
    if pixel.shape != expected_pixel:
        raise ValueError("pixel_logits expected {}, got {}".format(
            expected_pixel, pixel.shape))

    if path_points.shape == (MAX_PATHS, 2, PATH_POINTS):
        path_points = path_points.transpose(0, 2, 1)
    if path_points.shape != (MAX_PATHS, PATH_POINTS, 2):
        raise ValueError("path_points expected [2,32,2], got {}".format(
            path_points.shape))
    path_points = np.clip(path_points, 0.0, 1.0)

    if path_scores.shape != (MAX_PATHS,):
        raise ValueError("path_scores expected [2], got {}".format(
            path_scores.shape))
    if path_count_scores.shape != (MAX_PATHS + 1,):
        raise ValueError("path_count_scores expected [3], got {}".format(
            path_count_scores.shape))
    path_scores = _clip_probability(path_scores, "path_scores")
    path_count_scores = _clip_probability(
        path_count_scores, "path_count_scores")
    return boxes, scores, pixel, path_points, path_scores, path_count_scores


def _box_iou_one_to_many(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(x2 - x1, 0.0) * np.maximum(y2 - y1, 0.0)
    area_a = np.maximum((box[2] - box[0]) * (box[3] - box[1]), 0.0)
    area_b = np.maximum(boxes[:, 2] - boxes[:, 0], 0.0) * np.maximum(
        boxes[:, 3] - boxes[:, 1], 0.0)
    return intersection / np.maximum(area_a + area_b - intersection, 1e-9)


def _numpy_nms_indices(boxes, scores, iou_threshold):
    order = np.argsort(scores)[::-1]
    kept = []
    while order.size:
        current = int(order[0])
        kept.append(current)
        if order.size == 1:
            break
        overlap = _box_iou_one_to_many(boxes[current], boxes[order[1:]])
        order = order[1:][overlap < iou_threshold]
    return np.asarray(kept, dtype=np.int32)


def detection_nms(boxes, scores, score_threshold=DET_SCORE_THRESHOLD,
                  iou_threshold=DET_NMS_THRESHOLD,
                  pre_nms_top_k=DET_PRE_NMS_TOP_K,
                  max_detections=MAX_DETECTIONS,
                  coin_min_short_side=COIN_MIN_SHORT_SIDE):
    """Filter invalid/tiny candidates, run class-wise NMS, then cap output."""
    results = []
    pre_nms_top_k = max(1, int(pre_nms_top_k))
    box_widths = boxes[:, 2] - boxes[:, 0]
    box_heights = boxes[:, 3] - boxes[:, 1]
    valid_boxes = (box_widths > 0.0) & (box_heights > 0.0)
    short_sides = np.minimum(box_widths, box_heights)
    coin_min_short_side = max(0.0, float(coin_min_short_side))

    for class_id in range(scores.shape[1]):
        class_scores = scores[:, class_id]
        class_valid = valid_boxes
        if CLASSES[class_id] == "Coin" and coin_min_short_side > 0.0:
            # Training ignores Coin boxes whose short side is below 10 px in
            # the fixed 640x480 input. Apply the same rule before NMS so tiny
            # quantization noise cannot reintroduce those false positives.
            class_valid = class_valid & (short_sides >= coin_min_short_side)
        indices = np.flatnonzero(
            class_valid & (class_scores >= score_threshold))
        if not indices.size:
            continue
        if indices.size > pre_nms_top_k:
            candidate_scores = class_scores[indices]
            top_positions = np.argpartition(
                candidate_scores, -pre_nms_top_k)[-pre_nms_top_k:]
            indices = indices[top_positions]
        candidate_boxes = boxes[indices]
        candidate_scores = class_scores[indices]
        xywh = candidate_boxes.copy()
        xywh[:, 2:] -= xywh[:, :2]
        try:
            # OpenCV executes the greedy loop in C++. The NumPy fallback keeps
            # compatibility with board images that ship an older cv2 build.
            kept_positions = cv2.dnn.NMSBoxes(
                xywh, candidate_scores, 0.0, float(iou_threshold),
                1.0, pre_nms_top_k)
            kept_positions = np.asarray(
                kept_positions, dtype=np.int32).reshape(-1)
        except (AttributeError, cv2.error):
            kept_positions = _numpy_nms_indices(
                candidate_boxes, candidate_scores, iou_threshold)

        for position in kept_positions:
            source_index = int(indices[int(position)])
            results.append((
                class_id, float(class_scores[source_index]),
                boxes[source_index]))

    results.sort(key=lambda item: item[1], reverse=True)
    return results[:max(0, int(max_detections))]


def build_detections(image_shape, nms_results):
    height, width = image_shape[:2]
    scale_x = float(width) / MODEL_WIDTH
    scale_y = float(height) / MODEL_HEIGHT
    image_area = max(1.0, float(width * height))
    detections = []
    for class_id, score, model_box in nms_results:
        left = min(max(float(model_box[0]) * scale_x, 0.0),
                   float(max(width - 1, 0)))
        top = min(max(float(model_box[1]) * scale_y, 0.0),
                  float(max(height - 1, 0)))
        right = min(max(float(model_box[2]) * scale_x, left + 1.0),
                    float(width))
        bottom = min(max(float(model_box[3]) * scale_y, top + 1.0),
                     float(height))
        label = CLASSES[class_id]
        box_xyxy = [left, top, right, bottom]
        detections.append({
            "class_id": int(class_id),
            "class_name": label,
            "label": label,
            "category": label,
            "score": float(score),
            "box_xyxy": box_xyxy,
            "bbox": box_xyxy,
            "area_ratio": ((right - left) * (bottom - top)) / image_area,
        })
    return detections


def decode_curve_paths(points, path_scores, path_count_scores, image_shape):
    """Map the active model slots directly into original-image coordinates."""
    height, width = image_shape[:2]
    path_count = int(np.argmax(path_count_scores))
    count_confidence = float(path_count_scores[path_count])
    if path_count == 0:
        roles = ()
    elif path_count == 1:
        roles = ("single",)
    else:
        roles = ("left", "right")
    paths = []
    for slot, role in enumerate(roles):
        points_xy = points[slot].copy()
        points_xy[:, 0] *= max(width - 1, 0)
        points_xy[:, 1] *= max(height - 1, 0)
        paths.append({
            "slot": int(slot),
            "role": role,
            "score": float(path_scores[slot]),
            "count_confidence": count_confidence,
            "points_normalized": points[slot].copy(),
            "points_xy": points_xy,
        })
    return paths, path_count, count_confidence


def _row_peaks(values, threshold, max_peaks=None, min_distance=3):
    """Return local maxima from one heatmap row, strongest first."""
    if len(values) < 3:
        return []
    mask = ((values[1:-1] >= values[:-2]) &
            (values[1:-1] >= values[2:]) &
            (values[1:-1] >= threshold))
    indices = np.flatnonzero(mask) + 1
    indices = sorted(indices, key=lambda index: values[index], reverse=True)
    selected = []
    for index in indices:
        if all(abs(int(index) - previous) >= min_distance
               for previous, _ in selected):
            selected.append((int(index), float(values[index])))
            if max_peaks is not None and len(selected) >= max_peaks:
                break
    return selected


def _trace_all_paths(rows, candidates, max_jump=20, max_missing_rows=2,
                     min_points=3):
    """Connect row peaks within one frame; this contains no temporal state."""
    tracks = []
    for row_index, (row, peaks) in enumerate(zip(rows, candidates)):
        active = [
            index for index, track in enumerate(tracks)
            if row_index - track["last_row_index"] <= max_missing_rows + 1
        ]
        pairs = []
        for track_index in active:
            previous_x = tracks[track_index]["points"][-1][0]
            for peak_index, (x, confidence) in enumerate(peaks):
                distance = abs(x - previous_x)
                if distance <= max_jump:
                    pairs.append((distance, track_index, peak_index))

        used_tracks = set()
        used_peaks = set()
        for _, track_index, peak_index in sorted(pairs):
            if track_index in used_tracks or peak_index in used_peaks:
                continue
            x, confidence = peaks[peak_index]
            tracks[track_index]["points"].append((x, row, confidence))
            tracks[track_index]["last_row_index"] = row_index
            used_tracks.add(track_index)
            used_peaks.add(peak_index)

        for peak_index, (x, confidence) in enumerate(peaks):
            if peak_index not in used_peaks:
                tracks.append({
                    "points": [(x, row, confidence)],
                    "last_row_index": row_index,
                })

    paths = [track["points"] for track in tracks
             if len(track["points"]) >= min_points]
    paths.sort(
        key=lambda path: (len(path), sum(point[2] for point in path)),
        reverse=True)
    return paths


def candidate_centerlines(road_probability, center_probability,
                          peak_threshold=PATH_HEATMAP_THRESHOLD,
                          max_peaks_per_row=None, row_step=3,
                          road_floor=0.25):
    """Trace the previous model's row peaks on one raw path heatmap."""
    score = center_probability * (
        road_floor + (1.0 - road_floor) * road_probability)
    rows = list(range(score.shape[0] - 2,
                      max(score.shape[0] // 5, 1), -row_step))
    sampled_rows = []
    candidates = []
    for row in rows:
        peaks = _row_peaks(
            score[row], peak_threshold, max_peaks_per_row)
        if peaks:
            sampled_rows.append(row)
            candidates.append(peaks)
    return _trace_all_paths(sampled_rows, candidates), score


def decode_heatmap_paths(road_probability, path_heatmaps, path_scores,
                         path_count_scores, image_shape):
    """Trace both heatmap slots without path-count gating or frame history."""
    height, width = image_shape[:2]
    path_count = int(np.argmax(path_count_scores))
    count_confidence = float(path_count_scores[path_count])
    roles = (("left", "right") if path_count == 2
             else ("single", "right"))
    paths = []
    score_maps = []
    for slot, role in enumerate(roles):
        candidates, score_map = candidate_centerlines(
            road_probability, path_heatmaps[slot])
        score_maps.append(score_map)
        if not candidates:
            continue
        points = np.asarray(candidates[0], dtype=np.float32)
        points_xy = np.empty((len(points), 2), dtype=np.float32)
        points_xy[:, 0] = (
            points[:, 0] * PIXEL_STRIDE * float(max(width - 1, 0)) /
            float(max(MODEL_WIDTH - 1, 1)))
        points_xy[:, 1] = (
            points[:, 1] * PIXEL_STRIDE * float(max(height - 1, 0)) /
            float(max(MODEL_HEIGHT - 1, 1)))
        paths.append({
            "slot": int(slot),
            "role": role,
            "source": "heatmap",
            "score": float(path_scores[slot]),
            "heatmap_score": float(np.mean(points[:, 2])),
            "count_confidence": count_confidence,
            "point_confidences": points[:, 2].copy(),
            "points_xy": points_xy,
        })
    return paths, path_count, count_confidence, score_maps


def decode_outputs(outputs, image_shape, include_path_heatmaps=True):
    """Decode all model tasks without performing route selection."""
    boxes, scores, pixel_logits, path_points, path_scores, path_count_scores = (
        parse_outputs(outputs))
    curve_paths, _, _ = decode_curve_paths(
        path_points, path_scores, path_count_scores, image_shape)

    road_probability = sigmoid(pixel_logits[0])
    road_mask = (pixel_logits[0] >= ROAD_LOGIT_THRESHOLD).astype(np.uint8)
    raw_path_heatmaps = sigmoid(pixel_logits[1:])
    paths, path_count, count_confidence, path_score_maps = (
        decode_heatmap_paths(
            road_probability, raw_path_heatmaps, path_scores,
            path_count_scores, image_shape))
    path_heatmaps = raw_path_heatmaps if include_path_heatmaps else None

    path_scores_list = path_scores.tolist()
    path_count_scores_list = path_count_scores.tolist()
    detections = build_detections(
        image_shape, detection_nms(boxes, scores))
    road = {
        "probability": road_probability,
        "mask": road_mask,
        "threshold": ROAD_THRESHOLD,
        "stride": PIXEL_STRIDE,
    }
    centerline = {
        "heatmaps": path_heatmaps,
        "paths": paths,
        "curve_paths": curve_paths,
        "score_maps": path_score_maps,
        "source": "heatmap",
        "path_scores": path_scores_list,
        "path_count": path_count,
        "path_count_scores": path_count_scores_list,
        "path_count_probabilities": path_count_scores_list,
        "count_confidence": count_confidence,
        "stride": PIXEL_STRIDE,
    }
    return {
        "detections": detections,
        "road_probability": road_probability,
        "road_mask": road_mask,
        "path_heatmaps": path_heatmaps,
        "path_count": path_count,
        "path_count_scores": path_count_scores_list,
        "paths": paths,
        "curve_paths": curve_paths,
        "road": road,
        "centerline": centerline,
        # Kept as a view so path heatmaps can be materialized only when the
        # full debug renderer needs them.
        "_pixel_logits": pixel_logits,
    }


def _overlay_binary_mask(image, mask, color, alpha):
    if alpha <= 0.0 or mask is None or not mask.size:
        return
    resized = cv2.resize(
        mask, (image.shape[1], image.shape[0]),
        interpolation=cv2.INTER_NEAREST)
    if not np.any(resized):
        return
    tinted = np.empty_like(image)
    tinted[:] = color
    cv2.addWeighted(image, 1.0 - alpha, tinted, alpha, 0.0, dst=tinted)
    cv2.copyTo(tinted, resized, image)


def _overlay_path_heatmaps(image, heatmaps, colors, alpha, threshold):
    if alpha <= 0.0 or heatmaps is None or not heatmaps.size:
        return
    denominator = max(1.0 - threshold, 1e-6)
    weights = np.clip(
        (np.asarray(heatmaps, dtype=np.float32) - threshold) / denominator,
        0.0, 1.0)
    # Color at native 160x120 resolution, then let OpenCV perform the only
    # full-frame operation. Pixel brightness remains proportional to the raw
    # sigmoid probability in heatmap mode.
    colored = (
        weights[0, ..., None] * np.asarray(colors[0], dtype=np.float32) +
        weights[1, ..., None] * np.asarray(colors[1], dtype=np.float32))
    colored = np.clip(colored, 0.0, 255.0).astype(np.uint8)
    colored = cv2.resize(
        colored, (image.shape[1], image.shape[0]),
        interpolation=cv2.INTER_LINEAR)
    cv2.addWeighted(image, 1.0, colored, alpha, 0.0, dst=image)


def _select_detections_for_render(detections, mode):
    threshold = (DET_SCORE_THRESHOLD if mode in {"heatmap", "full"}
                 else RENDER_DET_THRESHOLD)
    if mode == "heatmap":
        total_limit = MAX_DETECTIONS
    else:
        total_limit = (RENDER_FULL_MAX_DETECTIONS if mode == "full"
                       else RENDER_MAX_DETECTIONS)
    if total_limit <= 0:
        return []
    per_class_limit = (total_limit if mode in {"heatmap", "full"}
                       else RENDER_MAX_PER_CLASS)
    counts = {}
    selected = []
    for detection in sorted(
            detections or [], key=lambda item: item.get("score", 0.0),
            reverse=True):
        if float(detection.get("score", 0.0)) < threshold:
            continue
        label = str(detection.get("label") or "")
        if counts.get(label, 0) >= per_class_limit:
            continue
        selected.append(detection)
        counts[label] = counts.get(label, 0) + 1
        if len(selected) >= total_limit:
            break
    return selected


def _draw_corner_box(image, bbox, color, thickness=1):
    left, top, right, bottom = [int(round(value)) for value in bbox]
    left = max(0, min(left, image.shape[1] - 1))
    right = max(0, min(right, image.shape[1] - 1))
    top = max(0, min(top, image.shape[0] - 1))
    bottom = max(0, min(bottom, image.shape[0] - 1))
    length = max(4, min(12, (right - left) // 4, (bottom - top) // 4))
    segments = (
        ((left, top), (left + length, top)),
        ((left, top), (left, top + length)),
        ((right, top), (right - length, top)),
        ((right, top), (right, top + length)),
        ((left, bottom), (left + length, bottom)),
        ((left, bottom), (left, bottom - length)),
        ((right, bottom), (right - length, bottom)),
        ((right, bottom), (right, bottom - length)),
    )
    for start, end in segments:
        cv2.line(image, start, end, color, thickness, cv2.LINE_AA)


def _draw_path_curve(image, points, color, dashed=False):
    if len(points) < 2:
        return
    if not dashed:
        cv2.polylines(image, [points], False, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.polylines(image, [points], False, color, 2, cv2.LINE_AA)
        return
    for index in range(0, len(points) - 1, 2):
        start = tuple(points[index])
        end = tuple(points[index + 1])
        cv2.line(image, start, end, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.line(image, start, end, color, 2, cv2.LINE_AA)


def render_result(image, result, mode=None):
    """Draw boxes and a per-frame heatmap centerline for model diagnosis."""
    mode = str(mode or RENDER_MODE).strip().lower()
    mode = {"path": "drive", "road": "debug"}.get(mode, mode)
    if mode == "off":
        return image
    if mode not in {"heatmap", "drive", "debug", "full"}:
        mode = "heatmap"

    if mode in {"debug", "full"}:
        _overlay_binary_mask(
            image, result.get("road_mask"), (70, 70, 210),
            ROAD_OVERLAY_ALPHA)

    if mode in {"heatmap", "full"}:
        heatmaps = result.get("path_heatmaps")
        if heatmaps is None:
            logits = result.get("_pixel_logits")
            heatmaps = sigmoid(logits[1:]) if logits is not None else None
        if heatmaps is not None:
            first_role = "single" if result.get("path_count") == 1 else "left"
            _overlay_path_heatmaps(
                image, heatmaps,
                (PATH_COLORS[first_role], PATH_COLORS["right"]),
                PATH_HEATMAP_ALPHA,
                0.0 if mode == "heatmap" else PATH_HEATMAP_THRESHOLD)

    drawn_detections = _select_detections_for_render(
        result.get("detections"), mode)
    for detection in drawn_detections:
        bbox = detection["bbox"]
        label = detection["label"]
        color = DETECTION_COLORS.get(label, (0, 220, 0))
        if mode in {"heatmap", "full"}:
            left, top, right, bottom = [
                int(round(value)) for value in bbox]
            cv2.rectangle(
                image, (left, top), (right, bottom), color, 2, cv2.LINE_AA)
        elif label == "Coin":
            left, top, right, bottom = bbox
            center = (int(round((left + right) * 0.5)),
                      int(round((top + bottom) * 0.5)))
            cv2.circle(image, center, 4, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.circle(image, center, 3, color, -1, cv2.LINE_AA)
        else:
            _draw_corner_box(
                image, bbox, color, thickness=1)
        if mode in {"heatmap", "full"}:
            left, top = int(bbox[0]), int(bbox[1])
            cv2.putText(
                image, "{} {:.2f}".format(label, detection["score"]),
                (left, max(18, top - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, color, 1, cv2.LINE_AA)

    for path_info in result.get("paths") or []:
        if (mode != "heatmap" and
                float(path_info.get("score", 0.0)) < RENDER_PATH_MIN_SCORE):
            continue
        if (mode != "heatmap" and
                float(path_info.get("count_confidence", 0.0)) <
                RENDER_PATH_MIN_COUNT_CONFIDENCE):
            continue
        points = np.rint(path_info["points_xy"]).astype(np.int32)
        if len(points) < 2:
            continue
        points[:, 0] = np.clip(points[:, 0], 0, image.shape[1] - 1)
        points[:, 1] = np.clip(points[:, 1], 0, image.shape[0] - 1)
        color = PATH_COLORS[path_info["role"]]
        _draw_path_curve(image, points, color, dashed=False)
        if mode == "full":
            for point in points:
                cv2.circle(image, tuple(point), 2, color, -1, cv2.LINE_AA)
            label_point = tuple(points[0])
            cv2.putText(
                image, "{} {:.2f}".format(
                    path_info["role"], path_info["score"]),
                (label_point[0] + 5, max(18, label_point[1] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2)

    if mode == "full":
        count = int(result.get("path_count", 0))
        scores = result.get("path_count_scores") or [0.0, 0.0, 0.0]
        centerline = result.get("centerline") or {}
        confidence = float(centerline.get(
            "count_confidence",
            scores[count] if 0 <= count < len(scores) else 0.0))
        temporal = result.get("temporal") or {}
        raw_count = int(temporal.get("raw_path_count", count))
        temporal_status = str(temporal.get("status") or "raw")
        cv2.putText(
            image, "PATHS {} RAW {} {:.2f} {}".format(
                count, raw_count, confidence, temporal_status),
            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
            (30, 230, 255), 2)
    return image


def inference_worker(rknn_lite, img_orin):
    """Run only preprocessing and RKNN inference on an NPU worker."""
    if img_orin is None or img_orin.ndim != 3:
        raise ValueError("input frame must be a BGR HxWx3 image")

    started = time.perf_counter()
    if img_orin.shape[:2] == (MODEL_HEIGHT, MODEL_WIDTH):
        resized = img_orin
    else:
        resized = cv2.resize(
            img_orin, (MODEL_WIDTH, MODEL_HEIGHT),
            interpolation=cv2.INTER_LINEAR)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    model_input = rgb[np.newaxis, ...]
    preprocessed = time.perf_counter()

    outputs = rknn_lite.inference(inputs=[model_input])
    inferred = time.perf_counter()
    return {
        "outputs": outputs,
        "image_shape": img_orin.shape,
        "source_frame": img_orin,
        "completed_at": inferred,
        "timings_ms": {
            "preprocess": (preprocessed - started) * 1000.0,
            "inference": (inferred - preprocessed) * 1000.0,
        },
    }


def finalize_inference(worker_result):
    """Decode a FIFO-ordered worker result while the NPU starts its next job."""
    if not isinstance(worker_result, dict) or "outputs" not in worker_result:
        raise ValueError("invalid RKNN worker result")
    started = time.perf_counter()
    source_frame = worker_result["source_frame"]
    result = decode_outputs(
        worker_result["outputs"], worker_result["image_shape"],
        include_path_heatmaps=True)
    postprocess_ms = (time.perf_counter() - started) * 1000.0
    worker_timings = worker_result.get("timings_ms") or {}
    preprocess_ms = float(worker_timings.get("preprocess", 0.0))
    inference_ms = float(worker_timings.get("inference", 0.0))
    fifo_wait_ms = max(
        0.0, (started - float(worker_result.get(
            "completed_at", started))) * 1000.0)
    result["timings_ms"] = {
        "preprocess": preprocess_ms,
        "inference": inference_ms,
        "fifo_wait": fifo_wait_ms,
        "postprocess": postprocess_ms,
        "temporal_filter": 0.0,
        "total": preprocess_ms + inference_ms + fifo_wait_ms + postprocess_ms,
    }
    # The frame already belongs to this asynchronous job. Return the same
    # object for matching preview/OCR instead of creating another full copy.
    result["_source_frame"] = source_frame
    result["frame"] = source_frame
    result["ocr_frame"] = (
        source_frame if any(item["label"] == "TurnSign"
                            for item in result["detections"]) else None)
    return result


def myFunc(rknn_lite, img_orin):
    """Synchronous compatibility entry point used by tests and small tools."""
    return finalize_inference(inference_worker(rknn_lite, img_orin))
