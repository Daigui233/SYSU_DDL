"""Fast CPU post-processing for the six-output RKNN multi-task model.

Raw curve anchors are decoded from the ordered row-classification head. Curve
cleanup, compensation, and fitting are performed once in vision_control.py.
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
ROW_ANCHORS = 32
ROW_BINS = 160
ROW_ANCHOR_TOP = 0.40

try:
    cv2.setNumThreads(max(1, int(os.environ.get(
        "MULTITASK_OPENCV_THREADS", "1"))))
except (TypeError, ValueError):
    cv2.setNumThreads(1)


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


DET_SCORE_THRESHOLD = _env_float("MULTITASK_DET_THRESHOLD", 0.50)
DET_TURNSIGN_SCORE_THRESHOLD = _env_float(
    "MULTITASK_TURNSIGN_THRESHOLD", 0.40)
DET_CAR_SCORE_THRESHOLD = _env_float("MULTITASK_CAR_THRESHOLD", 0.35)
DET_NMS_THRESHOLD = _env_float("MULTITASK_NMS_THRESHOLD", 0.45)
DET_PRE_NMS_TOP_K = max(1, _env_int("MULTITASK_PRE_NMS_TOP_K", 1000))
MAX_DETECTIONS = max(1, _env_int("MULTITASK_MAX_DETECTIONS", 100))
COIN_MIN_SHORT_SIDE = max(
    0.0, _env_float("MULTITASK_COIN_MIN_SHORT_SIDE", 10.0))
ROAD_THRESHOLD = float(np.clip(
    _env_float("MULTITASK_ROAD_THRESHOLD", 0.50), 0.0, 1.0))
ROAD_TOP_CROP_RATIO = float(np.clip(
    _env_float("MULTITASK_ROAD_TOP_CROP_RATIO", 0.34), 0.0, 0.8))
ROAD_MIN_RATIO = max(0.0, _env_float("MULTITASK_ROAD_MIN_RATIO", 0.001))
ROAD_MAX_RATIO = min(1.0, _env_float("MULTITASK_ROAD_MAX_RATIO", 0.60))
ROAD_MAX_RAW_RATIO = min(
    1.0, _env_float("MULTITASK_ROAD_MAX_RAW_RATIO", 0.86))
ROAD_MIN_COMPONENT_AREA_RATIO = max(
    0.0, _env_float("MULTITASK_ROAD_MIN_COMPONENT_AREA_RATIO", 0.0015))
ROAD_MAX_COMPONENTS = max(1, _env_int("MULTITASK_ROAD_MAX_COMPONENTS", 3))
ROAD_OVERLAY_ALPHA = float(np.clip(
    _env_float("MULTITASK_ROAD_OVERLAY_ALPHA", 0.28), 0.0, 1.0))
RENDER_DET_THRESHOLD = float(np.clip(
    _env_float("MULTITASK_RENDER_DET_THRESHOLD", 0.45), 0.0, 1.0))
RENDER_MAX_DETECTIONS = max(
    0, _env_int("MULTITASK_RENDER_MAX_DETECTIONS", 6))
RENDER_MAX_PER_CLASS = max(
    0, _env_int("MULTITASK_RENDER_MAX_PER_CLASS", 2))
RENDER_FULL_MAX_DETECTIONS = max(
    0, _env_int("MULTITASK_RENDER_FULL_MAX_DETECTIONS", 20))
RENDER_MAX_BOX_WIDTH_RATIO = float(np.clip(
    _env_float("MULTITASK_RENDER_MAX_BOX_WIDTH_RATIO", 0.70), 0.0, 1.0))
RENDER_MAX_BOX_HEIGHT_RATIO = float(np.clip(
    _env_float("MULTITASK_RENDER_MAX_BOX_HEIGHT_RATIO", 0.70), 0.0, 1.0))
RENDER_MAX_BOX_AREA_RATIO = float(np.clip(
    _env_float("MULTITASK_RENDER_MAX_BOX_AREA_RATIO", 0.30), 0.0, 1.0))
RENDER_PATH_MIN_SCORE = float(np.clip(
    _env_float("MULTITASK_RENDER_PATH_MIN_SCORE", 0.35), 0.0, 1.0))
RENDER_PATH_MIN_COUNT_CONFIDENCE = float(np.clip(
    _env_float("MULTITASK_RENDER_PATH_MIN_COUNT_CONFIDENCE", 0.40),
    0.0, 1.0))
RENDER_MODE = os.environ.get(
    "MULTITASK_RENDER_MODE", "drive").strip().lower()
RENDER_MODE = {"path": "drive", "road": "debug"}.get(
    RENDER_MODE, RENDER_MODE)
if RENDER_MODE not in {"off", "drive", "debug", "full"}:
    RENDER_MODE = "drive"

PATH_COLORS = {
    "single": (255, 0, 0),
    "left": (255, 0, 0),
    "right": (0, 255, 0),
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


def _road_component_score(labels, stats, label, height, width):
    left, top, component_width, component_height, area = stats[label]
    bottom_y = int(height * 0.72)
    bottom_roi = labels[
        bottom_y:, left:left + component_width] == label
    bottom_contact = float(bottom_roi.sum()) / float(max(
        1, component_width * max(1, height - bottom_y)))
    xs = np.flatnonzero(bottom_roi.sum(axis=0))
    if xs.size:
        bottom_center = left + (xs[0] + xs[-1]) * 0.5
        center_score = 1.0 - min(
            1.0,
            abs(bottom_center - width * 0.5) / max(1.0, width * 0.5),
        )
    else:
        center_score = 0.0
    area_score = float(area) / float(max(1, height * width))
    height_score = float(component_height) / float(max(1, height))
    return (
        area_score * 3.0 + bottom_contact * 1.8 +
        center_score * 0.6 + height_score * 0.35)


def clean_road_mask(raw_mask, top_crop_ratio=ROAD_TOP_CROP_RATIO,
                    return_info=False):
    """Adapt the main-branch road-mask cleanup to the 120x160 output."""
    raw = (np.asarray(raw_mask) != 0).astype(np.uint8)
    raw_ratio = float(np.mean(raw)) if raw.size else 0.0
    mask = raw.copy()
    height, width = mask.shape
    mask[:int(height * float(top_crop_ratio)), :] = 0
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask, connectivity=8)
    min_area = max(
        6, int(round(height * width * ROAD_MIN_COMPONENT_AREA_RATIO)))
    components = []
    for label in range(1, component_count):
        left, top, component_width, component_height, area = stats[label]
        if area < min_area or component_width < 2 or component_height < 2:
            continue
        score = _road_component_score(
            labels, stats, label, height, width)
        components.append((score, label))
    components.sort(reverse=True)
    selected = np.zeros_like(mask)
    for _score, label in components[:ROAD_MAX_COMPONENTS]:
        selected[labels == label] = 1
    if np.any(selected):
        selected = cv2.morphologyEx(
            selected, cv2.MORPH_CLOSE,
            np.ones((3, 3), dtype=np.uint8))

    ratio = float(np.mean(selected)) if selected.size else 0.0
    if raw_ratio > ROAD_MAX_RAW_RATIO:
        valid, reason = False, "raw-full"
    elif ratio < ROAD_MIN_RATIO:
        valid, reason = False, "small"
    elif ratio > ROAD_MAX_RATIO:
        valid, reason = False, "large"
    else:
        valid, reason = True, "ok"
    if not valid:
        selected.fill(0)
    info = {
        "valid": valid,
        "reason": reason,
        "ratio": ratio if valid else 0.0,
        "raw_ratio": raw_ratio,
        "component_count": min(len(components), ROAD_MAX_COMPONENTS),
    }
    return (selected, info) if return_info else selected


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
            "[boxes, scores, pixel, row_path_logits, path_scores, "
            "path_count_scores]; got {}".format(count))

    boxes = _squeeze_batch(outputs[0], "det_boxes")
    scores = _squeeze_batch(outputs[1], "det_scores")
    pixel = _squeeze_batch(outputs[2], "pixel_logits")
    row_path_logits = _squeeze_batch(outputs[3], "row_path_logits")
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

    if row_path_logits.shape == (ROW_ANCHORS, MAX_PATHS, ROW_BINS + 1):
        row_path_logits = row_path_logits.transpose(1, 0, 2)
    if row_path_logits.shape == (ROW_ANCHORS, ROW_BINS + 1, MAX_PATHS):
        row_path_logits = row_path_logits.transpose(2, 0, 1)
    expected_row_logits = (MAX_PATHS, ROW_ANCHORS, ROW_BINS + 1)
    if row_path_logits.shape != expected_row_logits:
        raise ValueError(
            "row_path_logits expected {}, got {}. This board package no "
            "longer supports the old B-spline [2,32,2] output.".format(
                expected_row_logits, row_path_logits.shape))

    if path_scores.shape != (MAX_PATHS,):
        raise ValueError("path_scores expected [2], got {}".format(
            path_scores.shape))
    if path_count_scores.shape != (MAX_PATHS + 1,):
        raise ValueError("path_count_scores expected [3], got {}".format(
            path_count_scores.shape))
    path_scores = _clip_probability(path_scores, "path_scores")
    path_count_scores = _clip_probability(
        path_count_scores, "path_count_scores")
    return (boxes, scores, pixel, row_path_logits, path_scores,
            path_count_scores)


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
                  coin_min_short_side=COIN_MIN_SHORT_SIDE,
                  turnsign_score_threshold=DET_TURNSIGN_SCORE_THRESHOLD,
                  car_score_threshold=DET_CAR_SCORE_THRESHOLD):
    """Apply main-branch argmax filtering and class-wise NMS."""
    results = []
    pre_nms_top_k = max(1, int(pre_nms_top_k))
    box_widths = boxes[:, 2] - boxes[:, 0]
    box_heights = boxes[:, 3] - boxes[:, 1]
    valid_boxes = (box_widths > 0.0) & (box_heights > 0.0)
    short_sides = np.minimum(box_widths, box_heights)
    coin_min_short_side = max(0.0, float(coin_min_short_side))
    best_classes = np.argmax(scores, axis=1).astype(np.int32, copy=False)
    best_scores = scores[
        np.arange(scores.shape[0], dtype=np.int32), best_classes]
    turnsign_class_id = CLASSES.index("TurnSign")
    car_class_id = CLASSES.index("Car")
    class_thresholds = np.full(
        best_classes.shape, float(score_threshold), dtype=np.float32)
    class_thresholds[best_classes == turnsign_class_id] = float(
        turnsign_score_threshold)
    class_thresholds[best_classes == car_class_id] = float(
        car_score_threshold)
    eligible = valid_boxes & (best_scores >= class_thresholds)

    for class_id in np.unique(best_classes[eligible]):
        class_valid = eligible & (best_classes == class_id)
        if CLASSES[class_id] == "TurnSign":
            class_valid = class_valid & (
                best_scores >= float(turnsign_score_threshold))
        if CLASSES[class_id] == "Coin" and coin_min_short_side > 0.0:
            # Training ignores Coin boxes whose short side is below 10 px in
            # the fixed 640x480 input. Apply the same rule before NMS so tiny
            # quantization noise cannot reintroduce those false positives.
            class_valid = class_valid & (short_sides >= coin_min_short_side)
        indices = np.flatnonzero(class_valid)
        if not indices.size:
            continue
        if indices.size > pre_nms_top_k:
            candidate_scores = best_scores[indices]
            top_positions = np.argpartition(
                candidate_scores, -pre_nms_top_k)[-pre_nms_top_k:]
            indices = indices[top_positions]
        candidate_boxes = boxes[indices]
        candidate_scores = best_scores[indices]
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
                int(class_id), float(best_scores[source_index]),
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


def softmax(values, axis=-1):
    values = np.asarray(values, dtype=np.float32)
    values = values - np.max(values, axis=axis, keepdims=True)
    values = np.exp(np.clip(values, -60.0, 30.0))
    return values / np.maximum(values.sum(axis=axis, keepdims=True), 1e-12)


def _row_anchor_y_normalized(count):
    """Return the exact bottom-to-top anchors used by MultitaskResize."""
    return np.linspace(1.0, ROW_ANCHOR_TOP, count, dtype=np.float32)


def decode_raw_curve_paths(
        row_path_logits, path_scores, path_count_scores, image_shape):
    """Decode every row-head anchor without thresholding or fitting."""
    height, width = image_shape[:2]
    probabilities = softmax(row_path_logits, axis=-1)
    coordinate_probabilities = probabilities[..., :ROW_BINS]
    coordinate_mass = coordinate_probabilities.sum(axis=-1)
    conditional = coordinate_probabilities / np.maximum(
        coordinate_mass[..., None], 1e-12)
    x_values = np.arange(ROW_BINS, dtype=np.float32)
    x_normalized = (conditional * x_values).sum(axis=-1) / float(ROW_BINS - 1)
    y_normalized = _row_anchor_y_normalized(ROW_ANCHORS)
    model_path_count = int(np.argmax(path_count_scores))
    count_confidence = float(path_count_scores[model_path_count])
    scale = np.asarray(
        [max(width - 1, 0), max(height - 1, 0)], dtype=np.float32)
    raw_paths = []
    for slot in range(MAX_PATHS):
        normalized = np.stack(
            (x_normalized[slot], y_normalized), axis=1).astype(np.float32)
        raw_paths.append({
            "slot": int(slot),
            "role": "left" if slot == 0 else "right",
            "source": "row_classifier_raw",
            "score": float(path_scores[slot]),
            "row_support": int(ROW_ANCHORS),
            "row_indices": np.arange(ROW_ANCHORS, dtype=np.int32),
            "point_confidences": coordinate_mass[slot].astype(np.float32),
            "points_normalized": normalized,
            "points_xy": (normalized * scale).astype(np.float32),
            "road_constrained": False,
        })
    return raw_paths, model_path_count, count_confidence


def decode_outputs(
        outputs, image_shape, include_debug_probabilities=True):
    """Decode detections, semantic road, and unfiltered curve anchors."""
    boxes, scores, pixel_logits, row_path_logits, path_scores, path_count_scores = (
        parse_outputs(outputs))

    road_probability = (
        sigmoid(pixel_logits[0]) if include_debug_probabilities else None)
    road_mask_raw = (
        pixel_logits[0] >= ROAD_LOGIT_THRESHOLD).astype(np.uint8)
    road_mask, road_quality = clean_road_mask(
        road_mask_raw, return_info=True)
    raw_curve_paths, model_path_count, count_confidence = \
        decode_raw_curve_paths(
            row_path_logits, path_scores, path_count_scores, image_shape)

    path_scores_list = path_scores.tolist()
    path_count_scores_list = path_count_scores.tolist()
    detections = build_detections(
        image_shape, detection_nms(boxes, scores))
    road = {
        "probability": road_probability,
        "mask": road_mask,
        "raw_mask": road_mask_raw,
        "threshold": ROAD_THRESHOLD,
        "stride": PIXEL_STRIDE,
        **road_quality,
    }
    centerline = {
        "raw_curve_paths": raw_curve_paths,
        "path_scores": path_scores_list,
        "path_count_scores": path_count_scores_list,
        "path_count_probabilities": path_count_scores_list,
        "model_path_count": model_path_count,
        "count_confidence": count_confidence,
        "stride": PIXEL_STRIDE,
    }
    return {
        "detections": detections,
        "road_probability": road_probability,
        "road_mask": road_mask,
        "road_mask_raw": road_mask_raw,
        "model_path_count": model_path_count,
        "path_count_scores": path_count_scores_list,
        "raw_curve_paths": raw_curve_paths,
        "road": road,
        "centerline": centerline,
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


def _select_detections_for_render(detections, mode):
    threshold = (
        DET_SCORE_THRESHOLD if mode == "full" else RENDER_DET_THRESHOLD)
    total_limit = (
        RENDER_FULL_MAX_DETECTIONS if mode == "full"
        else RENDER_MAX_DETECTIONS)
    if total_limit <= 0:
        return []
    per_class_limit = (
        total_limit if mode == "full" else RENDER_MAX_PER_CLASS)
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


def _is_oversized_render_box(detection, image_shape):
    """Hide frame-spanning preview boxes without changing perception data."""
    height, width = image_shape[:2]
    if width <= 0 or height <= 0:
        return False
    left, top, right, bottom = detection.get("bbox", (0, 0, 0, 0))
    width_ratio = max(0.0, float(right) - float(left)) / float(width)
    height_ratio = max(0.0, float(bottom) - float(top)) / float(height)
    area_ratio = width_ratio * height_ratio
    return (
        width_ratio >= RENDER_MAX_BOX_WIDTH_RATIO or
        height_ratio >= RENDER_MAX_BOX_HEIGHT_RATIO or
        area_ratio >= RENDER_MAX_BOX_AREA_RATIO)


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
    """Draw detections, semantic road, and finalized control paths."""
    mode = str(mode or RENDER_MODE).strip().lower()
    mode = {"path": "drive", "road": "debug"}.get(mode, mode)
    if mode == "off":
        return image
    if mode not in {"drive", "debug", "full"}:
        mode = "drive"

    if mode in {"debug", "full"}:
        _overlay_binary_mask(
            image, result.get("road_mask"), (70, 70, 210),
            ROAD_OVERLAY_ALPHA)

    drawn_detections = _select_detections_for_render(
        result.get("detections"), mode)
    for detection in drawn_detections:
        if _is_oversized_render_box(detection, image.shape):
            continue
        bbox = detection["bbox"]
        label = detection["label"]
        color = DETECTION_COLORS.get(label, (0, 220, 0))
        if mode == "full":
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
        if mode == "full":
            left, top = int(bbox[0]), int(bbox[1])
            cv2.putText(
                image, "{} {:.2f}".format(label, detection["score"]),
                (left, max(18, top - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, color, 1, cv2.LINE_AA)

    display_paths = result.get("paths") or []
    if result.get("vision_control_path_overlay"):
        display_paths = ()
    for path_info in list(display_paths)[:MAX_PATHS]:
        if float(path_info.get("score", 0.0)) < RENDER_PATH_MIN_SCORE:
            continue
        if (float(path_info.get("count_confidence", 0.0)) <
                RENDER_PATH_MIN_COUNT_CONFIDENCE):
            continue
        color = PATH_COLORS[path_info["role"]]
        segments = path_info.get("display_segments_xy")
        if segments is None:
            segments = [path_info["points_xy"]]
        drawn_points = []
        for segment in segments:
            points = np.rint(segment).astype(np.int32)
            if len(points) < 2:
                continue
            points[:, 0] = np.clip(points[:, 0], 0, image.shape[1] - 1)
            points[:, 1] = np.clip(points[:, 1], 0, image.shape[0] - 1)
            _draw_path_curve(image, points, color, dashed=False)
            drawn_points.append(points)
            if mode == "full":
                for point in points:
                    cv2.circle(image, tuple(point), 2, color, -1, cv2.LINE_AA)
        if mode == "full" and drawn_points:
            label_point = tuple(drawn_points[0][0])
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
        include_debug_probabilities=False)
    # The realtime controller only needs the semantic road mask and raw row
    # anchors. Do not retain logits after post-processing.
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
