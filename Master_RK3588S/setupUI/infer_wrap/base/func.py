"""RK3588S CPU post-processing for the single-RKNN perception model.

The RKNN graph deliberately exports raw detection, pixel and topology outputs.
NMS and path selection stay here so they can be tuned without rebuilding RKNN.
"""

import os
from pathlib import Path

import cv2
import numpy as np


MODEL_WIDTH = 640
MODEL_HEIGHT = 480
PIXEL_STRIDE = 4
TOPOLOGY_CLASSES = ("normal", "fork", "merge", "unknown")


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
ROAD_THRESHOLD = _env_float("MULTITASK_ROAD_THRESHOLD", 0.50)
ROAD_OVERLAY_ALPHA = _env_float("MULTITASK_ROAD_OVERLAY_ALPHA", 0.28)
CENTERLINE_THRESHOLD = _env_float("MULTITASK_CENTERLINE_THRESHOLD", 0.25)
CENTERLINE_MIN_POINTS = max(
    3, _env_int("MULTITASK_CENTERLINE_MIN_POINTS", 8))
CENTERLINE_MAX_PEAKS_PER_ROW = max(
    1, _env_int("MULTITASK_CENTERLINE_MAX_PEAKS_PER_ROW", 3))
CENTERLINE_MAX_JUMP = max(
    1.0, _env_float("MULTITASK_CENTERLINE_MAX_JUMP", 8.0))
CENTERLINE_ROAD_FLOOR = float(np.clip(
    _env_float("MULTITASK_CENTERLINE_ROAD_FLOOR", 0.10), 0.0, 1.0))
CENTERLINE_FIT_BLEND = float(np.clip(
    _env_float("MULTITASK_CENTERLINE_FIT_BLEND", 0.45), 0.0, 1.0))
TOPOLOGY_THRESHOLD = _env_float("MULTITASK_TOPOLOGY_THRESHOLD", 0.45)
MAX_DETECTIONS = 100
MAX_CENTERLINE_PATHS = 2


def _load_classes():
    names_path = Path(__file__).resolve().parent / "model" / "coco.names"
    try:
        classes = tuple(
            line.strip()
            for line in names_path.read_text(encoding="utf-8").splitlines()
            if line.strip())
    except OSError as exc:
        raise RuntimeError("failed to read labels: {}".format(names_path)) from exc
    if len(classes) != 8:
        raise RuntimeError(
            "multitask RKNN requires 8 labels, got {} in {}".format(
                len(classes), names_path))
    return classes


CLASSES = _load_classes()


def sigmoid(values):
    values = np.clip(np.asarray(values, dtype=np.float32), -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-values))


def softmax(values, axis=-1):
    values = np.asarray(values, dtype=np.float32)
    values = values - np.max(values, axis=axis, keepdims=True)
    exp_values = np.exp(values)
    return exp_values / np.maximum(
        exp_values.sum(axis=axis, keepdims=True), 1e-9)


def _squeeze_batch(value, name):
    value = np.asarray(value)
    if value.ndim and value.shape[0] == 1:
        value = value[0]
    if not np.all(np.isfinite(value)):
        raise ValueError("{} contains NaN or Inf".format(name))
    return value.astype(np.float32, copy=False)


def parse_outputs(outputs):
    """Validate and normalize RKNN outputs to their documented layouts."""
    if not isinstance(outputs, (list, tuple)) or len(outputs) != 4:
        raise ValueError(
            "multitask RKNN must return 4 outputs: boxes, scores, pixel, topology; "
            "got {}".format(len(outputs) if isinstance(outputs, (list, tuple)) else type(outputs)))

    boxes = _squeeze_batch(outputs[0], "det_boxes")
    scores = _squeeze_batch(outputs[1], "det_scores")
    pixel = _squeeze_batch(outputs[2], "pixel_logits")
    topology = _squeeze_batch(outputs[3], "topology_logits").reshape(-1)

    if boxes.ndim != 2:
        raise ValueError("det_boxes must be rank 2 after batch squeeze, got {}".format(boxes.shape))
    if boxes.shape[-1] != 4 and boxes.shape[0] == 4:
        boxes = boxes.T
    if boxes.shape[-1] != 4:
        raise ValueError("det_boxes expected [N,4], got {}".format(boxes.shape))

    if scores.ndim != 2:
        raise ValueError("det_scores must be rank 2 after batch squeeze, got {}".format(scores.shape))
    if scores.shape[-1] != len(CLASSES) and scores.shape[0] == len(CLASSES):
        scores = scores.T
    if scores.shape != (boxes.shape[0], len(CLASSES)):
        raise ValueError(
            "det_scores expected [{},{}], got {}".format(
                boxes.shape[0], len(CLASSES), scores.shape))

    if pixel.ndim != 3:
        raise ValueError("pixel_logits expected rank 3 after batch squeeze, got {}".format(pixel.shape))
    if pixel.shape[0] != 2 and pixel.shape[-1] == 2:
        pixel = pixel.transpose(2, 0, 1)
    expected_pixel_shape = (2, MODEL_HEIGHT // PIXEL_STRIDE,
                            MODEL_WIDTH // PIXEL_STRIDE)
    if pixel.shape != expected_pixel_shape:
        raise ValueError("pixel_logits expected {}, got {}".format(expected_pixel_shape, pixel.shape))
    if topology.shape != (len(TOPOLOGY_CLASSES),):
        raise ValueError("topology_logits expected [4], got {}".format(topology.shape))
    return boxes, scores, pixel, topology


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


def detection_nms(boxes, scores, score_threshold=DET_SCORE_THRESHOLD,
                  iou_threshold=DET_NMS_THRESHOLD,
                  max_detections=MAX_DETECTIONS):
    """Run class-wise NMS on already decoded PP-YOLOE xyxy boxes."""
    results = []
    for class_id in range(scores.shape[1]):
        class_scores = scores[:, class_id]
        indices = np.flatnonzero(class_scores >= score_threshold)
        if not indices.size:
            continue
        order = indices[np.argsort(class_scores[indices])[::-1]]
        while order.size:
            current = int(order[0])
            results.append((class_id, float(class_scores[current]), boxes[current].copy()))
            if len(results) >= max_detections or order.size == 1:
                break
            overlap = _box_iou_one_to_many(boxes[current], boxes[order[1:]])
            order = order[1:][overlap < iou_threshold]
        if len(results) >= max_detections:
            break
    results.sort(key=lambda item: item[1], reverse=True)
    return results[:max_detections]


def build_detections(image_shape, nms_results):
    height, width = image_shape[:2]
    scale_x = float(width) / MODEL_WIDTH
    scale_y = float(height) / MODEL_HEIGHT
    image_area = max(1.0, float(width * height))
    detections = []
    for class_id, score, model_box in nms_results:
        left = float(np.clip(model_box[0] * scale_x, 0, max(width - 1, 0)))
        top = float(np.clip(model_box[1] * scale_y, 0, max(height - 1, 0)))
        right = float(np.clip(model_box[2] * scale_x, left + 1, width))
        bottom = float(np.clip(model_box[3] * scale_y, top + 1, height))
        label = CLASSES[class_id]
        detections.append({
            "class_id": int(class_id),
            "label": label,
            "category": label,
            "score": float(score),
            "bbox": [left, top, right, bottom],
            "area_ratio": ((right - left) * (bottom - top)) / image_area,
        })
    return detections


def _row_peaks(values, threshold, max_peaks=None, min_distance=3,
               refine_radius=2):
    if len(values) < 3:
        return []
    mask = ((values[1:-1] >= values[:-2]) &
            (values[1:-1] >= values[2:]) &
            (values[1:-1] >= threshold))
    indices = np.flatnonzero(mask) + 1
    indices = sorted(indices, key=lambda x: values[x], reverse=True)
    selected = []
    selected_indices = []
    for index in indices:
        if all(abs(int(index) - previous) >= min_distance
               for previous in selected_indices):
            left = max(0, int(index) - refine_radius)
            right = min(len(values), int(index) + refine_radius + 1)
            positions = np.arange(left, right, dtype=np.float32)
            weights = np.maximum(
                np.asarray(values[left:right], dtype=np.float32) - threshold,
                0.0,
            )
            weight_sum = float(weights.sum())
            refined_x = (
                float(np.dot(positions, weights) / weight_sum)
                if weight_sum > 1e-9 else float(index)
            )
            selected_indices.append(int(index))
            selected.append((refined_x, float(values[index])))
            if max_peaks is not None and len(selected) >= max_peaks:
                break
    return selected


def _trace_all_paths(rows, candidates, max_jump=CENTERLINE_MAX_JUMP,
                     max_missing_rows=2,
                     min_points=CENTERLINE_MIN_POINTS):
    """Connect every row peak into a path without selecting a driving route."""
    tracks = []
    row_spacing = (
        max(1, abs(int(rows[1]) - int(rows[0]))) if len(rows) > 1 else 1)
    max_row_gap = row_spacing * (max_missing_rows + 1)
    for row, peaks in zip(rows, candidates):
        active = [
            index for index, track in enumerate(tracks)
            if abs(int(row) - track["last_row"]) <= max_row_gap
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
            tracks[track_index]["last_row"] = int(row)
            used_tracks.add(track_index)
            used_peaks.add(peak_index)

        # A newly visible branch becomes another path at the row where it splits.
        for peak_index, (x, confidence) in enumerate(peaks):
            if peak_index not in used_peaks:
                tracks.append({
                    "points": [(x, row, confidence)],
                    "last_row": int(row),
                })

    paths = [track["points"] for track in tracks
             if len(track["points"]) >= min_points]
    return paths


def _smooth_path(path, fit_blend=CENTERLINE_FIT_BLEND):
    """Blend a confidence-weighted curve fit with the decoded peak points."""
    if len(path) < 3 or fit_blend <= 0.0:
        return path
    x_values = np.asarray([point[0] for point in path], dtype=np.float32)
    y_values = np.asarray([point[1] for point in path], dtype=np.float32)
    confidences = np.asarray([point[2] for point in path], dtype=np.float32)
    y_center = float(y_values.mean())
    y_scale = max(float(np.ptp(y_values)), 1.0)
    normalized_y = (y_values - y_center) / y_scale
    degree = 2 if len(path) >= 5 else 1
    design = (
        np.column_stack((normalized_y * normalized_y,
                         normalized_y,
                         np.ones_like(normalized_y)))
        if degree == 2
        else np.column_stack((normalized_y, np.ones_like(normalized_y)))
    )
    weights = np.sqrt(np.maximum(confidences, 1e-3))
    weighted_design = design * weights[:, None]
    weighted_x = x_values * weights
    try:
        normal_matrix = weighted_design.T @ weighted_design
        normal_matrix += np.eye(
            normal_matrix.shape[0], dtype=np.float32) * 1e-6
        coefficients = np.linalg.solve(
            normal_matrix,
            weighted_design.T @ weighted_x,
        )
        fitted_x = design @ coefficients
    except (FloatingPointError, TypeError, ValueError, np.linalg.LinAlgError):
        return path
    blend = float(np.clip(fit_blend, 0.0, 1.0))
    smoothed_x = np.clip(
        x_values * (1.0 - blend) + fitted_x * blend,
        0.0,
        (MODEL_WIDTH // PIXEL_STRIDE) - 1.0,
    )
    return [
        (float(x), int(y), float(confidence))
        for x, (_, y, confidence) in zip(smoothed_x, path)
    ]


def candidate_centerlines(road_probability, center_probability,
                          peak_threshold=CENTERLINE_THRESHOLD,
                          max_peaks_per_row=CENTERLINE_MAX_PEAKS_PER_ROW,
                          row_step=3,
                          road_floor=CENTERLINE_ROAD_FLOOR,
                          min_path_points=CENTERLINE_MIN_POINTS,
                          max_paths=MAX_CENTERLINE_PATHS,
                          max_jump=CENTERLINE_MAX_JUMP,
                          fit_blend=CENTERLINE_FIT_BLEND):
    """Return at most the two most confident sufficiently long paths."""
    score = center_probability * (
        road_floor + (1.0 - road_floor) * road_probability)
    rows = list(range(score.shape[0] - 2,
                      max(score.shape[0] // 5, 1), -row_step))
    sampled_rows = []
    candidates = []
    for row in rows:
        peaks = _row_peaks(score[row], peak_threshold, max_peaks_per_row)
        sampled_rows.append(row)
        candidates.append(peaks)
    paths = _trace_all_paths(
        sampled_rows,
        candidates,
        max_jump=max(1.0, float(max_jump)),
        min_points=max(3, int(min_path_points)),
    )
    paths.sort(
        key=lambda path: (
            sum(point[2] for point in path) / len(path),
            len(path),
        ),
        reverse=True,
    )
    paths = paths[:max(0, min(int(max_paths), MAX_CENTERLINE_PATHS))]
    paths = [_smooth_path(path, fit_blend) for path in paths]
    return [[(int(round(x * PIXEL_STRIDE)),
              int(round(y * PIXEL_STRIDE)), float(confidence))
             for x, y, confidence in path] for path in paths], score


def decode_topology(logits):
    probabilities = softmax(logits).reshape(-1)
    class_id = int(np.argmax(probabilities))
    confidence = float(probabilities[class_id])
    return {
        "class_id": class_id,
        "label": TOPOLOGY_CLASSES[class_id],
        "confidence": confidence,
        "reliable": confidence >= TOPOLOGY_THRESHOLD,
        "probabilities": probabilities.tolist(),
    }


def decode_outputs(outputs, image_shape):
    boxes, scores, pixel_logits, topology_logits = parse_outputs(outputs)
    nms_results = detection_nms(boxes, scores)
    road_probability = sigmoid(pixel_logits[0])
    center_probability = sigmoid(pixel_logits[1])
    paths, path_score = candidate_centerlines(
        road_probability, center_probability)
    return {
        "detections": build_detections(image_shape, nms_results),
        "road": {
            "probability": road_probability,
            "mask": (road_probability >= ROAD_THRESHOLD).astype(np.uint8),
            "threshold": ROAD_THRESHOLD,
            "stride": PIXEL_STRIDE,
        },
        "centerline": {
            "probability": center_probability,
            "score": path_score,
            "paths": paths,
            "stride": PIXEL_STRIDE,
        },
        "topology": decode_topology(topology_logits),
    }


def _draw_result(image, result):
    road_mask = result["road"]["mask"]
    if ROAD_OVERLAY_ALPHA > 0.0 and road_mask.size:
        resized_mask = cv2.resize(
            road_mask, (image.shape[1], image.shape[0]),
            interpolation=cv2.INTER_NEAREST).astype(bool)
        if np.any(resized_mask):
            overlay_color = np.asarray((70, 70, 210), dtype=np.float32)
            pixels = image[resized_mask].astype(np.float32)
            image[resized_mask] = np.clip(
                pixels * (1.0 - ROAD_OVERLAY_ALPHA) +
                overlay_color * ROAD_OVERLAY_ALPHA,
                0, 255).astype(np.uint8)

    for detection in result["detections"]:
        left, top, right, bottom = [int(value) for value in detection["bbox"]]
        cv2.rectangle(image, (left, top), (right, bottom), (0, 220, 0), 2)
        cv2.putText(
            image, "{} {:.2f}".format(detection["label"], detection["score"]),
            (left, max(18, top - 6)), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 0, 255), 2)

    centerline_color = (0, 255, 255)
    scale_x = float(image.shape[1]) / MODEL_WIDTH
    scale_y = float(image.shape[0]) / MODEL_HEIGHT
    for path in result["centerline"]["paths"]:
        points = np.asarray(
            [[int(x * scale_x), int(y * scale_y)] for x, y, _ in path],
            dtype=np.int32)
        if len(points) >= 2:
            cv2.polylines(image, [points], False,
                          centerline_color, 2,
                          cv2.LINE_AA)
    topology = result["topology"]
    topology_text = "TOPO {} {:.2f}".format(
        topology["label"], topology["confidence"])
    text_size, _ = cv2.getTextSize(
        topology_text, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
    topology_x = max(10, image.shape[1] - text_size[0] - 10)
    cv2.putText(
        image, topology_text, (topology_x, 24),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (30, 230, 255), 2)


def myFunc(rknn_lite, img_orin):
    if img_orin is None or img_orin.ndim != 3:
        raise ValueError("input frame must be a BGR HxWx3 image")
    source_frame = img_orin.copy()
    rgb = cv2.cvtColor(source_frame, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (MODEL_WIDTH, MODEL_HEIGHT),
                     interpolation=cv2.INTER_LINEAR)
    # RKNN conversion owns ImageNet mean/std normalization. Keep this uint8.
    model_input = np.expand_dims(np.ascontiguousarray(rgb), 0)
    outputs = rknn_lite.inference(inputs=[model_input])
    result = decode_outputs(outputs, source_frame.shape)
    result["ocr_frame"] = (
        source_frame.copy()
        if any(item["label"] == "TurnSign" for item in result["detections"])
        else None)
    _draw_result(source_frame, result)
    result["frame"] = source_frame
    return result
