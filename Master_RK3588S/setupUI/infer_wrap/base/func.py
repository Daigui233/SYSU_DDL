"""CPU post-processing for the six-output RKNN multi-task perception model.

The model exports decoded B-spline candidate paths.  CPU post-processing only
scales and displays those paths; it never reconnects heatmap peaks into a
different path.  Road and per-slot heatmaps remain available for diagnostics.
"""

import os
from pathlib import Path

import cv2
import numpy as np


MODEL_WIDTH = 640
MODEL_HEIGHT = 480
PIXEL_STRIDE = 4
MAX_PATHS = 2
PATH_POINTS = 32
PATH_COLORS = ((0, 255, 255), (255, 80, 255))  # BGR: left/single, right


def _env_float(name, default):
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return float(default)


DET_SCORE_THRESHOLD = _env_float("MULTITASK_DET_THRESHOLD", 0.25)
DET_NMS_THRESHOLD = _env_float("MULTITASK_NMS_THRESHOLD", 0.60)
ROAD_THRESHOLD = _env_float("MULTITASK_ROAD_THRESHOLD", 0.50)
ROAD_OVERLAY_ALPHA = _env_float("MULTITASK_ROAD_OVERLAY_ALPHA", 0.28)
# Only used if path-count predicts zero.  A predicted one/two-path frame keeps
# every corresponding slot so the board does not choose or hide a candidate.
PATH_SCORE_FALLBACK_THRESHOLD = _env_float(
    "MULTITASK_PATH_SCORE_FALLBACK_THRESHOLD", 0.20)
MAX_DETECTIONS = 100


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


def softmax(values, axis=-1):
    values = np.asarray(values, dtype=np.float32)
    values = values - np.max(values, axis=axis, keepdims=True)
    exp_values = np.exp(values)
    return exp_values / np.maximum(exp_values.sum(axis=axis, keepdims=True), 1e-9)


def _squeeze_batch(value, name):
    value = np.asarray(value)
    if value.ndim and value.shape[0] == 1:
        value = value[0]
    if not np.all(np.isfinite(value)):
        raise ValueError("{} contains NaN or Inf".format(name))
    return value.astype(np.float32, copy=False)


def parse_outputs(outputs):
    """Validate the fixed six-output deployment contract."""
    if not isinstance(outputs, (list, tuple)) or len(outputs) != 6:
        count = len(outputs) if isinstance(outputs, (list, tuple)) else type(outputs)
        raise ValueError("multitask RKNN must return 6 outputs "
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
        raise ValueError("det_boxes must be [N,4], got {}".format(boxes.shape))
    if boxes.shape[-1] != 4 and boxes.shape[0] == 4:
        boxes = boxes.T
    if boxes.shape[-1] != 4:
        raise ValueError("det_boxes expected [N,4], got {}".format(boxes.shape))

    if scores.ndim != 2:
        raise ValueError("det_scores must be [N,8], got {}".format(scores.shape))
    if scores.shape[-1] != len(CLASSES) and scores.shape[0] == len(CLASSES):
        scores = scores.T
    if scores.shape != (boxes.shape[0], len(CLASSES)):
        raise ValueError("det_scores expected [{},{}], got {}".format(
            boxes.shape[0], len(CLASSES), scores.shape))

    if pixel.ndim != 3:
        raise ValueError("pixel_logits must be [3,120,160], got {}".format(
            pixel.shape))
    if pixel.shape[0] != 3 and pixel.shape[-1] == 3:
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
    if np.any(path_points < -0.05) or np.any(path_points > 1.05):
        raise ValueError("path_points must be normalized coordinates in [0,1]")
    path_points = np.clip(path_points, 0.0, 1.0)
    if path_scores.shape != (MAX_PATHS,):
        raise ValueError("path_scores expected [2], got {}".format(path_scores.shape))
    if path_count_scores.shape != (MAX_PATHS + 1,):
        raise ValueError("path_count_scores expected [3], got {}".format(
            path_count_scores.shape))
    return boxes, scores, pixel, path_points, path_scores, path_count_scores


def _box_iou_one_to_many(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(x2 - x1, 0.0) * np.maximum(y2 - y1, 0.0)
    area_a = np.maximum((box[2] - box[0]) * (box[3] - box[1]), 0.0)
    area_b = np.maximum((boxes[:, 2] - boxes[:, 0]) *
                        (boxes[:, 3] - boxes[:, 1]), 0.0)
    return intersection / np.maximum(area_a + area_b - intersection, 1e-9)


def detection_nms(boxes, scores, score_threshold=DET_SCORE_THRESHOLD,
                  iou_threshold=DET_NMS_THRESHOLD,
                  max_detections=MAX_DETECTIONS):
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
    scale_x, scale_y = float(width) / MODEL_WIDTH, float(height) / MODEL_HEIGHT
    image_area = max(1.0, float(width * height))
    detections = []
    for class_id, score, model_box in nms_results:
        left = float(np.clip(model_box[0] * scale_x, 0, max(width - 1, 0)))
        top = float(np.clip(model_box[1] * scale_y, 0, max(height - 1, 0)))
        right = float(np.clip(model_box[2] * scale_x, left + 1, width))
        bottom = float(np.clip(model_box[3] * scale_y, top + 1, height))
        label = CLASSES[class_id]
        detections.append({
            "class_id": int(class_id), "label": label, "category": label,
            "score": float(score), "bbox": [left, top, right, bottom],
            "area_ratio": ((right - left) * (bottom - top)) / image_area,
        })
    return detections


def decode_curve_paths(points, score_logits, count_logits):
    """Decode every model-predicted candidate path without route selection."""
    path_probabilities = sigmoid(score_logits)
    count_probabilities = softmax(count_logits)
    predicted_count = int(np.argmax(count_probabilities))
    if predicted_count > 0:
        active_slots = range(predicted_count)
    else:
        active_slots = [index for index, score in enumerate(path_probabilities)
                        if score >= PATH_SCORE_FALLBACK_THRESHOLD]
    paths = []
    for slot in active_slots:
        # Training canonicalizes slot 0 as single/left and slot 1 as right.
        path = [(float(x * (MODEL_WIDTH - 1)), float(y * (MODEL_HEIGHT - 1)),
                 float(path_probabilities[slot]))
                for x, y in points[slot]]
        paths.append({"slot": int(slot), "score": float(path_probabilities[slot]),
                      "points": path})
    return paths, path_probabilities, predicted_count, count_probabilities


def decode_outputs(outputs, image_shape):
    boxes, scores, pixel_logits, path_points, path_score_logits, count_logits = (
        parse_outputs(outputs))
    road_probability = sigmoid(pixel_logits[0])
    path_heatmaps = sigmoid(pixel_logits[1:])
    paths, path_scores, path_count, count_probabilities = decode_curve_paths(
        path_points, path_score_logits, count_logits)
    return {
        "detections": build_detections(image_shape, detection_nms(boxes, scores)),
        "road": {"probability": road_probability,
                 "mask": (road_probability >= ROAD_THRESHOLD).astype(np.uint8),
                 "threshold": ROAD_THRESHOLD, "stride": PIXEL_STRIDE},
        "centerline": {
            "heatmaps": path_heatmaps,
            "paths": paths,
            "path_scores": path_scores.tolist(),
            "path_count": path_count,
            "path_count_probabilities": count_probabilities.tolist(),
            "stride": PIXEL_STRIDE,
        },
    }


def _draw_result(image, result):
    road_mask = result["road"]["mask"]
    if ROAD_OVERLAY_ALPHA > 0.0 and road_mask.size:
        resized_mask = cv2.resize(road_mask, (image.shape[1], image.shape[0]),
                                  interpolation=cv2.INTER_NEAREST).astype(bool)
        if np.any(resized_mask):
            overlay_color = np.asarray((70, 70, 210), dtype=np.float32)
            image[resized_mask] = np.clip(
                image[resized_mask].astype(np.float32) * (1.0 - ROAD_OVERLAY_ALPHA) +
                overlay_color * ROAD_OVERLAY_ALPHA, 0, 255).astype(np.uint8)

    for detection in result["detections"]:
        left, top, right, bottom = [int(value) for value in detection["bbox"]]
        cv2.rectangle(image, (left, top), (right, bottom), (0, 220, 0), 2)
        cv2.putText(image, "{} {:.2f}".format(detection["label"], detection["score"]),
                    (left, max(18, top - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 255), 2)

    scale_x, scale_y = float(image.shape[1]) / MODEL_WIDTH, float(image.shape[0]) / MODEL_HEIGHT
    for path_info in result["centerline"]["paths"]:
        points = np.asarray([[int(x * scale_x), int(y * scale_y)]
                             for x, y, _ in path_info["points"]], dtype=np.int32)
        if len(points) >= 2:
            color = PATH_COLORS[path_info["slot"] % len(PATH_COLORS)]
            cv2.polylines(image, [points], False, color, 2, cv2.LINE_AA)


def myFunc(rknn_lite, img_orin):
    if img_orin is None or img_orin.ndim != 3:
        raise ValueError("input frame must be a BGR HxWx3 image")
    source_frame = img_orin.copy()
    rgb = cv2.cvtColor(source_frame, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (MODEL_WIDTH, MODEL_HEIGHT), interpolation=cv2.INTER_LINEAR)
    model_input = np.expand_dims(np.ascontiguousarray(rgb), 0)
    outputs = rknn_lite.inference(inputs=[model_input])
    result = decode_outputs(outputs, source_frame.shape)
    result["ocr_frame"] = (source_frame.copy() if any(
        item["label"] == "TurnSign" for item in result["detections"]) else None)
    _draw_result(source_frame, result)
    result["frame"] = source_frame
    return result
