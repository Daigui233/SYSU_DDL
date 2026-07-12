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


DET_SCORE_THRESHOLD = _env_float("MULTITASK_DET_THRESHOLD", 0.25)
DET_NMS_THRESHOLD = _env_float("MULTITASK_NMS_THRESHOLD", 0.60)
ROAD_THRESHOLD = _env_float("MULTITASK_ROAD_THRESHOLD", 0.50)
CENTERLINE_THRESHOLD = _env_float("MULTITASK_CENTERLINE_THRESHOLD", 0.25)
TOPOLOGY_THRESHOLD = _env_float("MULTITASK_TOPOLOGY_THRESHOLD", 0.45)
MAX_DETECTIONS = 100


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


def _row_peaks(values, threshold, max_peaks):
    if len(values) < 3:
        return []
    mask = ((values[1:-1] >= values[:-2]) &
            (values[1:-1] >= values[2:]) &
            (values[1:-1] >= threshold))
    indices = np.flatnonzero(mask) + 1
    indices = sorted(indices, key=lambda x: values[x], reverse=True)
    return [(int(x), float(values[x])) for x in indices[:max_peaks]]


def _best_path(rows, candidates):
    if not rows:
        return []
    costs = []
    parents = []
    for row_index, row_candidates in enumerate(candidates):
        row_cost = np.full(len(row_candidates), np.inf, dtype=np.float32)
        row_parent = np.full(len(row_candidates), -1, dtype=np.int32)
        for index, (x, confidence) in enumerate(row_candidates):
            node_cost = -np.log(max(confidence, 1e-6))
            if row_index == 0:
                row_cost[index] = node_cost
                continue
            for previous_index, (previous_x, _) in enumerate(candidates[row_index - 1]):
                transition = 0.035 * abs(x - previous_x)
                value = costs[-1][previous_index] + transition + node_cost
                if value < row_cost[index]:
                    row_cost[index] = value
                    row_parent[index] = previous_index
        costs.append(row_cost)
        parents.append(row_parent)
    index = int(np.argmin(costs[-1]))
    path = []
    for row_index in range(len(rows) - 1, -1, -1):
        x, confidence = candidates[row_index][index]
        path.append((x, rows[row_index], confidence))
        index = int(parents[row_index][index])
        if row_index and index < 0:
            return []
    return path[::-1]


def candidate_centerlines(road_probability, center_probability,
                          peak_threshold=CENTERLINE_THRESHOLD,
                          max_peaks_per_row=3, row_step=3, road_floor=0.25):
    """Extract at most two continuous path candidates without skeletonization."""
    score = center_probability * (
        road_floor + (1.0 - road_floor) * road_probability)
    rows = list(range(score.shape[0] - 2,
                      max(score.shape[0] // 5, 1), -row_step))
    valid_rows = []
    candidates = []
    for row in rows:
        peaks = _row_peaks(score[row], peak_threshold, max_peaks_per_row)
        if peaks:
            valid_rows.append(row)
            candidates.append(peaks)
    first = _best_path(valid_rows, candidates)
    paths = [first] if first else []
    if first:
        suppressed = [list(items) for items in candidates]
        for row_index, point in enumerate(first):
            suppressed[row_index] = [
                item for item in suppressed[row_index]
                if abs(item[0] - point[0]) >= 4]
        if all(suppressed):
            second = _best_path(valid_rows, suppressed)
            if second:
                paths.append(second)
    return [[(int(x * PIXEL_STRIDE), int(y * PIXEL_STRIDE), float(confidence))
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
            "primary": paths[0] if paths else [],
            "stride": PIXEL_STRIDE,
        },
        "topology": decode_topology(topology_logits),
    }


def _draw_result(image, result):
    for detection in result["detections"]:
        left, top, right, bottom = [int(value) for value in detection["bbox"]]
        cv2.rectangle(image, (left, top), (right, bottom), (0, 220, 0), 2)
        cv2.putText(
            image, "{} {:.2f}".format(detection["label"], detection["score"]),
            (left, max(18, top - 6)), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 0, 255), 2)

    colors = ((0, 255, 255), (255, 128, 0))
    scale_x = float(image.shape[1]) / MODEL_WIDTH
    scale_y = float(image.shape[0]) / MODEL_HEIGHT
    for path_index, path in enumerate(result["centerline"]["paths"]):
        points = np.asarray(
            [[int(x * scale_x), int(y * scale_y)] for x, y, _ in path],
            dtype=np.int32)
        if len(points) >= 2:
            cv2.polylines(image, [points], False, colors[path_index], 2,
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
