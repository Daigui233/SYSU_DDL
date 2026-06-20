#以下代码改自https://github.com/rockchip-linux/rknn-toolkit2/tree/master/examples/onnx/yolov5
from pathlib import Path

import os
import time

import cv2
import numpy as np

OBJ_THRESH, NMS_THRESH = 0.5, 0.45
IMG_SIZE = (640, 640)

_DEFAULT_CLASSES = (
    "Door",
    "SpeedSign",
    "TurnSign",
    "Stone",
    "BeginSign",
    "EndSign",
    "Crosswalk",
    "TrafficLight",
    "Coin",
    "Human",
    "Car",
)

def _load_classes():
    names_path = Path(__file__).resolve().parent / "model" / "coco.names"
    try:
        if names_path.is_file():
            text = names_path.read_text(encoding="utf-8")
            classes = tuple(line.strip() for line in text.splitlines() if line.strip())
            if classes:
                return classes
    except OSError:
        pass
    return _DEFAULT_CLASSES


CLASSES = _load_classes()
NUM_CLASSES = len(CLASSES)


# 
# def sigmoid(x):
#     return 1 / (1 + np.exp(-x))


def xywh2xyxy(x):
    # Convert [x, y, w, h] to [x1, y1, x2, y2]
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def process(input, mask, anchors):

    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    box_confidence = input[..., 4]
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    box_class_probs = input[..., 5:]

    box_xy = input[..., :2] *2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE/grid_h)

    box_wh = pow(input[..., 2:4] *2, 2)
    box_wh = box_wh * anchors

    return np.concatenate((box_xy, box_wh), axis=-1), box_confidence, box_class_probs

def filter_boxes(boxes, box_confidences, box_class_probs):
    """过滤低置信度框（纯NumPy实现）
    # Arguments
        boxes: ndarray, boxes of objects.
        box_confidences: ndarray, confidences of objects.
        box_class_probs: ndarray, class_probs of objects.

    # Returns
        boxes: ndarray, filtered boxes.
        classes: ndarray, classes for boxes.
        scores: ndarray, scores for boxes.
    """
    box_confidences = box_confidences.reshape(-1)
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[_class_pos]
    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    """NMS算法（纯NumPy实现）
    # Arguments
        boxes: ndarray, boxes of objects.
        scores: ndarray, scores of objects.

    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def dfl(position):
    """分布焦点损失(DFL)解码（纯NumPy实现，移除Torch依赖）"""
    n, c, h, w = position.shape
    p_num = 4  # x,y,w,h四个参数
    mc = c // p_num  # 每个参数的分布数

    # 重塑为 (n, p_num, mc, h, w)
    y = position.reshape(n, p_num, mc, h, w)
    
    # NumPy实现softmax
    exp_y = np.exp(y - np.max(y, axis=2, keepdims=True))  # 数值稳定版softmax
    y_softmax = exp_y / np.sum(exp_y, axis=2, keepdims=True)
    
    # 计算分布加权和（替代Torch的矩阵乘法）
    acc_metrix = np.arange(mc, dtype=np.float32).reshape(1, 1, mc, 1, 1)
    y = np.sum(y_softmax * acc_metrix, axis=2)  # 等效于Torch的sum(2)
    return y


def box_process(position, size_im=IMG_SIZE):
    """边界框解码（纯NumPy实现）"""
    grid_h, grid_w = position.shape[2:4]
    # 生成网格坐标
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w).astype(np.float32)
    row = row.reshape(1, 1, grid_h, grid_w).astype(np.float32)
    grid = np.concatenate((col, row), axis=1)
    
    # 计算步长（适配RK3588多尺度特征图）
    stride = np.array([size_im[1] // grid_h, size_im[0] // grid_w], dtype=np.float32).reshape(1, 2, 1, 1)
    # 

    # DFL解码
    position = dfl(position)
    
    # 计算边界框坐标
    box_xy = grid + 0.5 - position[:, 0:2, :, :]
    box_xy2 = grid + 0.5 + position[:, 2:4, :, :]
    xyxy = np.concatenate((box_xy * stride, box_xy2 * stride), axis=1)

    return xyxy


def _decode_branch_candidates(position, class_output, size_im):
    """Filter cheap class scores before the comparatively expensive DFL decode."""
    position = np.asarray(position)
    class_output = np.asarray(class_output)
    if position.ndim != 4 or class_output.ndim != 4:
        raise ValueError("detection outputs must be NCHW tensors")

    n, channels, grid_h, grid_w = position.shape
    if n != 1 or class_output.shape[0] != 1:
        raise ValueError("only batch size 1 detection outputs are supported")
    if channels % 4 != 0:
        raise ValueError(f"invalid DFL channel count: {channels}")
    if class_output.shape[1] != NUM_CLASSES:
        raise ValueError(
            f"detection model outputs {class_output.shape[1]} classes, "
            f"but {NUM_CLASSES} labels are configured"
        )
    if class_output.shape[2:] != (grid_h, grid_w):
        raise ValueError("detection position/class grid shapes do not match")

    class_flat = class_output[0].reshape(NUM_CLASSES, -1)
    classes = np.argmax(class_flat, axis=0)
    scores = np.max(class_flat, axis=0)
    candidate_indices = np.flatnonzero(scores >= OBJ_THRESH)
    if candidate_indices.size == 0:
        return None, None, None

    distribution_bins = channels // 4
    distribution = position[0].reshape(4, distribution_bins, -1)[:, :, candidate_indices]
    distribution = np.asarray(distribution, dtype=np.float32)
    distribution -= np.max(distribution, axis=1, keepdims=True)
    np.exp(distribution, out=distribution)
    distribution /= np.sum(distribution, axis=1, keepdims=True)
    distances = np.sum(
        distribution * np.arange(distribution_bins, dtype=np.float32)[None, :, None],
        axis=1,
    )

    rows, cols = np.divmod(candidate_indices, grid_w)
    center_x = cols.astype(np.float32) + 0.5
    center_y = rows.astype(np.float32) + 0.5
    stride_x = float(size_im[1] // grid_w)
    stride_y = float(size_im[0] // grid_h)
    boxes = np.stack(
        (
            (center_x - distances[0]) * stride_x,
            (center_y - distances[1]) * stride_y,
            (center_x + distances[2]) * stride_x,
            (center_y + distances[3]) * stride_y,
        ),
        axis=1,
    )
    return boxes, classes[candidate_indices], scores[candidate_indices]


def post_process(input_data, img_shape=(640, 640)):
    """后处理（纯NumPy实现，适配RK3588输出格式）"""
    boxes, scores, classes = [], [], []
    defualt_branch = 3
    pair_per_branch = len(input_data) // defualt_branch

    # The class tensors are much cheaper to inspect than the 68-channel DFL
    # tensors. Real frames normally contain only a handful of candidates, so
    # reject background locations before DFL instead of decoding all 8400.
    for i in range(defualt_branch):
        branch_boxes, branch_classes, branch_scores = _decode_branch_candidates(
            input_data[pair_per_branch * i],
            input_data[pair_per_branch * i + 1],
            img_shape,
        )
        if branch_boxes is not None:
            boxes.append(branch_boxes)
            classes.append(branch_classes)
            scores.append(branch_scores)

    # 合并所有尺度的结果
    if not boxes:
        return None, None, None
    boxes = np.concatenate(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    # Candidates are already thresholded above; preserve the original NMS
    # ordering and output contract (boxes, classes, scores).
    # 按类别进行NMS
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c_cls = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c_cls[keep])
            nscores.append(s[keep])

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)
    # print(boxes)
    # 合成一个结果

    return boxes, classes, scores


def _class_name(class_id):
    idx = int(class_id)
    if 0 <= idx < len(CLASSES):
        return CLASSES[idx]
    return f"cls_{idx}"


def draw(image, boxes, scores, classes):
    if boxes is None or scores is None or classes is None:
        return image
    if len(boxes) == 0:
        return image

    h, w = image.shape[:2]
    for box, score, cl in zip(boxes, scores, classes):
        left, top, right, bottom = [int(_b) for _b in box]
        left = max(0, min(w - 1, left))
        right = max(0, min(w - 1, right))
        top = max(0, min(h - 1, top))
        bottom = max(0, min(h - 1, bottom))
        if right <= left or bottom <= top:
            continue
        cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(
            image,
            f"{_class_name(cl)} {float(score):.2f}",
            (left, max(20, top - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
    return image


def myFunc(rknn_lite, img_orin):
    img_rgb_orin = cv2.cvtColor(img_orin, cv2.COLOR_BGR2RGB)
    size_orin = img_orin.shape[:2]
    img_rgb_in = cv2.resize(img_rgb_orin, IMG_SIZE)
    img_rgb_in = np.expand_dims(img_rgb_in, 0)
    npu_started = time.perf_counter()
    outputs = rknn_lite.inference(inputs=[img_rgb_in])
    rknn_lite._last_inference_ms = (time.perf_counter() - npu_started) * 1000.0
    boxes, classes, scores = post_process(outputs, size_orin)
    return boxes, scores, classes
