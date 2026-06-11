import cv2
import numpy as np


# ===== 红绿灯颜色识别参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# 该模块只在检测模型给出的 TrafficLight bbox 内做颜色判断，不在整幅图里找灯。
TRAFFIC_LIGHT_COLOR_DEFAULTS = {
    # bbox 向内收缩比例；红绿灯框边缘常包含外壳/背景，收缩可减少误判。
    "ROI_INSET_RATIO": 0.08,

    # HSV 中参与颜色统计的最小饱和度/亮度，过滤灰白背景和暗外壳。
    "MIN_SATURATION": 70,
    "MIN_VALUE": 80,

    # OpenCV HSV 的 H 范围为 0..179；红色跨 0，需要两段阈值。
    "RED_HUE_LOW_MAX": 12,
    "RED_HUE_HIGH_MIN": 165,

    # 绿色阈值范围；如果现场灯偏青/偏黄，优先微调这两个值。
    "GREEN_HUE_MIN": 35,
    "GREEN_HUE_MAX": 92,

    # 红/绿有效像素占 ROI 面积的最小比例，太小视为 unknown。
    "MIN_COLOR_RATIO": 0.018,

    # 红/绿有效像素数量下限，避免小框或噪点触发。
    "MIN_COLOR_PIXELS": 8,

    # 主导颜色至少要比另一种颜色多这个倍数，减少红绿同时有噪声时的跳变。
    "DOMINANCE_RATIO": 1.25,
}


def _clamp_int(value, low, high):
    return max(int(low), min(int(high), int(value)))


def classify_traffic_light_color(frame, bbox, params=None):
    """Classify red/green inside a detector-provided TrafficLight bbox."""
    params = dict(TRAFFIC_LIGHT_COLOR_DEFAULTS if params is None else params)
    if frame is None or not hasattr(frame, "shape") or bbox is None:
        return {
            "traffic_light_state": "unknown",
            "traffic_light_confidence": 0.0,
            "traffic_light_red_ratio": 0.0,
            "traffic_light_green_ratio": 0.0,
        }

    h, w = frame.shape[:2]
    try:
        left, top, right, bottom = [float(v) for v in bbox]
    except Exception:
        left, top, right, bottom = 0.0, 0.0, 0.0, 0.0

    inset = max(0.0, float(params["ROI_INSET_RATIO"]))
    box_w = max(0.0, right - left)
    box_h = max(0.0, bottom - top)
    x0 = _clamp_int(round(left + box_w * inset), 0, w - 1)
    x1 = _clamp_int(round(right - box_w * inset), x0 + 1, w)
    y0 = _clamp_int(round(top + box_h * inset), 0, h - 1)
    y1 = _clamp_int(round(bottom - box_h * inset), y0 + 1, h)
    roi = frame[y0:y1, x0:x1]
    if roi.size == 0:
        area = 1
        red_count = 0
        green_count = 0
    else:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        valid = (sat >= int(params["MIN_SATURATION"])) & (val >= int(params["MIN_VALUE"]))
        red_mask = valid & (
            (hue <= int(params["RED_HUE_LOW_MAX"]))
            | (hue >= int(params["RED_HUE_HIGH_MIN"]))
        )
        green_mask = valid & (
            (hue >= int(params["GREEN_HUE_MIN"]))
            & (hue <= int(params["GREEN_HUE_MAX"]))
        )
        area = max(1, int(roi.shape[0] * roi.shape[1]))
        red_count = int(np.count_nonzero(red_mask))
        green_count = int(np.count_nonzero(green_mask))

    red_ratio = float(red_count) / float(area)
    green_ratio = float(green_count) / float(area)
    min_pixels = int(params["MIN_COLOR_PIXELS"])
    min_ratio = float(params["MIN_COLOR_RATIO"])
    dominance = float(params["DOMINANCE_RATIO"])

    state = "unknown"
    confidence = max(red_ratio, green_ratio)
    if red_count >= min_pixels and red_ratio >= min_ratio and red_ratio >= green_ratio * dominance:
        state = "red"
        confidence = red_ratio
    elif green_count >= min_pixels and green_ratio >= min_ratio and green_ratio >= red_ratio * dominance:
        state = "green"
        confidence = green_ratio

    return {
        "traffic_light_state": state,
        "traffic_light_confidence": float(confidence),
        "traffic_light_red_ratio": float(red_ratio),
        "traffic_light_green_ratio": float(green_ratio),
    }
