import math


# ===== 比赛事件状态机参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# 本模块只使用图像检测框和框内颜色识别结果，不使用 AR 坐标。
RACE_STATE_DEFAULTS = {
    # 比赛总圈数；当前规则为 3 圈。
    "TOTAL_LAPS": 3,

    # 检测结果最大可用年龄，单位 s；过旧检测不参与红绿灯/门/标志状态判断。
    "MAX_DETECTION_AGE": 1.00,

    # Door 置信度和面积阈值；面积比例基于整幅 640x480 图像，0.002 约等于 614 px。
    "DOOR_MIN_SCORE": 0.35,
    "DOOR_MIN_AREA_RATIO": 0.0020,

    # Door bbox 底边进入画面该比例以下时，认为接近起终点门，可触发一次过门事件。
    "DOOR_CROSS_MIN_BOTTOM_RATIO": 0.42,

    # 过门计数冷却时间，避免 Door 连续存在时每帧重复加圈。
    "DOOR_CROSS_COOLDOWN": 1.20,

    # BeginSign/EndSign 的基础置信度和面积阈值。
    "SIGN_MIN_SCORE": 0.35,
    "SIGN_MIN_AREA_RATIO": 0.00025,

    # 判断标志是否在 Door 下方/内部时，允许 Door bbox 横向扩展的比例。
    "SIGN_UNDER_DOOR_X_MARGIN_RATIO": 0.30,

    # BeginSign 曾经在 Door 下方出现后，这段时间内过 Door 都可认为是比赛开始。
    "BEGIN_SIGN_HOLD_TTL": 1.00,

    # 看到 EndSign 后不立刻停车；EndSign 消失超过该时间后进入 ENDSIGN_STOP。
    "ENDSIGN_LOST_STOP_DELAY": 0.45,

    # 若打开，则只有已经通过 BeginSign 开始比赛后，EndSign 才能触发终点停车。
    # 先关闭，避免 BeginSign 漏检导致第三圈无法停车。
    "ENDSIGN_REQUIRE_RACE_STARTED": False,

    # TrafficLight 置信度和面积阈值；面积比例 0.00025 约等于 77 px。
    "TRAFFIC_LIGHT_MIN_SCORE": 0.35,
    "TRAFFIC_LIGHT_MIN_AREA_RATIO": 0.00025,

    # 红绿灯有效触发区。底边至少进入画面高度 26% 且横向不能太偏，避免远处/侧边灯抢占。
    "TRAFFIC_LIGHT_MIN_BOTTOM_RATIO": 0.26,
    "TRAFFIC_LIGHT_CENTER_LATERAL_RATIO": 0.42,

    # OpenCV 框内红/绿识别置信度下限。
    "TRAFFIC_LIGHT_MIN_COLOR_CONFIDENCE": 0.018,

    # 红灯最近出现后的保持时间，防止红灯检测短时闪断；绿灯有效出现会立刻解除红灯保持。
    "TRAFFIC_LIGHT_RED_HOLD_TTL": 0.45,
}


def _finite_float(value, default=None):
    try:
        result = float(value)
    except Exception:
        return default
    if not math.isfinite(result):
        return default
    return result


def _detection_category(det):
    return str((det or {}).get("category") or (det or {}).get("label") or "").strip().lower()


def _data_age(data, now):
    age = _finite_float((data or {}).get("age"))
    if age is None:
        timestamp = _finite_float((data or {}).get("timestamp"))
        if timestamp is not None:
            age = max(0.0, float(now) - timestamp)
    return age


def _fresh_detections(detections, now, max_age):
    result = []
    for det in detections or []:
        age = _data_age(det, now)
        if age is not None and age > max_age:
            continue
        result.append(det)
    return result


def _geometry(det, frame_w, frame_h):
    bbox = (det or {}).get("bbox") or [0.0, 0.0, 0.0, 0.0]
    center = (det or {}).get("center") or [None, None]
    try:
        left, top, right, bottom = [float(v) for v in bbox]
    except Exception:
        return None
    if right <= left or bottom <= top:
        return None
    cx = _finite_float(center[0], (left + right) * 0.5)
    cy = _finite_float(center[1], (top + bottom) * 0.5)
    if cx is None or cy is None:
        return None
    area_ratio = _finite_float((det or {}).get("area_ratio"), 0.0)
    return {
        "left": left,
        "top": top,
        "right": right,
        "bottom": min(max(0.0, bottom), float(frame_h - 1)),
        "cx": cx,
        "cy": cy,
        "width": right - left,
        "height": bottom - top,
        "area_ratio": area_ratio,
        "bottom_ratio": bottom / float(max(1, frame_h)),
        "lateral_ratio": abs(cx - frame_w * 0.5) / float(max(1, frame_w)),
    }


def _best_detection(detections, category, min_score, min_area_ratio):
    best = None
    best_rank = -1.0
    for det in detections:
        if _detection_category(det) != category:
            continue
        score = _finite_float(det.get("score"), 0.0)
        area_ratio = _finite_float(det.get("area_ratio"), 0.0)
        if score < min_score or area_ratio < min_area_ratio:
            continue
        rank = score + area_ratio * 10.0
        if rank > best_rank:
            best = det
            best_rank = rank
    return best


def _sign_under_door(sign_det, door_det, frame_w, frame_h, params):
    sign_geom = _geometry(sign_det, frame_w, frame_h)
    door_geom = _geometry(door_det, frame_w, frame_h)
    if sign_geom is None or door_geom is None:
        return False

    margin = door_geom["width"] * float(params["SIGN_UNDER_DOOR_X_MARGIN_RATIO"])
    x_ok = (door_geom["left"] - margin) <= sign_geom["cx"] <= (door_geom["right"] + margin)
    y_ok = sign_geom["cy"] >= door_geom["top"] - door_geom["height"] * 0.15
    return bool(x_ok and y_ok)


class RaceStateMachine:
    """Tracks image-space race events: Door, BeginSign, EndSign and traffic light."""

    def __init__(self, params=None):
        self.params = dict(RACE_STATE_DEFAULTS)
        if params:
            self.params.update(params)

        self.race_started = False
        self.completed_laps = 0
        self.current_lap = 0
        self.door_cross_ready = True
        self.door_cooldown_until = 0.0
        self.last_begin_seen_ts = None
        self.finish_armed = False
        self.finish_stop = False
        self.last_end_seen_ts = None
        self.last_red_seen_ts = None

    def update(self, perception, now):
        now = float(now)
        frame_shape = (perception or {}).get("frame_shape") or [0, 0]
        frame_h = int(frame_shape[0]) if len(frame_shape) >= 1 else 0
        frame_w = int(frame_shape[1]) if len(frame_shape) >= 2 else 0
        frame_w = max(frame_w, 1)
        frame_h = max(frame_h, 1)
        detections = _fresh_detections(
            (perception or {}).get("detections") or [],
            now,
            self.params["MAX_DETECTION_AGE"],
        )

        door = _best_detection(
            detections,
            "door",
            self.params["DOOR_MIN_SCORE"],
            self.params["DOOR_MIN_AREA_RATIO"],
        )
        begin = _best_detection(
            detections,
            "begin_sign",
            self.params["SIGN_MIN_SCORE"],
            self.params["SIGN_MIN_AREA_RATIO"],
        )
        end = _best_detection(
            detections,
            "end_sign",
            self.params["SIGN_MIN_SCORE"],
            self.params["SIGN_MIN_AREA_RATIO"],
        )
        traffic_light = _best_detection(
            detections,
            "traffic_light",
            self.params["TRAFFIC_LIGHT_MIN_SCORE"],
            self.params["TRAFFIC_LIGHT_MIN_AREA_RATIO"],
        )

        door_geom = _geometry(door, frame_w, frame_h) if door is not None else None
        door_active = bool(
            door_geom is not None
            and door_geom["bottom_ratio"] >= float(self.params["DOOR_CROSS_MIN_BOTTOM_RATIO"])
        )
        door_cross_event = self._update_door_cross(door_active, now)

        begin_under_door = _sign_under_door(begin, door, frame_w, frame_h, self.params)
        end_under_door = _sign_under_door(end, door, frame_w, frame_h, self.params)
        if begin_under_door:
            self.last_begin_seen_ts = now
        if end_under_door:
            self.finish_armed = True
            self.last_end_seen_ts = now

        lap_event = self._update_laps(door_cross_event, now)
        traffic_state, traffic_stop = self._update_traffic_light(traffic_light, frame_w, frame_h, now)
        if self._should_finish_stop(now):
            self.finish_stop = True

        return {
            "race_started": bool(self.race_started),
            "completed_laps": int(self.completed_laps),
            "current_lap": int(self.current_lap),
            "total_laps": int(self.params["TOTAL_LAPS"]),
            "door_visible": bool(door is not None),
            "door_active": bool(door_active),
            "door_cross_event": bool(door_cross_event),
            "lap_event": lap_event,
            "begin_under_door": bool(begin_under_door),
            "end_under_door": bool(end_under_door),
            "finish_armed": bool(self.finish_armed),
            "finish_stop": bool(self.finish_stop),
            "traffic_light_visible": bool(traffic_light is not None),
            "traffic_light_state": traffic_state,
            "traffic_light_stop": bool(traffic_stop),
        }

    def _update_door_cross(self, door_active, now):
        if not door_active:
            if now >= self.door_cooldown_until:
                self.door_cross_ready = True
            return False

        if self.door_cross_ready and now >= self.door_cooldown_until:
            self.door_cross_ready = False
            self.door_cooldown_until = now + float(self.params["DOOR_CROSS_COOLDOWN"])
            return True
        return False

    def _update_laps(self, door_cross_event, now):
        if not door_cross_event:
            return None

        begin_recent = (
            self.last_begin_seen_ts is not None
            and now - self.last_begin_seen_ts <= float(self.params["BEGIN_SIGN_HOLD_TTL"])
        )
        if not self.race_started and begin_recent:
            self.race_started = True
            self.completed_laps = 0
            self.current_lap = 1
            return "race_start"

        if self.race_started:
            total_laps = int(self.params["TOTAL_LAPS"])
            self.completed_laps = min(total_laps, self.completed_laps + 1)
            self.current_lap = min(total_laps, self.completed_laps + 1)
            return f"lap_{self.completed_laps}"
        return None

    def _update_traffic_light(self, traffic_light, frame_w, frame_h, now):
        state = "none"
        if traffic_light is not None:
            geom = _geometry(traffic_light, frame_w, frame_h)
            color_state = str(traffic_light.get("traffic_light_state") or "unknown").lower()
            color_conf = _finite_float(traffic_light.get("traffic_light_confidence"), 0.0)
            effective = bool(
                geom is not None
                and geom["bottom_ratio"] >= float(self.params["TRAFFIC_LIGHT_MIN_BOTTOM_RATIO"])
                and geom["lateral_ratio"] <= float(self.params["TRAFFIC_LIGHT_CENTER_LATERAL_RATIO"])
                and color_conf >= float(self.params["TRAFFIC_LIGHT_MIN_COLOR_CONFIDENCE"])
            )
            state = color_state if effective else "far_or_unknown"
            if effective and color_state == "red":
                self.last_red_seen_ts = now
            elif effective and color_state == "green":
                self.last_red_seen_ts = None

        stop = bool(
            self.last_red_seen_ts is not None
            and now - self.last_red_seen_ts <= float(self.params["TRAFFIC_LIGHT_RED_HOLD_TTL"])
        )
        return state, stop

    def _should_finish_stop(self, now):
        if self.finish_stop or not self.finish_armed or self.last_end_seen_ts is None:
            return self.finish_stop
        if self.params["ENDSIGN_REQUIRE_RACE_STARTED"] and not self.race_started:
            return False
        return (now - self.last_end_seen_ts) >= float(self.params["ENDSIGN_LOST_STOP_DELAY"])
