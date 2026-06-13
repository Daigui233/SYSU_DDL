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

    # EndSign 误识别会直接导致终点停车，所以比 BeginSign 更严格。
    # confirm_seconds 表示需要连续稳定看到多久才允许进入终点停车准备。
    "ENDSIGN_MIN_SCORE": 0.60,
    "ENDSIGN_MIN_AREA_RATIO": 0.00035,
    "ENDSIGN_CONFIRM_SECONDS": 0.30,
    "ENDSIGN_CONFIRM_GAP": 0.20,

    # 判断标志是否在 Door 下方/内部时，允许 Door bbox 横向扩展的比例。
    "SIGN_UNDER_DOOR_X_MARGIN_RATIO": 0.30,

    # BeginSign 曾经在 Door 下方出现后，这段时间内过 Door 都可认为是比赛开始。
    "BEGIN_SIGN_HOLD_TTL": 1.00,

    # 看到 EndSign 后不立刻停车；EndSign 消失超过该时间后进入 ENDSIGN_STOP。
    "ENDSIGN_LOST_STOP_DELAY": 0.45,

    # EndSign 终点停车启用；误识别时可在完全停车后手动搬动车体 1m 自动解除锁存。
    "ENDSIGN_STOP_ENABLED": True,

    # ENDSIGN_STOP 后，车完全停住再记录定位锚点；若 x/z 平面位移超过该值，视为手动拿走车并解除误识别停车。
    "ENDSIGN_MANUAL_RESET_DISTANCE_M": 1.0,
    "ENDSIGN_STOPPED_SPEED_THRESHOLD": 0.03,
    "ENDSIGN_STOP_SETTLE_SECONDS": 0.80,
    "ENDSIGN_FEEDBACK_MAX_AGE": 1.0,
    "ENDSIGN_POSE_MAX_AGE": 1.0,
    "ENDSIGN_MANUAL_RESET_COOLDOWN": 1.0,

    # EndSign 必须在比赛已经开始、且已经进入最后一圈后才允许触发。
    "ENDSIGN_REQUIRE_RACE_STARTED": True,
    "ENDSIGN_REQUIRE_FINAL_LAP": True,

    # TrafficLight 置信度和面积阈值；面积比例 0.00025 约等于 77 px。
    "TRAFFIC_LIGHT_MIN_SCORE": 0.35,
    "TRAFFIC_LIGHT_MIN_AREA_RATIO": 0.00025,

    # 红绿灯有效触发区。底边至少进入画面高度 26% 且横向不能太偏，避免远处/侧边灯抢占。
    "TRAFFIC_LIGHT_MIN_BOTTOM_RATIO": 0.26,
    "TRAFFIC_LIGHT_CENTER_LATERAL_RATIO": 0.42,

    # 红灯/黄灯只有进入更近的停车区才硬停；远处只记录为 *_far，不抢占近处避障。
    "TRAFFIC_LIGHT_STOP_MIN_BOTTOM_RATIO": 0.50,
    "TRAFFIC_LIGHT_STOP_CENTER_LATERAL_RATIO": 0.34,

    # OpenCV 框内红/黄/绿识别置信度下限。
    "TRAFFIC_LIGHT_MIN_COLOR_CONFIDENCE": 0.018,

    # 红灯/黄灯最近停车后的保持时间，防止灯色检测短时闪断；有效绿灯会立刻解除保持。
    "TRAFFIC_LIGHT_RED_HOLD_TTL": 0.45,

    # 红灯/黄灯进入近处停车区后仍需连续确认，避免单帧误识别导致硬停。
    "TRAFFIC_LIGHT_RED_CONFIRM_SECONDS": 0.20,
    "TRAFFIC_LIGHT_RED_CONFIRM_GAP": 0.30,
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
        self.end_confirm_first_ts = None
        self.last_end_seen_ts = None
        self.end_stop_enter_ts = None
        self.end_stop_pose_anchor = None
        self.end_stop_pose_anchor_ts = None
        self.end_stop_reset_armed = False
        self.end_stop_reset_distance = 0.0
        self.end_stop_reset_reason = "idle"
        self.end_stop_reset_count = 0
        self.end_stop_reset_cooldown_until = 0.0
        self.last_red_seen_ts = None
        self.red_confirm_first_ts = None
        self.last_red_stop_ts = None

    def update(self, perception, now, pose_packet=None, car_feedback=None):
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
            self.params["ENDSIGN_MIN_SCORE"],
            self.params["ENDSIGN_MIN_AREA_RATIO"],
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

        lap_event = self._update_laps(door_cross_event, now)
        end_stop_enabled = bool(self.params["ENDSIGN_STOP_ENABLED"])
        end_sign_allowed = self._end_sign_allowed()
        end_sign_can_stop = bool(end_stop_enabled and end_sign_allowed)
        end_confirm_age = self._update_end_sign(end_under_door, end_sign_can_stop, now)
        traffic_state, traffic_stop, traffic_stop_zone, red_confirm_age = self._update_traffic_light(
            traffic_light,
            frame_w,
            frame_h,
            now,
        )
        if self._should_finish_stop(now):
            if not self.finish_stop:
                self._start_finish_stop(now)
            self.finish_stop = True
        if self.finish_stop:
            self._update_end_stop_manual_reset(pose_packet, car_feedback, now)
        else:
            self._clear_end_stop_anchor("not_stopping")

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
            "end_sign_allowed": bool(end_sign_allowed),
            "end_sign_stop_enabled": bool(end_stop_enabled),
            "end_confirm_age": float(end_confirm_age),
            "finish_armed": bool(self.finish_armed),
            "finish_stop": bool(self.finish_stop),
            "end_stop_reset_armed": bool(self.end_stop_reset_armed),
            "end_stop_reset_distance": float(self.end_stop_reset_distance),
            "end_stop_reset_reason": self.end_stop_reset_reason,
            "end_stop_reset_count": int(self.end_stop_reset_count),
            "traffic_light_visible": bool(traffic_light is not None),
            "traffic_light_state": traffic_state,
            "traffic_light_stop_zone": bool(traffic_stop_zone),
            "traffic_light_red_confirm_age": float(red_confirm_age),
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

    def _end_sign_allowed(self):
        if self.params["ENDSIGN_REQUIRE_RACE_STARTED"] and not self.race_started:
            return False
        if self.params["ENDSIGN_REQUIRE_FINAL_LAP"]:
            total_laps = max(1, int(self.params["TOTAL_LAPS"]))
            if self.completed_laps < max(0, total_laps - 1):
                return False
        return True

    def _update_end_sign(self, end_under_door, end_sign_allowed, now):
        gap = float(self.params["ENDSIGN_CONFIRM_GAP"])
        if now < float(self.end_stop_reset_cooldown_until):
            self.end_confirm_first_ts = None
            self.last_end_seen_ts = None
            return 0.0
        if not end_under_door or not end_sign_allowed:
            if self.last_end_seen_ts is None or now - self.last_end_seen_ts > gap:
                self.end_confirm_first_ts = None
            return 0.0

        if (
            self.end_confirm_first_ts is None
            or self.last_end_seen_ts is None
            or now - self.last_end_seen_ts > gap
        ):
            self.end_confirm_first_ts = now

        self.last_end_seen_ts = now
        confirm_age = max(0.0, now - self.end_confirm_first_ts)
        if confirm_age >= float(self.params["ENDSIGN_CONFIRM_SECONDS"]):
            self.finish_armed = True
        return confirm_age

    def _start_finish_stop(self, now):
        self.end_stop_enter_ts = float(now)
        self._clear_end_stop_anchor("waiting_stopped")

    def _clear_end_stop_anchor(self, reason):
        self.end_stop_pose_anchor = None
        self.end_stop_pose_anchor_ts = None
        self.end_stop_reset_armed = False
        self.end_stop_reset_distance = 0.0
        self.end_stop_reset_reason = str(reason)

    def _pose_ground_xy(self, pose_packet, now):
        if not isinstance(pose_packet, dict):
            return None
        timestamp = _finite_float(pose_packet.get("timestamp"), None)
        max_age = float(self.params["ENDSIGN_POSE_MAX_AGE"])
        if timestamp is not None and float(now) - timestamp > max_age:
            return None
        pos = pose_packet.get("pos")
        if not isinstance(pos, (list, tuple)) or len(pos) < 2:
            return None
        x = _finite_float(pos[0], None)
        z_index = 2 if len(pos) >= 3 else 1
        z = _finite_float(pos[z_index], None)
        if x is None or z is None:
            return None
        return (float(x), float(z))

    def _car_feedback_stopped(self, car_feedback, now):
        if isinstance(car_feedback, dict) and car_feedback.get("online"):
            age = _finite_float(car_feedback.get("age"), None)
            speed = _finite_float(car_feedback.get("actual_speed"), None)
            if age is not None and age <= float(self.params["ENDSIGN_FEEDBACK_MAX_AGE"]) and speed is not None:
                return abs(speed) <= float(self.params["ENDSIGN_STOPPED_SPEED_THRESHOLD"])

        if self.end_stop_enter_ts is None:
            return False
        return float(now) - float(self.end_stop_enter_ts) >= float(self.params["ENDSIGN_STOP_SETTLE_SECONDS"])

    def _reset_finish_stop_after_manual_move(self, now, distance):
        self.finish_stop = False
        self.finish_armed = False
        self.end_confirm_first_ts = None
        self.last_end_seen_ts = None
        self.end_stop_pose_anchor = None
        self.end_stop_pose_anchor_ts = None
        self.end_stop_reset_armed = False
        self.end_stop_reset_distance = float(distance)
        self.end_stop_reset_reason = "manual_pose_move_reset"
        self.end_stop_reset_count += 1
        self.end_stop_reset_cooldown_until = float(now) + float(self.params["ENDSIGN_MANUAL_RESET_COOLDOWN"])

    def _update_end_stop_manual_reset(self, pose_packet, car_feedback, now):
        pose_xy = self._pose_ground_xy(pose_packet, now)
        if pose_xy is None:
            self.end_stop_reset_reason = "waiting_pose"
            return False

        if not self._car_feedback_stopped(car_feedback, now):
            self.end_stop_reset_reason = "waiting_stopped"
            return False

        if self.end_stop_pose_anchor is None:
            self.end_stop_pose_anchor = pose_xy
            self.end_stop_pose_anchor_ts = float(now)
            self.end_stop_reset_armed = True
            self.end_stop_reset_distance = 0.0
            self.end_stop_reset_reason = "armed"
            return False

        dx = float(pose_xy[0]) - float(self.end_stop_pose_anchor[0])
        dz = float(pose_xy[1]) - float(self.end_stop_pose_anchor[1])
        distance = math.hypot(dx, dz)
        self.end_stop_reset_distance = float(distance)
        self.end_stop_reset_armed = True
        threshold = float(self.params["ENDSIGN_MANUAL_RESET_DISTANCE_M"])
        if distance >= threshold:
            self._reset_finish_stop_after_manual_move(now, distance)
            return True
        self.end_stop_reset_reason = "holding"
        return False

    def _update_traffic_light(self, traffic_light, frame_w, frame_h, now):
        state = "none"
        stop_zone = False
        red_confirm_age = 0.0
        if traffic_light is not None:
            geom = _geometry(traffic_light, frame_w, frame_h)
            color_state = str(traffic_light.get("traffic_light_state") or "unknown").lower()
            color_conf = _finite_float(traffic_light.get("traffic_light_confidence"), 0.0)
            seen_effective = bool(
                geom is not None
                and geom["bottom_ratio"] >= float(self.params["TRAFFIC_LIGHT_MIN_BOTTOM_RATIO"])
                and geom["lateral_ratio"] <= float(self.params["TRAFFIC_LIGHT_CENTER_LATERAL_RATIO"])
                and color_conf >= float(self.params["TRAFFIC_LIGHT_MIN_COLOR_CONFIDENCE"])
            )
            stop_zone = bool(
                seen_effective
                and geom["bottom_ratio"] >= float(self.params["TRAFFIC_LIGHT_STOP_MIN_BOTTOM_RATIO"])
                and geom["lateral_ratio"] <= float(self.params["TRAFFIC_LIGHT_STOP_CENTER_LATERAL_RATIO"])
            )
            if seen_effective and color_state == "green":
                state = "green"
                self.last_red_seen_ts = None
                self.red_confirm_first_ts = None
                self.last_red_stop_ts = None
            elif seen_effective and color_state in ("red", "yellow"):
                state = f"{color_state}_far"
                if stop_zone:
                    gap = float(self.params["TRAFFIC_LIGHT_RED_CONFIRM_GAP"])
                    if (
                        self.red_confirm_first_ts is None
                        or self.last_red_seen_ts is None
                        or now - self.last_red_seen_ts > gap
                    ):
                        self.red_confirm_first_ts = now
                    self.last_red_seen_ts = now
                    red_confirm_age = max(0.0, now - self.red_confirm_first_ts)
                    if red_confirm_age >= float(self.params["TRAFFIC_LIGHT_RED_CONFIRM_SECONDS"]):
                        state = f"{color_state}_stop_zone"
                        self.last_red_stop_ts = now
                    else:
                        state = f"{color_state}_confirming"
            elif seen_effective:
                state = color_state
            else:
                state = "far_or_unknown"
        else:
            if (
                self.last_red_seen_ts is None
                or now - self.last_red_seen_ts > float(self.params["TRAFFIC_LIGHT_RED_CONFIRM_GAP"])
            ):
                self.red_confirm_first_ts = None

        stop = bool(
            self.last_red_stop_ts is not None
            and now - self.last_red_stop_ts <= float(self.params["TRAFFIC_LIGHT_RED_HOLD_TTL"])
        )
        return state, stop, stop_zone, red_confirm_age

    def _should_finish_stop(self, now):
        if not self.params["ENDSIGN_STOP_ENABLED"]:
            self.finish_armed = False
            self.finish_stop = False
            self._clear_end_stop_anchor("disabled")
            return False
        if self.finish_stop or not self.finish_armed or self.last_end_seen_ts is None:
            return self.finish_stop
        if self.params["ENDSIGN_REQUIRE_RACE_STARTED"] and not self.race_started:
            return False
        if self.params["ENDSIGN_REQUIRE_FINAL_LAP"]:
            total_laps = max(1, int(self.params["TOTAL_LAPS"]))
            if self.completed_laps < max(0, total_laps - 1):
                return False
        return (now - self.last_end_seen_ts) >= float(self.params["ENDSIGN_LOST_STOP_DELAY"])
