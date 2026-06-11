import math
from dataclasses import dataclass, field

from control_race_state_machine import RaceStateMachine
from control_states import PlannerMode, TaskState


# ===== 调车参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# 不同任务状态下发给 TC264D 的 target_speed，单位 m/s。
# 当前不使用环境变量覆盖，避免源码默认值和实际运行值不一致。
TASK_SPEED_DEFAULTS = {
    # 默认视觉巡线速度；当前新车模低速调车先统一 0.05 m/s。
    TaskState.NORMAL_TRACK: 0.05,

    # 静态车障绕行速度；当前先与巡线统一，后续实车再细分。
    TaskState.AVOID_CAR: 0.05,

    # 避人速度；当前先与巡线统一，后续实车再细分。
    TaskState.AVOID_HUMAN: 0.05,

    # 石头触发分岔路线选择速度；当前先与巡线统一，后续实车再细分。
    TaskState.AVOID_STONE: 0.05,

    # 靠近金币速度；当前先与巡线统一，后续实车再细分。
    TaskState.COLLECT_GOLD: 0.05,

    # 红灯停车速度，必须保持 0；绿灯不进入该状态，继续正常循迹。
    TaskState.TRAFFIC_LIGHT_STOP: 0.0,

    # 终点停车速度，必须保持 0。
    TaskState.ENDSIGN_STOP: 0.0,

    # 短时丢线恢复速度；当前先与巡线统一，后续实车再细分。
    TaskState.RECOVER_LINE: 0.05,

    # 连续丢线超时后的目标速度，必须保持 0。
    TaskState.LINE_LOSS_SAFE_STOP: 0.0,
}

# 状态保持/失效判断参数，单位 s。状态机统一负责这些上位机状态判断。
TASK_TIMING_DEFAULTS = {
    # human 检测短暂丢失后的保持时间；调大可减少抖动，但会更久停留在 AVOID_HUMAN。
    "HUMAN_HOLD_TTL": 0.8,

    # 中线/感知质量连续无效超过该时间后才 safe stop；短时丢线先 RECOVER_LINE。
    "LINE_LOSS_SAFE_STOP_TIMEOUT": 3.0,
}

# 检测任务规则参数。后续实车调车时优先改这里，不要去 Rule 内部找散落阈值。
TASK_RULE_DEFAULTS = {
    # 通用检测排序中 area_ratio 的权重；调大时更偏向选择面积大的目标。
    "GENERIC_AREA_RANK_GAIN": 1.0,

    # 近处目标风险区：检测框底边进入画面底部这部分区域才参与任务抢占。
    # 0.40 表示只优先处理画面下方 40% 内的目标，远处目标只画框不触发避障。
    "NEAR_ROI_BOTTOM_RATIO": 0.40,

    # 近处目标还需要靠近赛道/图像中线；该值是横向允许范围占画面宽度的比例。
    "NEAR_CENTER_LATERAL_RATIO": 0.32,

    # 横向允许范围会随 bbox 宽度增大，用于适配近处大目标。
    "NEAR_BOX_LATERAL_GAIN": 0.75,

    # 横向允许范围的像素下限，防止小目标时范围过窄。
    "NEAR_MIN_LATERAL_LIMIT_PX": 52.0,

    # 近处候选排序时的类别微弱偏置，只在远近非常接近时起作用。
    "NEAR_HUMAN_TIE_BIAS": 0.030,
    "NEAR_STONE_TIE_BIAS": 0.025,
    "NEAR_CAR_TIE_BIAS": 0.020,
    "NEAR_GOLD_TIE_BIAS": 0.000,

    # human 置信度门限；调高减少误触发，调低更容易提前避人。
    "HUMAN_MIN_SCORE": 0.35,

    # car 置信度门限；调高减少误触发，调低更容易提前避车。
    "CAR_MIN_SCORE": 0.35,

    # car 必须出现在画面前方区域才触发避车；0.35 表示 y 大于画面高度 35%。
    "CAR_FRONT_MIN_Y_RATIO": 0.35,

    # car 距离画面中线的横向允许范围；调大更容易把旁边的车也当成前方障碍。
    "CAR_CENTER_LATERAL_RATIO": 0.24,

    # car 横向允许范围会随 bbox 宽度增大；适配近处大目标。
    "CAR_BOX_LATERAL_GAIN": 0.75,

    # car 横向允许范围的像素下限；防止低分辨率/小目标时范围过窄。
    "CAR_MIN_LATERAL_LIMIT_PX": 48.0,

    # car 目标排序中面积的权重；调大更偏向处理近处/更大的车。
    "CAR_AREA_RANK_GAIN": 6.0,

    # stone 置信度门限；stone 用于触发分岔路线选择，不直接作为普通障碍绕行。
    "STONE_MIN_SCORE": 0.35,

    # stone 最小面积比例；太小通常表示距离远或误检，先不触发路线切换。
    "STONE_MIN_AREA_RATIO": 0.0005,

    # gold 置信度门限；调高减少误收集，调低更容易尝试靠近金币。
    "GOLD_MIN_SCORE": 0.35,

    # gold 最小面积比例；太小通常表示距离远或误检，不值得偏离中线。
    "GOLD_MIN_AREA_RATIO": 0.001,

    # gold 离画面中线的最大横向比例；调大允许捡更偏的金币，但路径代价更高。
    "GOLD_MAX_LATERAL_RATIO": 0.36,

    # gold 目标线与当前中线误差差距上限；调小会只捡顺路金币。
    "GOLD_MAX_TRACK_COST_RATIO": 0.42,
}

# 感知质量参数。状态机先用这些字段判断 segmentation/detections 是否可信。
# age 单位为 s；road_ratio 是语义分割 road mask 占整幅图比例。
PERCEPTION_QUALITY_DEFAULTS = {
    # segmentation 结果最大可用年龄；超过后视为过旧，进入 RECOVER_LINE。
    "MAX_SEGMENTATION_AGE": 0.80,

    # road mask 最小面积比例；太小通常说明没有看到路或分割失败。
    "MIN_ROAD_RATIO": 0.001,

    # road mask 最大面积比例；太大可能是误把大面积背景识别成路。
    "MAX_ROAD_RATIO": 0.60,

    # 是否要求 road_valid 为 True；关掉会更宽松，但更容易相信坏分割。
    "REQUIRE_ROAD_VALID": True,

    # 是否要求 midline_valid 为 True；关掉会允许没有稳定中线时继续正常状态。
    "REQUIRE_MIDLINE_VALID": True,

    # 是否允许使用分割后处理的 HOLD 结果；打开可抗短时抖动，关闭更严格。
    "ALLOW_HELD_SEGMENTATION": True,

    # 可接受的 road_state；通常 OK/HOLD 可用，LOST 不可用。
    "VALID_ROAD_STATES": ("OK", "HOLD"),

    # 可接受的 midline_state；通常 OK/HOLD 可用，LOST 不可用。
    "VALID_MIDLINE_STATES": ("OK", "HOLD"),

    # detection 最大可用年龄；超过后不参与 human/car/gold/stone 任务判断。
    # Door/BeginSign/EndSign/TrafficLight 由 control_race_state_machine.py 单独管理。
    "MAX_DETECTION_AGE": 1.00,
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
    return str(det.get("category") or det.get("label") or "").strip().lower()


def _copy_params(defaults, overrides=None):
    params = dict(defaults)
    if overrides:
        params.update(overrides)
    return params


def _data_age(data, now):
    age = _finite_float(data.get("age"))
    if age is None:
        timestamp = _finite_float(data.get("timestamp"))
        if timestamp is not None:
            age = max(0.0, float(now) - timestamp)
    return age


def _fresh_detections(detections, now, max_age):
    result = []
    for det in detections:
        age = _data_age(det, now)
        if age is not None and age > max_age:
            continue
        result.append(det)
    return result


def _segmentation_quality(segmentation, track_error, now, params):
    reasons = []
    age = _data_age(segmentation, now)
    if age is not None and age > params["MAX_SEGMENTATION_AGE"]:
        reasons.append("seg_stale")

    line_valid = bool(segmentation.get("line_valid") and track_error is not None)
    if not line_valid:
        reasons.append("line_invalid")

    road_ratio = _finite_float(segmentation.get("road_ratio"), 0.0)
    if road_ratio < params["MIN_ROAD_RATIO"]:
        reasons.append("road_small")
    if road_ratio > params["MAX_ROAD_RATIO"]:
        reasons.append("road_large")

    road_state = str(segmentation.get("road_state") or "LOST").upper()
    midline_state = str(segmentation.get("midline_state") or "LOST").upper()
    valid_road_states = set(params["VALID_ROAD_STATES"])
    valid_midline_states = set(params["VALID_MIDLINE_STATES"])
    if road_state not in valid_road_states:
        reasons.append(f"road_{road_state.lower()}")
    if midline_state not in valid_midline_states:
        reasons.append(f"midline_{midline_state.lower()}")

    if not params["ALLOW_HELD_SEGMENTATION"] and (road_state == "HOLD" or midline_state == "HOLD"):
        reasons.append("seg_held")
    if params["REQUIRE_ROAD_VALID"] and not bool(segmentation.get("road_valid")):
        reasons.append("road_invalid")
    if params["REQUIRE_MIDLINE_VALID"] and not bool(segmentation.get("midline_valid")):
        reasons.append("midline_invalid")

    return {
        "line_valid": not reasons,
        "reason": "ok" if not reasons else ",".join(reasons),
        "age": age,
        "source": str(segmentation.get("source") or "missing"),
        "road_ratio": road_ratio,
        "road_state": road_state,
        "midline_state": midline_state,
        "raw_line_valid": line_valid,
    }


def _best_detection(detections, category, min_score, area_rank_gain):
    best = None
    best_score = -1.0
    for det in detections:
        if _detection_category(det) != category:
            continue
        score = _finite_float(det.get("score"), 0.0)
        if score < min_score:
            continue
        rank = score + _finite_float(det.get("area_ratio"), 0.0) * area_rank_gain
        if rank > best_score:
            best = det
            best_score = rank
    return best


def _category_min_score(category, params):
    if category == "human":
        return float(params["HUMAN_MIN_SCORE"])
    if category == "car":
        return float(params["CAR_MIN_SCORE"])
    if category == "stone":
        return float(params["STONE_MIN_SCORE"])
    if category == "gold":
        return float(params["GOLD_MIN_SCORE"])
    return 1.0


def _category_tie_bias(category, params):
    if category == "human":
        return float(params["NEAR_HUMAN_TIE_BIAS"])
    if category == "stone":
        return float(params["NEAR_STONE_TIE_BIAS"])
    if category == "car":
        return float(params["NEAR_CAR_TIE_BIAS"])
    if category == "gold":
        return float(params["NEAR_GOLD_TIE_BIAS"])
    return 0.0


def _detection_geometry(det, frame_w, frame_h):
    bbox = det.get("bbox") or [0.0, 0.0, 0.0, 0.0]
    center = det.get("center") or [None, None]
    size = det.get("size") or [0.0, 0.0]
    try:
        left, top, right, bottom = [float(v) for v in bbox]
    except Exception:
        return None
    cx = _finite_float(center[0], (left + right) * 0.5)
    cy = _finite_float(center[1], (top + bottom) * 0.5)
    box_w = _finite_float(size[0], right - left)
    box_h = _finite_float(size[1], bottom - top)
    if cx is None or cy is None or right <= left or bottom <= top:
        return None
    bottom = max(0.0, min(float(frame_h - 1), bottom))
    return {
        "cx": cx,
        "cy": cy,
        "bottom": bottom,
        "box_w": max(0.0, box_w),
        "box_h": max(0.0, box_h),
        "bottom_ratio": bottom / float(max(1, frame_h)),
    }


def _near_detection_candidates(ctx, params):
    frame_w = int(ctx["frame_w"])
    frame_h = int(ctx["frame_h"])
    center_x = float(ctx["center_x"])
    min_bottom_y = frame_h * (1.0 - float(params["NEAR_ROI_BOTTOM_RATIO"]))
    candidates = []

    for det in ctx["detections"]:
        category = _detection_category(det)
        if category not in ("human", "stone", "car", "gold"):
            continue
        score = _finite_float(det.get("score"), 0.0)
        if score < _category_min_score(category, params):
            continue
        geom = _detection_geometry(det, frame_w, frame_h)
        if geom is None:
            continue
        if geom["bottom"] < min_bottom_y:
            continue

        lateral_dist = abs(geom["cx"] - center_x)
        lateral_limit = max(
            frame_w * float(params["NEAR_CENTER_LATERAL_RATIO"]),
            geom["box_w"] * float(params["NEAR_BOX_LATERAL_GAIN"]),
            float(params["NEAR_MIN_LATERAL_LIMIT_PX"]),
        )
        if lateral_dist > lateral_limit:
            continue

        if category == "stone":
            area_ratio = _finite_float(det.get("area_ratio"), 0.0)
            if area_ratio < params["STONE_MIN_AREA_RATIO"]:
                continue

        if category == "gold":
            area_ratio = _finite_float(det.get("area_ratio"), 0.0)
            if area_ratio < params["GOLD_MIN_AREA_RATIO"]:
                continue
            target_error = geom["cx"] - center_x
            track_error = _finite_float(ctx["track_error"], 0.0)
            if abs(target_error - track_error) > frame_w * params["GOLD_MAX_TRACK_COST_RATIO"]:
                continue

        target = dict(det)
        risk = {
            "bottom_ratio": float(geom["bottom_ratio"]),
            "lateral_ratio": float(lateral_dist / float(max(1, frame_w))),
            "near_roi": True,
        }
        target["risk"] = risk
        rank = (
            float(geom["bottom_ratio"]),
            -float(lateral_dist / float(max(1, frame_w))),
            _category_tie_bias(category, params),
            score * 0.01,
        )
        candidates.append({
            "category": category,
            "target": target,
            "rank": rank,
        })

    candidates.sort(key=lambda item: item["rank"], reverse=True)
    return candidates


def _front_center_detection(detections, category, frame_w, frame_h, params):
    center_x = frame_w * 0.5
    best = None
    best_rank = -1.0
    for det in detections:
        if _detection_category(det) != category:
            continue
        score = _finite_float(det.get("score"), 0.0)
        if score < params["CAR_MIN_SCORE"]:
            continue
        center = det.get("center") or [None, None]
        size = det.get("size") or [0.0, 0.0]
        cx = _finite_float(center[0])
        cy = _finite_float(center[1])
        box_w = _finite_float(size[0], 0.0)
        if cx is None or cy is None:
            continue
        if cy < frame_h * params["CAR_FRONT_MIN_Y_RATIO"]:
            continue
        lateral_limit = max(
            frame_w * params["CAR_CENTER_LATERAL_RATIO"],
            box_w * params["CAR_BOX_LATERAL_GAIN"],
            params["CAR_MIN_LATERAL_LIMIT_PX"],
        )
        lateral_dist = abs(cx - center_x)
        if lateral_dist > lateral_limit:
            continue
        rank = (
            score
            + _finite_float(det.get("area_ratio"), 0.0) * params["CAR_AREA_RANK_GAIN"]
            - lateral_dist / max(1.0, frame_w)
        )
        if rank > best_rank:
            best = det
            best_rank = rank
    return best


@dataclass
class TaskDecision:
    task_state: TaskState
    desired_speed: float
    planner_intent: dict = field(default_factory=dict)
    line_valid: bool = True
    line_loss_age: float = 0.0
    reason: str = ""
    perception_quality: dict = field(default_factory=dict)
    race_state: dict = field(default_factory=dict)

    def as_dict(self):
        return {
            "task_state": self.task_state.value,
            "desired_speed": float(self.desired_speed),
            "planner_intent": dict(self.planner_intent),
            "line_valid": bool(self.line_valid),
            "line_loss_age": float(self.line_loss_age),
            "reason": self.reason,
            "perception_quality": dict(self.perception_quality),
            "race_state": dict(self.race_state),
        }


class TaskRule:
    priority = 0

    def evaluate(self, ctx):
        return None


class HumanAvoidRule(TaskRule):
    priority = 100

    def __init__(self, rule_params, hold_ttl=0.8):
        self.rule_params = rule_params
        self.hold_ttl = float(hold_ttl)
        self.min_score = float(rule_params["HUMAN_MIN_SCORE"])
        self.last_seen_ts = None
        self.last_target = None

    def evaluate(self, ctx):
        target = _best_detection(
            ctx["detections"],
            "human",
            self.min_score,
            self.rule_params["GENERIC_AREA_RANK_GAIN"],
        )
        held = False
        if target is not None:
            self.last_seen_ts = ctx["now"]
            self.last_target = dict(target)
        elif self.last_seen_ts is not None and ctx["now"] - self.last_seen_ts <= self.hold_ttl:
            target = dict(self.last_target or {})
            held = True
        else:
            return None

        return TaskDecision(
            task_state=TaskState.AVOID_HUMAN,
            desired_speed=ctx["human_speed"],
            planner_intent={
                "mode": PlannerMode.AVOID_OBSTACLE.value,
                "category": "human",
                "target": target,
                "dynamic": True,
                "held": held,
            },
            line_valid=True,
            reason="human" + ("_ttl" if held else ""),
            perception_quality=ctx["perception_quality"],
        )


class CarAvoidRule(TaskRule):
    priority = 70

    def __init__(self, rule_params):
        self.rule_params = rule_params

    def evaluate(self, ctx):
        target = _front_center_detection(
            ctx["detections"],
            "car",
            ctx["frame_w"],
            ctx["frame_h"],
            self.rule_params,
        )
        if target is None:
            return None
        return TaskDecision(
            task_state=TaskState.AVOID_CAR,
            desired_speed=ctx["avoid_speed"],
            planner_intent={
                "mode": PlannerMode.AVOID_OBSTACLE.value,
                "category": "car",
                "target": target,
                "dynamic": False,
            },
            line_valid=True,
            reason="front_center_car",
            perception_quality=ctx["perception_quality"],
        )


class GoldCollectRule(TaskRule):
    priority = 40

    def __init__(self, rule_params):
        self.rule_params = rule_params
        self.min_score = float(rule_params["GOLD_MIN_SCORE"])
        self.min_area_ratio = float(rule_params["GOLD_MIN_AREA_RATIO"])
        self.max_lateral_ratio = float(rule_params["GOLD_MAX_LATERAL_RATIO"])
        self.max_track_cost_ratio = float(rule_params["GOLD_MAX_TRACK_COST_RATIO"])

    def evaluate(self, ctx):
        target = _best_detection(
            ctx["detections"],
            "gold",
            self.min_score,
            self.rule_params["GENERIC_AREA_RANK_GAIN"],
        )
        if target is None:
            return None

        center = target.get("center") or [None, None]
        cx = _finite_float(center[0])
        if cx is None:
            return None
        area_ratio = _finite_float(target.get("area_ratio"), 0.0)
        if area_ratio < self.min_area_ratio:
            return None

        lateral = abs(cx - ctx["center_x"])
        if lateral > ctx["frame_w"] * self.max_lateral_ratio:
            return None

        track_error = _finite_float(ctx["track_error"], 0.0)
        target_error = cx - ctx["center_x"]
        if abs(target_error - track_error) > ctx["frame_w"] * self.max_track_cost_ratio:
            return None

        return TaskDecision(
            task_state=TaskState.COLLECT_GOLD,
            desired_speed=ctx["collect_speed"],
            planner_intent={
                "mode": PlannerMode.APPROACH_TARGET.value,
                "category": "gold",
                "target": target,
            },
            line_valid=True,
            reason="near_gold",
            perception_quality=ctx["perception_quality"],
        )


class TaskStateMachine:
    """Selects a high-level RK task from structured perception."""

    def __init__(
        self,
        track_speed=None,
        fallback_speed=None,
        line_loss_safe_stop_timeout=None,
        human_speed=None,
        stone_speed=None,
        avoid_speed=None,
        collect_speed=None,
        human_hold_ttl=None,
        rule_params=None,
        perception_quality_params=None,
        race_state_machine=None,
    ):
        self.rule_params = _copy_params(TASK_RULE_DEFAULTS, rule_params)
        self.perception_quality_params = _copy_params(PERCEPTION_QUALITY_DEFAULTS, perception_quality_params)
        self.race_state_machine = race_state_machine or RaceStateMachine()
        self.track_speed = float(
            track_speed if track_speed is not None else TASK_SPEED_DEFAULTS[TaskState.NORMAL_TRACK]
        )
        self.fallback_speed = float(
            fallback_speed if fallback_speed is not None else TASK_SPEED_DEFAULTS[TaskState.RECOVER_LINE]
        )
        self.line_loss_safe_stop_timeout = float(
            line_loss_safe_stop_timeout
            if line_loss_safe_stop_timeout is not None
            else TASK_TIMING_DEFAULTS["LINE_LOSS_SAFE_STOP_TIMEOUT"]
        )
        self.human_speed = float(
            human_speed
            if human_speed is not None
            else TASK_SPEED_DEFAULTS[TaskState.AVOID_HUMAN]
        )
        self.stone_speed = float(
            stone_speed
            if stone_speed is not None
            else TASK_SPEED_DEFAULTS[TaskState.AVOID_STONE]
        )
        self.avoid_speed = float(
            avoid_speed
            if avoid_speed is not None
            else TASK_SPEED_DEFAULTS[TaskState.AVOID_CAR]
        )
        self.collect_speed = float(
            collect_speed
            if collect_speed is not None
            else TASK_SPEED_DEFAULTS[TaskState.COLLECT_GOLD]
        )
        human_hold_ttl = (
            human_hold_ttl if human_hold_ttl is not None else TASK_TIMING_DEFAULTS["HUMAN_HOLD_TTL"]
        )
        self.line_missing_since_ts = None
        self.human_hold_ttl = float(human_hold_ttl)
        self.last_human_seen_ts = None
        self.last_human_target = None

    def update(self, perception, now):
        segmentation = (perception or {}).get("segmentation") or {}
        detections = list((perception or {}).get("detections") or [])
        frame_shape = (perception or {}).get("frame_shape") or [0, 0]
        frame_h = int(frame_shape[0]) if len(frame_shape) >= 1 else 0
        frame_w = int(frame_shape[1]) if len(frame_shape) >= 2 else 0
        frame_w = max(frame_w, 1)
        frame_h = max(frame_h, 1)
        center_x = _finite_float(segmentation.get("center_x"), frame_w * 0.5)
        track_error = _finite_float(segmentation.get("track_error"))
        race_state = self.race_state_machine.update(perception, now)
        perception_quality = _segmentation_quality(
            segmentation,
            track_error,
            now,
            self.perception_quality_params,
        )
        line_valid = bool(perception_quality["line_valid"])

        if race_state.get("finish_stop"):
            return self._race_stop_decision(
                TaskState.ENDSIGN_STOP,
                "endsign_lost_after_seen",
                perception_quality,
                race_state,
            ).as_dict()

        if race_state.get("traffic_light_stop"):
            return self._race_stop_decision(
                TaskState.TRAFFIC_LIGHT_STOP,
                "traffic_light_red",
                perception_quality,
                race_state,
            ).as_dict()

        if not line_valid:
            return self._line_loss_decision(now, perception_quality, race_state)

        self.line_missing_since_ts = None
        detections = _fresh_detections(
            detections,
            now,
            self.perception_quality_params["MAX_DETECTION_AGE"],
        )
        ctx = {
            "now": float(now),
            "perception": perception,
            "segmentation": segmentation,
            "detections": detections,
            "frame_w": frame_w,
            "frame_h": frame_h,
            "center_x": center_x,
            "track_error": track_error,
            "track_speed": self.track_speed,
            "human_speed": self.human_speed,
            "stone_speed": self.stone_speed,
            "avoid_speed": self.avoid_speed,
            "collect_speed": self.collect_speed,
            "perception_quality": perception_quality,
            "race_state": race_state,
        }

        near_decision = self._near_target_decision(ctx)
        if near_decision is not None:
            return near_decision.as_dict()

        return TaskDecision(
            task_state=TaskState.NORMAL_TRACK,
            desired_speed=self.track_speed,
            planner_intent={"mode": PlannerMode.TRACK_CENTER.value},
            line_valid=True,
            reason="line_track",
            perception_quality=perception_quality,
            race_state=race_state,
        ).as_dict()

    def _race_stop_decision(self, task_state, reason, perception_quality, race_state):
        return TaskDecision(
            task_state=task_state,
            desired_speed=TASK_SPEED_DEFAULTS[task_state],
            planner_intent={"mode": PlannerMode.SAFE_STOP.value},
            line_valid=bool((perception_quality or {}).get("line_valid", False)),
            line_loss_age=0.0,
            reason=reason,
            perception_quality=perception_quality or {},
            race_state=race_state or {},
        )

    def _near_target_decision(self, ctx):
        candidates = _near_detection_candidates(ctx, self.rule_params)
        if candidates:
            selected = candidates[0]
            category = selected["category"]
            target = selected["target"]
            if category == "human":
                self.last_human_seen_ts = ctx["now"]
                self.last_human_target = dict(target)
                return self._build_target_decision(
                    ctx,
                    category,
                    target,
                    held=False,
                    reason="near_human",
                )
            if category == "stone":
                return self._build_target_decision(
                    ctx,
                    category,
                    target,
                    held=False,
                    reason="near_stone",
                )
            if category == "car":
                return self._build_target_decision(
                    ctx,
                    category,
                    target,
                    held=False,
                    reason="near_car",
                )
            if category == "gold":
                return self._build_target_decision(
                    ctx,
                    category,
                    target,
                    held=False,
                    reason="near_gold",
                )

        if (
            self.last_human_seen_ts is not None
            and ctx["now"] - self.last_human_seen_ts <= self.human_hold_ttl
            and self.last_human_target is not None
        ):
            return self._build_target_decision(
                ctx,
                "human",
                dict(self.last_human_target),
                held=True,
                reason="near_human_ttl",
            )
        return None

    def _build_target_decision(self, ctx, category, target, held, reason):
        if category == "human":
            return TaskDecision(
                task_state=TaskState.AVOID_HUMAN,
                desired_speed=ctx["human_speed"],
                planner_intent={
                    "mode": PlannerMode.AVOID_OBSTACLE.value,
                    "category": "human",
                    "target": target,
                    "dynamic": True,
                    "held": bool(held),
                },
                line_valid=True,
                reason=reason,
                perception_quality=ctx["perception_quality"],
                race_state=ctx["race_state"],
            )

        if category == "car":
            return TaskDecision(
                task_state=TaskState.AVOID_CAR,
                desired_speed=ctx["avoid_speed"],
                planner_intent={
                    "mode": PlannerMode.AVOID_OBSTACLE.value,
                    "category": "car",
                    "target": target,
                    "dynamic": False,
                    "held": bool(held),
                },
                line_valid=True,
                reason=reason,
                perception_quality=ctx["perception_quality"],
                race_state=ctx["race_state"],
            )

        if category == "stone":
            return TaskDecision(
                task_state=TaskState.AVOID_STONE,
                desired_speed=ctx["stone_speed"],
                planner_intent={
                    "mode": PlannerMode.BRANCH_SELECT.value,
                    "category": "stone",
                    "target": target,
                    "default_branch": "outer",
                    "stone_branch": "inner",
                    "held": bool(held),
                },
                line_valid=True,
                reason=reason,
                perception_quality=ctx["perception_quality"],
                race_state=ctx["race_state"],
            )

        if category == "gold":
            return TaskDecision(
                task_state=TaskState.COLLECT_GOLD,
                desired_speed=ctx["collect_speed"],
                planner_intent={
                    "mode": PlannerMode.APPROACH_TARGET.value,
                    "category": "gold",
                    "target": target,
                    "held": bool(held),
                },
                line_valid=True,
                reason=reason,
                perception_quality=ctx["perception_quality"],
                race_state=ctx["race_state"],
            )

        return None

    def _line_loss_decision(self, now, perception_quality=None, race_state=None):
        now = float(now)
        if self.line_missing_since_ts is None:
            self.line_missing_since_ts = now
        line_loss_age = max(0.0, now - self.line_missing_since_ts)
        perception_quality = dict(perception_quality or {})
        quality_reason = perception_quality.get("reason") or "line_invalid"

        if line_loss_age >= self.line_loss_safe_stop_timeout:
            return TaskDecision(
                task_state=TaskState.LINE_LOSS_SAFE_STOP,
                desired_speed=TASK_SPEED_DEFAULTS[TaskState.LINE_LOSS_SAFE_STOP],
                planner_intent={"mode": PlannerMode.SAFE_STOP.value},
                line_valid=False,
                line_loss_age=line_loss_age,
                reason=f"line_loss_timeout:{quality_reason}",
                perception_quality=perception_quality,
                race_state=race_state or {},
            ).as_dict()

        return TaskDecision(
            task_state=TaskState.RECOVER_LINE,
            desired_speed=self.fallback_speed,
            planner_intent={"mode": PlannerMode.HOLD_LAST.value},
            line_valid=False,
            line_loss_age=line_loss_age,
            reason=f"line_recover:{quality_reason}",
            perception_quality=perception_quality,
            race_state=race_state or {},
        ).as_dict()
