import math
import os
import time
from dataclasses import dataclass

import cv2
import numpy as np


STATE_TRACK = 1
STATE_AVOID_CAR = 2
STATE_AVOID_HUMAN = 3
STATE_COLLECT_GOLD = 4
STATE_RECOVER_LINE = 5
STATE_LINE_LOSS_SAFE_STOP = 6
STATE_SAFE_STOP = 7
CONTROL_FLAG_USE_TARGET_SPEED = 0x01

ROUTE_NONE = "NONE"
ROUTE_SINGLE = "SINGLE"
ROUTE_MULTI_FORK = "MULTI_FORK"
ROUTE_AMBIGUOUS = "AMBIGUOUS"


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


def _env_flag(name, default):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _path_source_from_env():
    value = os.environ.get(
        "VISION_CONTROL_PATH_SOURCE",
        os.environ.get("MULTITASK_PATH_SOURCE", "heatmap"),
    ).strip().lower()
    return value if value in {"curve", "heatmap"} else "heatmap"


def _finite_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _clamp(value, low, high):
    return max(float(low), min(float(high), float(value)))


def _interp_path_x(points, target_y):
    if points is None or len(points) == 0:
        return None
    points = np.asarray(points, dtype=np.float32)
    order = np.argsort(points[:, 1])
    ys = points[order, 1]
    xs = points[order, 0]
    target_y = float(target_y)
    if target_y <= float(ys[0]):
        return float(xs[0])
    if target_y >= float(ys[-1]):
        return float(xs[-1])
    return float(np.interp(target_y, ys, xs))


def _interp_path_x_many(points, target_ys):
    points = np.asarray(points, dtype=np.float32)
    target_ys = np.asarray(target_ys, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        return np.full(target_ys.shape, np.nan, dtype=np.float32)
    order = np.argsort(points[:, 1])
    ys = points[order, 1]
    xs = points[order, 0]
    ys, unique_indices = np.unique(ys, return_index=True)
    xs = xs[unique_indices]
    if len(ys) == 1:
        return np.full(target_ys.shape, xs[0], dtype=np.float32)
    return np.interp(target_ys, ys, xs).astype(np.float32)


@dataclass
class VisionControlConfig:
    visual_center_x: float = 0.50
    lookahead_y_ratio: float = 0.625
    max_track_error_640: float = 210.0
    max_error_step_640: float = 36.0
    normal_speed_mps: float = 0.15
    recover_speed_mps: float = 0.15
    obstacle_speed_mps: float = 0.15
    human_speed_mps: float = 0.15
    human_pass_speed_mps: float = 0.35
    collect_speed_mps: float = 0.15
    turnsign_slow_speed_mps: float = 0.08
    heat_threshold: float = 0.22
    road_mask_threshold: float = 0.20
    heat_peak_top_k: int = 6
    row_step: int = 2
    min_path_points: int = 12
    min_path_coverage: float = 0.28
    min_mean_heat: float = 0.28
    max_link_jump_px: float = 16.0
    road_penalty_weight: float = 0.75
    history_weight: float = 0.035
    jump_weight: float = 0.018
    path_ema_alpha: float = 0.32
    path_smooth_window: int = 5
    path_max_step_px_640: float = 40.0
    path_state_hold_frames: int = 8
    route_confirm_frames: int = 6
    branch_release_frames: int = 20
    branch_separation_px_640: float = 70.0
    branch_separation_rows: int = 8
    overlap_px_640: float = 28.0
    default_outer_after_s: float = 15.0
    ocr_lock_lifetime_s: float = 10.0
    ocr_confirm_frames: int = 1
    outer_slot: int = 0
    no_path_stop_s: float = 0.8
    recover_hold_s: float = 0.5
    hazard_bottom_ratio: float = 0.58
    hazard_lateral_ratio: float = 0.18
    coin_bottom_ratio: float = 0.55
    coin_lateral_ratio: float = 0.24
    sign_stop_height_ratio: float = 0.24
    sign_stop_area_ratio: float = 0.035
    sign_stop_line_margin_ratio: float = 0.08
    sign_slow_min_score: float = 0.35
    sign_ocr_timeout_s: float = 8.0
    sign_ocr_pulse_speed_mps: float = 0.25
    sign_ocr_pulse_duration_s: float = 0.30
    human_stop_line_margin_ratio: float = 0.08
    human_stop_progress_ratio: float = 0.75
    human_cross_release_px_640: float = 45.0
    human_pass_offset_px_640: float = 38.0
    human_speed_hold_s: float = 0.5
    car_avoid_offset_px_640: float = 55.0
    human_avoid_offset_px_640: float = 75.0
    avoid_box_width_gain: float = 0.35
    gold_bias_gain: float = 0.45
    gold_max_bias_px_640: float = 75.0
    min_human_score: float = 0.35
    min_car_score: float = 0.35
    min_coin_score: float = 0.35
    path_source: str = "heatmap"

    @classmethod
    def from_env(cls):
        return cls(
            visual_center_x=_clamp(_env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8),
            lookahead_y_ratio=_clamp(_env_float("VISION_CONTROL_LOOKAHEAD_Y_RATIO", 0.625), 0.25, 0.95),
            max_track_error_640=max(1.0, _env_float("VISION_CONTROL_MAX_ERROR_640", 210.0)),
            max_error_step_640=max(1.0, _env_float("VISION_CONTROL_MAX_STEP_640", 36.0)),
            normal_speed_mps=max(0.0, _env_float("VISION_CONTROL_NORMAL_SPEED", 0.15)),
            recover_speed_mps=max(0.0, _env_float("VISION_CONTROL_RECOVER_SPEED", 0.15)),
            obstacle_speed_mps=max(0.0, _env_float("VISION_CONTROL_OBSTACLE_SPEED", 0.15)),
            human_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED", 0.15)),
            human_pass_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_SPEED", 0.35)),
            collect_speed_mps=max(0.0, _env_float("VISION_CONTROL_COLLECT_SPEED", 0.15)),
            turnsign_slow_speed_mps=max(0.0, _env_float("VISION_CONTROL_TURNSIGN_SLOW_SPEED", 0.08)),
            heat_threshold=_clamp(_env_float("VISION_CONTROL_HEAT_THRESHOLD", 0.22), 0.01, 0.99),
            road_mask_threshold=_clamp(_env_float("VISION_CONTROL_ROAD_MASK_THRESHOLD", 0.20), 0.0, 1.0),
            heat_peak_top_k=max(1, _env_int("VISION_CONTROL_HEAT_TOP_K", 6)),
            row_step=max(1, _env_int("VISION_CONTROL_ROW_STEP", 2)),
            min_path_points=max(3, _env_int("VISION_CONTROL_MIN_PATH_POINTS", 12)),
            min_path_coverage=_clamp(_env_float("VISION_CONTROL_MIN_PATH_COVERAGE", 0.28), 0.01, 1.0),
            min_mean_heat=_clamp(_env_float("VISION_CONTROL_MIN_MEAN_HEAT", 0.28), 0.01, 0.99),
            max_link_jump_px=max(2.0, _env_float("VISION_CONTROL_MAX_LINK_JUMP", 16.0)),
            road_penalty_weight=max(0.0, _env_float("VISION_CONTROL_ROAD_WEIGHT", 0.75)),
            history_weight=max(0.0, _env_float("VISION_CONTROL_HISTORY_WEIGHT", 0.035)),
            jump_weight=max(0.0, _env_float("VISION_CONTROL_JUMP_WEIGHT", 0.018)),
            path_ema_alpha=_clamp(_env_float("VISION_CONTROL_PATH_EMA_ALPHA", 0.32), 0.0, 1.0),
            path_smooth_window=max(1, _env_int("VISION_CONTROL_PATH_SMOOTH_WINDOW", 5)),
            path_max_step_px_640=max(1.0, _env_float("VISION_CONTROL_PATH_MAX_STEP_640", 40.0)),
            path_state_hold_frames=max(1, _env_int("VISION_CONTROL_PATH_HOLD_FRAMES", 8)),
            route_confirm_frames=max(1, _env_int("VISION_CONTROL_ROUTE_CONFIRM_FRAMES", 6)),
            branch_release_frames=max(1, _env_int("VISION_CONTROL_BRANCH_RELEASE_FRAMES", 20)),
            branch_separation_px_640=max(1.0, _env_float("VISION_CONTROL_BRANCH_SEP_640", 70.0)),
            branch_separation_rows=max(1, _env_int("VISION_CONTROL_BRANCH_SEP_ROWS", 8)),
            overlap_px_640=max(1.0, _env_float("VISION_CONTROL_OVERLAP_640", 28.0)),
            default_outer_after_s=max(0.0, _env_float("VISION_CONTROL_DEFAULT_OUTER_AFTER", 15.0)),
            ocr_lock_lifetime_s=max(
                0.1, _env_float("VISION_CONTROL_OCR_LOCK_LIFETIME_S", 10.0)),
            ocr_confirm_frames=max(1, _env_int("VISION_CONTROL_OCR_CONFIRM_FRAMES", 1)),
            outer_slot=max(0, min(1, _env_int("VISION_CONTROL_OUTER_SLOT", 0))),
            no_path_stop_s=max(0.1, _env_float("VISION_CONTROL_NO_PATH_STOP_S", 0.8)),
            recover_hold_s=max(0.0, _env_float("VISION_CONTROL_RECOVER_HOLD_S", 0.5)),
            hazard_bottom_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_BOTTOM_RATIO", 0.58), 0.0, 1.0),
            hazard_lateral_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_LATERAL_RATIO", 0.18), 0.01, 0.5),
            coin_bottom_ratio=_clamp(_env_float("VISION_CONTROL_COIN_BOTTOM_RATIO", 0.55), 0.0, 1.0),
            coin_lateral_ratio=_clamp(_env_float("VISION_CONTROL_COIN_LATERAL_RATIO", 0.24), 0.01, 0.5),
            sign_stop_height_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_HEIGHT_RATIO", 0.24), 0.01, 1.0),
            sign_stop_area_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_AREA_RATIO", 0.035), 0.001, 1.0),
            sign_stop_line_margin_ratio=_clamp(_env_float("VISION_CONTROL_SIGN_STOP_LINE_MARGIN_RATIO", 0.08), 0.0, 0.5),
            sign_slow_min_score=_clamp(_env_float("VISION_CONTROL_SIGN_SLOW_MIN_SCORE", 0.35), 0.0, 1.0),
            sign_ocr_timeout_s=max(0.1, _env_float("VISION_CONTROL_SIGN_OCR_TIMEOUT_S", 8.0)),
            sign_ocr_pulse_speed_mps=max(0.0, _env_float("VISION_CONTROL_SIGN_OCR_PULSE_SPEED", 0.25)),
            sign_ocr_pulse_duration_s=max(0.0, _env_float("VISION_CONTROL_SIGN_OCR_PULSE_DURATION_S", 0.30)),
            human_stop_line_margin_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_LINE_MARGIN_RATIO", 0.08), 0.0, 0.5),
            human_stop_progress_ratio=_clamp(_env_float("VISION_CONTROL_HUMAN_STOP_PROGRESS_RATIO", 0.75), 0.0, 1.0),
            human_cross_release_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_CROSS_RELEASE_640", 45.0)),
            human_pass_offset_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_PASS_OFFSET_640", 38.0)),
            human_speed_hold_s=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED_HOLD_S", 0.5)),
            car_avoid_offset_px_640=max(0.0, _env_float("VISION_CONTROL_CAR_AVOID_OFFSET_640", 55.0)),
            human_avoid_offset_px_640=max(0.0, _env_float("VISION_CONTROL_HUMAN_AVOID_OFFSET_640", 75.0)),
            avoid_box_width_gain=max(0.0, _env_float("VISION_CONTROL_AVOID_BOX_WIDTH_GAIN", 0.35)),
            gold_bias_gain=max(0.0, _env_float("VISION_CONTROL_GOLD_BIAS_GAIN", 0.45)),
            gold_max_bias_px_640=max(0.0, _env_float("VISION_CONTROL_GOLD_MAX_BIAS_640", 75.0)),
            min_human_score=_clamp(_env_float("VISION_CONTROL_HUMAN_MIN_SCORE", 0.35), 0.0, 1.0),
            min_car_score=_clamp(_env_float("VISION_CONTROL_CAR_MIN_SCORE", 0.35), 0.0, 1.0),
            min_coin_score=_clamp(_env_float("VISION_CONTROL_COIN_MIN_SCORE", 0.35), 0.0, 1.0),
            path_source=_path_source_from_env(),
        )


class HeatmapPathSearch:
    """Small Viterbi-style path search over one 120x160 heatmap channel."""

    def __init__(self, config=None):
        self.config = config or VisionControlConfig.from_env()

    def search(self, heatmap, road_mask=None, image_shape=(480, 640, 3), slot=0, history_points=None):
        if heatmap is None:
            return None
        heatmap = np.asarray(heatmap, dtype=np.float32)
        if heatmap.ndim != 2 or heatmap.size == 0:
            return None
        heatmap = np.clip(heatmap, 1e-4, 1.0)
        h, w = heatmap.shape[:2]
        road = self._prepare_road(road_mask, (h, w))
        road_threshold = float(self.config.road_mask_threshold)
        if road is not None and np.any(road >= road_threshold):
            heatmap = np.where(
                road >= road_threshold, heatmap, 1e-4).astype(np.float32)
        else:
            road = None
        rows = list(range(h - 2, max(int(h * 0.18), 1), -self.config.row_step))
        layers = []
        expected_rows = len(rows)
        for row in rows:
            peaks = self._row_peaks(heatmap[row], self.config.heat_threshold, self.config.heat_peak_top_k)
            if not peaks:
                continue
            nodes = []
            for x, probability in peaks:
                road_support = float(road[row, x]) if road is not None else 1.0
                if road is not None and road_support < road_threshold:
                    continue
                nodes.append({
                    "x": int(x),
                    "y": int(row),
                    "p": float(probability),
                    "road": road_support,
                    "cost": 0.0,
                    "prev": None,
                })
            if nodes:
                layers.append(nodes)
        if not layers:
            return None

        anchor_x = self._history_anchor_x(history_points, image_shape, w)
        center_anchor = self.config.visual_center_x * float(w - 1)
        if anchor_x is None:
            anchor_x = center_anchor
        max_jump = self.config.max_link_jump_px
        for layer_index, nodes in enumerate(layers):
            if layer_index == 0:
                for node in nodes:
                    node["cost"] = self._node_cost(node) + abs(node["x"] - anchor_x) * self.config.history_weight
                    node["length"] = 1
                continue
            previous_nodes = layers[layer_index - 1]
            row_gap = max(1.0, abs(float(nodes[0]["y"] - previous_nodes[0]["y"])) / float(self.config.row_step))
            allowed_jump = max_jump * (1.0 + 0.35 * (row_gap - 1.0))
            for node in nodes:
                best_cost = None
                best_previous = None
                for previous in previous_nodes:
                    jump = abs(float(node["x"] - previous["x"]))
                    if jump > allowed_jump:
                        continue
                    cost = previous["cost"] + self._node_cost(node) + jump * self.config.jump_weight
                    if best_cost is None or cost < best_cost:
                        best_cost = cost
                        best_previous = previous
                if best_previous is None:
                    node["cost"] = self._node_cost(node) + abs(node["x"] - anchor_x) * self.config.history_weight
                    node["length"] = 1
                else:
                    node["cost"] = best_cost
                    node["prev"] = best_previous
                    node["length"] = int(best_previous.get("length", 1)) + 1

        possible_end_nodes = [
            node
            for layer in layers
            for node in layer
            if int(node.get("length", 1)) >= self.config.min_path_points
        ]
        if not possible_end_nodes:
            return None
        max_length = max(int(node.get("length", 1)) for node in possible_end_nodes)
        min_length = max(self.config.min_path_points, int(round(max_length * 0.75)))
        possible_end_nodes = [
            node
            for node in possible_end_nodes
            if int(node.get("length", 1)) >= min_length
        ]
        end_node = min(
            possible_end_nodes,
            key=lambda item: float(item["cost"]) / float(max(1, int(item.get("length", 1)))),
        )
        chain = []
        node = end_node
        while node is not None:
            chain.append(node)
            node = node.get("prev")
        chain.reverse()
        if len(chain) < self.config.min_path_points:
            return None
        coverage = len(chain) / float(max(1, expected_rows))
        mean_heat = float(np.mean([node["p"] for node in chain]))
        if coverage < self.config.min_path_coverage or mean_heat < self.config.min_mean_heat:
            return None

        img_h, img_w = image_shape[:2]
        points = np.zeros((len(chain), 2), dtype=np.float32)
        points[:, 0] = [node["x"] * float(max(img_w - 1, 1)) / float(max(w - 1, 1)) for node in chain]
        points[:, 1] = [node["y"] * float(max(img_h - 1, 1)) / float(max(h - 1, 1)) for node in chain]
        return {
            "slot": int(slot),
            "role": "left" if int(slot) == 0 else "right",
            "source": "heatmap_viterbi",
            "points_xy": points,
            "score": mean_heat,
            "heatmap_score": mean_heat,
            "coverage": coverage,
            "road_support": float(np.mean([node["road"] for node in chain])),
            "point_confidences": np.asarray([node["p"] for node in chain], dtype=np.float32),
        }

    def _node_cost(self, node):
        heat_cost = -math.log(max(1e-4, float(node["p"])))
        road_cost = (1.0 - float(node["road"])) * self.config.road_penalty_weight
        return heat_cost + road_cost

    @staticmethod
    def _prepare_road(road_mask, shape):
        if road_mask is None:
            return None
        road = np.asarray(road_mask, dtype=np.float32)
        if road.ndim != 2 or road.size == 0:
            return None
        if road.shape[:2] != shape:
            road = cv2.resize(road, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
        return np.clip(road, 0.0, 1.0)

    @staticmethod
    def _row_peaks(values, threshold, top_k):
        values = np.asarray(values, dtype=np.float32)
        if values.size < 3:
            return []
        mask = (
            (values[1:-1] >= values[:-2])
            & (values[1:-1] >= values[2:])
            & (values[1:-1] >= float(threshold))
        )
        xs = np.flatnonzero(mask) + 1
        if xs.size == 0:
            return []
        order = sorted(xs.tolist(), key=lambda x: float(values[x]), reverse=True)
        selected = []
        for x in order:
            if all(abs(int(x) - previous_x) >= 3 for previous_x, _ in selected):
                selected.append((int(x), float(values[x])))
            if len(selected) >= int(top_k):
                break
        return selected

    @staticmethod
    def _history_anchor_x(history_points, image_shape, heatmap_width):
        if history_points is None or len(history_points) == 0:
            return None
        points = np.asarray(history_points, dtype=np.float32)
        bottom = points[np.argmax(points[:, 1])]
        img_w = max(1.0, float(image_shape[1] - 1))
        return float(bottom[0]) * float(max(heatmap_width - 1, 1)) / img_w


class VisionControlPlanner:
    """Convert pure visual perception into one TC264D command and debug packet."""

    def __init__(self, config=None, log_func=None):
        self.config = config or VisionControlConfig.from_env()
        self.path_search = HeatmapPathSearch(self.config)
        self.log_func = log_func
        self.last_selected_points = None
        self.last_slot_points = {}
        self.last_valid_ts = 0.0
        self.last_error = 0.0
        self.route_state = ROUTE_NONE
        self.route_reason = "initial"
        self.route_initialized = False
        self.pending_route_state = None
        self.pending_route_frames = 0
        self.branch_lock = None
        self.branch_lock_source = None
        self.selected_slot_lock = None
        self.selected_slot_missing_frames = 0
        self.path_missing_frames = {}
        self.fork_seen_since = None
        self.single_seen_frames = 0
        self.last_valid_ocr_ts = 0.0
        self.last_ocr_direction = None
        self.ocr_pending_direction = None
        self.ocr_pending_frames = 0
        self.ocr_confirmed_current = False
        self.human_waiting_cross = False
        self.human_last_side = None
        self.human_pass_active = False
        self.human_pass_side = None
        self.human_speed_hold_until = 0.0
        self.sign_ocr_active_since = None
        self.sign_ocr_pulse_until = 0.0
        self.sign_ocr_pulse_sent = False

    def update(self, perception_result, ocr_response=None, now=None):
        now = float(time.monotonic() if now is None else now)
        start = time.perf_counter()
        perception_result = perception_result if isinstance(perception_result, dict) else {}
        image_shape = self._image_shape(perception_result)
        candidates, search_ms = self._extract_candidates(perception_result, image_shape)
        raw_route_state, raw_route_reason = self._classify_routes(
            candidates, image_shape)
        route_state, route_reason = self._stabilize_route_state(
            raw_route_state, raw_route_reason)
        raw_ocr_direction, raw_ocr_current = self._extract_ocr_direction(ocr_response)
        ocr_direction, ocr_current = self._confirm_ocr_direction(
            raw_ocr_direction, raw_ocr_current)
        ocr_lock_expired = False
        if ocr_current:
            self.last_valid_ocr_ts = now
            self.last_ocr_direction = ocr_direction
            self.branch_lock = ocr_direction
            self.branch_lock_source = "ocr"
            self.selected_slot_lock = 0 if ocr_direction == "left" else 1
            self.selected_slot_missing_frames = 0
        else:
            ocr_lock_expired = self._expire_ocr_lock(now)
        self._update_default_outer(route_state, now, ocr_current)
        selected = self._select_candidate(candidates, route_state, image_shape)
        command, control_target = self._build_command(
            selected, route_state, route_reason, perception_result,
            image_shape, now, ocr_response)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        debug = {
            "enabled": True,
            "route_state": route_state,
            "route_reason": route_reason,
            "raw_route_state": raw_route_state,
            "raw_route_reason": raw_route_reason,
            "pending_route_state": self.pending_route_state,
            "pending_route_frames": self.pending_route_frames,
            "branch_lock": self.branch_lock,
            "branch_lock_source": self.branch_lock_source,
            "selected_slot_lock": self.selected_slot_lock,
            "raw_ocr_direction": raw_ocr_direction if raw_ocr_current else None,
            "raw_ocr_current": bool(raw_ocr_current),
            "ocr_pending_direction": self.ocr_pending_direction,
            "ocr_pending_frames": self.ocr_pending_frames,
            "ocr_confirm_frames": self.config.ocr_confirm_frames,
            "ocr_direction": ocr_direction if ocr_current else None,
            "ocr_current": bool(ocr_current),
            "ocr_lock_expired": bool(ocr_lock_expired),
            "ocr_lock_age_s": self._ocr_lock_age(now),
            "ocr_lock_remaining_s": self._ocr_lock_remaining(now),
            "default_outer_elapsed": self._default_outer_elapsed(now),
            "selected_slot": None if selected is None else int(selected.get("slot", -1)),
            "candidate_count": len(candidates),
            "candidates": [self._summarize_candidate(item, image_shape) for item in candidates],
            "heatmap_lines": [
                self._debug_path_points(item)
                for item in perception_result.get("heatmap_debug_lines", [])
            ],
            "candidate_paths": [self._debug_path_points(item) for item in candidates],
            "selected_path": None if selected is None else self._debug_path_points(selected),
            "control_target": control_target,
            "command": dict(command) if command else None,
            "timings_ms": {
                "path_search": float(search_ms),
                "control_total": float(elapsed_ms),
            },
        }
        perception_result["vision_control"] = debug
        return command, debug

    def _merge_close_display_paths(self, candidates, image_shape):
        display = []
        for candidate in candidates[:2]:
            item = dict(candidate)
            item["points_xy"] = np.asarray(candidate.get("points_xy"), dtype=np.float32).copy()
            item["display_segments_xy"] = [item["points_xy"].copy()]
            display.append(item)
        if len(display) != 2:
            return display
        first = display[0]["points_xy"]
        second = display[1]["points_xy"]
        if len(first) < 2 or len(second) < 2:
            return display
        overlap_threshold = (
            self.config.overlap_px_640 *
            float(max(1, image_shape[1])) / 640.0)
        merged_first = first.copy()
        merged_second = second.copy()
        first_other_x = _interp_path_x_many(second, first[:, 1])
        second_other_x = _interp_path_x_many(first, second[:, 1])
        first_overlap = (
            np.isfinite(first_other_x) &
            (np.abs(first[:, 0] - first_other_x) <= overlap_threshold)
        )
        second_overlap = (
            np.isfinite(second_other_x) &
            (np.abs(second[:, 0] - second_other_x) <= overlap_threshold)
        )
        merged_first[first_overlap, 0] = 0.5 * (
            first[first_overlap, 0] + first_other_x[first_overlap])
        merged_second[second_overlap, 0] = 0.5 * (
            second[second_overlap, 0] + second_other_x[second_overlap])
        second_is_separate = ~second_overlap
        display[0]["points_xy"] = merged_first
        display[1]["points_xy"] = merged_second
        display[0]["display_segments_xy"] = [merged_first]
        display[1]["display_segments_xy"] = self._split_visible_segments(
            merged_second, second_is_separate)
        return display

    @staticmethod
    def _split_visible_segments(points, visible):
        points = np.asarray(points, dtype=np.float32)
        visible = np.asarray(visible, dtype=bool)
        segments = []
        start = None
        for index, is_visible in enumerate(visible.tolist() + [False]):
            if is_visible and start is None:
                start = index
            elif not is_visible and start is not None:
                segment = points[start:index]
                if len(segment) >= 2:
                    segments.append(segment.copy())
                start = None
        return segments

    def _extract_candidates(self, result, image_shape):
        started = time.perf_counter()
        centerline = result.get("centerline") or {}
        if self.config.path_source == "curve":
            paths = (centerline.get("curve_paths") or
                     result.get("curve_paths") or [])
            candidates = []
            for path in paths:
                points = np.asarray(path.get("points_xy"), dtype=np.float32)
                if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
                    continue
                candidate = dict(path)
                candidate["points_xy"] = points
                candidate["source"] = "direct_curve"
                candidate["coverage"] = 1.0
                candidate["road_support"] = None
                candidates.append(candidate)
        else:
            heatmaps = centerline.get("heatmaps")
            if heatmaps is None:
                heatmaps = result.get("path_heatmaps")
            if heatmaps is None:
                return [], 0.0
            heatmaps = np.asarray(heatmaps, dtype=np.float32)
            if heatmaps.ndim != 3 or heatmaps.shape[0] < 1:
                return [], 0.0
            road = (result.get("road") or {}).get("mask")
            if road is None:
                road = result.get("road_mask")
            result["heatmap_debug_lines"] = self._extract_heatmap_debug_lines(
                heatmaps, road, image_shape)
            candidates = []
            for slot in range(min(2, heatmaps.shape[0])):
                candidate = self.path_search.search(
                    heatmaps[slot],
                    road_mask=road,
                    image_shape=image_shape,
                    slot=slot,
                    history_points=self.last_slot_points.get(slot),
                )
                if candidate is not None:
                    candidates.append(candidate)

        candidates.sort(key=lambda item: int(item.get("slot", 99)))
        candidates = self._smooth_candidates(candidates, image_shape)
        self._publish_filtered_paths(result, candidates)
        elapsed = (time.perf_counter() - started) * 1000.0
        return candidates, elapsed

    def _extract_heatmap_debug_lines(self, heatmaps, road_mask, image_shape):
        heatmaps = np.asarray(heatmaps, dtype=np.float32)
        if heatmaps.ndim != 3:
            return []
        h, w = heatmaps.shape[1:3]
        road = HeatmapPathSearch._prepare_road(road_mask, (h, w))
        road_threshold = float(self.config.road_mask_threshold)
        has_road = road is not None and np.any(road >= road_threshold)
        lines = []
        for slot in range(min(2, heatmaps.shape[0])):
            heatmap = np.clip(heatmaps[slot], 0.0, 1.0)
            mask = heatmap >= float(self.config.heat_threshold)
            if has_road:
                mask &= road >= road_threshold
            components, labels, stats, _centroids = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), connectivity=8)
            for label in range(1, components):
                if int(stats[label, cv2.CC_STAT_AREA]) < max(
                        4, self.config.min_path_points // 2):
                    continue
                points = self._component_centerline_points(labels, label, heatmap, image_shape)
                if len(points) < max(3, self.config.min_path_points // 2):
                    continue
                lines.append({
                    "slot": int(slot),
                    "points_xy": points,
                    "score": float(np.mean(heatmap[labels == label])),
                })
        return lines

    def _find_heatmap_intersection(self, heatmaps, road, image_shape):
        if heatmaps.shape[0] < 2:
            return None
        threshold = float(self.config.heat_threshold)
        support = heatmaps[:2] >= threshold
        if not np.any(support[0]) or not np.any(support[1]):
            return None

        if _skeletonize is not None:
            skeletons = np.stack([
                _skeletonize(channel).astype(np.uint8)
                for channel in support
            ])
            method = "skimage_skeleton"
        else:
            skeletons = support.astype(np.uint8)
            method = "binary_support"

        h, w = support.shape[1:3]
        radius = max(
            1,
            int(math.ceil(
                self.config.overlap_px_640 * float(w) / 640.0)),
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1))
        near_first = skeletons[0].astype(bool) & (
            cv2.dilate(skeletons[1], kernel) != 0)
        near_second = skeletons[1].astype(bool) & (
            cv2.dilate(skeletons[0], kernel) != 0)
        seeds = (near_first | near_second).astype(np.uint8)
        if not np.any(seeds):
            return None

        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            seeds, connectivity=8)
        kept = np.zeros_like(seeds)
        min_rows = max(1, int(self.config.intersection_min_rows))
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            if max(area, height) >= min_rows:
                kept[labels == label] = 1
        if not np.any(kept):
            return None

        row_flags = np.any(kept != 0, axis=1)
        gap_rows = max(0, int(self.config.intersection_gap_rows))
        if gap_rows:
            row_flags = cv2.morphologyEx(
                row_flags.astype(np.uint8).reshape((-1, 1)),
                cv2.MORPH_CLOSE,
                np.ones((gap_rows * 2 + 1, 1), dtype=np.uint8),
            ).reshape((-1,)).astype(bool)
        regions = self._true_runs(row_flags, min_rows)
        if not regions:
            return None

        kept_rows = np.zeros(h, dtype=bool)
        for start, end in regions:
            kept_rows[start:end + 1] = True
        kept &= kept_rows[:, np.newaxis].astype(np.uint8)

        expanded = cv2.dilate(kept, kernel) != 0
        overlap_probability = np.minimum(
            cv2.dilate(heatmaps[0], kernel),
            cv2.dilate(heatmaps[1], kernel),
        )
        intersection_mask = (
            expanded & np.any(support, axis=0) &
            (road >= float(self.config.road_mask_threshold)) &
            kept_rows[:, np.newaxis]
        )

        raw_centers = np.full(h, np.nan, dtype=np.float32)
        for y in np.flatnonzero(np.any(kept != 0, axis=1)):
            xs = np.flatnonzero(kept[y] != 0)
            weights = overlap_probability[y, xs].astype(np.float32)
            if float(np.sum(weights)) > 0.0:
                raw_centers[y] = float(np.average(xs, weights=weights))
            else:
                raw_centers[y] = float(np.mean(xs))

        known_rows = np.flatnonzero(np.isfinite(raw_centers))
        if not known_rows.size:
            return None
        centers = raw_centers.copy()
        requested_rows = np.flatnonzero(kept_rows)
        centers[requested_rows] = np.interp(
            requested_rows, known_rows, raw_centers[known_rows])

        img_h, img_w = image_shape[:2]
        points = np.asarray([
            [
                float(centers[y]) * float(max(img_w - 1, 1)) /
                float(max(w - 1, 1)),
                float(y) * float(max(img_h - 1, 1)) /
                float(max(h - 1, 1)),
            ]
            for y in requested_rows[::-1]
        ], dtype=np.float32)
        image_regions = [
            [
                float(start) * float(max(img_h - 1, 1)) /
                float(max(h - 1, 1)),
                float(end) * float(max(img_h - 1, 1)) /
                float(max(h - 1, 1)),
            ]
            for start, end in regions
        ]
        return {
            "mask": intersection_mask,
            "points_xy": points,
            "regions_y": image_regions,
            "heatmap_regions_y": [list(item) for item in regions],
            "score": float(np.mean(overlap_probability[kept != 0])),
            "method": method,
        }

    @staticmethod
    def _true_runs(flags, min_length):
        flags = np.asarray(flags, dtype=bool)
        runs = []
        start = None
        for index, value in enumerate(flags.tolist() + [False]):
            if value and start is None:
                start = index
            elif not value and start is not None:
                if index - start >= int(min_length):
                    runs.append((start, index - 1))
                start = None
        return runs

    def _attach_intersection_paths(self, lines, intersection, image_shape):
        if intersection is None:
            self.intersection_missing_frames += 1
            if self.intersection_missing_frames > self.config.path_state_hold_frames:
                self.last_intersection_points = None
            return lines

        shared_points = np.asarray(
            intersection.get("points_xy"), dtype=np.float32)
        if len(shared_points) < 2:
            return lines
        shared_points = self._smooth_path_points(
            shared_points, self.last_intersection_points, image_shape[1])
        self.last_intersection_points = shared_points.copy()
        self.intersection_missing_frames = 0
        intersection["points_xy"] = shared_points

        common = {
            "points_xy": shared_points,
            "score": float(intersection.get("score", 0.0)),
            "component_count": 1,
        }
        if not lines:
            return [{
                "slot": 0,
                "source_slot": 0,
                "role": "left",
                "points_xy": shared_points.copy(),
                "branch_points_xy": np.empty((0, 2), dtype=np.float32),
                "score": float(intersection.get("score", 0.0)),
                "component_count": 1,
                "component_rows": len(shared_points),
                "vertical_span_px": self._vertical_span(shared_points),
                "intersection_points_xy": shared_points.copy(),
                "intersection_y_ranges": list(intersection.get("regions_y") or []),
                "intersection_method": intersection.get("method"),
                "shared_only": True,
            }]

        attached = []
        for line in lines:
            branch_points = np.asarray(
                line.get("points_xy"), dtype=np.float32).copy()
            merged = self._blend_heatmap_evidence(line, common)
            merged["branch_points_xy"] = branch_points
            merged["intersection_points_xy"] = shared_points.copy()
            merged["intersection_y_ranges"] = list(
                intersection.get("regions_y") or [])
            merged["intersection_method"] = intersection.get("method")
            attached.append(merged)
        return attached

    def _lock_intersection_points(self, candidates):
        locked = []
        for candidate in candidates:
            shared_value = candidate.get("intersection_points_xy")
            ranges = candidate.get("intersection_y_ranges") or []
            if shared_value is None or not ranges:
                locked.append(candidate)
                continue
            shared = np.asarray(shared_value, dtype=np.float32)
            if len(shared) < 2 or not ranges:
                locked.append(candidate)
                continue
            item = dict(candidate)
            points = np.asarray(
                candidate.get("points_xy"), dtype=np.float32).copy()
            for index, point in enumerate(points):
                if not VisionControlPlanner._y_in_ranges(point[1], ranges):
                    continue
                shared_x = _interp_path_x(shared, point[1])
                if shared_x is not None:
                    points[index, 0] = shared_x
            item["points_xy"] = points
            self.last_slot_points[int(item.get("slot", -1))] = points.copy()
            locked.append(item)
        return locked

    @staticmethod
    def _y_in_ranges(y, ranges):
        return any(
            min(float(start), float(end)) <= float(y) <=
            max(float(start), float(end))
            for start, end in ranges
        )

    @staticmethod
    def _intersection_debug(intersection):
        if intersection is None:
            return {"active": False, "regions_y": [], "points_xy": []}
        points = np.asarray(intersection.get("points_xy"), dtype=np.float32)
        return {
            "active": True,
            "method": str(intersection.get("method") or ""),
            "score": float(intersection.get("score", 0.0)),
            "regions_y": [list(item) for item in intersection.get("regions_y", [])],
            "points_xy": VisionControlPlanner._debug_points(points),
        }

    @staticmethod
    def _vertical_span(points):
        points = np.asarray(points, dtype=np.float32)
        if len(points) < 2:
            return 0.0
        return float(np.max(points[:, 1]) - np.min(points[:, 1]))

    def _join_heatmap_fragments(self, lines, image_shape):
        ordered = sorted(
            lines,
            key=lambda item: (
                self._vertical_span(item.get("points_xy", [])),
                len(item.get("points_xy", [])),
                float(item.get("score", 0.0)),
            ),
            reverse=True,
        )
        tracks = []
        for line in ordered:
            best_index = None
            best_cost = None
            for index, track in enumerate(tracks):
                if int(track.get("source_slot", -1)) != int(line.get("source_slot", -1)):
                    continue
                cost = self._fragment_join_cost(track, line, image_shape)
                if cost is not None and (best_cost is None or cost < best_cost):
                    best_index = index
                    best_cost = cost
            if best_index is None:
                item = dict(line)
                item["points_xy"] = np.asarray(
                    line.get("points_xy"), dtype=np.float32).copy()
                tracks.append(item)
            else:
                tracks[best_index] = self._merge_heatmap_fragments(
                    tracks[best_index], line)
        return tracks

    def _fragment_join_cost(self, first, second, image_shape):
        first_points = np.asarray(first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(second.get("points_xy"), dtype=np.float32)
        if len(first_points) < 2 or len(second_points) < 2:
            return None

        first_min = float(np.min(first_points[:, 1]))
        first_max = float(np.max(first_points[:, 1]))
        second_min = float(np.min(second_points[:, 1]))
        second_max = float(np.max(second_points[:, 1]))
        if first_min > second_max:
            first_end = first_points[np.argmin(first_points[:, 1])]
            second_end = second_points[np.argmax(second_points[:, 1])]
            gap = first_min - second_max
        elif second_min > first_max:
            first_end = first_points[np.argmax(first_points[:, 1])]
            second_end = second_points[np.argmin(second_points[:, 1])]
            gap = second_min - first_max
        else:
            return None

        max_gap = max(12.0, float(image_shape[0]) * 0.35)
        if gap > max_gap:
            return None
        base_x_limit = max(
            self.config.overlap_px_640 * float(max(1, image_shape[1])) / 320.0,
            self.config.max_link_jump_px * float(max(1, image_shape[1])) / 160.0,
        )
        x_limit = base_x_limit * (1.0 + 0.5 * gap / max_gap)
        x_gap = abs(float(first_end[0]) - float(second_end[0]))
        if x_gap > x_limit:
            return None
        return x_gap + gap * 0.10

    def _merge_heatmap_fragments(self, first, second):
        first_points = np.asarray(first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(second.get("points_xy"), dtype=np.float32)
        points = np.concatenate((first_points, second_points), axis=0)
        points = points[np.argsort(points[:, 1])[::-1]]
        first_count = max(1, len(first_points))
        second_count = max(1, len(second_points))
        merged = dict(first)
        merged["points_xy"] = points
        merged["score"] = (
            float(first.get("score", 0.0)) * first_count +
            float(second.get("score", 0.0)) * second_count
        ) / float(first_count + second_count)
        merged["component_rows"] = (
            int(first.get("component_rows", first_count)) +
            int(second.get("component_rows", second_count))
        )
        merged["component_count"] = (
            int(first.get("component_count", 1)) +
            int(second.get("component_count", 1))
        )
        merged["vertical_span_px"] = self._vertical_span(points)
        return merged

    def _select_heatmap_lines(self, lines, image_shape):
        if len(lines) <= 2:
            return list(lines)

        qualities = [self._heatmap_line_quality(item) for item in lines]
        best_pair = None
        best_score = None
        for first_index in range(len(lines) - 1):
            for second_index in range(first_index + 1, len(lines)):
                first = lines[first_index]
                second = lines[second_index]
                distance = self._path_distance_640(
                    first.get("points_xy"), second.get("points_xy"), image_shape)
                diversity = min(
                    distance,
                    self.config.branch_separation_px_640 * 2.0,
                )
                score = (
                    qualities[first_index] + qualities[second_index] +
                    diversity * 2.0 +
                    self._history_match_bonus(first, qualities[first_index], image_shape) +
                    self._history_match_bonus(second, qualities[second_index], image_shape)
                )
                if best_score is None or score > best_score:
                    best_score = score
                    best_pair = (first, second)
        selected = list(best_pair or [])
        selected_ids = {id(item) for item in selected}
        for line in lines:
            if id(line) in selected_ids or not selected:
                continue
            distances = [
                self._path_distance_640(
                    line.get("points_xy"), item.get("points_xy"), image_shape)
                for item in selected
            ]
            nearest = int(np.argmin(distances))
            if distances[nearest] <= self.config.overlap_px_640:
                selected[nearest] = self._blend_heatmap_evidence(
                    selected[nearest], line)
        return selected

    def _blend_heatmap_evidence(self, first, second):
        first_points = np.asarray(first.get("points_xy"), dtype=np.float32)
        second_points = np.asarray(second.get("points_xy"), dtype=np.float32)
        rows = {}
        for line, points in ((first, first_points), (second, second_points)):
            weight = max(1e-4, float(line.get("score", 0.0)))
            for x, y in points:
                key = round(float(y), 3)
                rows.setdefault(key, []).append((float(x), weight))
        merged_points = []
        for y in sorted(rows, reverse=True):
            observations = rows[y]
            weights = np.asarray([item[1] for item in observations], dtype=np.float32)
            xs = np.asarray([item[0] for item in observations], dtype=np.float32)
            merged_points.append([float(np.average(xs, weights=weights)), float(y)])
        merged = dict(first)
        merged["points_xy"] = np.asarray(merged_points, dtype=np.float32)
        first_count = max(1, len(first_points))
        second_count = max(1, len(second_points))
        merged["score"] = (
            float(first.get("score", 0.0)) * first_count +
            float(second.get("score", 0.0)) * second_count
        ) / float(first_count + second_count)
        merged["component_count"] = (
            int(first.get("component_count", 1)) +
            int(second.get("component_count", 1))
        )
        merged["vertical_span_px"] = self._vertical_span(merged_points)
        return merged

    def _heatmap_line_quality(self, line):
        span = max(1.0, self._vertical_span(line.get("points_xy", [])))
        return span * (0.5 + float(line.get("score", 0.0)))

    def _history_match_bonus(self, line, quality, image_shape):
        if not self.last_slot_points:
            return 0.0
        distance = min(
            self._path_distance_640(
                line.get("points_xy"), previous, image_shape)
            for previous in self.last_slot_points.values()
        )
        gate = max(
            self.config.branch_separation_px_640 * 2.0,
            self.config.path_max_step_px_640 * 3.0,
        )
        return quality * 0.45 * max(0.0, 1.0 - distance / gate)

    @staticmethod
    def _path_distance_640(first_points, second_points, image_shape):
        first = np.asarray(first_points, dtype=np.float32)
        second = np.asarray(second_points, dtype=np.float32)
        if len(first) < 2 or len(second) < 2:
            return 1e9
        low = max(float(np.min(first[:, 1])), float(np.min(second[:, 1])))
        high = min(float(np.max(first[:, 1])), float(np.max(second[:, 1])))
        if low <= high:
            distances = []
            for y in np.linspace(low, high, num=24):
                first_x = _interp_path_x(first, y)
                second_x = _interp_path_x(second, y)
                if first_x is not None and second_x is not None:
                    distances.append(abs(first_x - second_x))
            if distances:
                return float(np.mean(distances)) * 640.0 / float(max(1, image_shape[1]))

        endpoint_distance = min(
            abs(float(first_point[0]) - float(second_point[0]))
            for first_point in (first[0], first[-1])
            for second_point in (second[0], second[-1])
        )
        return endpoint_distance * 640.0 / float(max(1, image_shape[1]))

    def _assign_heatmap_slots(self, lines, image_shape):
        if not lines:
            return []
        assigned = []
        if len(lines) == 1:
            item = dict(lines[0])
            item["slot"] = max(0, min(1, int(item.get("source_slot", 0))))
            assigned.append(item)
        else:
            first = dict(lines[0])
            second = dict(lines[1])
            source_slots = {
                int(first.get("source_slot", -1)),
                int(second.get("source_slot", -1)),
            }
            if source_slots == {0, 1}:
                ordered = sorted(
                    (first, second), key=lambda item: int(item.get("source_slot", 0)))
            elif 0 in self.last_slot_points and 1 in self.last_slot_points:
                direct = (
                    self._path_distance_640(first.get("points_xy"), self.last_slot_points[0], image_shape) +
                    self._path_distance_640(second.get("points_xy"), self.last_slot_points[1], image_shape)
                )
                swapped = (
                    self._path_distance_640(first.get("points_xy"), self.last_slot_points[1], image_shape) +
                    self._path_distance_640(second.get("points_xy"), self.last_slot_points[0], image_shape)
                )
                ordered = [first, second] if direct <= swapped else [second, first]
            else:
                lookahead_y = image_shape[0] * self.config.lookahead_y_ratio
                ordered = sorted(
                    (first, second),
                    key=lambda item: float(
                        _interp_path_x(item.get("points_xy"), lookahead_y) or 0.0),
                )
            for slot, item in enumerate(ordered):
                item["slot"] = slot
                assigned.append(item)
        for item in assigned:
            item["points_xy"] = np.asarray(item.get("points_xy"), dtype=np.float32)
            item["role"] = "left" if int(item["slot"]) == 0 else "right"
        return assigned

    @staticmethod
    def _component_centerline_points(labels, label, heatmap, image_shape):
        ys, xs = np.nonzero(labels == label)
        if len(xs) == 0:
            return []
        img_h, img_w = image_shape[:2]
        points = []
        for y in sorted(set(int(value) for value in ys), reverse=True):
            row_xs = xs[ys == y]
            weights = heatmap[y, row_xs].astype(np.float32)
            if float(np.sum(weights)) <= 0.0:
                center_x = float(np.mean(row_xs))
            else:
                center_x = float(np.average(row_xs, weights=weights))
            points.append([
                center_x * float(max(img_w - 1, 1)) / float(max(labels.shape[1] - 1, 1)),
                float(y) * float(max(img_h - 1, 1)) / float(max(labels.shape[0] - 1, 1)),
            ])
        if len(points) >= 5:
            arr = np.asarray(points, dtype=np.float32)
            kernel = np.asarray([1.0, 2.0, 3.0, 2.0, 1.0], dtype=np.float32)
            kernel /= float(np.sum(kernel))
            padded = np.pad(arr[:, 0], (2, 2), mode="edge")
            arr[:, 0] = np.convolve(padded, kernel, mode="valid")
            points = arr.tolist()
        return points

    def _smooth_candidates(self, candidates, image_shape):
        present_slots = set()
        smoothed_candidates = []
        for candidate in candidates:
            slot = int(candidate.get("slot", -1))
            points = np.asarray(candidate.get("points_xy"), dtype=np.float32)
            if slot < 0 or points.ndim != 2 or points.shape[1] != 2:
                continue
            present_slots.add(slot)
            filtered = dict(candidate)
            filtered["raw_points_xy"] = points.copy()
            previous = self.last_slot_points.get(slot)
            filtered_points = self._smooth_path_points(
                points, previous, image_shape[1])
            filtered["points_xy"] = filtered_points
            filtered["spatial_smoothed"] = self.config.path_smooth_window > 1
            filtered["temporal_smoothed"] = previous is not None
            self.last_slot_points[slot] = filtered_points.copy()
            self.path_missing_frames[slot] = 0
            smoothed_candidates.append(filtered)

        for slot in list(self.last_slot_points):
            if slot in present_slots:
                continue
            missing = self.path_missing_frames.get(slot, 0) + 1
            self.path_missing_frames[slot] = missing
            if missing > self.config.path_state_hold_frames:
                self.last_slot_points.pop(slot, None)
                self.path_missing_frames.pop(slot, None)
        return smoothed_candidates

    def _smooth_path_points(self, points, previous, image_width):
        points = np.asarray(points, dtype=np.float32).copy()
        window = max(1, int(self.config.path_smooth_window))
        if window % 2 == 0:
            window += 1
        if window > 1 and len(points) >= 3:
            window = min(window, len(points) if len(points) % 2 else len(points) - 1)
            half = window // 2
            weights = np.concatenate((
                np.arange(1, half + 2, dtype=np.float32),
                np.arange(half, 0, -1, dtype=np.float32),
            ))
            padded_x = np.pad(points[:, 0], (half, half), mode="edge")
            points[:, 0] = np.convolve(
                padded_x, weights / np.sum(weights), mode="valid")

        if previous is None or len(previous) < 2 or not len(points):
            return points
        previous = np.asarray(previous, dtype=np.float32)
        order = np.argsort(previous[:, 1])
        previous_y = previous[order, 1]
        previous_x = previous[order, 0]
        previous_y, unique_indices = np.unique(previous_y, return_index=True)
        previous_x = previous_x[unique_indices]
        if len(previous_y) < 2:
            return points
        overlap = (
            (points[:, 1] >= previous_y[0]) &
            (points[:, 1] <= previous_y[-1]))
        if not np.any(overlap):
            return points
        projected_x = np.interp(
            points[overlap, 1], previous_y, previous_x).astype(np.float32)
        max_step = (
            self.config.path_max_step_px_640 *
            float(max(1, image_width)) / 640.0)
        bounded_x = projected_x + np.clip(
            points[overlap, 0] - projected_x, -max_step, max_step)
        alpha = float(self.config.path_ema_alpha)
        points[overlap, 0] = (
            bounded_x * alpha + projected_x * (1.0 - alpha))
        return points

    def _publish_filtered_paths(self, result, candidates):
        raw_paths = list(result.get("paths") or [])
        result["raw_paths"] = raw_paths
        result["paths"] = candidates
        result["temporal"] = {
            "enabled": True,
            "status": "tracking" if candidates else "path_unavailable",
            "alpha": float(self.config.path_ema_alpha),
            "source": "vision_control_slot_filter",
        }
        centerline = result.get("centerline")
        if isinstance(centerline, dict):
            centerline["raw_paths"] = raw_paths
            centerline["paths"] = candidates
            centerline["temporal"] = result["temporal"]

    @staticmethod
    def _publish_display_paths(result, display_candidates):
        result["display_paths"] = display_candidates
        centerline = result.get("centerline")
        if isinstance(centerline, dict):
            centerline["display_paths"] = display_candidates

    def _stabilize_route_state(self, raw_state, raw_reason):
        if not self.route_initialized:
            self.route_initialized = True
            self.route_state = raw_state
            self.route_reason = raw_reason
            return self.route_state, self.route_reason
        if raw_state == self.route_state:
            self.pending_route_state = None
            self.pending_route_frames = 0
            self.route_reason = raw_reason
            return self.route_state, self.route_reason
        if raw_state != self.pending_route_state:
            self.pending_route_state = raw_state
            self.pending_route_frames = 1
        else:
            self.pending_route_frames += 1
        if self.pending_route_frames >= self.config.route_confirm_frames:
            self.route_state = raw_state
            self.route_reason = raw_reason
            self.pending_route_state = None
            self.pending_route_frames = 0
            return self.route_state, self.route_reason
        return self.route_state, "hold_{}_pending_{}".format(
            self.route_state.lower(), raw_state.lower())

    def _classify_routes(self, candidates, image_shape):
        if not candidates:
            return ROUTE_NONE, "no_candidate"
        if len(candidates) == 1:
            return ROUTE_SINGLE, "one_candidate"
        first, second = candidates[:2]
        first_points = first.get("branch_points_xy", first["points_xy"])
        second_points = second.get("branch_points_xy", second["points_xy"])
        stats = self._path_pair_stats(
            first_points, second_points, image_shape)
        if stats["mean_distance_640"] <= self.config.overlap_px_640:
            return ROUTE_SINGLE, "overlap_dedup"
        if stats["separated_rows"] >= self.config.branch_separation_rows:
            return ROUTE_MULTI_FORK, "separated_candidates"
        return ROUTE_AMBIGUOUS, "weak_separation"

    def _update_default_outer(self, route_state, now, ocr_current):
        if route_state == ROUTE_MULTI_FORK:
            self.single_seen_frames = 0
            if self.fork_seen_since is None:
                self.fork_seen_since = now
            if (
                not ocr_current
                and self.branch_lock is None
                and now - self.fork_seen_since >= self.config.default_outer_after_s
            ):
                self.branch_lock = "left" if self.config.outer_slot == 0 else "right"
                self.branch_lock_source = "default"
            return
        if route_state == ROUTE_SINGLE:
            self.fork_seen_since = None
            self.single_seen_frames += 1
            if self.single_seen_frames == self.config.branch_release_frames:
                self._clear_branch_lock()
            return
        if route_state in {ROUTE_NONE, ROUTE_AMBIGUOUS}:
            self.single_seen_frames = 0
            return

    def _select_candidate(self, candidates, route_state, image_shape):
        if not candidates:
            if self.selected_slot_lock is not None:
                self.selected_slot_missing_frames += 1
                if (self.branch_lock is None and
                        self.selected_slot_missing_frames >
                        self.config.path_state_hold_frames):
                    self.selected_slot_lock = None
                    self.selected_slot_missing_frames = 0
                    self.last_selected_points = None
            return None
        if self.branch_lock in {"left", "right"}:
            wanted_slot = 0 if self.branch_lock == "left" else 1
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == wanted_slot:
                    return self._remember_selection(candidate)
            self.selected_slot_lock = wanted_slot
            self.selected_slot_missing_frames += 1
            return None
        if self.selected_slot_lock is not None:
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == self.selected_slot_lock:
                    return self._remember_selection(candidate)
            self.selected_slot_missing_frames += 1
            if (self.selected_slot_missing_frames <=
                    self.config.path_state_hold_frames):
                return None
            self.selected_slot_lock = None
            self.selected_slot_missing_frames = 0
        if self.last_selected_points is not None:
            previous_x = _interp_path_x(self.last_selected_points, image_shape[0] * self.config.lookahead_y_ratio)
            if previous_x is not None:
                return self._remember_selection(min(
                    candidates,
                    key=lambda item: abs(
                        (_interp_path_x(item["points_xy"], image_shape[0] * self.config.lookahead_y_ratio) or previous_x)
                        - previous_x
                    ),
                ))
        if route_state == ROUTE_MULTI_FORK:
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == self.config.outer_slot:
                    return self._remember_selection(candidate)
        center_x = image_shape[1] * self.config.visual_center_x
        return self._remember_selection(min(
            candidates,
            key=lambda item: abs((_interp_path_x(item["points_xy"], image_shape[0] * self.config.lookahead_y_ratio) or center_x) - center_x),
        ))

    def _remember_selection(self, candidate):
        self.selected_slot_lock = int(candidate.get("slot", -1))
        self.selected_slot_missing_frames = 0
        return candidate

    def _build_command(self, selected, route_state, route_reason, result, image_shape, now, ocr_response=None):
        lookahead_y = image_shape[0] * self.config.lookahead_y_ratio
        task_state, speed, task_reason, target_override_x = self._task_from_detections(
            result, selected, image_shape, lookahead_y, route_state, now, ocr_response=ocr_response)
        if selected is None:
            age = now - self.last_valid_ts if self.last_valid_ts else 1e9
            if self.last_valid_ts and age <= self.config.recover_hold_s:
                error = self.last_error
                return self._command(error, self.config.recover_speed_mps, STATE_RECOVER_LINE), {
                    "target_x": None,
                    "track_error_640": error,
                    "reason": "recover_hold",
                    "task_reason": task_reason,
                }
            state = STATE_LINE_LOSS_SAFE_STOP if age >= self.config.no_path_stop_s else STATE_RECOVER_LINE
            return self._command(0.0, 0.0 if state == STATE_LINE_LOSS_SAFE_STOP else self.config.recover_speed_mps, state, flags=0 if state == STATE_LINE_LOSS_SAFE_STOP else CONTROL_FLAG_USE_TARGET_SPEED), {
                "target_x": None,
                "track_error_640": 0.0,
                "reason": route_reason,
                "task_reason": task_reason,
            }
        if route_state == ROUTE_AMBIGUOUS:
            return self._command(0.0, 0.0, STATE_SAFE_STOP, flags=0), {
                "target_x": None,
                "track_error_640": 0.0,
                "reason": route_reason,
                "task_reason": "ambiguous_stop",
            }

        target_x = target_override_x
        if target_x is None:
            target_x = _interp_path_x(selected["points_xy"], lookahead_y)
        if target_x is None:
            return self._command(0.0, 0.0, STATE_SAFE_STOP, flags=0), {
                "target_x": None,
                "track_error_640": 0.0,
                "reason": "missing_target_x",
                "task_reason": task_reason,
            }
        raw_error = (float(target_x) / float(max(1, image_shape[1])) - self.config.visual_center_x) * 640.0
        error = self._limit_error(raw_error)
        self.last_error = error
        self.last_valid_ts = now
        self.last_selected_points = np.asarray(selected["points_xy"], dtype=np.float32).copy()
        self.last_slot_points[int(selected["slot"])] = self.last_selected_points
        return self._command(error, speed, task_state), {
            "target_x": float(target_x),
            "lookahead_y": float(lookahead_y),
            "track_error_640": float(error),
            "raw_track_error_640": float(raw_error),
            "reason": route_reason,
            "task_reason": task_reason,
        }

    def _task_from_detections(self, result, selected, image_shape, lookahead_y, route_state, now, ocr_response=None):
        detections = result.get("detections") or []
        if selected is None:
            return STATE_RECOVER_LINE, self.config.recover_speed_mps, "no_path", None
        sign_action = self._turnsign_action(detections, image_shape, lookahead_y, ocr_response, now)
        sign_mode = None
        sign_target_x = None
        if isinstance(sign_action, tuple):
            sign_mode, sign_target_x = sign_action
        else:
            sign_mode = sign_action
        if sign_mode == "stop":
            return STATE_SAFE_STOP, 0.0, "turnsign_stop", None
        if sign_mode == "pulse":
            return STATE_TRACK, self.config.sign_ocr_pulse_speed_mps, "turnsign_ocr_timeout_pulse", None
        target_path_x = _interp_path_x(selected["points_xy"], image_shape[0] * 0.68)
        if target_path_x is None:
            target_path_x = image_shape[1] * self.config.visual_center_x
        lookahead_path_x = _interp_path_x(selected["points_xy"], lookahead_y)
        if lookahead_path_x is None:
            lookahead_path_x = target_path_x
        human_action = self._human_action(
            detections, selected, image_shape, lookahead_y, lookahead_path_x)
        if human_action is not None:
            if int(human_action[0]) == STATE_AVOID_HUMAN:
                self.human_speed_hold_until = float(now) + self.config.human_speed_hold_s
            return human_action
        if now < self.human_speed_hold_until:
            return STATE_AVOID_HUMAN, self.config.human_pass_speed_mps, "human_speed_hold", None
        hazard_limit = image_shape[1] * self.config.hazard_lateral_ratio
        for det in detections:
            label = self._normalized_label(det)
            if label != "car":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_car_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None or geom["bottom_ratio"] < self.config.hazard_bottom_ratio:
                continue
            if abs(geom["cx"] - target_path_x) > max(hazard_limit, geom["box_w"] * 0.75):
                continue
            avoid_target_x = self._avoid_target_x(label, geom, target_path_x, image_shape)
            return STATE_AVOID_CAR, self.config.obstacle_speed_mps, "car_in_path_bias", avoid_target_x
        coin = self._best_coin(detections, image_shape, target_path_x)
        if coin is not None:
            return STATE_COLLECT_GOLD, self.config.collect_speed_mps, "coin_bias", self._coin_target_x(coin, target_path_x, image_shape)
        if sign_mode == "slow":
            return (
                STATE_TRACK,
                min(self.config.normal_speed_mps, self.config.turnsign_slow_speed_mps),
                "turnsign_slow",
                sign_target_x,
            )
        return STATE_TRACK, self.config.normal_speed_mps, "track", None

    def _turnsign_action(self, detections, image_shape, lookahead_y, ocr_response, now):
        has_active_ocr = bool((ocr_response or {}).get("active"))
        if self._ocr_has_current_direction(ocr_response):
            self._clear_sign_ocr_state()
            return None
        has_sign = False
        should_stop = False
        sign_target_x = None
        best_rank = -1.0
        for det in detections:
            if not self._is_turnsign_detection(det):
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.sign_slow_min_score:
                continue
            has_sign = True
            geom = self._detection_geom(det, image_shape)
            if geom is not None:
                rank = float(score) + float(geom.get("area_ratio") or 0.0) * 8.0
                if rank > best_rank:
                    best_rank = rank
                    sign_target_x = float(geom["cx"])
                if self._sign_should_stop(geom, image_shape, lookahead_y):
                    should_stop = True
        if has_active_ocr:
            if self.sign_ocr_active_since is None:
                self.sign_ocr_active_since = float(now)
                self.sign_ocr_pulse_until = 0.0
                self.sign_ocr_pulse_sent = False
            if not has_sign:
                return "stop", None
            return ("stop", None) if should_stop else ("slow", sign_target_x)
        self._clear_sign_ocr_state()
        if should_stop:
            return "stop", None
        return ("slow", sign_target_x) if has_sign or has_active_ocr else None

    def _clear_sign_ocr_state(self):
        self.sign_ocr_active_since = None
        self.sign_ocr_pulse_until = 0.0
        self.sign_ocr_pulse_sent = False

    def _human_action(self, detections, selected, image_shape, lookahead_y, lookahead_path_x):
        humans = []
        for det in detections:
            if self._normalized_label(det) != "human":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_human_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None:
                continue
            path_x = _interp_path_x(selected["points_xy"], geom["cy"])
            if path_x is None:
                path_x = lookahead_path_x
            distance = float(geom["cx"]) - float(path_x)
            side = self._sign(distance)
            path_margin = max(
                float(image_shape[1]) * self.config.hazard_lateral_ratio,
                float(geom["box_w"]) * 0.75,
            )
            humans.append({
                "geom": geom,
                "score": score,
                "path_x": path_x,
                "distance": distance,
                "side": side,
                "on_path": abs(distance) <= path_margin,
            })
        if not humans:
            self._clear_human_state()
            return None

        human = max(
            humans,
            key=lambda item: (
                bool(item["on_path"]),
                float(item["geom"]["bottom_ratio"]),
                float(item["score"]),
            ),
        )
        geom = human["geom"]
        side = human["side"] or self.human_last_side or 1
        if human["side"] != 0:
            if self.human_last_side is None:
                self.human_last_side = human["side"]

        if self.human_pass_active:
            self.human_pass_side = side
            return self._human_pass_command(lookahead_path_x, image_shape, side)

        crossed = (
            self.human_waiting_cross
            and human["side"] != 0
            and self.human_last_side is not None
            and human["side"] != self.human_last_side
        )
        release_distance = (
            self.config.human_cross_release_px_640 *
            float(max(1, image_shape[1])) / 640.0)
        released_to_side = (
            self.human_waiting_cross
            and not human["on_path"]
            and abs(float(human["distance"])) >= release_distance
        )
        if crossed or released_to_side:
            self.human_pass_active = True
            self.human_pass_side = side
            return self._human_pass_command(lookahead_path_x, image_shape, side)

        if human["on_path"] and self._human_on_stop_line(geom, image_shape, lookahead_y):
            self.human_waiting_cross = True
            if human["side"] != 0:
                self.human_last_side = human["side"]
            return STATE_SAFE_STOP, 0.0, "human_half_lookahead_stop", None

        if not human["on_path"] and not self.human_waiting_cross:
            self._clear_human_state()
        return None

    def _human_pass_command(self, path_x, image_shape, human_side):
        scale = float(max(1, image_shape[1])) / 640.0
        offset = self.config.human_pass_offset_px_640 * scale
        target_x = _clamp(
            float(path_x) - float(human_side or 1) * offset,
            0.0,
            float(max(0, image_shape[1] - 1)),
        )
        return STATE_AVOID_HUMAN, self.config.human_pass_speed_mps, "human_cross_pass", target_x

    def _human_on_stop_line(self, geom, image_shape, lookahead_y):
        stop_y = float(image_shape[0]) - (
            float(image_shape[0]) - float(lookahead_y)
        ) * self.config.human_stop_progress_ratio
        line_margin = float(image_shape[0]) * self.config.human_stop_line_margin_ratio
        return (
            float(geom["top"]) - line_margin <= stop_y <= float(geom["bottom"]) + line_margin
            or abs(float(geom["cy"]) - stop_y) <= line_margin
        )

    def _clear_human_state(self):
        self.human_waiting_cross = False
        self.human_last_side = None
        self.human_pass_active = False
        self.human_pass_side = None

    def _sign_should_stop(self, geom, image_shape, lookahead_y):
        height_ratio = float(geom["box_h"]) / float(max(1, image_shape[0]))
        area_ratio = geom.get("area_ratio")
        if area_ratio is None:
            area_ratio = (float(geom["box_w"]) * float(geom["box_h"])) / float(max(1, image_shape[0] * image_shape[1]))
        large_enough = (
            height_ratio >= self.config.sign_stop_height_ratio
            or float(area_ratio) >= self.config.sign_stop_area_ratio
        )
        if not large_enough:
            return False
        line_margin = float(image_shape[0]) * self.config.sign_stop_line_margin_ratio
        return (
            float(geom["top"]) - line_margin <= float(lookahead_y) <= float(geom["bottom"]) + line_margin
            or abs(float(geom["bottom"]) - float(lookahead_y)) <= line_margin
        )

    def _best_coin(self, detections, image_shape, path_x):
        best = None
        best_rank = -1.0
        for det in detections:
            label = str(det.get("label") or det.get("category") or "").lower()
            if label != "coin":
                continue
            score = _finite_float(det.get("score"), 0.0)
            if score < self.config.min_coin_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None or geom["bottom_ratio"] < self.config.coin_bottom_ratio:
                continue
            lateral = abs(geom["cx"] - path_x) / float(max(1, image_shape[1]))
            if lateral > self.config.coin_lateral_ratio:
                continue
            rank = score + geom["bottom_ratio"] - lateral
            if rank > best_rank:
                best = geom
                best_rank = rank
        return best

    def _avoid_target_x(self, label, geom, path_x, image_shape):
        scale = float(max(1, image_shape[1])) / 640.0
        base_offset = (
            self.config.human_avoid_offset_px_640
            if label == "human"
            else self.config.car_avoid_offset_px_640
        )
        offset = max(float(base_offset) * scale, float(geom["box_w"]) * self.config.avoid_box_width_gain)
        side = 1.0 if geom["cx"] <= path_x else -1.0
        return _clamp(float(path_x) + side * offset, 0.0, float(max(0, image_shape[1] - 1)))

    def _coin_target_x(self, geom, path_x, image_shape):
        scale = float(max(1, image_shape[1])) / 640.0
        max_bias = self.config.gold_max_bias_px_640 * scale
        bias = _clamp((float(geom["cx"]) - float(path_x)) * self.config.gold_bias_gain, -max_bias, max_bias)
        return _clamp(float(path_x) + bias, 0.0, float(max(0, image_shape[1] - 1)))

    @staticmethod
    def _detection_geom(det, image_shape):
        bbox = det.get("bbox") or det.get("box_xyxy")
        center = det.get("center")
        size = det.get("size")
        if bbox and len(bbox) >= 4:
            try:
                left, top, right, bottom = [float(value) for value in bbox[:4]]
            except (TypeError, ValueError):
                return None
        elif center and size and len(center) >= 2 and len(size) >= 2:
            try:
                cx, cy = [float(value) for value in center[:2]]
                box_w, box_h = [float(value) for value in size[:2]]
            except (TypeError, ValueError):
                return None
            left = cx - box_w * 0.5
            right = cx + box_w * 0.5
            top = cy - box_h * 0.5
            bottom = cy + box_h * 0.5
        else:
            return None
        if right <= left or bottom <= top:
            return None
        return {
            "left": left,
            "top": top,
            "right": right,
            "bottom": bottom,
            "cx": (left + right) * 0.5,
            "cy": (top + bottom) * 0.5,
            "box_w": right - left,
            "box_h": bottom - top,
            "bottom_ratio": bottom / float(max(1, image_shape[0])),
            "area_ratio": _finite_float(det.get("area_ratio")),
        }

    @staticmethod
    def _normalized_label(det):
        label = str(det.get("label") or det.get("category") or det.get("class_name") or "")
        return label.strip().lower().replace("_", "").replace("-", "").replace(" ", "")

    @staticmethod
    def _sign(value):
        value = float(value)
        if value > 0.0:
            return 1
        if value < 0.0:
            return -1
        return 0

    def _is_turnsign_detection(self, det):
        return self._normalized_label(det) in {"turnsign", "roadsign", "sign"}

    def _limit_error(self, error):
        error = _clamp(error, -self.config.max_track_error_640, self.config.max_track_error_640)
        delta = _clamp(error - self.last_error, -self.config.max_error_step_640, self.config.max_error_step_640)
        return float(self.last_error + delta)

    @staticmethod
    def _command(error, speed, state, flags=CONTROL_FLAG_USE_TARGET_SPEED):
        if int(state) in {STATE_SAFE_STOP, STATE_LINE_LOSS_SAFE_STOP} and float(speed) == 0.0:
            flags = 0
        return {
            "track_error": float(error),
            "target_speed": float(speed),
            "state_cmd": int(state),
            "flags": int(flags),
            "safe_stop": int(state) in {STATE_SAFE_STOP, STATE_LINE_LOSS_SAFE_STOP} and float(speed) == 0.0,
        }

    @staticmethod
    def _extract_ocr_direction(response):
        response = response if isinstance(response, dict) else {}
        instruction = response.get("instruction") or response.get("latest_instruction") or {}
        direction = str(instruction.get("direction") or instruction.get("preferred_branch") or "").lower()
        if bool(response.get("instruction_current")) and direction in {"left", "right"}:
            return direction, True
        return None, False

    def _ocr_has_current_direction(self, response):
        return bool(self.ocr_confirmed_current)

    def _confirm_ocr_direction(self, direction, is_current):
        if not is_current or direction not in {"left", "right"}:
            self.ocr_pending_direction = None
            self.ocr_pending_frames = 0
            self.ocr_confirmed_current = False
            return None, False
        if direction == self.ocr_pending_direction:
            self.ocr_pending_frames += 1
        else:
            self.ocr_pending_direction = direction
            self.ocr_pending_frames = 1
        self.ocr_confirmed_current = (
            self.ocr_pending_frames >= self.config.ocr_confirm_frames)
        if not self.ocr_confirmed_current:
            return None, False
        return direction, True

    def _expire_ocr_lock(self, now):
        if self.branch_lock_source != "ocr" or self.last_valid_ocr_ts <= 0.0:
            return False
        if now - self.last_valid_ocr_ts < self.config.ocr_lock_lifetime_s:
            return False
        self._clear_branch_lock()
        return True

    def _clear_branch_lock(self):
        self.branch_lock = None
        self.branch_lock_source = None
        self.selected_slot_lock = None
        self.selected_slot_missing_frames = 0
        self.last_selected_points = None

    def _ocr_lock_age(self, now):
        if self.branch_lock_source != "ocr" or self.last_valid_ocr_ts <= 0.0:
            return None
        return max(0.0, now - self.last_valid_ocr_ts)

    def _ocr_lock_remaining(self, now):
        age = self._ocr_lock_age(now)
        if age is None:
            return None
        return max(0.0, self.config.ocr_lock_lifetime_s - age)

    def _default_outer_elapsed(self, now):
        if self.fork_seen_since is None:
            return 0.0
        return max(0.0, now - self.fork_seen_since)

    @staticmethod
    def _image_shape(result):
        frame = result.get("_source_frame")
        if frame is None:
            frame = result.get("frame")
        if frame is not None and hasattr(frame, "shape") and len(frame.shape) >= 2:
            return frame.shape
        shape = result.get("image_shape") or result.get("frame_shape")
        if isinstance(shape, (list, tuple)) and len(shape) >= 2:
            return int(shape[0]), int(shape[1]), 3
        return 480, 640, 3

    def _path_pair_stats(self, first_points, second_points, image_shape):
        first = np.asarray(first_points, dtype=np.float32)
        second = np.asarray(second_points, dtype=np.float32)
        if len(first) == 0 or len(second) == 0:
            return {"mean_distance_640": 1e9, "separated_rows": 0}
        ys = np.linspace(
            max(float(np.min(first[:, 1])), float(np.min(second[:, 1]))),
            min(float(np.max(first[:, 1])), float(np.max(second[:, 1]))),
            num=24,
        )
        if ys.size == 0:
            return {"mean_distance_640": 1e9, "separated_rows": 0}
        distances = []
        for y in ys:
            ax = _interp_path_x(first, y)
            bx = _interp_path_x(second, y)
            if ax is None or bx is None:
                continue
            distances.append(abs(ax - bx) * 640.0 / float(max(1, image_shape[1])))
        if not distances:
            return {"mean_distance_640": 1e9, "separated_rows": 0}
        distances = np.asarray(distances, dtype=np.float32)
        return {
            "mean_distance_640": float(np.mean(distances)),
            "separated_rows": int(np.count_nonzero(distances >= self.config.branch_separation_px_640)),
        }

    @staticmethod
    def _summarize_candidate(candidate, image_shape):
        points = np.asarray(candidate.get("points_xy"), dtype=np.float32)
        road_support = _finite_float(candidate.get("road_support"))
        return {
            "slot": int(candidate.get("slot", -1)),
            "role": str(candidate.get("role") or ""),
            "source": str(candidate.get("source") or "unknown"),
            "points": int(len(points)),
            "score": float(candidate.get("score", 0.0)),
            "coverage": float(candidate.get("coverage", 0.0)),
            "road_support": road_support,
            "near_x": None if len(points) == 0 else float(points[np.argmax(points[:, 1]), 0]),
            "lookahead_x": _interp_path_x(points, image_shape[0] * 0.62),
        }

    @staticmethod
    def _debug_path_points(candidate):
        points = np.asarray(candidate.get("points_xy"), dtype=np.float32)
        return VisionControlPlanner._debug_points(points)

    @staticmethod
    def _debug_points(points):
        points = np.asarray(points, dtype=np.float32)
        if len(points) == 0:
            return []
        stride = max(1, int(math.ceil(len(points) / 48.0)))
        return [[float(x), float(y)] for x, y in points[::stride]]


def render_vision_control_debug(frame, result):
    if frame is None or result is None:
        return frame
    debug = (result.get("vision_control") if isinstance(result, dict) else None) or {}
    if not debug:
        return frame
    h, w = frame.shape[:2]
    colors = [(60, 230, 70), (255, 220, 40)]
    selected_slot = debug.get("selected_slot")
    center_x = int(round(w * _clamp(_env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8)))
    cv2.line(frame, (center_x, int(h * 0.45)), (center_x, h - 1), (210, 210, 210), 1, cv2.LINE_AA)

    path_segments = debug.get("candidate_path_segments") or []
    candidate_paths = debug.get("candidate_paths") or []
    for index, pts in enumerate(candidate_paths):
        summaries = debug.get("candidates") or []
        slot = int(summaries[index].get("slot", index)) if index < len(summaries) else index
        color = colors[slot % len(colors)]
        thickness = 3 if slot == selected_slot else 1
        segments = path_segments[index] if index < len(path_segments) else [pts]
        for segment in segments:
            segment = np.asarray(segment, dtype=np.int32)
            if len(segment) < 2:
                continue
            cv2.polylines(
                frame, [segment.reshape((-1, 1, 2))], False,
                color, thickness, cv2.LINE_AA)

    selected_path = np.asarray(debug.get("selected_path") or [], dtype=np.int32)
    if not path_segments and len(selected_path) >= 2:
        cv2.polylines(frame, [selected_path.reshape((-1, 1, 2))], False, (255, 0, 255), 2, cv2.LINE_AA)

    target = debug.get("control_target") or {}
    target_x = _finite_float(target.get("target_x"))
    lookahead_y = _finite_float(target.get("lookahead_y"), h * 0.62)
    if target_x is not None:
        x = int(round(_clamp(target_x, 0, w - 1)))
        y = int(round(_clamp(lookahead_y, 0, h - 1)))
        cv2.circle(frame, (x, y), 7, (255, 0, 255), -1, cv2.LINE_AA)
        cv2.line(frame, (0, y), (w - 1, y), (255, 0, 255), 1, cv2.LINE_AA)

    command = debug.get("command") or {}
    text = "{} raw={} lock={}/{} slot={} err={:.1f}".format(
        debug.get("route_state", "-"),
        debug.get("raw_route_state", "-"),
        debug.get("branch_lock") or "-",
        "-" if debug.get("selected_slot_lock") is None else debug.get("selected_slot_lock"),
        "-" if selected_slot is None else selected_slot,
        float(command.get("track_error", 0.0)),
    )
    cv2.putText(frame, text, (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2, cv2.LINE_AA)
    return frame
