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
        os.environ.get("MULTITASK_PATH_SOURCE", "curve"),
    ).strip().lower()
    return value if value in {"curve", "heatmap"} else "curve"


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


@dataclass
class VisionControlConfig:
    visual_center_x: float = 0.50
    lookahead_y_ratio: float = 0.625
    max_track_error_640: float = 240.0
    max_error_step_640: float = 42.0
    normal_speed_mps: float = 0.05
    recover_speed_mps: float = 0.05
    obstacle_speed_mps: float = 0.05
    human_speed_mps: float = 0.05
    collect_speed_mps: float = 0.05
    heat_threshold: float = 0.22
    heat_peak_top_k: int = 6
    row_step: int = 2
    min_path_points: int = 12
    min_path_coverage: float = 0.28
    min_mean_heat: float = 0.28
    max_link_jump_px: float = 18.0
    road_penalty_weight: float = 0.75
    history_weight: float = 0.035
    jump_weight: float = 0.018
    branch_separation_px_640: float = 70.0
    branch_separation_rows: int = 8
    overlap_px_640: float = 28.0
    default_outer_after_s: float = 15.0
    outer_slot: int = 0
    no_path_stop_s: float = 0.8
    recover_hold_s: float = 0.5
    hazard_bottom_ratio: float = 0.58
    hazard_lateral_ratio: float = 0.18
    coin_bottom_ratio: float = 0.55
    coin_lateral_ratio: float = 0.24
    car_avoid_offset_px_640: float = 55.0
    human_avoid_offset_px_640: float = 75.0
    avoid_box_width_gain: float = 0.35
    gold_bias_gain: float = 0.45
    gold_max_bias_px_640: float = 75.0
    min_human_score: float = 0.35
    min_car_score: float = 0.35
    min_coin_score: float = 0.35
    path_source: str = "curve"

    @classmethod
    def from_env(cls):
        return cls(
            visual_center_x=_clamp(_env_float("VISION_CONTROL_CENTER_X", 0.50), 0.2, 0.8),
            lookahead_y_ratio=_clamp(_env_float("VISION_CONTROL_LOOKAHEAD_Y_RATIO", 0.625), 0.25, 0.95),
            max_track_error_640=max(1.0, _env_float("VISION_CONTROL_MAX_ERROR_640", 240.0)),
            max_error_step_640=max(1.0, _env_float("VISION_CONTROL_MAX_STEP_640", 42.0)),
            normal_speed_mps=max(0.0, _env_float("VISION_CONTROL_NORMAL_SPEED", 0.05)),
            recover_speed_mps=max(0.0, _env_float("VISION_CONTROL_RECOVER_SPEED", 0.05)),
            obstacle_speed_mps=max(0.0, _env_float("VISION_CONTROL_OBSTACLE_SPEED", 0.05)),
            human_speed_mps=max(0.0, _env_float("VISION_CONTROL_HUMAN_SPEED", 0.05)),
            collect_speed_mps=max(0.0, _env_float("VISION_CONTROL_COLLECT_SPEED", 0.05)),
            heat_threshold=_clamp(_env_float("VISION_CONTROL_HEAT_THRESHOLD", 0.22), 0.01, 0.99),
            heat_peak_top_k=max(1, _env_int("VISION_CONTROL_HEAT_TOP_K", 6)),
            row_step=max(1, _env_int("VISION_CONTROL_ROW_STEP", 2)),
            min_path_points=max(3, _env_int("VISION_CONTROL_MIN_PATH_POINTS", 12)),
            min_path_coverage=_clamp(_env_float("VISION_CONTROL_MIN_PATH_COVERAGE", 0.28), 0.01, 1.0),
            min_mean_heat=_clamp(_env_float("VISION_CONTROL_MIN_MEAN_HEAT", 0.28), 0.01, 0.99),
            max_link_jump_px=max(2.0, _env_float("VISION_CONTROL_MAX_LINK_JUMP", 18.0)),
            road_penalty_weight=max(0.0, _env_float("VISION_CONTROL_ROAD_WEIGHT", 0.75)),
            history_weight=max(0.0, _env_float("VISION_CONTROL_HISTORY_WEIGHT", 0.035)),
            jump_weight=max(0.0, _env_float("VISION_CONTROL_JUMP_WEIGHT", 0.018)),
            branch_separation_px_640=max(1.0, _env_float("VISION_CONTROL_BRANCH_SEP_640", 70.0)),
            branch_separation_rows=max(1, _env_int("VISION_CONTROL_BRANCH_SEP_ROWS", 8)),
            overlap_px_640=max(1.0, _env_float("VISION_CONTROL_OVERLAP_640", 28.0)),
            default_outer_after_s=max(0.0, _env_float("VISION_CONTROL_DEFAULT_OUTER_AFTER", 15.0)),
            outer_slot=max(0, min(1, _env_int("VISION_CONTROL_OUTER_SLOT", 0))),
            no_path_stop_s=max(0.1, _env_float("VISION_CONTROL_NO_PATH_STOP_S", 0.8)),
            recover_hold_s=max(0.0, _env_float("VISION_CONTROL_RECOVER_HOLD_S", 0.5)),
            hazard_bottom_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_BOTTOM_RATIO", 0.58), 0.0, 1.0),
            hazard_lateral_ratio=_clamp(_env_float("VISION_CONTROL_HAZARD_LATERAL_RATIO", 0.18), 0.01, 0.5),
            coin_bottom_ratio=_clamp(_env_float("VISION_CONTROL_COIN_BOTTOM_RATIO", 0.55), 0.0, 1.0),
            coin_lateral_ratio=_clamp(_env_float("VISION_CONTROL_COIN_LATERAL_RATIO", 0.24), 0.01, 0.5),
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
                nodes.append({
                    "x": int(x),
                    "y": int(row),
                    "p": float(probability),
                    "road": road_support,
                    "cost": 0.0,
                    "prev": None,
                })
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
        self.branch_lock = None
        self.fork_seen_since = None
        self.single_seen_frames = 0
        self.last_valid_ocr_ts = 0.0
        self.last_ocr_direction = None

    def update(self, perception_result, ocr_response=None, now=None):
        now = float(time.monotonic() if now is None else now)
        start = time.perf_counter()
        perception_result = perception_result if isinstance(perception_result, dict) else {}
        image_shape = self._image_shape(perception_result)
        candidates, search_ms = self._extract_candidates(perception_result, image_shape)
        route_state, route_reason = self._classify_routes(candidates, image_shape)
        self.route_state = route_state
        ocr_direction, ocr_current = self._extract_ocr_direction(ocr_response)
        if ocr_current:
            self.last_valid_ocr_ts = now
            self.last_ocr_direction = ocr_direction
            self.branch_lock = ocr_direction
        self._update_default_outer(route_state, now, ocr_current)
        selected = self._select_candidate(candidates, route_state, image_shape)
        command, control_target = self._build_command(selected, route_state, route_reason, perception_result, image_shape, now)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        debug = {
            "enabled": True,
            "route_state": route_state,
            "route_reason": route_reason,
            "branch_lock": self.branch_lock,
            "ocr_direction": ocr_direction if ocr_current else None,
            "ocr_current": bool(ocr_current),
            "default_outer_elapsed": self._default_outer_elapsed(now),
            "selected_slot": None if selected is None else int(selected.get("slot", -1)),
            "candidate_count": len(candidates),
            "candidates": [self._summarize_candidate(item, image_shape) for item in candidates],
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
            candidates.sort(key=lambda item: int(item.get("slot", 99)))
            elapsed = (time.perf_counter() - started) * 1000.0
            return candidates, elapsed

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
        elapsed = (time.perf_counter() - started) * 1000.0
        return candidates, elapsed

    def _classify_routes(self, candidates, image_shape):
        if not candidates:
            return ROUTE_NONE, "no_candidate"
        if len(candidates) == 1:
            return ROUTE_SINGLE, "one_candidate"
        first, second = candidates[:2]
        stats = self._path_pair_stats(first["points_xy"], second["points_xy"], image_shape)
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
            return
        if route_state == ROUTE_SINGLE:
            self.fork_seen_since = None
            self.single_seen_frames += 1
            if self.single_seen_frames >= 6:
                self.branch_lock = None
            return
        if route_state in {ROUTE_NONE, ROUTE_AMBIGUOUS}:
            self.single_seen_frames = 0
            return

    def _select_candidate(self, candidates, route_state, image_shape):
        if not candidates:
            return None
        if self.branch_lock in {"left", "right"}:
            wanted_slot = 0 if self.branch_lock == "left" else 1
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == wanted_slot:
                    return candidate
        if self.last_selected_points is not None:
            previous_x = _interp_path_x(self.last_selected_points, image_shape[0] * self.config.lookahead_y_ratio)
            if previous_x is not None:
                return min(
                    candidates,
                    key=lambda item: abs(
                        (_interp_path_x(item["points_xy"], image_shape[0] * self.config.lookahead_y_ratio) or previous_x)
                        - previous_x
                    ),
                )
        if route_state == ROUTE_MULTI_FORK:
            for candidate in candidates:
                if int(candidate.get("slot", -1)) == self.config.outer_slot:
                    return candidate
        center_x = image_shape[1] * self.config.visual_center_x
        return min(
            candidates,
            key=lambda item: abs((_interp_path_x(item["points_xy"], image_shape[0] * self.config.lookahead_y_ratio) or center_x) - center_x),
        )

    def _build_command(self, selected, route_state, route_reason, result, image_shape, now):
        task_state, speed, task_reason, target_override_x = self._task_from_detections(result, selected, image_shape)
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

        lookahead_y = image_shape[0] * self.config.lookahead_y_ratio
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

    def _task_from_detections(self, result, selected, image_shape):
        detections = result.get("detections") or []
        if selected is None:
            return STATE_RECOVER_LINE, self.config.recover_speed_mps, "no_path", None
        target_path_x = _interp_path_x(selected["points_xy"], image_shape[0] * 0.68)
        if target_path_x is None:
            target_path_x = image_shape[1] * self.config.visual_center_x
        hazard_limit = image_shape[1] * self.config.hazard_lateral_ratio
        for det in detections:
            label = str(det.get("label") or det.get("category") or "").lower()
            if label not in {"human", "car"}:
                continue
            score = _finite_float(det.get("score"), 0.0)
            if label == "human" and score < self.config.min_human_score:
                continue
            if label == "car" and score < self.config.min_car_score:
                continue
            geom = self._detection_geom(det, image_shape)
            if geom is None or geom["bottom_ratio"] < self.config.hazard_bottom_ratio:
                continue
            if abs(geom["cx"] - target_path_x) > max(hazard_limit, geom["box_w"] * 0.75):
                continue
            avoid_target_x = self._avoid_target_x(label, geom, target_path_x, image_shape)
            if label == "human":
                return STATE_AVOID_HUMAN, self.config.human_speed_mps, "human_in_path_bias", avoid_target_x
            return STATE_AVOID_CAR, self.config.obstacle_speed_mps, "car_in_path_bias", avoid_target_x
        coin = self._best_coin(detections, image_shape, target_path_x)
        if coin is not None:
            return STATE_COLLECT_GOLD, self.config.collect_speed_mps, "coin_bias", self._coin_target_x(coin, target_path_x, image_shape)
        return STATE_TRACK, self.config.normal_speed_mps, "track", None

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
            "cx": (left + right) * 0.5,
            "cy": (top + bottom) * 0.5,
            "box_w": right - left,
            "box_h": bottom - top,
            "bottom_ratio": bottom / float(max(1, image_shape[0])),
        }

    def _limit_error(self, error):
        error = _clamp(error, -self.config.max_track_error_640, self.config.max_track_error_640)
        delta = _clamp(error - self.last_error, -self.config.max_error_step_640, self.config.max_error_step_640)
        return float(self.last_error + delta)

    @staticmethod
    def _command(error, speed, state, flags=CONTROL_FLAG_USE_TARGET_SPEED):
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

    for index, pts in enumerate(debug.get("candidate_paths") or []):
        slot = index
        pts = np.asarray(pts, dtype=np.int32)
        if len(pts) < 2:
            continue
        color = colors[slot % len(colors)]
        thickness = 3 if slot == selected_slot else 1
        cv2.polylines(frame, [pts.reshape((-1, 1, 2))], False, color, thickness, cv2.LINE_AA)

    selected_path = np.asarray(debug.get("selected_path") or [], dtype=np.int32)
    if len(selected_path) >= 2:
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
    text = "{} lock={} slot={} err={:.1f}".format(
        debug.get("route_state", "-"),
        debug.get("branch_lock") or "-",
        "-" if selected_slot is None else selected_slot,
        float(command.get("track_error", 0.0)),
    )
    cv2.putText(frame, text, (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2, cv2.LINE_AA)
    return frame
