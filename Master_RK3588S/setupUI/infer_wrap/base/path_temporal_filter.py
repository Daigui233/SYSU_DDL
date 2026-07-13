"""Causal temporal filtering for ordered multi-task path points."""

import os
import time

import numpy as np


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


class PathTemporalFilter:
    """Smooth paths without buffering future frames or adding frame latency."""

    def __init__(self, alpha=None, count_confirm_frames=None,
                 lost_hold_frames=None, max_jump_ratio=None,
                 reset_gap_seconds=None, enabled=None):
        self.enabled = (
            _env_flag("MULTITASK_PATH_TEMPORAL_FILTER", True)
            if enabled is None else bool(enabled))
        self.alpha = float(np.clip(
            _env_float("MULTITASK_PATH_EMA_ALPHA", 0.45)
            if alpha is None else alpha, 0.0, 1.0))
        self.count_confirm_frames = max(1, int(
            _env_int("MULTITASK_PATH_COUNT_CONFIRM_FRAMES", 2)
            if count_confirm_frames is None else count_confirm_frames))
        self.lost_hold_frames = max(1, int(
            _env_int("MULTITASK_PATH_LOST_HOLD_FRAMES", 3)
            if lost_hold_frames is None else lost_hold_frames))
        self.max_jump_ratio = max(0.0, float(
            _env_float("MULTITASK_PATH_MAX_JUMP_RATIO", 0.15)
            if max_jump_ratio is None else max_jump_ratio))
        self.reset_gap_seconds = max(0.0, float(
            _env_float("MULTITASK_PATH_RESET_GAP_SECONDS", 0.50)
            if reset_gap_seconds is None else reset_gap_seconds))
        self.reset()

    def reset(self):
        self.stable_count = None
        self.stable_count_confidence = 0.0
        self.pending_count = None
        self.pending_frames = 0
        self.states = {}
        self.last_update_ts = None

    @staticmethod
    def _copy_path(path):
        copied = dict(path)
        copied["points_xy"] = np.asarray(
            path["points_xy"], dtype=np.float32).copy()
        normalized = path.get("points_normalized")
        if normalized is not None:
            copied["points_normalized"] = np.asarray(
                normalized, dtype=np.float32).copy()
        return copied

    @staticmethod
    def _path_map(paths):
        return {str(path.get("role")): path for path in paths}

    def _required_count_frames(self, next_count):
        return (self.lost_hold_frames if next_count == 0
                else self.count_confirm_frames)

    def _update_stable_count(self, raw_count, raw_confidence):
        if self.stable_count is None:
            self.stable_count = raw_count
            self.stable_count_confidence = raw_confidence
            return True
        if raw_count == self.stable_count:
            self.pending_count = None
            self.pending_frames = 0
            self.stable_count_confidence = raw_confidence
            return False
        if raw_count != self.pending_count:
            self.pending_count = raw_count
            self.pending_frames = 1
        else:
            self.pending_frames += 1
        if self.pending_frames < self._required_count_frames(raw_count):
            return False
        self.stable_count = raw_count
        self.stable_count_confidence = raw_confidence
        self.pending_count = None
        self.pending_frames = 0
        self.states = {}
        return True

    def _new_state(self, path):
        copied = self._copy_path(path)
        return {
            "path": copied,
            "score": float(path.get("score", 0.0)),
            "rejected_frames": 0,
        }

    def _smooth_path(self, current, state):
        current_points = np.asarray(current["points_xy"], dtype=np.float32)
        previous_points = state["path"]["points_xy"]
        current_normalized = current.get("points_normalized")
        previous_normalized = state["path"].get("points_normalized")
        if current_normalized is not None and previous_normalized is not None:
            normalized_delta = (
                np.asarray(current_normalized, dtype=np.float32) -
                np.asarray(previous_normalized, dtype=np.float32))
        else:
            frame_scale = np.maximum(np.max(previous_points, axis=0), 1.0)
            normalized_delta = (current_points - previous_points) / frame_scale
        jump = float(np.median(np.linalg.norm(normalized_delta, axis=1)))

        if self.max_jump_ratio > 0.0 and jump > self.max_jump_ratio:
            state["rejected_frames"] += 1
            if state["rejected_frames"] < self.lost_hold_frames:
                held = self._copy_path(state["path"])
                held["held"] = True
                held["jump_rejected"] = True
                held["score"] = state["score"] * (
                    0.85 ** state["rejected_frames"])
                return held
            state.clear()
            state.update(self._new_state(current))
            accepted = self._copy_path(state["path"])
            accepted["reinitialized"] = True
            return accepted

        state["rejected_frames"] = 0
        alpha = self.alpha
        smoothed = current_points * alpha + previous_points * (1.0 - alpha)
        filtered = self._copy_path(current)
        filtered["points_xy"] = smoothed.astype(np.float32, copy=False)

        if current_normalized is not None and previous_normalized is not None:
            filtered["points_normalized"] = (
                np.asarray(current_normalized, dtype=np.float32) * alpha +
                np.asarray(previous_normalized, dtype=np.float32) *
                (1.0 - alpha)
            ).astype(np.float32, copy=False)
        state["score"] = (
            float(current.get("score", 0.0)) * alpha +
            state["score"] * (1.0 - alpha))
        filtered["score"] = state["score"]
        filtered["held"] = False
        state["path"] = self._copy_path(filtered)
        return filtered

    def _held_paths(self):
        held = []
        for role in self._roles_for_count(self.stable_count):
            state = self.states.get(role)
            if state is None:
                continue
            path = self._copy_path(state["path"])
            path["held"] = True
            path["count_transition_hold"] = True
            held.append(path)
        return held

    @staticmethod
    def _roles_for_count(path_count):
        if path_count == 1:
            return ("single",)
        if path_count == 2:
            return ("left", "right")
        return ()

    def update(self, result, timestamp=None):
        """Apply filtering in place and return the same result dictionary."""
        if not self.enabled or not isinstance(result, dict):
            return result

        now = float(time.monotonic() if timestamp is None else timestamp)
        if (self.last_update_ts is not None and self.reset_gap_seconds > 0.0
                and now - self.last_update_ts > self.reset_gap_seconds):
            self.reset()
        self.last_update_ts = now

        raw_count = int(result.get("path_count", 0))
        raw_paths = list(result.get("paths") or [])
        count_scores = result.get("path_count_scores") or [0.0, 0.0, 0.0]
        raw_confidence = (
            float(count_scores[raw_count])
            if 0 <= raw_count < len(count_scores) else 0.0)
        result["raw_path_count"] = raw_count
        result["raw_paths"] = raw_paths

        count_changed = self._update_stable_count(raw_count, raw_confidence)
        stable_count = int(self.stable_count or 0)
        expected_roles = self._roles_for_count(stable_count)

        if stable_count == 0:
            filtered_paths = []
            status = "no_path"
        elif raw_count != stable_count:
            filtered_paths = self._held_paths()
            status = "count_transition_hold"
        else:
            current_by_role = self._path_map(raw_paths)
            filtered_paths = []
            for role in expected_roles:
                current = current_by_role.get(role)
                if current is None:
                    continue
                state = self.states.get(role)
                if state is None or count_changed:
                    self.states[role] = self._new_state(current)
                    filtered = self._copy_path(current)
                    filtered["held"] = False
                else:
                    filtered = self._smooth_path(current, state)
                filtered["count_confidence"] = self.stable_count_confidence
                filtered_paths.append(filtered)
            status = "tracking" if filtered_paths else "path_unavailable"

        result["path_count"] = stable_count
        result["paths"] = filtered_paths
        result["temporal"] = {
            "enabled": True,
            "status": status,
            "raw_path_count": raw_count,
            "stable_path_count": stable_count,
            "pending_path_count": self.pending_count,
            "pending_frames": self.pending_frames,
            "alpha": self.alpha,
        }
        centerline = result.get("centerline")
        if isinstance(centerline, dict):
            centerline["raw_path_count"] = raw_count
            centerline["raw_paths"] = raw_paths
            centerline["path_count"] = stable_count
            centerline["paths"] = filtered_paths
            centerline["count_confidence"] = self.stable_count_confidence
            centerline["temporal"] = result["temporal"]
        return result
