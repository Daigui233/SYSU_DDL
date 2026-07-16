"""Low-overhead JSONL telemetry for diagnosing vision-control oscillation."""

import json
import os
from pathlib import Path
import time

import numpy as np


def _env_flag(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _number(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if np.isfinite(result) else default


def _path_x_at(points, target_y):
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        return None
    order = np.argsort(points[:, 1])
    ys = points[order, 1]
    xs = points[order, 0]
    target_y = float(target_y)
    if target_y <= float(ys[0]):
        return float(xs[0])
    if target_y >= float(ys[-1]):
        return float(xs[-1])
    return float(np.interp(target_y, ys, xs))


class VisionTelemetryLogger:
    """Write one compact, flush-on-each-frame record per vision update."""

    def __init__(self, directory=None):
        default_directory = Path(__file__).resolve().parent / "dist" / "vision_runs"
        self.directory = Path(directory or default_directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = self.directory / f"vision_run_{stamp}_{os.getpid()}.jsonl"
        self._file = self.path.open("w", encoding="utf-8", buffering=1)
        self._file.write(json.dumps({
            "record_type": "metadata",
            "created_wall_time": time.time(),
            "pid": os.getpid(),
            "format": 1,
        }, ensure_ascii=False) + "\n")

    @classmethod
    def from_env(cls):
        if not _env_flag("AR_VISION_TELEMETRY", True):
            return None
        return cls(os.environ.get("AR_VISION_TELEMETRY_DIR"))

    @staticmethod
    def _path_summary(paths, lookahead_y):
        summary = []
        for path in list(paths or [])[:2]:
            points = path.get("points_xy") if isinstance(path, dict) else None
            points_array = np.asarray(points, dtype=np.float32)
            if (points_array.ndim != 2 or points_array.shape[1] != 2 or
                    len(points_array) == 0):
                continue
            summary.append({
                "slot": int(path.get("slot", -1)),
                "role": str(path.get("role") or ""),
                "point_count": int(len(points_array)),
                "min_y": _number(np.min(points_array[:, 1])),
                "max_y": _number(np.max(points_array[:, 1])),
                "x_y220": _path_x_at(points_array, 220.0),
                "x_lookahead": _path_x_at(points_array, lookahead_y),
                "x_y380": _path_x_at(points_array, 380.0),
                "x_y420": _path_x_at(points_array, 420.0),
                "coverage": _number(path.get("coverage"), 0.0),
                "row_support": int(path.get("row_support", 0)),
            })
        return summary

    @staticmethod
    def _frame_stats(frame):
        if not isinstance(frame, np.ndarray) or frame.size == 0:
            return {"mean": None, "std": None, "min": None, "max": None}
        sample = frame[::16, ::16]
        return {
            "mean": _number(np.mean(sample)),
            "std": _number(np.std(sample)),
            "min": int(np.min(sample)),
            "max": int(np.max(sample)),
        }

    def log_frame(
            self, frame_id, frame, perception=None, command=None,
            vision_debug=None, runtime_snapshot=None,
            camera_status="ok"):
        if self._file is None:
            return
        debug = vision_debug if isinstance(vision_debug, dict) else {}
        target = debug.get("control_target") or {}
        command = command if isinstance(command, dict) else {}
        runtime_snapshot = (
            runtime_snapshot if isinstance(runtime_snapshot, dict) else {})
        serial = runtime_snapshot.get("serial") or {}
        last_command = runtime_snapshot.get("last_command") or {}
        result = perception if isinstance(perception, dict) else {}
        temporal = result.get("temporal") or {}
        record = {
            "record_type": "frame",
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
            "frame_id": int(frame_id),
            "camera_status": str(camera_status),
            "frame": self._frame_stats(frame),
            "path": {
                "lookahead_y": _number(target.get("lookahead_y")),
                "path_target_x": _number(target.get("path_target_x")),
                "path_target_y": _number(target.get("path_target_y")),
                "path_target_adaptive_y": bool(
                    target.get("path_target_adaptive_y", False)),
                "path_target_held": bool(
                    target.get("path_target_held", False)),
                "path_raw_error_640": _number(
                    target.get("path_raw_track_error_640")),
                "raw_error_640": _number(target.get("raw_track_error_640")),
                "path_heading_feedforward_640": _number(
                    target.get("path_heading_feedforward_640"), 0.0),
                "curve_feedforward_640": _number(
                    target.get("curve_feedforward_640"), 0.0),
                "right_circle_feedforward_640": _number(
                    target.get("right_circle_feedforward_640"), 0.0),
                "right_circle_feedforward_active": bool(
                    target.get("right_circle_feedforward_active", False)),
                "right_circle_trim_640": _number(
                    target.get("right_circle_trim_640"), 0.0),
                "right_circle_compensation_640": _number(
                    target.get("right_circle_compensation_640"), 0.0),
                "right_circle_anchor_weight": _number(
                    target.get("right_circle_anchor_weight"), 0.0),
                "right_circle_compensation_gain": _number(
                    target.get("right_circle_compensation_gain"), 0.0),
                "right_circle_output_offset_640": _number(
                    target.get("right_circle_output_offset_640"), 0.0),
                "line_based_track_error_640": _number(
                    target.get("line_based_track_error_640")),
                "total_feedforward_640": _number(
                    target.get("total_feedforward_640"), 0.0),
                "road_shape_state": target.get(
                    "road_shape_state", debug.get("road_shape_state")),
                "road_shape_valid": bool(target.get(
                    "road_shape_valid", debug.get("road_shape_valid", False))),
                "road_geometry_reason": target.get(
                    "road_geometry_reason"),
                "road_geometry_sample_rows_y": target.get(
                    "road_geometry_sample_rows_y") or [],
                "road_geometry_support_y": target.get(
                    "road_geometry_support_y") or [],
                "road_heading_delta_640": _number(target.get(
                    "road_heading_delta_640",
                    debug.get("road_heading_delta_640"))),
                "road_curvature_640": _number(target.get(
                    "road_curvature_640", debug.get("road_curvature_640"))),
                "selected_path_curvature_640": _number(
                    target.get("selected_path_curvature_640")),
                "road_curve_evidence": (
                    target.get("road_curve_evidence")
                    or debug.get("road_curve_evidence")
                    or {}),
                "paths": self._path_summary(
                    result.get("paths") or (result.get("centerline") or {}).get("paths"),
                    _number(target.get("lookahead_y"), 300.0)),
            },
            "control": {
                "track_error": _number(
                    command.get("track_error"),
                    _number(target.get("track_error_640"))),
                "target_speed": _number(command.get("target_speed")),
                "state_cmd": command.get("state_cmd"),
                "task_reason": target.get("task_reason"),
                "reason": target.get("reason"),
                "response": target.get("track_error_response"),
                "trend_frames": target.get("track_error_trend_frames"),
                "task_offset_x": _number(target.get("task_offset_x")),
                "line_loss_hold": bool(
                    target.get("line_loss_hold", False)),
                "line_loss_indefinite": bool(
                    target.get("line_loss_indefinite", False)),
                "line_loss_held_speed_mps": _number(
                    target.get("line_loss_held_speed_mps")),
            },
            "selection": {
                "selected_slot": debug.get("selected_slot"),
                "selected_slot_lock": debug.get("selected_slot_lock"),
                "branch_lock": debug.get("branch_lock"),
                "selection_reason": debug.get("selection_reason"),
                "line_loss_active": bool(
                    debug.get("line_loss_active", False)),
                "line_loss_reason": debug.get("line_loss_reason"),
                "line_connection_reason": debug.get(
                    "line_connection_reason"),
                "line_connection_metrics": debug.get(
                    "line_connection_metrics") or {},
                "curve_merge_override": bool(
                    debug.get("curve_merge_override", False)),
                "curve_merge_reason": debug.get("curve_merge_reason"),
            },
            "temporal": {
                "mode": temporal.get("mode"),
                "status": temporal.get("status"),
            },
            "runtime": {
                "vision_age": _number(runtime_snapshot.get("vision_age")),
                "serial_input_error": _number(
                    serial.get("input_track_error")),
                "serial_servo_output": serial.get("servo_output"),
                "serial_servo_raw_output": _number(
                    serial.get("servo_raw_output")),
                "serial_servo_limited_output": _number(
                    serial.get("servo_limited_output")),
                "runtime_last_track_error": _number(
                    last_command.get("track_error")),
            },
        }
        try:
            self._file.write(json.dumps(record, ensure_ascii=False) + "\n")
        except (OSError, TypeError, ValueError):
            # Telemetry must never interrupt camera/control processing.
            pass

    def close(self):
        if self._file is not None:
            try:
                self._file.flush()
                self._file.close()
            except OSError:
                pass
            self._file = None
