#!/usr/bin/env python3
"""Interactive steering calibration tool for the SYSU_DDL vehicle.

The tool keeps sending a zero-speed TRACK command while steering output is
enabled, records measured left/right Ackermann wheel angles, and exports the
result as JSON, CSV, or a C lookup-table header.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import threading
import time
import webbrowser
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional
from urllib.parse import urlparse

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

from control_serial_comm import (
    CONTROL_FLAG_USE_TARGET_SPEED,
    STATE_SAFE_STOP,
    STATE_TRACK,
    CarController,
)


APP_DIR = Path(__file__).resolve().parent
UI_PATH = APP_DIR / "steering_calibration_ui.html"
DEFAULT_DATA_DIR = APP_DIR / "steering_calibration_data"
COMMAND_PERIOD_S = 0.05
ERROR_MIN = -160.0
ERROR_MAX = 160.0
ACK_MAX_AGE_S = 0.25
ACK_ERROR_TOLERANCE = 0.02
CONTROL_FB_FLAG_INPUT_TIMEOUT = 0x0001


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是数字") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    return result


def calculate_ackermann(
    left_deg: float,
    right_deg: float,
    wheelbase_mm: float,
    track_width_mm: float,
) -> Dict[str, Optional[float]]:
    """Estimate the axle-center steering angle and curvature.

    User-facing angles follow the vehicle convention: clockwise/right is
    positive, counter-clockwise/left is negative, and magnitude is acute.
    """

    left = finite_float(left_deg, "左轮角度")
    right = finite_float(right_deg, "右轮角度")
    wheelbase = finite_float(wheelbase_mm, "轴距")
    track = finite_float(track_width_mm, "前轮轮距")
    if wheelbase <= 0.0:
        raise ValueError("轴距必须大于 0")
    if track <= 0.0:
        raise ValueError("前轮轮距必须大于 0")
    if abs(left) >= 90.0 or abs(right) >= 90.0:
        raise ValueError("左右轮角度必须是 (-90°, 90°) 内的锐角")

    radii: List[float] = []
    left_radius = None
    right_radius = None
    if abs(left) > 1e-6:
        left_radius = track / 2.0 + wheelbase / math.tan(math.radians(-left))
        radii.append(left_radius)
    if abs(right) > 1e-6:
        right_radius = -track / 2.0 + wheelbase / math.tan(math.radians(-right))
        radii.append(right_radius)

    if not radii:
        return {
            "center_angle_deg": 0.0,
            "turn_radius_mm": None,
            "curvature_per_m": 0.0,
            "radius_mismatch_mm": None,
        }

    mathematical_radius = sum(radii) / len(radii)
    if abs(mathematical_radius) < 1e-9:
        center_angle = math.copysign(90.0, -mathematical_radius or 1.0)
        curvature = None
    else:
        center_angle = -math.degrees(math.atan(wheelbase / mathematical_radius))
        curvature = -1000.0 / mathematical_radius

    mismatch = None
    if left_radius is not None and right_radius is not None:
        mismatch = right_radius - left_radius

    return {
        "center_angle_deg": center_angle,
        "turn_radius_mm": -mathematical_radius,
        "curvature_per_m": curvature,
        "radius_mismatch_mm": mismatch,
    }


def aggregate_samples(samples: Iterable[Dict[str, Any]]) -> List[Dict[str, float]]:
    """Average repeated measurements made at the same error value."""

    groups: Dict[float, List[Dict[str, Any]]] = {}
    for sample in samples:
        key = round(float(sample["error"]), 6)
        groups.setdefault(key, []).append(sample)

    result = []
    for error in sorted(groups):
        rows = groups[error]
        result.append(
            {
                "error": error,
                "left_angle_deg": sum(float(row["left_angle_deg"]) for row in rows) / len(rows),
                "right_angle_deg": sum(float(row["right_angle_deg"]) for row in rows) / len(rows),
                "center_angle_deg": sum(float(row["center_angle_deg"]) for row in rows) / len(rows),
                "sample_count": float(len(rows)),
            }
        )
    return result


def _solve_linear_system(matrix: List[List[float]], vector: List[float]) -> List[float]:
    size = len(vector)
    augmented = [list(matrix[row]) + [float(vector[row])] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("拟合矩阵不可逆")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                augmented[row][index] - factor * augmented[column][index]
                for index in range(size + 1)
            ]
    return [augmented[row][-1] for row in range(size)]


def polynomial_fit(
    x_values: Iterable[float],
    y_values: Iterable[float],
    max_degree: int = 3,
) -> Dict[str, Any]:
    pairs = [
        (float(x), float(y))
        for x, y in zip(x_values, y_values)
        if math.isfinite(float(x)) and math.isfinite(float(y))
    ]
    unique_x = sorted({round(x, 9) for x, _ in pairs})
    if len(pairs) < 2 or len(unique_x) < 2:
        return {"available": False, "reason": "至少需要两个不同的标定输入点"}

    degree = min(max(1, int(max_degree)), len(unique_x) - 1, len(pairs) - 1)
    matrix = []
    vector = []
    for row_power in range(degree + 1):
        matrix.append(
            [
                sum(x ** (row_power + column_power) for x, _ in pairs)
                for column_power in range(degree + 1)
            ]
        )
        vector.append(sum(y * (x ** row_power) for x, y in pairs))
    try:
        coefficients = _solve_linear_system(matrix, vector)
    except ValueError:
        return {"available": False, "reason": "标定点不足以形成稳定拟合"}

    predictions = [
        sum(coefficient * (x ** power) for power, coefficient in enumerate(coefficients))
        for x, _ in pairs
    ]
    residual_sum = sum((prediction - y) ** 2 for prediction, (_, y) in zip(predictions, pairs))
    rmse = math.sqrt(residual_sum / len(pairs))
    mean_y = sum(y for _, y in pairs) / len(pairs)
    total_sum = sum((y - mean_y) ** 2 for _, y in pairs)
    r_squared = None if total_sum < 1e-12 else 1.0 - residual_sum / total_sum
    terms = []
    for power, coefficient in enumerate(coefficients):
        if power == 0:
            terms.append(f"{coefficient:.8g}")
        elif power == 1:
            terms.append(f"{coefficient:+.8g}*x")
        else:
            terms.append(f"{coefficient:+.8g}*x^{power}")
    return {
        "available": True,
        "degree": degree,
        "coefficients": coefficients,
        "rmse": rmse,
        "r_squared": r_squared,
        "equation": "y=" + "".join(terms),
        "sample_count": len(pairs),
    }


def build_fit_report(samples: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    sample_list = list(samples)
    points = aggregate_samples(sample_list)
    error_values = [row["error"] for row in points]
    center_values = [row["center_angle_deg"] for row in points]
    models = {
        "left_angle_from_error": {
            "input": "error",
            "output": "left_angle_deg",
            **polynomial_fit(error_values, [row["left_angle_deg"] for row in points]),
        },
        "right_angle_from_error": {
            "input": "error",
            "output": "right_angle_deg",
            **polynomial_fit(error_values, [row["right_angle_deg"] for row in points]),
        },
        "center_angle_from_error": {
            "input": "error",
            "output": "center_angle_deg",
            **polynomial_fit(error_values, center_values),
        },
        "error_from_center_angle": {
            "input": "center_angle_deg",
            "output": "error",
            **polynomial_fit(center_values, error_values),
        },
    }
    return {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "raw_sample_count": len(sample_list),
        "unique_error_count": len(points),
        "recommended_control_model": "piecewise_linear_inverse_lut",
        "models": models,
    }


def inverse_calibration_points(samples: Iterable[Dict[str, Any]]) -> List[Dict[str, float]]:
    grouped: Dict[float, List[Dict[str, float]]] = {}
    for row in aggregate_samples(samples):
        key = round(float(row["center_angle_deg"]), 6)
        grouped.setdefault(key, []).append(row)
    result = []
    for center_angle in sorted(grouped):
        rows = grouped[center_angle]
        result.append(
            {
                "center_angle_deg": center_angle,
                "error": sum(row["error"] for row in rows) / len(rows),
            }
        )
    return result


@dataclass
class CalibrationPaths:
    json_path: Path
    csv_path: Path
    header_path: Path
    fit_path: Path


class CalibrationStore:
    def __init__(self, data_dir: Path):
        self.data_dir = Path(data_dir).resolve()
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.paths = CalibrationPaths(
            json_path=self.data_dir / "steering_calibration.json",
            csv_path=self.data_dir / "steering_calibration.csv",
            header_path=self.data_dir / "steering_calibration_lut.h",
            fit_path=self.data_dir / "steering_calibration_fit.json",
        )
        self._lock = threading.RLock()
        self.config: Dict[str, Any] = {
            "wheelbase_mm": 200.0,
            "track_width_mm": 140.0,
            "angle_convention": "right_turn_positive_acute",
        }
        self.samples: List[Dict[str, Any]] = []
        self.next_id = 1
        self.load()

    def load(self) -> None:
        if not self.paths.json_path.exists():
            return
        try:
            payload = json.loads(self.paths.json_path.read_text(encoding="utf-8"))
            config = payload.get("config", {})
            samples = payload.get("samples", [])
            if isinstance(config, dict):
                self.config.update(config)
                self.config["angle_convention"] = "right_turn_positive_acute"
            if isinstance(samples, list):
                self.samples = [row for row in samples if isinstance(row, dict)]
                self.next_id = max((int(row.get("id", 0)) for row in self.samples), default=0) + 1
        except (OSError, ValueError, TypeError):
            # Keep the tool usable if an interrupted/manual edit damaged the file.
            self.samples = []
            self.next_id = 1

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "config": dict(self.config),
                "samples": [dict(row) for row in self.samples],
                "paths": {
                    "json": str(self.paths.json_path),
                    "csv": str(self.paths.csv_path),
                    "header": str(self.paths.header_path),
                    "fit": str(self.paths.fit_path),
                },
                "fit": build_fit_report(self.samples),
            }

    def update_config(self, wheelbase_mm: Any, track_width_mm: Any) -> Dict[str, Any]:
        wheelbase = finite_float(wheelbase_mm, "轴距")
        track = finite_float(track_width_mm, "前轮轮距")
        if wheelbase <= 0.0 or track <= 0.0:
            raise ValueError("轴距和轮距必须大于 0")
        with self._lock:
            self.config["wheelbase_mm"] = wheelbase
            self.config["track_width_mm"] = track
            self._recalculate_locked()
            self.save_locked()
            return dict(self.config)

    def add_sample(
        self,
        error: Any,
        servo_duty: Optional[Any],
        left_angle_deg: Any,
        right_angle_deg: Any,
        note: Any = "",
    ) -> Dict[str, Any]:
        error_value = clamp(finite_float(error, "error"), ERROR_MIN, ERROR_MAX)
        left = finite_float(left_angle_deg, "左轮角度")
        right = finite_float(right_angle_deg, "右轮角度")
        duty = None
        if servo_duty is not None:
            duty = int(finite_float(servo_duty, "舵机占空比"))

        with self._lock:
            derived = calculate_ackermann(
                left,
                right,
                self.config["wheelbase_mm"],
                self.config["track_width_mm"],
            )
            sample = {
                "id": self.next_id,
                "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
                "error": error_value,
                "servo_duty": duty,
                "left_angle_deg": left,
                "right_angle_deg": right,
                "center_angle_deg": derived["center_angle_deg"],
                "turn_radius_mm": derived["turn_radius_mm"],
                "curvature_per_m": derived["curvature_per_m"],
                "radius_mismatch_mm": derived["radius_mismatch_mm"],
                "note": str(note or "").strip(),
            }
            self.samples.append(sample)
            self.next_id += 1
            self.save_locked()
            return dict(sample)

    def delete_sample(self, sample_id: int) -> bool:
        with self._lock:
            old_length = len(self.samples)
            self.samples = [row for row in self.samples if int(row.get("id", -1)) != int(sample_id)]
            changed = len(self.samples) != old_length
            if changed:
                self.save_locked()
            return changed

    def save(self) -> None:
        with self._lock:
            self.save_locked()

    def save_locked(self) -> None:
        payload = {
            "version": 1,
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "config": self.config,
            "samples": self.samples,
        }
        json_text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
        self._atomic_write(self.paths.json_path, json_text)
        self._atomic_write(self.paths.csv_path, self.csv_text())
        self._atomic_write(self.paths.header_path, self.header_text())
        fit_text = json.dumps(build_fit_report(self.samples), ensure_ascii=False, indent=2) + "\n"
        self._atomic_write(self.paths.fit_path, fit_text)

    def _recalculate_locked(self) -> None:
        for sample in self.samples:
            derived = calculate_ackermann(
                sample["left_angle_deg"],
                sample["right_angle_deg"],
                self.config["wheelbase_mm"],
                self.config["track_width_mm"],
            )
            sample.update(derived)

    @staticmethod
    def _atomic_write(path: Path, content: str) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(content, encoding="utf-8", newline="")
        temporary.replace(path)

    def csv_text(self) -> str:
        output = io.StringIO(newline="")
        fields = [
            "id",
            "timestamp",
            "error",
            "servo_duty",
            "left_angle_deg",
            "right_angle_deg",
            "center_angle_deg",
            "turn_radius_mm",
            "curvature_per_m",
            "radius_mismatch_mm",
            "note",
        ]
        writer = csv.DictWriter(output, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(self.samples)
        return output.getvalue()

    def header_text(self) -> str:
        rows = aggregate_samples(self.samples)
        inverse_rows = inverse_calibration_points(self.samples)
        lines = [
            "/* Auto-generated by steering_calibration_tool.py. */",
            "#ifndef STEERING_CALIBRATION_LUT_H_",
            "#define STEERING_CALIBRATION_LUT_H_",
            "",
            "#include <stdint.h>",
            "",
            "typedef struct",
            "{",
            "    float error;",
            "    float left_angle_deg;",
            "    float right_angle_deg;",
            "    float center_angle_deg;",
            "} steering_calibration_point_t;",
            "",
            f"#define STEERING_CALIBRATION_POINT_COUNT ({len(rows)}U)",
        ]
        if rows:
            lines.extend(
                [
                    "",
                    "static const steering_calibration_point_t steering_calibration_lut[] =",
                    "{",
                ]
            )
            for row in rows:
                lines.append(
                    "    {%.6ff, %.6ff, %.6ff, %.6ff},"
                    % (
                        row["error"],
                        row["left_angle_deg"],
                        row["right_angle_deg"],
                        row["center_angle_deg"],
                    )
                )
            lines.extend(["};"])
        lines.extend(
            [
                "",
                "static float steering_center_angle_from_error(float error)",
                "{",
            ]
        )
        if not rows:
            lines.append("    (void)error;")
            lines.append("    return 0.0f;")
        elif len(rows) == 1:
            lines.append("    (void)error;")
            lines.append(f"    return {rows[0]['center_angle_deg']:.6f}f;")
        else:
            lines.extend(
                [
                    "    uint32_t i;",
                    "    if (error <= steering_calibration_lut[0].error)",
                    "    {",
                    "        return steering_calibration_lut[0].center_angle_deg;",
                    "    }",
                    "    if (error >= steering_calibration_lut[STEERING_CALIBRATION_POINT_COUNT - 1U].error)",
                    "    {",
                    "        return steering_calibration_lut[STEERING_CALIBRATION_POINT_COUNT - 1U].center_angle_deg;",
                    "    }",
                    "    for (i = 1U; i < STEERING_CALIBRATION_POINT_COUNT; i++)",
                    "    {",
                    "        if (error <= steering_calibration_lut[i].error)",
                    "        {",
                    "            const steering_calibration_point_t *low = &steering_calibration_lut[i - 1U];",
                    "            const steering_calibration_point_t *high = &steering_calibration_lut[i];",
                    "            float ratio = (error - low->error) / (high->error - low->error);",
                    "            return low->center_angle_deg + ratio * (high->center_angle_deg - low->center_angle_deg);",
                    "        }",
                    "    }",
                    "    return 0.0f;",
                ]
            )
        lines.extend(["}", ""])

        lines.extend(
            [
                "typedef struct",
                "{",
                "    float center_angle_deg;",
                "    float error;",
                "} steering_inverse_point_t;",
                "",
                f"#define STEERING_INVERSE_POINT_COUNT ({len(inverse_rows)}U)",
            ]
        )
        if inverse_rows:
            lines.extend(
                [
                    "",
                    "static const steering_inverse_point_t steering_inverse_lut[] =",
                    "{",
                ]
            )
            for row in inverse_rows:
                lines.append(
                    "    {%.6ff, %.6ff}," % (row["center_angle_deg"], row["error"])
                )
            lines.extend(["};"])
        lines.extend(["", "static float steering_error_from_center_angle(float center_angle_deg)", "{"])
        if not inverse_rows:
            lines.append("    (void)center_angle_deg;")
            lines.append("    return 0.0f;")
        elif len(inverse_rows) == 1:
            lines.append("    (void)center_angle_deg;")
            lines.append(f"    return {inverse_rows[0]['error']:.6f}f;")
        else:
            lines.extend(
                [
                    "    uint32_t i;",
                    "    if (center_angle_deg <= steering_inverse_lut[0].center_angle_deg)",
                    "    {",
                    "        return steering_inverse_lut[0].error;",
                    "    }",
                    "    if (center_angle_deg >= steering_inverse_lut[STEERING_INVERSE_POINT_COUNT - 1U].center_angle_deg)",
                    "    {",
                    "        return steering_inverse_lut[STEERING_INVERSE_POINT_COUNT - 1U].error;",
                    "    }",
                    "    for (i = 1U; i < STEERING_INVERSE_POINT_COUNT; i++)",
                    "    {",
                    "        if (center_angle_deg <= steering_inverse_lut[i].center_angle_deg)",
                    "        {",
                    "            const steering_inverse_point_t *low = &steering_inverse_lut[i - 1U];",
                    "            const steering_inverse_point_t *high = &steering_inverse_lut[i];",
                    "            float ratio = (center_angle_deg - low->center_angle_deg)",
                    "                        / (high->center_angle_deg - low->center_angle_deg);",
                    "            return low->error + ratio * (high->error - low->error);",
                    "        }",
                    "    }",
                    "    return 0.0f;",
                ]
            )
        lines.extend(["}"])
        lines.extend(["", "#endif", ""])
        return "\n".join(lines)


class SteeringSession:
    def __init__(self, default_port: str = "/dev/ttyUSB0", baudrate: int = 460800):
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self.controller: Optional[CarController] = None
        self.port = str(default_port)
        self.baudrate = int(baudrate)
        self.error = 0.0
        self.output_enabled = False
        self.last_send_ok: Optional[bool] = None
        self.last_send_time = 0.0
        self.last_ack_time = 0.0
        self.last_ack_error: Optional[float] = None
        self.last_ack_feedback_seq: Optional[int] = None
        self.last_stop_confirmed: Optional[bool] = None
        self.send_attempt_count = 0
        self.send_success_count = 0
        self.ack_count = 0
        self.consecutive_send_failures = 0
        self._thread = threading.Thread(target=self._command_loop, name="steering-calibration-tx", daemon=True)
        self._thread.start()

    def connect(self, port: Any, baudrate: Any = 460800) -> None:
        port_name = str(port or "").strip()
        if not port_name:
            raise ValueError("串口不能为空")
        baud = int(finite_float(baudrate, "波特率"))
        if baud <= 0:
            raise ValueError("波特率必须大于 0")
        self.disconnect()
        controller = CarController(port=port_name, baudrate=baud)
        with self._lock:
            self.port = port_name
            self.baudrate = baud
            self.controller = controller
            self.last_send_ok = None
            self.last_send_time = 0.0
            self.last_ack_time = 0.0
            self.last_ack_error = None
            self.last_ack_feedback_seq = None
            self.last_stop_confirmed = None
            self.consecutive_send_failures = 0

    def disconnect(self) -> bool:
        with self._lock:
            controller = self.controller
            self.controller = None
            self.output_enabled = False
            self.last_ack_time = 0.0
            self.last_ack_error = None
        stop_confirmed = True
        if controller is not None:
            stop_confirmed = self._send_safe_stop(controller)
            controller.close()
        with self._lock:
            self.last_stop_confirmed = stop_confirmed
        return stop_confirmed

    def set_control(self, error: Any, output_enabled: Any) -> Dict[str, Any]:
        error_value = clamp(finite_float(error, "error"), ERROR_MIN, ERROR_MAX)
        enabled = bool(output_enabled)
        with self._lock:
            controller = self.controller
            was_enabled = self.output_enabled
            error_changed = abs(error_value - self.error) > 1e-9
            self.error = error_value
            self.output_enabled = enabled and controller is not None
            if error_changed or (enabled and not was_enabled):
                self.last_ack_time = 0.0
                self.last_ack_error = None
        if was_enabled and not enabled and controller is not None:
            stop_confirmed = self._send_safe_stop(controller)
            with self._lock:
                self.last_stop_confirmed = stop_confirmed
        return {"error": error_value, "output_enabled": self.output_enabled}

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            controller = self.controller
            desired_error = self.error
            output_enabled = self.output_enabled
            result = {
                "port": self.port,
                "baudrate": self.baudrate,
                "controller_created": controller is not None,
                "output_enabled": output_enabled,
                "error": desired_error,
                "last_send_ok": self.last_send_ok,
                "last_send_age": time.time() - self.last_send_time if self.last_send_time else None,
                "last_stop_confirmed": self.last_stop_confirmed,
                "send_attempt_count": self.send_attempt_count,
                "send_success_count": self.send_success_count,
                "ack_count": self.ack_count,
                "consecutive_send_failures": self.consecutive_send_failures,
                "command_written": bool(
                    output_enabled
                    and self.last_send_ok
                    and self.last_send_time
                    and time.time() - self.last_send_time <= ACK_MAX_AGE_S
                ),
            }
        feedback = controller.get_feedback() if controller is not None else {
            "online": False,
            "count": 0,
            "bad": 0,
            "error": "",
        }
        confirmed = self._observe_feedback(feedback, desired_error, output_enabled)
        with self._lock:
            result.update(
                {
                    "command_confirmed": confirmed,
                    "last_ack_error": self.last_ack_error,
                    "last_ack_age": time.time() - self.last_ack_time if self.last_ack_time else None,
                }
            )
        result["feedback"] = feedback
        return result

    def shutdown(self) -> None:
        self._stop_event.set()
        self.disconnect()
        self._thread.join(timeout=1.0)

    def _command_loop(self) -> None:
        while not self._stop_event.wait(COMMAND_PERIOD_S):
            with self._lock:
                controller = self.controller
                enabled = self.output_enabled
                error = self.error
            if controller is None or not enabled:
                continue
            ok = controller.send_cmd(
                track_error=error,
                target_speed=0.0,
                state_cmd=STATE_TRACK,
                flags=CONTROL_FLAG_USE_TARGET_SPEED,
            )
            with self._lock:
                self.send_attempt_count += 1
                self.last_send_ok = ok
                self.last_send_time = time.time()
                if ok:
                    self.send_success_count += 1
                    self.consecutive_send_failures = 0
                else:
                    self.consecutive_send_failures += 1
            if ok:
                self._observe_feedback(controller.get_feedback(), error, True)

    def _observe_feedback(
        self,
        feedback: Dict[str, Any],
        desired_error: float,
        output_enabled: bool,
    ) -> bool:
        if not output_enabled or not feedback.get("online"):
            return False
        try:
            echoed_error = float(feedback["input_track_error"])
            feedback_age = float(feedback.get("age", float("inf")))
            input_age_ms = feedback.get("input_age_ms")
            safety_flags = int(feedback.get("safety_flags", CONTROL_FB_FLAG_INPUT_TIMEOUT))
            state = int(feedback.get("state", -1))
            input_target_speed = float(feedback.get("input_target_speed", float("inf")))
            motor_target = float(feedback.get("motor_target", float("inf")))
            motor_output = int(feedback.get("motor_output", 1))
        except (KeyError, TypeError, ValueError):
            return False
        if (
            not math.isfinite(echoed_error)
            or abs(echoed_error - desired_error) > ACK_ERROR_TOLERANCE
            or feedback_age > ACK_MAX_AGE_S
            or (input_age_ms is not None and int(input_age_ms) > int(ACK_MAX_AGE_S * 1000.0))
            or safety_flags & CONTROL_FB_FLAG_INPUT_TIMEOUT
            or state != STATE_TRACK
            or abs(input_target_speed) > 1e-6
            or abs(motor_target) > 1e-6
            or motor_output != 0
        ):
            return False

        sequence = feedback.get("feedback_seq")
        with self._lock:
            if sequence is None or sequence != self.last_ack_feedback_seq:
                self.ack_count += 1
                self.last_ack_feedback_seq = sequence
            self.last_ack_time = time.time()
            self.last_ack_error = echoed_error
        return True

    @staticmethod
    def _send_safe_stop(controller: CarController) -> bool:
        for _ in range(5):
            if not controller.send_cmd(0.0, 0.0, state_cmd=STATE_SAFE_STOP, flags=0):
                time.sleep(0.03)
                continue
            time.sleep(0.03)
            feedback = controller.get_feedback()
            try:
                if (
                    feedback.get("online")
                    and int(feedback.get("state", -1)) == STATE_SAFE_STOP
                    and float(feedback.get("age", float("inf"))) <= ACK_MAX_AGE_S
                    and not (int(feedback.get("safety_flags", 0)) & CONTROL_FB_FLAG_INPUT_TIMEOUT)
                ):
                    return True
            except (TypeError, ValueError):
                pass
        return False


class CalibrationApplication:
    def __init__(self, data_dir: Path, default_port: str, baudrate: int):
        self.store = CalibrationStore(data_dir)
        self.session = SteeringSession(default_port=default_port, baudrate=baudrate)
        self.ui_bytes = UI_PATH.read_bytes()

    def state(self) -> Dict[str, Any]:
        return {
            "limits": {"error_min": ERROR_MIN, "error_max": ERROR_MAX},
            "serial": self.session.snapshot(),
            **self.store.snapshot(),
        }

    @staticmethod
    def ports() -> List[Dict[str, str]]:
        if list_ports is None:
            return []
        return [
            {
                "device": port.device,
                "description": port.description or "",
                "hwid": port.hwid or "",
            }
            for port in list_ports.comports()
        ]

    def shutdown(self) -> None:
        self.session.shutdown()


class CalibrationHandler(BaseHTTPRequestHandler):
    server_version = "SYSU-DDL-SteeringCalibration/1.0"

    @property
    def app(self) -> CalibrationApplication:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        print("[%s] %s" % (self.log_date_time_string(), fmt % args))

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send_bytes(self.app.ui_bytes, "text/html; charset=utf-8")
            return
        if path == "/api/state":
            self._send_json(self.app.state())
            return
        if path == "/api/ports":
            self._send_json({"ports": self.app.ports()})
            return
        if path == "/download/json":
            self.app.store.save()
            self._send_download(self.app.store.paths.json_path, "application/json")
            return
        if path == "/download/csv":
            self.app.store.save()
            self._send_download(self.app.store.paths.csv_path, "text/csv; charset=utf-8")
            return
        if path == "/download/header":
            self.app.store.save()
            self._send_download(self.app.store.paths.header_path, "text/x-c; charset=utf-8")
            return
        if path == "/download/fit":
            self.app.store.save()
            self._send_download(self.app.store.paths.fit_path, "application/json")
            return
        self._send_json({"error": "未找到该接口"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            payload = self._read_json()
            if path == "/api/connect":
                self.app.session.connect(payload.get("port"), payload.get("baudrate", 460800))
                self._send_json({"ok": True, "serial": self.app.session.snapshot()})
                return
            if path == "/api/disconnect":
                stop_confirmed = self.app.session.disconnect()
                self._send_json({"ok": True, "stop_confirmed": stop_confirmed})
                return
            if path == "/api/control":
                result = self.app.session.set_control(
                    payload.get("error", 0.0),
                    payload.get("output_enabled", False),
                )
                self._send_json({"ok": True, **result})
                return
            if path == "/api/config":
                config = self.app.store.update_config(
                    payload.get("wheelbase_mm"),
                    payload.get("track_width_mm"),
                )
                self._send_json({"ok": True, "config": config})
                return
            if path == "/api/samples":
                serial_state = self.app.session.snapshot()
                if not serial_state.get("command_written"):
                    raise ValueError("当前 error 尚未成功写入串口，不能保存该测量")
                feedback = serial_state.get("feedback", {})
                servo_duty = feedback.get("servo_output")
                sample = self.app.store.add_sample(
                    error=serial_state["error"],
                    servo_duty=servo_duty,
                    left_angle_deg=payload.get("left_angle_deg"),
                    right_angle_deg=payload.get("right_angle_deg"),
                    note=payload.get("note", ""),
                )
                self._send_json({"ok": True, "sample": sample}, HTTPStatus.CREATED)
                return
            if path == "/api/save":
                self.app.store.save()
                self._send_json({"ok": True, "paths": self.app.store.snapshot()["paths"]})
                return
            self._send_json({"error": "未找到该接口"}, HTTPStatus.NOT_FOUND)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._send_json({"error": f"内部错误: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)

    def do_DELETE(self) -> None:
        path = urlparse(self.path).path
        prefix = "/api/samples/"
        if not path.startswith(prefix):
            self._send_json({"error": "未找到该接口"}, HTTPStatus.NOT_FOUND)
            return
        try:
            sample_id = int(path[len(prefix):])
        except ValueError:
            self._send_json({"error": "记录 ID 无效"}, HTTPStatus.BAD_REQUEST)
            return
        if not self.app.store.delete_sample(sample_id):
            self._send_json({"error": "记录不存在"}, HTTPStatus.NOT_FOUND)
            return
        self._send_json({"ok": True})

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        data = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("请求内容必须是 JSON 对象")
        return data

    def _send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self._send_bytes(data, "application/json; charset=utf-8", status)

    def _send_download(self, path: Path, content_type: str) -> None:
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(
        self,
        data: bytes,
        content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SYSU_DDL 阿克曼转向标定工具")
    parser.add_argument("--host", default="127.0.0.1", help="界面监听地址")
    parser.add_argument("--port", type=int, default=8765, help="界面监听端口")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0", help="默认 TC264D 串口")
    parser.add_argument("--baudrate", type=int, default=460800, help="TC264D 串口波特率")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR, help="标定数据目录")
    parser.add_argument("--no-browser", action="store_true", help="启动后不自动打开浏览器")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    app = CalibrationApplication(args.data_dir, args.serial_port, args.baudrate)
    server = ThreadingHTTPServer((args.host, args.port), CalibrationHandler)
    server.app = app  # type: ignore[attr-defined]
    url = f"http://{args.host}:{server.server_address[1]}"
    print(f"转向标定界面: {url}")
    print("按 Ctrl+C 退出；退出时会发送安全停车并关闭串口。")
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
