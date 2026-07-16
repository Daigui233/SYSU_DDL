"""Summarize a JSONL file produced by collect_vision_run.py."""

import json
import math
from pathlib import Path
import sys


def _values(records, getter):
    values = []
    for record in records:
        try:
            value = getter(record)
            if value is not None and math.isfinite(float(value)):
                values.append(float(value))
        except (KeyError, TypeError, ValueError):
            pass
    return values


def _percentile(values, ratio):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * ratio))))
    return ordered[index]


def _summary(path):
    records = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("record_type") == "frame":
                records.append(record)
    if not records:
        return {"file": str(path), "frames": 0}

    timestamps = _values(records, lambda item: item["monotonic_time"])
    errors = _values(records, lambda item: item["control"]["track_error"])
    raw_errors = _values(records, lambda item: item["path"]["raw_error_640"])
    target_x = _values(records, lambda item: item["path"]["path_target_x"])
    feedforward = _values(
        records, lambda item: item["path"]["curve_feedforward_640"])
    heading_feedforward = _values(
        records,
        lambda item: item["path"]["path_heading_feedforward_640"])
    task_records = [
        item for item in records
        if item.get("control", {}).get("task_reason") == "track"
    ]
    signs = [1 if value > 2 else -1 if value < -2 else 0 for value in errors]
    sign_flips = sum(
        1 for previous, current in zip(signs, signs[1:])
        if previous and current and previous != current)
    target_steps = [
        abs(current - previous)
        for previous, current in zip(target_x, target_x[1:])]
    slot_values = [item.get("selection", {}).get("selected_slot") for item in records]
    slot_switches = sum(
        1 for previous, current in zip(slot_values, slot_values[1:])
        if previous is not None and current is not None and previous != current)
    duration = max(0.0, timestamps[-1] - timestamps[0]) if len(timestamps) >= 2 else 0.0
    rms = lambda values: (
        math.sqrt(sum(value * value for value in values) / len(values))
        if values else None)
    return {
        "file": str(path),
        "frames": len(records),
        "duration_s": duration,
        "effective_fps": len(records) / duration if duration > 0.0 else None,
        "blank_camera_frames": sum(
            1 for item in records if item.get("camera_status") != "ok"),
        "track_frames": len(task_records),
        "track_error_rms": rms(errors),
        "raw_error_rms": rms(raw_errors),
        "track_error_sign_flips": sign_flips,
        "sign_flips_per_s": sign_flips / duration if duration > 0.0 else None,
        "target_step_median_px": _percentile(target_steps, 0.50),
        "target_step_p95_px": _percentile(target_steps, 0.95),
        "curve_feedforward_abs_p95_px": _percentile(
            [abs(value) for value in feedforward], 0.95),
        "heading_feedforward_abs_p95_px": _percentile(
            [abs(value) for value in heading_feedforward], 0.95),
        "line_loss_frames": sum(
            1 for item in records
            if item.get("selection", {}).get("line_loss_active")),
        "selected_slot_switches": slot_switches,
        "adaptive_target_frames": sum(
            1 for item in records
            if item.get("path", {}).get("path_target_adaptive_y")),
        "servo_output_sign_flips": _servo_sign_flips(records),
    }


def _servo_sign_flips(records):
    values = _values(
        records,
        lambda item: item["runtime"]["serial_servo_limited_output"])
    signs = [1 if value > 2 else -1 if value < -2 else 0 for value in values]
    return sum(
        1 for previous, current in zip(signs, signs[1:])
        if previous and current and previous != current)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) != 1:
        raise SystemExit("usage: python3 analyze_vision_run.py <vision_run.jsonl>")
    print(json.dumps(_summary(argv[0]), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
