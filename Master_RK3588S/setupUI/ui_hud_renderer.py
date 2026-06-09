import glob
import json
import os
import time
from urllib import request as urlrequest
from urllib.parse import quote

import cv2
import numpy as np


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
REFEREE_RECORDS_DIR = os.path.join(BASE_DIR, "dist")
REFEREE_RECORDS_GLOB = "match_record_*.json"

# ===== 左侧裁判事件提示面板参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# OpenCV 默认字体不支持中文，所以这里使用英文短标签，避免预览窗口乱码。
REFEREE_PANEL_DEFAULTS = {
    # 左侧面板宽度，单位 px；调大可显示更长事件文本，但会占用更多窗口宽度。
    "PANEL_WIDTH": 285,

    # 重新扫描 dist/match_record_*.json 的间隔，单位 s；调小更实时，调大会减少磁盘读取。
    "SCAN_INTERVAL": 1.0,

    # 裁判系统事件 API，参考 SmartCar-Auto-Refresh-Events 的 Python 脚本。
    # RK 上通常是本机 5000；若裁判系统部署在别处，可改成对应地址。
    "API_BASE_URL": "http://127.0.0.1:5000",

    # API 请求超时时间，单位 s；调大更能容忍慢响应，但会拖慢 HUD 刷新。
    "API_TIMEOUT": 0.35,

    # 显示最近几条事件；调大可看到更多历史事件，但面板会更拥挤。
    "MAX_EVENTS": 4,

    # 最近多少秒内文件有更新时显示为 LIVE；超过后显示 STALE，提醒去刷新/检查裁判系统。
    "FRESH_FILE_SECONDS": 5.0,

    # 单行最大字符数；调大会更长但容易超出面板。
    "MAX_LINE_CHARS": 36,
}

_referee_cache = {
    "next_scan_ts": 0.0,
    "summary": None,
}


def short_text(text, max_len=74):
    text = str(text or "")
    if len(text) <= max_len:
        return text
    return text[:max_len - 3] + "..."


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


def _latest_match_record(records_dir):
    pattern = os.path.join(records_dir, REFEREE_RECORDS_GLOB)
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=lambda path: os.path.getmtime(path))


def _read_json_url(url, timeout):
    with urlrequest.urlopen(url, timeout=timeout) as response:
        if response.status != 200:
            raise RuntimeError(f"HTTP {response.status}")
        raw = response.read()
    return json.loads(raw.decode("utf-8"))


def _format_event(event):
    event_type = str(event.get("type") or "EVENT")
    message = str(event.get("message") or "")
    time_text = str(event.get("time_str") or "")
    elapsed = event.get("elapsed_seconds")
    if elapsed is not None:
        time_text = f"{_safe_float(elapsed):.1f}s"
    prefix = f"{time_text} " if time_text else ""
    return f"{prefix}{event_type} {message}".strip()


def _read_referee_api_summary(now, params):
    base_url = str(params["API_BASE_URL"]).rstrip("/")
    timeout = float(params["API_TIMEOUT"])
    records_payload = _read_json_url(f"{base_url}/api/match_records", timeout)
    records = list(records_payload.get("records") or [])
    if not records:
        return {
            "ok": True,
            "state": "API EMPTY",
            "source": "API",
            "file": None,
            "file_age": 0.0,
            "total_events": 0,
            "events": [],
            "error": None,
        }

    latest_record = records[0]
    filename = str(latest_record.get("filename") or "")
    if not filename:
        raise RuntimeError("latest record has no filename")

    record_url = f"{base_url}/api/match_record/{quote(filename)}"
    record_payload = _read_json_url(record_url, timeout)
    data = record_payload.get("data") if isinstance(record_payload.get("data"), dict) else record_payload
    events = list((data or {}).get("events") or [])
    max_events = int(params["MAX_EVENTS"])
    recent = events[-max_events:][::-1]
    return {
        "ok": True,
        "state": "API LIVE",
        "source": "API",
        "file": filename,
        "file_age": 0.0,
        "total_events": int(latest_record.get("total_events", len(events)) or len(events)),
        "events": recent,
        "error": None,
    }


def _read_referee_summary(records_dir, now, params):
    api_error = None
    try:
        return _read_referee_api_summary(now, params)
    except Exception as exc:
        api_error = str(exc)

    latest_path = _latest_match_record(records_dir)
    if latest_path is None:
        return {
            "ok": False,
            "state": "NO RECORD",
            "source": "LOCAL",
            "file": None,
            "file_age": None,
            "total_events": 0,
            "events": [],
            "error": api_error,
        }

    file_mtime = os.path.getmtime(latest_path)
    file_age = max(0.0, now - file_mtime)
    state = "LIVE" if file_age <= params["FRESH_FILE_SECONDS"] else "STALE"
    try:
        with open(latest_path, "r", encoding="utf-8") as f:
            payload = json.load(f)
    except Exception as exc:
        return {
            "ok": False,
            "state": "READ ERR",
            "source": "LOCAL",
            "file": os.path.basename(latest_path),
            "file_age": file_age,
            "total_events": 0,
            "events": [],
            "error": str(exc),
        }

    events = list(payload.get("events") or [])
    max_events = int(params["MAX_EVENTS"])
    recent = events[-max_events:][::-1]
    return {
        "ok": True,
        "state": state,
        "source": "LOCAL",
        "file": os.path.basename(latest_path),
        "file_age": file_age,
        "total_events": int(payload.get("total_events", len(events)) or 0),
        "events": recent,
        "error": api_error,
    }


def _referee_summary(now, records_dir=REFEREE_RECORDS_DIR, params=None):
    params = params or REFEREE_PANEL_DEFAULTS
    if now >= _referee_cache["next_scan_ts"]:
        _referee_cache["summary"] = _read_referee_summary(records_dir, now, params)
        _referee_cache["next_scan_ts"] = now + float(params["SCAN_INTERVAL"])
    return _referee_cache["summary"] or {}


def draw_pose_status(
    frame,
    pose_bridge,
    fps=None,
    track_error=None,
    control_state="N/A",
    gamepad_status=None,
    get_car_feedback=None,
    pose_input_port=9005,
    enabled=True,
):
    if not enabled:
        return frame

    now = time.time()
    info = pose_bridge.snapshot()
    age = now - info["last_ts"] if info["last_ts"] else None
    fresh = age is not None and age < 1.0
    status_l = info["status"].lower()
    color = (70, 240, 70) if fresh else (0, 220, 255)
    if "error" in status_l or "missing" in status_l or "no device" in status_l:
        color = (40, 80, 255)

    packet = info["last_packet"]
    pose_line = "POSE: waiting"
    if packet:
        pos = packet.get("pos", [0.0, 0.0, 0.0])
        euler = packet.get("euler", [0.0, 0.0, 0.0])
        age_text = f"{age:.1f}s" if age is not None else "N/A"
        pose_line = (
            f"POSE x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f} "
            f"yaw={euler[1]:.1f} age={age_text}"
        )

    source_line = (
        f"WIN-UDP {short_text(info['status'], 18)} "
        f"ok={info['packet_count']} bad={info['invalid_count']}"
    )
    udp_line = f"AR-FWD ok={info['udp_send_count']} fail={info['udp_fail_count']}"
    if track_error is not None and np.isfinite(track_error):
        control_line = f"CTRL {control_state} err={track_error:.1f}"
    else:
        control_line = f"CTRL {control_state} err=N/A"

    gamepad_lines = []
    if gamepad_status is not None:
        gp_age = gamepad_status.get("age")
        gp_age_text = f"{gp_age:.1f}s" if gp_age is not None else "N/A"
        gp_state = "ACTIVE" if gamepad_status.get("active") else "idle"
        gamepad_lines.append(
            f"GAMEPAD {gp_state} ok={gamepad_status.get('packet_count', 0)} "
            f"bad={gamepad_status.get('invalid_count', 0)} age={gp_age_text}"
        )
        if gamepad_status.get("last_packet"):
            inputs = gamepad_status.get("inputs") or {}
            gamepad_lines.append(
                f"GP v={gamepad_status.get('target_speed', 0.0):.2f} "
                f"err={gamepad_status.get('track_error', 0.0):.0f} "
                f"RT={inputs.get('rt', 0.0):.2f} LT={inputs.get('lt', 0.0):.2f} B={int(bool(inputs.get('b', False)))}"
            )

    feedback = get_car_feedback() if get_car_feedback is not None else {"online": False, "error": "waiting"}
    car_lines = []
    if feedback.get("online"):
        fb_age = feedback.get("age")
        fb_age_text = f"{fb_age:.1f}s" if fb_age is not None else "N/A"
        flags = feedback.get("flags")
        flag_text = "N/A" if flags is None else f"0x{int(flags):02X}"
        car_lines.append(
            f"TC264 fb={feedback.get('count', 0)} age={fb_age_text} "
            f"st={feedback.get('state', 'N/A')} flags={flag_text}"
        )
        car_lines.append(
            f"RX raw={feedback.get('raw_rx', 0)} drop={feedback.get('raw_drop', 0)} "
            f"bad={feedback.get('bad', 0)}"
        )
        car_lines.append(
            f"SPD in={feedback.get('input_target_speed', 0.0):.2f} "
            f"tgt={feedback.get('motor_target', 0.0):.2f} "
            f"act={feedback.get('actual_speed', 0.0):.2f}m/s"
        )
        car_lines.append(
            f"OUT m={feedback.get('motor_output', 0)} s={feedback.get('servo_output', 0)}"
        )
        car_lines.append(
            f"PID {feedback.get('motor_kp', 0.0):.1f}/{feedback.get('motor_ki', 0.0):.1f}/{feedback.get('motor_kd', 0.0):.1f} "
            f"SV {feedback.get('servo_kp', 0.0):.1f}/{feedback.get('servo_kd', 0.0):.1f}"
        )
    else:
        err = short_text(feedback.get("error", "waiting"), 28)
        car_lines.append(
            f"TC264 waiting fb={feedback.get('count', 0)} raw={feedback.get('raw_rx', 0)} "
            f"drop={feedback.get('raw_drop', 0)} bad={feedback.get('bad', 0)}"
        )
        if feedback.get("last_rx"):
            car_lines.append(f"last rx {short_text(feedback.get('last_rx', ''), 28)}")
        car_lines.append(err)
    fps_line = f"FPS {fps:.1f}" if fps is not None else "FPS N/A"

    lines = [
        "DEBUG STATUS",
        source_line,
        pose_line,
        udp_line,
        control_line,
    ]
    lines.extend(gamepad_lines)
    lines.extend(car_lines)
    lines.append(fps_line)

    h = frame.shape[0]
    left_panel = _draw_referee_panel(h, frame.dtype, now)

    panel_w = 430
    panel = np.zeros((h, panel_w, 3), dtype=frame.dtype)
    panel[:, :] = (12, 16, 16)
    cv2.rectangle(panel, (0, 0), (panel_w - 1, h - 1), color, 1)

    line_h = 22
    y = 28
    for i, line in enumerate(lines):
        text_color = color if i == 0 else (230, 245, 245)
        cv2.putText(panel, short_text(line, 58), (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, text_color, 1, cv2.LINE_AA)
        y += line_h
        if i in (0, 4):
            cv2.line(panel, (10, y - 8), (panel_w - 12, y - 8), (55, 70, 70), 1, cv2.LINE_AA)

    return np.hstack([left_panel, frame, panel])


def _draw_referee_panel(h, dtype, now):
    params = REFEREE_PANEL_DEFAULTS
    summary = _referee_summary(now, params=params)
    panel_w = int(params["PANEL_WIDTH"])
    panel = np.zeros((h, panel_w, 3), dtype=dtype)
    panel[:, :] = (10, 12, 15)

    state = summary.get("state") or "NO RECORD"
    if state in ("LIVE", "API LIVE"):
        color = (70, 240, 70)
    elif state == "STALE":
        color = (0, 220, 255)
    else:
        color = (40, 80, 255)
    cv2.rectangle(panel, (0, 0), (panel_w - 1, h - 1), color, 1)

    file_age = summary.get("file_age")
    file_age_text = "N/A" if file_age is None else f"{float(file_age):.1f}s"
    lines = [
        "REFEREE EVENTS",
        f"STATE {state}",
        f"SRC {summary.get('source', 'N/A')}",
        f"REFRESH auto {params['SCAN_INTERVAL']:.1f}s",
        f"FILE age {file_age_text}",
        f"TOTAL {summary.get('total_events', 0)}",
    ]
    if summary.get("file"):
        lines.append(short_text(summary["file"], params["MAX_LINE_CHARS"]))
    if summary.get("error"):
        lines.append(short_text(f"API {summary['error']}", params["MAX_LINE_CHARS"]))

    events = summary.get("events") or []
    if events:
        lines.append("RECENT")
        for event in events:
            lines.append(short_text(_format_event(event), params["MAX_LINE_CHARS"]))
    else:
        lines.append("RECENT none")
        lines.append("Use WebUI refresh")
        lines.append("or wait for engine")

    line_h = 22
    y = 28
    for i, line in enumerate(lines):
        text_color = color if i == 0 else (230, 245, 245)
        cv2.putText(
            panel,
            short_text(line, params["MAX_LINE_CHARS"]),
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            text_color,
            1,
            cv2.LINE_AA,
        )
        y += line_h
        if i in (0, 5):
            cv2.line(panel, (10, y - 8), (panel_w - 12, y - 8), (55, 70, 70), 1, cv2.LINE_AA)
        if y > h - 12:
            break

    return panel
