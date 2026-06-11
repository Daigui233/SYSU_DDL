import glob
import json
import os
import time
from urllib import request as urlrequest
from urllib.parse import quote

import cv2
import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:
    Image = None
    ImageDraw = None
    ImageFont = None


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
REFEREE_RECORDS_DIR = os.path.join(BASE_DIR, "dist")
REFEREE_RECORDS_GLOB = "match_record_*.json"

# ===== 左侧裁判事件提示面板参数区：修改这里后需要重启 ar_receiver.py 生效 =====
# OpenCV 默认字体不支持中文，所以这里使用英文短标签，避免预览窗口乱码。
REFEREE_PANEL_DEFAULTS = {
    # 左侧面板宽度，单位 px；调大可显示更长事件文本，但会占用更多窗口宽度。
    "PANEL_WIDTH": 300,

    # Referee event polling interval, seconds. The reference project uses a 500 ms cycle.
    "SCAN_INTERVAL": 0.5,

    # 裁判系统事件 API，参考 SmartCar-Auto-Refresh-Events 的 Python 脚本。
    # RK 上通常是本机 5000；若裁判系统部署在别处，可改成对应地址。
    "API_BASE_URL": "http://127.0.0.1:5000",

    # API 请求超时时间，单位 s；调大更能容忍慢响应，但会拖慢 HUD 刷新。
    "API_TIMEOUT": 0.35,

    # Number of latest referee events kept on the left panel.
    "MAX_EVENTS": 4,
    "MAX_BUFFER_EVENTS": 24,

    # 最近多少秒内文件有更新时显示为 LIVE；超过后显示 STALE，提醒去刷新/检查裁判系统。
    "FRESH_FILE_SECONDS": 5.0,

    # Max characters per line when the fallback OpenCV font is used.
    "MAX_LINE_CHARS": 36,

    # Optional Chinese font for referee messages. If none of these exists, the HUD falls back
    # to ASCII text so it does not show OpenCV question marks.
    "FONT_SIZE": 16,
    "FONT_PATHS": [
        os.environ.get("AR_HUD_FONT_PATH", ""),
        "C:/Windows/Fonts/msyh.ttc",
        "C:/Windows/Fonts/simhei.ttf",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
    ],
}

_referee_cache = {
    "next_scan_ts": 0.0,
    "summary": None,
    "filename": "",
    "event_count": 0,
    "events": [],
}

_font_cache = {
    "font": None,
    "tried": False,
}

TC264_STATE_NAMES = {
    0: "STATE_IDLE",
    1: "STATE_TRACK",
    2: "STATE_AVOID_CAR",
    3: "STATE_AVOID_HUMAN",
    4: "STATE_COLLECT_GOLD",
    5: "STATE_RECOVER_LINE",
    6: "STATE_LINE_LOSS_SAFE_STOP",
    7: "STATE_SAFE_STOP",
    8: "STATE_AVOID_STONE",
    9: "STATE_TRAFFIC_LIGHT_STOP",
    10: "STATE_ENDSIGN_STOP",
}


def short_text(text, max_len=74):
    text = str(text or "")
    if len(text) <= max_len:
        return text
    return text[:max_len - 3] + "..."


def _ascii_hud_text(text, fallback=""):
    text = str(text or "")
    text = text.encode("ascii", "ignore").decode("ascii")
    text = text.replace("?", "")
    text = " ".join(text.split())
    return text or fallback


def _clean_referee_text(text):
    text = str(text or "")
    text = " ".join(text.split())
    if not text:
        return ""
    if text.count("?") >= min(3, max(1, len(text) // 2)):
        return ""
    return text


def _safe_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return default


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
    event = event or {}
    event_type = _clean_referee_text(event.get("type") or event.get("event_type") or event.get("name") or "EVENT")
    message = _clean_referee_text(event.get("message") or event.get("msg") or event.get("description") or "")
    time_text = _clean_referee_text(event.get("time_str") or event.get("time") or "")
    elapsed = event.get("elapsed_seconds")
    if elapsed is not None:
        time_text = f"{_safe_float(elapsed):.1f}s"
    prefix = f"{time_text} " if time_text else ""
    return f"{prefix}{event_type} {message}".strip()


def _append_referee_events(filename, events, total_events, params):
    max_buffer = int(params["MAX_BUFFER_EVENTS"])
    if filename != _referee_cache["filename"]:
        _referee_cache["filename"] = filename
        _referee_cache["event_count"] = 0
        _referee_cache["events"] = []

    old_count = int(_referee_cache["event_count"])
    total_events = max(total_events, len(events))
    if total_events < old_count:
        old_count = 0
        _referee_cache["events"] = []

    new_events = list(events[old_count:total_events])
    if new_events:
        _referee_cache["events"].extend(new_events)
        _referee_cache["events"] = _referee_cache["events"][-max_buffer:]
    _referee_cache["event_count"] = total_events


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
            "total_events": int(_referee_cache["event_count"]),
            "events": list(_referee_cache["events"])[-int(params["MAX_EVENTS"]):][::-1],
            "error": None,
        }

    latest_record = records[0]
    filename = str(latest_record.get("filename") or "")
    if not filename:
        raise RuntimeError("latest record has no filename")

    listed_total = latest_record.get("total_events", latest_record.get("event_count"))
    listed_total = None if listed_total is None else _safe_int(listed_total, None)
    if (
        listed_total is not None
        and filename == _referee_cache["filename"]
        and listed_total <= int(_referee_cache["event_count"])
    ):
        return {
            "ok": True,
            "state": "API LIVE",
            "source": "API",
            "file": filename,
            "file_age": 0.0,
            "total_events": int(_referee_cache["event_count"]),
            "events": list(_referee_cache["events"])[-int(params["MAX_EVENTS"]):][::-1],
            "error": None,
        }

    record_url = f"{base_url}/api/match_record/{quote(filename)}"
    record_payload = _read_json_url(record_url, timeout)
    data = record_payload.get("data") if isinstance(record_payload.get("data"), dict) else record_payload
    events = list((data or {}).get("events") or [])
    total_events = len(events)
    _append_referee_events(filename, events, total_events, params)
    recent = list(_referee_cache["events"])[-int(params["MAX_EVENTS"]):][::-1]
    return {
        "ok": True,
        "state": "API LIVE",
        "source": "API",
        "file": filename,
        "file_age": 0.0,
        "total_events": int(_referee_cache["event_count"]),
        "events": recent,
        "error": None,
    }


def _read_referee_summary(records_dir, now, params):
    try:
        return _read_referee_api_summary(now, params)
    except Exception as exc:
        return {
            "ok": False,
            "state": "API OFFLINE",
            "source": "API",
            "file": _referee_cache.get("filename") or None,
            "file_age": None,
            "total_events": int(_referee_cache["event_count"]),
            "events": list(_referee_cache["events"])[-int(params["MAX_EVENTS"]):][::-1],
            "error": str(exc),
        }


def _referee_summary(now, records_dir=REFEREE_RECORDS_DIR, params=None):
    params = params or REFEREE_PANEL_DEFAULTS
    if now >= _referee_cache["next_scan_ts"]:
        _referee_cache["summary"] = _read_referee_summary(records_dir, now, params)
        _referee_cache["next_scan_ts"] = now + float(params["SCAN_INTERVAL"])
    return _referee_cache["summary"] or {}


def _format_perf_lines(performance_status):
    if not performance_status or not performance_status.get("enabled"):
        return []

    stages = performance_status.get("stages_ms") or {}
    system = performance_status.get("system") or {}
    raw_fps = _safe_float(performance_status.get("raw_fps"))
    loop_fps = _safe_float(performance_status.get("loop_fps"))
    total_ms = _safe_float(performance_status.get("total_ms"))

    def ms_text(name):
        value = _safe_float(stages.get(name))
        return "N/A" if value is None else f"{value:.1f}"

    raw_text = "N/A" if raw_fps is None else f"{raw_fps:.1f}"
    loop_text = "N/A" if loop_fps is None else f"{loop_fps:.1f}"
    total_text = "N/A" if total_ms is None else f"{total_ms:.1f}"
    lines = [
        f"PERF raw={raw_text} loop={loop_text}",
        f"MS rd={ms_text('read_ms')} ai={ms_text('vision_ms')} cmd={ms_text('command_ms')}",
        f"MS hud={ms_text('hud_ms')} disp={ms_text('display_ms')} tot={total_text}",
    ]

    npu_load = system.get("npu_load")
    if isinstance(npu_load, (list, tuple)) and npu_load:
        load_text = "/".join(f"{_safe_float(v, 0.0):.0f}" for v in npu_load)
    else:
        load_text = "N/A"
    npu_freq = _safe_float(system.get("npu_freq_mhz"))
    freq_text = "N/A" if npu_freq is None else f"{npu_freq:.0f}MHz"
    lines.append(f"NPU load={load_text} freq={freq_text}")

    temp_c = _safe_float(system.get("cpu_temp_c"))
    loadavg = _safe_float(system.get("loadavg_1m"))
    if temp_c is not None or loadavg is not None:
        temp_text = "N/A" if temp_c is None else f"{temp_c:.1f}C"
        load_text = "N/A" if loadavg is None else f"{loadavg:.2f}"
        lines.append(f"SYS temp={temp_text} load={load_text}")
    return lines


def _format_compact_perf_lines(performance_status, fps):
    if fps is None:
        view_text = "N/A"
    else:
        view_text = f"{fps:.1f}"

    if not performance_status or not performance_status.get("enabled"):
        return [f"FPS view={view_text}"]

    stages = performance_status.get("stages_ms") or {}
    raw_fps = _safe_float(performance_status.get("raw_fps"), None)
    loop_fps = _safe_float(performance_status.get("loop_fps"), None)
    total_ms = _safe_float(performance_status.get("total_ms"), None)
    vision_ms = _safe_float(stages.get("vision_ms"), None)
    command_ms = _safe_float(stages.get("command_ms"), None)

    def fmt(value):
        return "N/A" if value is None else f"{value:.1f}"

    lines = [
        f"FPS view={view_text} raw={fmt(raw_fps)} loop={fmt(loop_fps)}",
    ]
    if vision_ms is not None or command_ms is not None or total_ms is not None:
        lines.append(f"MS ai={fmt(vision_ms)} cmd={fmt(command_ms)} total={fmt(total_ms)}")
    return lines


def _tc264_state_name(value):
    try:
        state_value = int(value)
    except Exception:
        return "STATE_N/A"
    return TC264_STATE_NAMES.get(state_value, f"STATE_{state_value}")


def _load_hud_font(params):
    if _font_cache["tried"]:
        return _font_cache["font"]
    _font_cache["tried"] = True
    if ImageFont is None:
        return None
    for path in params.get("FONT_PATHS", []):
        if not path:
            continue
        try:
            if os.path.exists(path):
                _font_cache["font"] = ImageFont.truetype(path, int(params["FONT_SIZE"]))
                return _font_cache["font"]
        except Exception:
            continue
    return None


def _fit_text_for_pil(draw, text, font, max_width):
    text = str(text or "")
    if draw.textlength(text, font=font) <= max_width:
        return text
    suffix = "..."
    for n in range(max(0, len(text) - 1), 0, -1):
        candidate = text[:n] + suffix
        if draw.textlength(candidate, font=font) <= max_width:
            return candidate
    return suffix


def _draw_text_lines(panel, lines, title_color, text_color, params, line_h=24):
    font = _load_hud_font(params)
    if font is None:
        y = 28
        for i, line in enumerate(lines):
            color = title_color if i == 0 else text_color
            cv2.putText(
                panel,
                short_text(_ascii_hud_text(line, "N/A"), params["MAX_LINE_CHARS"]),
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                1,
                cv2.LINE_AA,
            )
            y += line_h
            if i in (0, 4):
                cv2.line(panel, (10, y - 8), (panel.shape[1] - 12, y - 8), (55, 70, 70), 1, cv2.LINE_AA)
            if y > panel.shape[0] - 12:
                break
        return panel

    image = Image.fromarray(cv2.cvtColor(panel, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    max_width = panel.shape[1] - 24
    y = 12
    for i, line in enumerate(lines):
        color = title_color if i == 0 else text_color
        rgb = (int(color[2]), int(color[1]), int(color[0]))
        text = _fit_text_for_pil(draw, line, font, max_width)
        draw.text((12, y), text, font=font, fill=rgb)
        y += line_h
        if i in (0, 4):
            draw.line((10, y - 2, panel.shape[1] - 12, y - 2), fill=(70, 70, 55), width=1)
        if y > panel.shape[0] - 18:
            break
    panel[:, :] = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
    return panel


def draw_pose_status(
    frame,
    pose_bridge,
    fps=None,
    track_error=None,
    control_state="N/A",
    gamepad_status=None,
    get_car_feedback=None,
    pose_input_port=9005,
    performance_status=None,
    target_speed=None,
    segmentation_status=None,
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

    if track_error is not None and np.isfinite(track_error):
        err_text = f"{track_error:.1f}"
    else:
        err_text = "N/A"

    speed_text = "N/A" if target_speed is None else f"{float(target_speed):.2f}"
    control_line = f"CTRL {control_state}"
    cmd_line = f"CMD v={speed_text} err={err_text}"
    fork_line = "FORK N/A"
    if isinstance(segmentation_status, dict):
        fork_cls = segmentation_status.get("fork_classifier") or {}
        fork_state = segmentation_status.get("fork_state") or "N/A"
        fork_mode = segmentation_status.get("fork_mode") or "single"
        fork_name = fork_cls.get("name") or "none"
        fork_conf = _safe_float(fork_cls.get("confidence"), None)
        split_rows = segmentation_status.get("fork_split_rows", 0)
        selected_side = segmentation_status.get("fork_selected_side") or "-"
        conf_text = "N/A" if fork_conf is None else f"{fork_conf:.2f}"
        fork_line = f"FORK {fork_state} {fork_mode} cls={fork_name}:{conf_text} split={split_rows} sel={selected_side}"

    feedback = get_car_feedback() if get_car_feedback is not None else {"online": False, "error": "waiting"}
    car_lines = []
    if feedback.get("online"):
        fb_age = feedback.get("age")
        fb_age_text = f"{fb_age:.1f}s" if fb_age is not None else "N/A"
        car_lines.append(
            f"CAR v={feedback.get('actual_speed', 0.0):.2f}m/s "
            f"target={feedback.get('input_target_speed', 0.0):.2f}"
        )
        car_lines.append(
            f"TC264 {_tc264_state_name(feedback.get('state'))} age={fb_age_text}"
        )
    else:
        err = short_text(_ascii_hud_text(feedback.get("error", "waiting"), "waiting"), 28)
        car_lines.append("CAR v=N/A target=N/A")
        car_lines.append(
            f"TC264 waiting fb={feedback.get('count', 0)} bad={feedback.get('bad', 0)}"
        )
        if err:
            car_lines.append(err)

    lines = [
        "RUN STATUS",
        control_line,
        cmd_line,
        fork_line,
        pose_line,
    ]
    lines.extend(car_lines)
    lines.extend(_format_compact_perf_lines(performance_status, fps))

    h = frame.shape[0]
    left_panel = _draw_referee_panel(h, frame.dtype, now)

    panel_w = 340
    panel = np.zeros((h, panel_w, 3), dtype=frame.dtype)
    panel[:, :] = (12, 16, 16)
    cv2.rectangle(panel, (0, 0), (panel_w - 1, h - 1), color, 1)

    line_h = 24
    y = 28
    for i, line in enumerate(lines):
        text_color = color if i == 0 else (230, 245, 245)
        cv2.putText(panel, short_text(line, 42), (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, text_color, 1, cv2.LINE_AA)
        y += line_h
        if i in (0, 3):
            cv2.line(panel, (10, y - 8), (panel_w - 12, y - 8), (55, 70, 70), 1, cv2.LINE_AA)

    return np.hstack([left_panel, frame, panel])


def _draw_referee_panel(h, dtype, now):
    params = REFEREE_PANEL_DEFAULTS
    summary = _referee_summary(now, params=params)
    panel_w = int(params["PANEL_WIDTH"])
    panel = np.zeros((h, panel_w, 3), dtype=dtype)
    panel[:, :] = (10, 12, 15)

    state = summary.get("state") or "NO RECORD"
    if state == "API LIVE":
        color = (70, 240, 70)
    elif state == "API EMPTY":
        color = (0, 220, 255)
    else:
        color = (40, 80, 255)
    cv2.rectangle(panel, (0, 0), (panel_w - 1, h - 1), color, 1)

    lines = [
        "REFEREE EVENTS",
        f"STATUS {state}",
        f"POLL {params['SCAN_INTERVAL']:.1f}s",
        f"TOTAL {summary.get('total_events', 0)}",
    ]
    if summary.get("file"):
        lines.append(short_text(f"FILE {summary['file']}", params["MAX_LINE_CHARS"]))

    events = summary.get("events") or []
    if events:
        lines.append("RECENT")
        for event in events:
            lines.append(_format_event(event))
    else:
        lines.append("RECENT none")
        if summary.get("error"):
            lines.append(short_text(f"ERR {_ascii_hud_text(summary['error'], 'offline')}", params["MAX_LINE_CHARS"]))

    return _draw_text_lines(panel, lines, color, (230, 245, 245), params, line_h=24)
