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

TC264_FB_FLAG_INPUT_TIMEOUT = 0x0001
TC264_FB_FLAG_STOP_STATE = 0x0002
TC264_FB_FLAG_SERVO_SATURATED = 0x0004
TC264_FB_FLAG_MOTOR_SATURATED = 0x0008
TC264_FB_FLAG_TARGET_DEADBAND = 0x0010


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
        control_text = "N/A"
    else:
        control_text = f"{fps:.1f}"

    if not performance_status or not performance_status.get("enabled"):
        return [f"FPS ctrl={control_text}"]

    stages = performance_status.get("stages_ms") or {}
    debug_render = performance_status.get("debug_render") or {}
    debug_stream = performance_status.get("debug_stream") or {}
    raw_fps = _safe_float(performance_status.get("raw_fps"), None)
    loop_fps = _safe_float(performance_status.get("loop_fps"), None)
    total_ms = _safe_float(performance_status.get("total_ms"), None)
    vision_ms = _safe_float(stages.get("vision_ms"), None)
    command_ms = _safe_float(stages.get("command_ms"), None)
    render_fps = _safe_float(debug_render.get("render_fps"), None)
    render_ms = _safe_float(debug_render.get("render_ms"), None)
    encode_fps = _safe_float(debug_stream.get("encode_fps"), None)
    encode_ms = _safe_float(debug_stream.get("encode_ms"), None)
    publish_fps = _safe_float(debug_stream.get("publish_fps"), None)
    render_drop = int(debug_render.get("drop_before_render") or 0)
    encode_drop = int(debug_stream.get("drop_before_encode") or 0)

    def fmt(value):
        return "N/A" if value is None else f"{value:.1f}"

    lines = [
        f"FPS ctrl={control_text} raw={fmt(raw_fps)} loop={fmt(loop_fps)}",
    ]
    if debug_render or debug_stream:
        lines.append(
            f"DBG r={fmt(render_fps)} enc={fmt(encode_fps)} pub={fmt(publish_fps)} "
            f"drop={render_drop}/{encode_drop}"
        )
    if vision_ms is not None or command_ms is not None or total_ms is not None:
        lines.append(
            f"MS ai={fmt(vision_ms)} cmd={fmt(command_ms)} ctrl={fmt(total_ms)} "
            f"dbg={fmt(render_ms)} jpg={fmt(encode_ms)}"
        )
    return lines


def _tc264_state_name(value):
    try:
        state_value = int(value)
    except Exception:
        return "STATE_N/A"
    return TC264_STATE_NAMES.get(state_value, f"STATE_{state_value}")


def _fb_flag(feedback, mask):
    try:
        return 1 if (int(feedback.get("safety_flags") or 0) & int(mask)) else 0
    except Exception:
        return 0


def _format_feedback_float(feedback, key, digits=1, default="N/A"):
    value = _safe_float(feedback.get(key), None)
    if value is None:
        return default
    return f"{value:.{digits}f}"


def _draw_key_status_card(frame, lines, color):
    lines = [short_text(_ascii_hud_text(line, "N/A"), 46) for line in lines if line]
    if not lines:
        return frame

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.56
    thickness = 1
    line_h = 25
    pad_x = 10
    pad_y = 8
    text_width = max(cv2.getTextSize(line, font, font_scale, thickness)[0][0] for line in lines)
    card_w = min(frame.shape[1] - 16, text_width + pad_x * 2)
    card_h = min(frame.shape[0] - 16, len(lines) * line_h + pad_y * 2)

    overlay = frame.copy()
    cv2.rectangle(overlay, (8, 8), (8 + card_w, 8 + card_h), (8, 12, 15), -1)
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0.0, frame)
    cv2.rectangle(frame, (8, 8), (8 + card_w, 8 + card_h), color, 1, cv2.LINE_AA)

    y = 8 + pad_y + 17
    for index, line in enumerate(lines):
        line_color = color if index == 0 else (235, 245, 245)
        cv2.putText(frame, line, (8 + pad_x, y), font, font_scale, line_color, thickness, cv2.LINE_AA)
        y += line_h
    return frame


def _draw_grouped_status_panel(h, dtype, color, sections, panel_w=390):
    panel = np.zeros((h, panel_w, 3), dtype=dtype)
    panel[:, :] = (12, 16, 16)
    cv2.rectangle(panel, (0, 0), (panel_w - 1, h - 1), color, 1)

    row_count = sum(1 + len(lines) for _title, lines in sections)
    line_h = max(16, min(21, (h - 18) // max(1, row_count)))
    font_scale = 0.46 if line_h <= 17 else 0.49
    y = 15
    for title, section_lines in sections:
        if y + line_h > h:
            break
        cv2.putText(panel, title, (12, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 1, cv2.LINE_AA)
        cv2.line(panel, (10, y + 5), (panel_w - 12, y + 5), (55, 70, 70), 1, cv2.LINE_AA)
        y += line_h
        for line in section_lines:
            if y + 3 > h:
                break
            cv2.putText(
                panel,
                short_text(_ascii_hud_text(line, "N/A"), 46),
                (16, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (230, 245, 245),
                1,
                cv2.LINE_AA,
            )
            y += line_h
    return panel


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
    detection_status=None,
    task_decision=None,
    plan_result=None,
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
    pose_drop = int(info.get("input_drop_count") or 0)
    pose_rx = int(info.get("datagram_count") or 0)
    pose_fwd_ms = _safe_float(info.get("last_forward_ms"), 0.0)
    pose_handle_ms = _safe_float(info.get("last_handle_ms"), 0.0)
    pose_io_line = f"POSE_IO rx={pose_rx} drop={pose_drop} ms={pose_fwd_ms:.1f}/{pose_handle_ms:.1f}"
    if packet:
        pos = packet.get("pos", [0.0, 0.0, 0.0])
        euler = packet.get("euler", [0.0, 0.0, 0.0])
        age_text = f"{age:.1f}s" if age is not None else "N/A"
        pose_line = f"POSE x={pos[0]:.2f} z={pos[2]:.2f} yaw={euler[1]:.1f} age={age_text}"

    if track_error is not None and np.isfinite(track_error):
        err_text = f"{track_error:.1f}"
    else:
        err_text = "N/A"

    speed_text = "N/A" if target_speed is None else f"{float(target_speed):.2f}"
    control_line = f"CTRL {control_state}"
    cmd_line = f"CMD v={speed_text} err={err_text}"
    seg_line = "SEG N/A"
    fork_line = "FORK N/A"
    road_line = "ROAD N/A"
    if isinstance(segmentation_status, dict):
        seg_source = segmentation_status.get("source") or "N/A"
        frame_id = segmentation_status.get("frame_id")
        seg_frame_id = segmentation_status.get("seg_frame_id")
        lag_frames = segmentation_status.get("seg_lag_frames")
        seg_age_ms = _safe_float(segmentation_status.get("seg_age_ms"), None)
        frame_text = "-" if frame_id is None else str(frame_id)
        seg_frame_text = "-" if seg_frame_id is None else str(seg_frame_id)
        lag_text = "-" if lag_frames is None else str(lag_frames)
        age_text = "-" if seg_age_ms is None else f"{seg_age_ms:.0f}ms"
        seg_line = f"SEG {seg_source} fid={frame_text} sid={seg_frame_text} lag={lag_text} age={age_text}"
        road_state = segmentation_status.get("road_state") or "N/A"
        midline_state = segmentation_status.get("midline_state") or "N/A"
        channel_state = segmentation_status.get("channel_state") or "TRACK"
        road_ratio = _safe_float(segmentation_status.get("road_ratio"), 0.0)
        road_line = f"ROAD {road_state} mid={midline_state} ch={channel_state} ratio={road_ratio:.1%}"
        fork_cls = segmentation_status.get("fork_classifier") or {}
        fork_state = segmentation_status.get("fork_state") or "N/A"
        fork_name = fork_cls.get("name") or "none"
        fork_conf = _safe_float(fork_cls.get("confidence"), None)
        split_rows = segmentation_status.get("fork_split_rows", 0)
        left_pixels = int(segmentation_status.get("fork_left_pixels") or 0)
        right_pixels = int(segmentation_status.get("fork_right_pixels") or 0)
        branch_pixel_threshold = int(segmentation_status.get("fork_branch_pixel_threshold") or 0)
        selected_side = segmentation_status.get("fork_selected_side") or "-"
        conf_text = "N/A" if fork_conf is None else f"{fork_conf:.2f}"
        fork_line = (
            f"FORK {fork_state} cls={fork_name}:{conf_text} "
            f"split={split_rows} pix={left_pixels}/{right_pixels}>{branch_pixel_threshold} sel={selected_side}"
        )

    det_source = (detection_status or {}).get("source") or "N/A"
    det_count = int((detection_status or {}).get("count") or 0)
    det_age_ms = _safe_float((detection_status or {}).get("det_age_ms"), None)
    det_age_text = "-" if det_age_ms is None else f"{det_age_ms:.0f}ms"
    det_line = f"DET {det_source} count={det_count} age={det_age_text}"

    task_decision = task_decision or {}
    plan_result = plan_result or {}
    task_reason = task_decision.get("reason") or "N/A"
    planner_reason = plan_result.get("planner_reason") or "N/A"
    planner_mode = (task_decision.get("planner_intent") or {}).get("mode") or "N/A"
    plan_line = f"PLAN {planner_mode}"
    reason_line = f"WHY task={task_reason} plan={planner_reason}"
    race_state = task_decision.get("race_state") or {}
    race_line = (
        f"RACE lap={race_state.get('current_lap', 0)}/{race_state.get('total_laps', 0)} "
        f"phase={race_state.get('sign_phase', 'N/A')} light={race_state.get('traffic_light_state', 'none')}"
    )

    feedback = get_car_feedback() if get_car_feedback is not None else {"online": False, "error": "waiting"}
    car_lines = []
    if feedback.get("online"):
        fb_age = feedback.get("age")
        fb_age_text = f"{fb_age:.1f}s" if fb_age is not None else "N/A"
        cmd_err = _safe_float(track_error, None)
        tc_err = _safe_float(feedback.get("input_track_error"), None)
        if cmd_err is not None and tc_err is not None:
            err_diff_text = f"{tc_err - cmd_err:.1f}"
        else:
            err_diff_text = "N/A"
        input_age_ms = feedback.get("input_age_ms")
        if input_age_ms is None:
            input_age_text = "sat" if feedback.get("input_age_saturated") else "N/A"
        else:
            input_age_text = f"{int(input_age_ms)}ms"
        seq_text = feedback.get("feedback_seq")
        seq_text = "N/A" if seq_text is None else str(int(seq_text))
        car_lines.append(
            f"ERR cmd={err_text} tc={_format_feedback_float(feedback, 'input_track_error', 1)} "
            f"d={err_diff_text}"
        )
        car_lines.append(
            f"SERVO pwm={feedback.get('servo_output', 0)} "
            f"raw={_format_feedback_float(feedback, 'servo_raw_output', 1)} "
            f"lim={_format_feedback_float(feedback, 'servo_limited_output', 1)}"
        )
        car_lines.append(
            f"MOTOR tgt={_format_feedback_float(feedback, 'input_target_speed', 2)} "
            f"act={_format_feedback_float(feedback, 'actual_speed', 2)} "
            f"out={feedback.get('motor_output', 0)}"
        )
        car_lines.append(
            f"MOTOR ff={_format_feedback_float(feedback, 'motor_feedforward', 0)} "
            f"pid={_format_feedback_float(feedback, 'motor_pid_correction', 0)}"
        )
        car_lines.append(
            f"SAFE to{_fb_flag(feedback, TC264_FB_FLAG_INPUT_TIMEOUT)} "
            f"st{_fb_flag(feedback, TC264_FB_FLAG_STOP_STATE)} "
            f"ss{_fb_flag(feedback, TC264_FB_FLAG_SERVO_SATURATED)} "
            f"ms{_fb_flag(feedback, TC264_FB_FLAG_MOTOR_SATURATED)} "
            f"db{_fb_flag(feedback, TC264_FB_FLAG_TARGET_DEADBAND)}"
        )
        car_lines.append(
            f"TC264 {_tc264_state_name(feedback.get('state'))} "
            f"in={input_age_text} fb={fb_age_text}"
        )
        car_lines.append(
            f"FB seq={seq_text} bad={feedback.get('bad', 0)} "
            f"drop={feedback.get('raw_drop', 0)} fmt={feedback.get('format', '?')}"
        )
    else:
        err = short_text(_ascii_hud_text(feedback.get("error", "waiting"), "waiting"), 28)
        car_lines.append("CAR v=N/A target=N/A")
        car_lines.append(
            f"TC264 waiting fb={feedback.get('count', 0)} bad={feedback.get('bad', 0)}"
        )
        if err:
            car_lines.append(err)

    h = frame.shape[0]
    left_panel = _draw_referee_panel(h, frame.dtype, now)
    key_line = road_line
    if race_state.get("traffic_light_stop"):
        key_line = "STOP traffic light"
    elif race_state.get("finish_stop"):
        key_line = "STOP finish"
    elif str(control_state) == "LINE_LOSS_SAFE_STOP":
        key_line = "STOP line lost"
    _draw_key_status_card(frame, [control_line, cmd_line, key_line], color)

    sections = [
        ("CONTROL", [control_line, cmd_line, plan_line, reason_line, race_line]),
        ("VISION", [road_line, seg_line, det_line, fork_line]),
        ("LINK", [pose_line, pose_io_line] + car_lines),
        ("PERF", _format_compact_perf_lines(performance_status, fps)),
    ]
    panel = _draw_grouped_status_panel(h, frame.dtype, color, sections)
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
