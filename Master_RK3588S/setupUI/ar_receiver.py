import json
import os
import fcntl
import queue
import shutil
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from multiprocessing import resource_tracker, shared_memory
from urllib.parse import urlparse

import cv2
import numpy as np


SHM_NAME = "shm_ar_video"
SHM_HEADER_SIZE = 16
PREVIEW_TITLE = "AR Preview"
INSTANCE_LOCK_PATH = "/tmp/sysu_ddl_ar_receiver.lock"
VISION_CONTROL_SEND_DEFAULT = True


def acquire_instance_lock():
    lock_path = os.environ.get("AR_RECEIVER_LOCK_PATH", INSTANCE_LOCK_PATH)
    lock_file = open(lock_path, "a+", encoding="ascii")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.seek(0)
        owner = lock_file.read().strip() or "unknown"
        lock_file.close()
        print("[AR_RECEIVER] another instance is already running pid={}".format(
            owner))
        return None
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    return lock_file


def activate_existing_preview():
    host = os.environ.get("AR_PREVIEW_CONTROL_HOST", "127.0.0.1")
    if host in {"", "0.0.0.0", "::"}:
        host = "127.0.0.1"
    port = int(env_float("AR_PREVIEW_CONTROL_PORT", 9105))
    request = urllib.request.Request(
        f"http://{host}:{port}/api/preview",
        data=json.dumps({"enabled": True}).encode("ascii"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=1.0) as response:
            activated = 200 <= int(response.status) < 300
    except (OSError, urllib.error.URLError, ValueError) as exc:
        print(f"[AR_RECEIVER] existing instance could not be activated: {exc}")
        return False
    if activated:
        print("[AR_RECEIVER] existing instance preview activated")
    return activated


def env_flag(name, default):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() not in {"0", "false", "no", "off"}


def env_float(name, default):
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return float(default)


def env_int(name, default):
    try:
        return int(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return int(default)


def instruction_direction(instruction):
    instruction = instruction if isinstance(instruction, dict) else {}
    direction = str(instruction.get("direction") or instruction.get("preferred_branch") or "").strip().lower()
    return direction if direction in {"left", "right"} else ""


class RuntimeState:
    def __init__(self, preview_enabled=True):
        self._lock = threading.Lock()
        self._preview_enabled = bool(preview_enabled)
        self._preview_running = False
        self._preview_error = ""
        self._ocr_status = "not_started"
        self._ocr_text = ""
        self._api_choice = ""
        self._api_reason = ""
        self._ocr_worker_ready = False
        self._ocr_error = ""
        self._process_fps = 0.0
        self._inference_fps = 0.0
        self._control_status = {"enabled": False, "status": "not_started"}
        self._perception_status = {"enabled": False, "status": "not_started"}

    def preview_enabled(self):
        with self._lock:
            return self._preview_enabled

    def set_preview_enabled(self, enabled):
        with self._lock:
            self._preview_enabled = bool(enabled)
            if not self._preview_enabled:
                self._preview_error = ""

    def set_preview_runtime(self, running, error=""):
        with self._lock:
            self._preview_running = bool(running)
            self._preview_error = str(error or "")

    def update_ocr(self, response):
        response = response if isinstance(response, dict) else {}
        instruction = response.get("instruction") or {}
        latest_instruction = response.get("latest_instruction") or {}
        display_instruction = (
            instruction if instruction_direction(instruction)
            else latest_instruction)
        ocr = response.get("ocr") or {}
        next_text = str(ocr.get("text") or display_instruction.get("source_text") or "")[:120]
        next_choice = instruction_direction(display_instruction)
        with self._lock:
            if response.get("clear_result"):
                self._ocr_text = ""
                self._api_choice = ""
                self._api_reason = ""
            self._ocr_status = str(
                response.get("control_phase") or
                response.get("status") or "not_started")
            if next_text:
                self._ocr_text = next_text
            if next_choice in {"left", "right"}:
                self._api_choice = next_choice
                self._api_reason = str(display_instruction.get("reason") or "")[:160]
            self._ocr_worker_ready = bool(response.get("worker_ready", self._ocr_worker_ready))
            self._ocr_error = str(response.get("error") or instruction.get("api_error") or "")[:240]

    def update_control(self, status):
        with self._lock:
            self._control_status = status if isinstance(status, dict) else {"enabled": False, "status": str(status)}

    def update_performance(self, process_fps, inference_fps):
        with self._lock:
            self._process_fps = float(process_fps)
            self._inference_fps = float(inference_fps)

    def update_perception(self, result=None, error=""):
        result = result if isinstance(result, dict) else {}
        detections = result.get("detections") or []
        road = result.get("road") or {}
        centerline = result.get("centerline") or {}
        labels = {}
        for detection in detections:
            label = str(detection.get("label") or "unknown")
            labels[label] = labels.get(label, 0) + 1
        road_mask = road.get("mask")
        road_coverage = float(np.mean(road_mask)) if isinstance(road_mask, np.ndarray) and road_mask.size else 0.0
        paths = centerline.get("paths") or []
        temporal = result.get("temporal") or {}
        vision_control = result.get("vision_control") or {}
        vision_timings = vision_control.get("timings_ms") or {}
        timings = result.get("timings_ms") or {}
        path_scores = centerline.get("path_scores")
        if isinstance(path_scores, np.ndarray):
            path_scores = path_scores.tolist()
        path_count_scores = centerline.get("path_count_scores")
        if path_count_scores is None:
            path_count_scores = centerline.get("path_count_probabilities")
        if isinstance(path_count_scores, np.ndarray):
            path_count_scores = path_count_scores.tolist()
        summary = {
            "enabled": not bool(error),
            "status": "error" if error else ("ready" if result else "not_started"),
            "error": str(error or "")[:240],
            "detection_count": len(detections),
            "detection_labels": labels,
            "road_coverage": road_coverage,
            "path_count": int(centerline.get(
                "detected_path_count",
                result.get(
                    "detected_path_count",
                    centerline.get("path_count", len(paths))))),
            "valid_path_count": int(centerline.get(
                "valid_path_count", len(paths))),
            "path_point_counts": [
                len(path.get("points_xy"))
                if path.get("points_xy") is not None else 0
                for path in paths
            ],
            "path_roles": [str(path.get("role") or "") for path in paths],
            "path_scores": list(path_scores or []),
            "path_count_scores": list(path_count_scores or []),
            "raw_path_count": int(temporal.get(
                "raw_path_count", centerline.get("path_count", len(paths)))),
            "temporal_status": str(temporal.get("status") or "disabled"),
            "temporal_pending_count": temporal.get("pending_path_count"),
            "vision_control": {
                "route_state": str(
                    vision_control.get("route_state") or "NONE"),
                "raw_route_state": str(
                    vision_control.get("raw_route_state") or "NONE"),
                "route_reason": str(
                    vision_control.get("route_reason") or ""),
                "pending_route_state": vision_control.get(
                    "pending_route_state"),
                "branch_lock": vision_control.get("branch_lock"),
                "branch_lock_source": vision_control.get(
                    "branch_lock_source"),
                "selected_slot_lock": vision_control.get(
                    "selected_slot_lock"),
                "selected_slot": vision_control.get("selected_slot"),
                "ocr_lock_expired": bool(vision_control.get(
                    "ocr_lock_expired", False)),
                "ocr_lock_remaining_s": vision_control.get(
                    "ocr_lock_remaining_s"),
                "turnsign_trim_separation_640": vision_control.get(
                    "turnsign_trim_separation_640"),
                "turnsign_trim_lookahead_separation_640": (
                    vision_control.get(
                        "turnsign_trim_lookahead_separation_640")),
                "turnsign_trim_direction": vision_control.get(
                    "turnsign_trim_direction"),
                "turnsign_trim_position_delta_640": vision_control.get(
                    "turnsign_trim_position_delta_640"),
                "turnsign_trim_fresh_frames": vision_control.get(
                    "turnsign_trim_fresh_frames"),
                "turnsign_trim_current_centered": vision_control.get(
                    "turnsign_trim_current_centered"),
                "turnsign_trim_stop_ready": vision_control.get(
                    "turnsign_trim_stop_ready"),
                "turnsign_trim_line_split_ready": vision_control.get(
                    "turnsign_trim_line_split_ready"),
                "turnsign_trim_line_ever_split": vision_control.get(
                    "turnsign_trim_line_ever_split"),
                "turnsign_trim_overshoot_latched": vision_control.get(
                    "turnsign_trim_overshoot_latched"),
                "turnsign_trim_accept_ready": vision_control.get(
                    "turnsign_trim_accept_ready"),
                "timings_ms": {
                    "path_search": float(
                        vision_timings.get("path_search", 0.0)),
                    "control_total": float(
                        vision_timings.get("control_total", 0.0)),
                },
            },
            "timings_ms": {
                key: float(timings.get(key, 0.0))
                for key in (
                    "preprocess", "inference", "fifo_wait", "postprocess",
                    "temporal_filter", "total")
            },
        }
        with self._lock:
            self._perception_status = summary

    def snapshot(self):
        with self._lock:
            return {
                "preview_enabled": self._preview_enabled,
                "preview_running": self._preview_running,
                "preview_error": self._preview_error,
                "ocr_status": self._ocr_status,
                "ocr_text": self._ocr_text,
                "api_choice": self._api_choice,
                "api_reason": self._api_reason,
                "ocr_worker_ready": self._ocr_worker_ready,
                "ocr_error": self._ocr_error,
                "process_fps": self._process_fps,
                "inference_fps": self._inference_fps,
                "control": self._control_status,
                "perception": self._perception_status,
            }


PREVIEW_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AR 本地预览控制</title>
  <style>
    body { font-family: sans-serif; color: #eaf7f4; background: #101817; margin: 0; }
    main { max-width: 620px; margin: 10vh auto; padding: 28px; background: #172321; border: 1px solid #31534c; border-radius: 14px; }
    .row { display: flex; align-items: center; justify-content: space-between; gap: 20px; }
    .switch { position: relative; width: 62px; height: 34px; flex: 0 0 auto; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; inset: 0; cursor: pointer; background: #58635f; border-radius: 34px; transition: .2s; }
    .slider:before { content: ""; position: absolute; width: 26px; height: 26px; left: 4px; top: 4px; background: white; border-radius: 50%; transition: .2s; }
    input:checked + .slider { background: #1ecf92; }
    input:checked + .slider:before { transform: translateX(28px); }
    #state { color: #8ce8c9; }
    #error { color: #ff8d8d; white-space: pre-wrap; }
    .muted { color: #9cadA8; font-size: .92rem; line-height: 1.6; }
    code { color: #8ce8c9; }
  </style>
</head>
<body>
<main>
  <div class="row">
    <div>
      <h2>RK3588 本地视频弹窗</h2>
      <div id="state">正在读取状态……</div>
    </div>
    <label class="switch" title="开启或关闭 RK3588 本地弹窗">
      <input id="preview-toggle" type="checkbox">
      <span class="slider"></span>
    </label>
  </div>
  <p class="muted">开：收到视频帧后弹出 <code>AR Preview</code>。关：关闭本地弹窗，但目标检测和 OCR/API 继续运行。</p>
  <p id="ocr" class="muted"></p>
  <p id="error"></p>
</main>
<script>
const toggle = document.getElementById('preview-toggle');
let writing = false;
async function refresh() {
  try {
    const response = await fetch('/api/preview', {cache: 'no-store'});
    const data = await response.json();
    if (!writing) toggle.checked = !!data.preview_enabled;
    document.getElementById('state').textContent = data.preview_enabled
      ? (data.preview_running ? '弹窗已开启' : '已开启，等待视频帧')
      : '弹窗已关闭';
    const control = data.control || {};
    const pose = control.pose || {};
    const gamepad = control.gamepad || {};
    const serial = control.serial || {};
    document.getElementById('ocr').textContent =
      `OCR：${data.ocr_status || '-'}　Worker：${data.ocr_worker_ready ? 'ready' : 'not ready'}　API：${data.api_choice || '-'}`
      + `　控制：${control.enabled ? (control.source || '-') : 'off'}`
      + `　定位包：${pose.packet_count ?? '-'}　手柄：${gamepad.active ? 'active' : 'idle'}　串口：${serial.online ? 'ON' : 'OFF'}`;
    document.getElementById('error').textContent = data.preview_error || data.ocr_error || '';
  } catch (error) {
    document.getElementById('state').textContent = '状态读取失败';
    document.getElementById('error').textContent = error.message;
  }
}
toggle.addEventListener('change', async () => {
  writing = true;
  try {
    await fetch('/api/preview', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({enabled: toggle.checked})
    });
  } catch (error) {
    document.getElementById('error').textContent = error.message;
  } finally {
    writing = false;
    refresh();
  }
});
refresh();
setInterval(refresh, 1000);
</script>
</body>
</html>
"""


class PreviewControlServer:
    def __init__(self, state, host="0.0.0.0", port=9105):
        self.state = state
        self.host = host
        self.port = int(port)
        self.server = None
        self.thread = None

    def start(self):
        state = self.state

        class Handler(BaseHTTPRequestHandler):
            def send_payload(self, payload, status=200, content_type="application/json; charset=utf-8"):
                if isinstance(payload, (dict, list)):
                    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                elif isinstance(payload, str):
                    body = payload.encode("utf-8")
                else:
                    body = bytes(payload)
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path in {"/", "/preview"}:
                    self.send_payload(PREVIEW_PAGE, content_type="text/html; charset=utf-8")
                    return
                if path == "/api/preview":
                    self.send_payload(state.snapshot())
                    return
                self.send_payload({"error": "not_found"}, status=404)

            def do_POST(self):
                if urlparse(self.path).path != "/api/preview":
                    self.send_payload({"error": "not_found"}, status=404)
                    return
                try:
                    length = min(4096, max(0, int(self.headers.get("Content-Length", "0"))))
                    payload = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                    enabled = payload.get("enabled")
                    if not isinstance(enabled, bool):
                        raise ValueError("enabled must be true or false")
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                    self.send_payload({"error": str(exc)}, status=400)
                    return
                state.set_preview_enabled(enabled)
                self.send_payload(state.snapshot())

            def log_message(self, _format, *_args):
                return

        try:
            self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as exc:
            print(f"[PREVIEW WEB] disabled: {exc}")
            return False
        self.port = int(self.server.server_address[1])
        self.thread = threading.Thread(
            target=self.server.serve_forever,
            name="preview-control-web",
            daemon=True,
        )
        self.thread.start()
        print(f"[PREVIEW WEB] http://127.0.0.1:{self.port}/")
        return True

    def stop(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None


class FfplayPreview:
    """Display the newest frame in a native window without OpenCV HighGUI."""

    def __init__(self, state, fps=30.0):
        self.state = state
        self.fps = max(1.0, float(fps))
        self.frame_interval = 1.0 / self.fps
        self.ffplay = shutil.which("ffplay")
        self.process = None
        self.writer = None
        self.frames = None
        self.stop_event = None
        self.frame_size = None
        self.next_frame_ts = 0.0
        self._lock = threading.Lock()
        self._intentional_stop = False

    def _start(self, width, height):
        if not self.ffplay:
            self.state.set_preview_runtime(False, "ffplay not found")
            return False
        self.stop(disable=False)
        command = [
            self.ffplay,
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostats",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-framedrop",
            "-sync",
            "ext",
            "-f",
            "rawvideo",
            "-pixel_format",
            "bgr24",
            "-video_size",
            f"{width}x{height}",
            "-framerate",
            f"{self.fps:g}",
            "-window_title",
            PREVIEW_TITLE,
            "-i",
            "-",
        ]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except OSError as exc:
            self.state.set_preview_runtime(False, str(exc))
            return False
        with self._lock:
            self.process = process
            self.frames = queue.Queue(maxsize=1)
            self.stop_event = threading.Event()
            self.frame_size = (width, height)
            self.next_frame_ts = 0.0
            self._intentional_stop = False
            self.writer = threading.Thread(
                target=self._write_frames,
                args=(process, self.frames, self.stop_event),
                name="ffplay-preview-writer",
                daemon=True,
            )
            self.writer.start()
        self.state.set_preview_runtime(True)
        return True

    def _write_frames(self, process, frames, stop_event):
        error = ""
        try:
            while not stop_event.is_set():
                try:
                    frame = frames.get(timeout=0.2)
                except queue.Empty:
                    if process.poll() is not None:
                        error = "preview window closed"
                        break
                    continue
                if frame is None:
                    break
                renderer = None
                if isinstance(frame, tuple):
                    frame, renderer = frame
                if renderer is not None:
                    frame = renderer(frame)
                payload = memoryview(np.ascontiguousarray(frame)).cast("B")
                while payload and not stop_event.is_set():
                    written = os.write(process.stdin.fileno(), payload)
                    if written <= 0:
                        raise BrokenPipeError("ffplay input closed")
                    payload = payload[written:]
        except Exception as exc:
            if not stop_event.is_set():
                error = str(exc)
        finally:
            with self._lock:
                intentional = self._intentional_stop
            if not intentional and not stop_event.is_set():
                self.state.set_preview_enabled(False)
            self.state.set_preview_runtime(False, "" if intentional else error)

    def show(self, frame, renderer=None):
        if frame is None or frame.size == 0:
            return
        height, width = frame.shape[:2]
        with self._lock:
            process = self.process
            frame_size = self.frame_size
        if process is None or process.poll() is not None or frame_size != (width, height):
            if not self._start(width, height):
                return
        now = time.monotonic()
        with self._lock:
            frames = self.frames
            if now < self.next_frame_ts:
                return
            self.next_frame_ts = now + self.frame_interval
        if frames is None:
            return
        try:
            frames.put_nowait((frame, renderer))
        except queue.Full:
            try:
                frames.get_nowait()
            except queue.Empty:
                pass
            try:
                frames.put_nowait((frame, renderer))
            except queue.Full:
                pass

    def stop(self, disable=False):
        if disable:
            self.state.set_preview_enabled(False)
        with self._lock:
            process = self.process
            writer = self.writer
            frames = self.frames
            stop_event = self.stop_event
            self._intentional_stop = True
            self.process = None
            self.writer = None
            self.frames = None
            self.stop_event = None
            self.frame_size = None
            self.next_frame_ts = 0.0
        if stop_event is not None:
            stop_event.set()
        if frames is not None:
            try:
                frames.put_nowait(None)
            except queue.Full:
                pass
        if process is not None:
            try:
                if process.stdin is not None:
                    process.stdin.close()
            except OSError:
                pass
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=0.6)
                except subprocess.TimeoutExpired:
                    process.kill()
        if writer is not None and writer is not threading.current_thread():
            writer.join(timeout=0.8)
        self.state.set_preview_runtime(False)


def remove_shm_from_resource_tracker():
    try:
        resource_tracker.unregister("/" + SHM_NAME, "shared_memory")
    except Exception:
        pass


def create_ocr_processor():
    if not env_flag("AR_TURNSIGN_OCR_ENABLED", True):
        print("[OCR] disabled by AR_TURNSIGN_OCR_ENABLED=0")
        return None
    try:
        from turnsign_ocr_api import AsyncTurnSignOcrApiProcessor

        processor = AsyncTurnSignOcrApiProcessor(
            worker_cpu_set=os.environ.get("AR_TURNSIGN_OCR_CPUSET", "0-3"),
            confirm_frames=max(1, int(env_float("AR_TURNSIGN_CONFIRM_FRAMES", 1))),
            snapshot_min_area_ratio=env_float(
                "AR_TURNSIGN_SNAPSHOT_MIN_AREA_RATIO", 0.02),
            snapshot_edge_margin_ratio=env_float(
                "AR_TURNSIGN_SNAPSHOT_EDGE_MARGIN_RATIO", 0.10),
            confirm_iou=env_float("AR_TURNSIGN_CONFIRM_IOU", 0.30),
            confirm_max_misses=max(
                0, int(env_float("AR_TURNSIGN_CONFIRM_MAX_MISSES", 2))),
            min_det_score=env_float("AR_TURNSIGN_MIN_DET_SCORE", 0.40),
            min_area_ratio=env_float("AR_TURNSIGN_MIN_AREA_RATIO", 0.01),
            door_conflict_score=env_float(
                "AR_TURNSIGN_DOOR_CONFLICT_SCORE", 0.80),
            door_conflict_distance_px_640=env_float(
                "AR_TURNSIGN_DOOR_CONFLICT_DISTANCE_PX_640", 120.0),
            detection_line_ratio=env_float(
                "AR_TURNSIGN_DETECTION_LINE_RATIO", 185.0 / 480.0),
            preconfirm_line_distance_px_480=env_float(
                "AR_TURNSIGN_PRECONFIRM_LINE_DISTANCE_480", 45.0),
            edge_margin_ratio=env_float(
                "AR_TURNSIGN_EDGE_MARGIN_RATIO", 0.15),
            confirmed_max_misses=max(
                0, int(env_float("AR_TURNSIGN_CONFIRMED_MAX_MISSES", 3))),
            session_absence_timeout_s=env_float(
                "AR_TURNSIGN_SESSION_ABSENCE_TIMEOUT_S", 3.0),
            ocr_response_timeout_s=env_float(
                "AR_TURNSIGN_RESPONSE_TIMEOUT_S", 10.0),
            min_ocr_confidence=env_float("AR_TURNSIGN_MIN_OCR_CONFIDENCE", 0.40),
            stable_frames=max(1, int(env_float("AR_TURNSIGN_STABLE_FRAMES", 2))),
            stable_duration_s=env_float("AR_TURNSIGN_STABLE_DURATION_S", 0.50),
            stable_bypass_confidence=env_float("AR_TURNSIGN_STABLE_BYPASS_CONFIDENCE", 0.90),
            stable_bypass_min_text_len=max(1, int(env_float("AR_TURNSIGN_STABLE_BYPASS_MIN_TEXT_LEN", 6))),
            ocr_interval=env_float("AR_TURNSIGN_OCR_INTERVAL", 0.10),
            api_cooldown=env_float("AR_TURNSIGN_API_COOLDOWN", 1.0),
            cache_ttl=0.0,
            async_api=True,
            log_func=lambda message: print(f"[OCR] {message}"),
        )
        print(
            "[OCR] TurnSign OCR/API enabled "
            "(1 complete box; det/OCR confidence>=0.40; "
            "snapshot area>=2% in center 80%; "
            "exit after 3s no TurnSign or 10s no OCR response)")
        return processor
    except Exception as exc:
        print(f"[OCR] unavailable: {exc}")
        return None


def create_control_runtime(state):
    if not env_flag("AR_CONTROL_RUNTIME_ENABLED", True):
        status = {"enabled": False, "status": "disabled"}
        state.update_control(status)
        print("[CONTROL] disabled by AR_CONTROL_RUNTIME_ENABLED=0")
        return None
    try:
        from control_runtime import ControlRuntime

        runtime = ControlRuntime(log_func=lambda message: print(f"[CONTROL] {message}"))
        runtime.start()
        status = runtime.snapshot()
        status["enabled"] = True
        status["status"] = "running"
        state.update_control(status)
        print("[CONTROL] runtime enabled: pose UDP, gamepad UDP, serial bridge")
        return runtime
    except Exception as exc:
        status = {"enabled": False, "status": "unavailable", "error": str(exc)}
        state.update_control(status)
        print(f"[CONTROL] unavailable: {exc}")
        return None


def create_vision_control_planner():
    if not (
        env_flag("AR_VISION_CONTROL_DEBUG", True)
        or env_flag("AR_VISION_CONTROL_SEND", VISION_CONTROL_SEND_DEFAULT)
    ):
        print("[VISION CONTROL] disabled by AR_VISION_CONTROL_DEBUG=0 and AR_VISION_CONTROL_SEND=0")
        return None, None
    try:
        from vision_control import VisionControlPlanner, render_vision_control_debug

        planner = VisionControlPlanner()
        mode = "send" if env_flag("AR_VISION_CONTROL_SEND", VISION_CONTROL_SEND_DEFAULT) else "debug-only"
        print(f"[VISION CONTROL] enabled ({mode})")
        return planner, render_vision_control_debug
    except Exception as exc:
        print(f"[VISION CONTROL] unavailable: {exc}")
        return None, None


def add_runtime_overlay(frame, process_fps, inference_fps, ocr_response):
    cv2.putText(
        frame,
        f"Actual FPS: {process_fps:.1f}  AI FPS: {inference_fps:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )
    if not isinstance(ocr_response, dict):
        return frame
    phase = str(ocr_response.get("control_phase") or "")
    phase_labels = {
        "no_turnsign": "NONE",
        "turnsign_low_score": "LOW_SCORE",
        "turnsign_too_small": "TOO_SMALL",
        "turnsign_too_far": "TOO_FAR",
        "turnsign_incomplete_bbox": "INCOMPLETE_BOX",
        "turnsign_door_conflict": "IGNORE_NEAR_DOOR",
        "turnsign_bad_bbox": "BAD_BOX",
        "turnsign_confirming": "CONFIRM",
        "turnsign_trim_forward": "TRIM_FORWARD_0.08_0.5S",
        "turnsign_trim_reverse": "TRIM_REVERSE_0.08_0.5S",
        "turnsign_trim_settle": "TRIM_OBSERVE_0.8S",
        "turnsign_trim_verify": "TRIM_VERIFY_SIGN",
        "turnsign_trim_ready": "TRIM_READY_TWO_LINES",
        "turnsign_approach": "APPROACH",
        "turnsign_edge_left": "EDGE_LEFT",
        "turnsign_edge_right": "EDGE_RIGHT",
        "turnsign_edge_over_line": "EDGE_OVER_LINE_STOP",
        "turnsign_missing_hold": "MISS_HOLD",
        "turnsign_missing_stop": "MISS_STOP",
        "turnsign_ocr_wait": "STOP_WAIT_OCR",
        "turnsign_consumed": "RESULT_LATCHED",
        "turnsign_exit_no_sign": "EXIT_NO_SIGN_3S",
        "turnsign_exit_ocr_timeout": "EXIT_TIMEOUT_10S",
        "ocr_wait_route": "WAIT_ROUTE",
        "ocr_route_ready": "ROUTE_READY",
    }
    status = phase_labels.get(
        phase, str(ocr_response.get("status") or "-"))
    if phase == "turnsign_confirming":
        status = (
            f"{status} {int(ocr_response.get('confirm_count') or 0)}/"
            f"{int(ocr_response.get('confirm_frames') or 1)}")
    instruction = ocr_response.get("instruction") or {}
    latest_instruction = ocr_response.get("latest_instruction") or {}
    choice = instruction_direction(instruction) or instruction_direction(latest_instruction) or "-"
    cv2.putText(
        frame,
        f"OCR: {status}  LOCK: {choice}",
        (10, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )
    line_y = int(round(float(ocr_response.get(
        "detection_line_y", frame.shape[0] * (185.0 / 480.0)))))
    left_edge = int(round(float(ocr_response.get(
        "left_edge_x", frame.shape[1] * 0.15))))
    right_edge = int(round(float(ocr_response.get(
        "right_edge_x", frame.shape[1] * 0.85))))
    cv2.line(
        frame, (0, line_y), (frame.shape[1] - 1, line_y),
        (0, 200, 255), 1, cv2.LINE_AA)
    cv2.line(
        frame, (left_edge, 0), (left_edge, line_y),
        (0, 200, 255), 1, cv2.LINE_AA)
    cv2.line(
        frame, (right_edge, 0), (right_edge, line_y),
        (0, 200, 255), 1, cv2.LINE_AA)
    return frame


def read_frame(shm):
    header = bytes(shm.buf[:SHM_HEADER_SIZE])
    frame_id, width, height = struct.unpack("QII", header)
    if frame_id == 0:
        return frame_id, None
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid frame size: {width}x{height}")
    size = int(width) * int(height) * 3
    if SHM_HEADER_SIZE + size > len(shm.buf):
        raise BufferError("shared-memory frame exceeds buffer")
    view = np.ndarray(
        (height, width, 3),
        dtype=np.uint8,
        buffer=shm.buf[SHM_HEADER_SIZE : SHM_HEADER_SIZE + size],
    )
    frame = view.copy()
    del view
    second_header = bytes(shm.buf[:SHM_HEADER_SIZE])
    second_id, second_width, second_height = struct.unpack("QII", second_header)
    if (second_id, second_width, second_height) != (frame_id, width, height):
        return 0, None
    return frame_id, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def main():
    instance_lock = acquire_instance_lock()
    if instance_lock is None:
        activate_existing_preview()
        return
    # Keep RKNN initialization under the main guard.  The OCR worker uses the
    # multiprocessing "spawn" method and must not initialize the NPU again.
    render_perception = None
    render_mode = "off"
    try:
        from infer_wrap import InferWrap
        from infer_wrap.base.func import CLASSES, RENDER_MODE, render_result

        render_perception = render_result
        render_mode = RENDER_MODE

        if not CLASSES:
            raise RuntimeError("detector class list is empty")
        infer = InferWrap(TPEs=max(1, env_int("MULTITASK_RKNN_TPES", 3)))
        print(
            f"[PERCEPTION] enabled model={infer.model_path} "
            f"classes={len(CLASSES)}: {', '.join(CLASSES)} "
            f"npu_mode={infer.npu_mode} "
            f"npu_workers={infer.TPEs} "
            f"pipeline_depth={infer.pipeline_depth} "
            f"render_mode={render_mode} "
            f"warmup_inference_ms="
            f"{','.join(f'{value:.1f}' for value in infer.warmup_inference_ms) or 'disabled'}"
        )
    except Exception as exc:
        infer = None
        print(f"[PERCEPTION] unavailable; preview-only mode: {exc}")

    state = RuntimeState(preview_enabled=env_flag("AR_LOCAL_PREVIEW", True))
    preview = FfplayPreview(state, fps=env_float("AR_PREVIEW_FPS", 60.0))
    web = PreviewControlServer(
        state,
        host=os.environ.get("AR_PREVIEW_CONTROL_HOST", "0.0.0.0"),
        port=int(env_float("AR_PREVIEW_CONTROL_PORT", 9105)),
    )
    web.start()
    ocr_processor = create_ocr_processor()
    control_runtime = create_control_runtime(state)
    vision_control_planner, render_vision_control = create_vision_control_planner()
    vision_control_send = env_flag("AR_VISION_CONTROL_SEND", VISION_CONTROL_SEND_DEFAULT)

    print("[AR_RECEIVER] ready; waiting for camera shared memory...")
    print(f"[AR_RECEIVER] DISPLAY={os.environ.get('DISPLAY', 'NOT SET')}")
    sys.stdout.flush()

    shm = None
    last_ocr_log_key = None
    next_control_status_ts = 0.0

    def refresh_control_status(force=False):
        nonlocal next_control_status_ts
        if control_runtime is None:
            return
        now = time.time()
        if not force and now < next_control_status_ts:
            return
        try:
            control_status = control_runtime.snapshot()
            control_status["enabled"] = True
            control_status["status"] = "running"
            state.update_control(control_status)
        except Exception as exc:
            state.update_control({"enabled": True, "status": "snapshot_error", "error": str(exc)})
        next_control_status_ts = now + 0.5

    try:
        while True:
            refresh_control_status()

            if shm is None:
                try:
                    shm = shared_memory.SharedMemory(name=SHM_NAME)
                    remove_shm_from_resource_tracker()
                    print("[AR_RECEIVER] camera shared memory connected")
                except FileNotFoundError:
                    if not state.preview_enabled():
                        preview.stop(disable=False)
                    time.sleep(0.5)
                    continue

            last_frame_id = 0
            fps_started = time.time()
            fps_frames = 0
            current_fps = 0.0
            inference_frames = 0
            current_inference_fps = 0.0
            latest_ocr_response = None

            while shm is not None:
                refresh_control_status()
                if not state.preview_enabled():
                    preview.stop(disable=False)
                try:
                    frame_id, frame = read_frame(shm)
                except (ValueError, struct.error, BufferError, FileNotFoundError):
                    print("[AR_RECEIVER] camera signal lost; waiting for recovery")
                    try:
                        shm.close()
                    except Exception:
                        pass
                    shm = None
                    preview.stop(disable=False)
                    time.sleep(0.5)
                    break

                if frame is None or frame_id == last_frame_id:
                    time.sleep(0.002)
                    continue
                last_frame_id = frame_id

                display_frame = frame
                detections = []
                ocr_source_frame = frame
                perception_result = None
                if infer is not None:
                    try:
                        infer_result, ready = infer.infer(frame)
                        if ready and isinstance(infer_result, dict):
                            inference_frames += 1
                            perception_result = infer_result
                            inferred_frame = infer_result.get("_source_frame")
                            if inferred_frame is None:
                                inferred_frame = infer_result.get("frame")
                            display_frame = inferred_frame if inferred_frame is not None else frame
                            detections = infer_result.get("detections") or []
                            ocr_source_frame = infer_result.get("ocr_frame")
                            if ocr_source_frame is None:
                                ocr_source_frame = display_frame
                    except Exception as exc:
                        print(f"[PERCEPTION] disabled after runtime error: {exc}")
                        state.update_perception(error=exc)
                        try:
                            infer.release()
                        except Exception:
                            pass
                        infer = None

                if ocr_processor is not None:
                    try:
                        latest_ocr_response = ocr_processor.process(
                            ocr_source_frame,
                            detections,
                            timestamp=time.time(),
                        )
                        state.update_ocr(latest_ocr_response)
                        instruction = (
                            latest_ocr_response.get("instruction")
                            or latest_ocr_response.get("latest_instruction")
                            or {}
                        )
                        log_key = (
                            latest_ocr_response.get("status"),
                            instruction.get("source_text"),
                            instruction.get("preferred_branch"),
                            latest_ocr_response.get("error") or instruction.get("api_error"),
                        )
                        if log_key != last_ocr_log_key and log_key[0] not in {"no_turnsign", "ocr_throttled"}:
                            print(
                                "[OCR] "
                                f"status={log_key[0]} text={log_key[1] or '-'} "
                                f"choice={log_key[2] or '-'} error={log_key[3] or '-'}"
                            )
                        last_ocr_log_key = log_key
                    except Exception as exc:
                        print(f"[OCR] process error: {exc}")

                if vision_control_planner is not None and perception_result is not None:
                    try:
                        command, _vision_debug = vision_control_planner.update(
                            perception_result,
                            latest_ocr_response,
                            now=time.monotonic(),
                        )
                        # Vision control may refine the OCR phase to the lock
                        # reverse brake, WAIT_ROUTE or ROUTE_READY status.
                        state.update_ocr(latest_ocr_response)
                        if vision_control_send and control_runtime is not None and command:
                            control_runtime.update_vision_command(
                                command["track_error"],
                                command["target_speed"],
                                state_cmd=command["state_cmd"],
                                flags=command["flags"],
                            )
                    except Exception as exc:
                        print(f"[VISION CONTROL] process error: {exc}")
                        if vision_control_send and control_runtime is not None:
                            control_runtime.clear_vision_command()
                if perception_result is not None:
                    state.update_perception(perception_result)

                fps_frames += 1
                now = time.time()
                if now - fps_started >= 1.0:
                    current_fps = fps_frames / max(1e-6, now - fps_started)
                    current_inference_fps = inference_frames / max(1e-6, now - fps_started)
                    fps_frames = 0
                    inference_frames = 0
                    fps_started = now
                    state.update_performance(
                        current_fps, current_inference_fps)

                if state.preview_enabled():
                    def render_preview(
                            target,
                            result=perception_result,
                            process_fps=current_fps,
                            inference_fps=current_inference_fps,
                            ocr_response=latest_ocr_response):
                        if result is not None and render_perception is not None:
                            render_perception(target, result, mode=render_mode)
                        if result is not None and render_vision_control is not None:
                            render_vision_control(target, result)
                        return add_runtime_overlay(
                            target, process_fps, inference_fps, ocr_response)

                    preview.show(display_frame, renderer=render_preview)
    except KeyboardInterrupt:
        print("\n[AR_RECEIVER] stopped by user")
    finally:
        preview.stop(disable=False)
        web.stop()
        if control_runtime is not None:
            control_runtime.stop()
        if ocr_processor is not None and hasattr(ocr_processor, "close"):
            ocr_processor.close()
        if infer is not None:
            infer.release()
        if shm is not None:
            try:
                shm.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
