import threading
import time
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


# ===== Debug stream parameter area =====
# This server publishes the fully rendered ar_receiver debug frame:
# vision overlay + local planner path + left/right HUD panels.
DEBUG_STREAM_DEFAULTS = {
    # Enable the browser-viewable debug video stream.
    "ENABLE": True,

    # Bind to all interfaces so Windows can open http://RK_IP:8090/debug_feed.
    "HOST": "0.0.0.0",

    # Keep this separate from official AR stream 8080 and status API 9105.
    "PORT": 8090,

    # MJPEG stream endpoint. Browsers can open this URL directly.
    "PATH": "/debug_feed",

    # Single JPEG snapshot endpoint, useful for quick checks.
    "SNAPSHOT_PATH": "/snapshot.jpg",

    # Legacy stream frame rate cap. 0 disables the cap so Windows sees every
    # encoded debug frame the board can produce.
    "FPS": 0.0,

    # JPEG quality. Higher is clearer but costs more bandwidth and CPU.
    "JPEG_QUALITY": 35,

    # Resize the published frame if it is wider than this value.
    # Use 0 or a negative value to disable resizing.
    "MAX_WIDTH": 1280,

    # If no frame arrives for this many seconds, /health reports stale.
    "STALE_SECONDS": 2.0,

    # Optional Linux CPU affinity for debug HTTP/encoder threads, e.g. "0-3".
    "CPUSET": os.environ.get("AR_DEBUG_STREAM_CPUSET", ""),
}


def _html_page(port, stream_path, snapshot_path):
    return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RK Debug Stream</title>
  <style>
    body {{ margin: 0; background: #080b0e; color: #e8fff8; font-family: Arial, sans-serif; }}
    header {{ padding: 10px 14px; background: #11181d; border-bottom: 1px solid #234; }}
    main {{ display: flex; justify-content: center; padding: 10px; }}
    img {{ max-width: 100%; height: auto; background: #000; border: 1px solid #234; }}
    code {{ color: #76ffd0; }}
  </style>
</head>
<body>
  <header>
    Debug HUD stream: <code>:{port}{stream_path}</code>
    &nbsp; Snapshot: <code>{snapshot_path}</code>
  </header>
  <main><img src="{stream_path}" alt="debug stream"></main>
</body>
</html>""".encode("utf-8")


def _waiting_frame(width=960, height=360):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = (10, 14, 18)
    cv2.putText(
        frame,
        "Waiting for ar_receiver debug frame...",
        (36, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (80, 240, 170),
        2,
        cv2.LINE_AA,
    )
    return frame


def _parse_cpu_set(value):
    text = str(value or "").strip()
    if not text:
        return None
    cpus = set()
    for part in text.replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            cpus.update(range(int(start), int(end) + 1))
        else:
            cpus.add(int(part))
    return cpus or None


def apply_current_thread_affinity(cpu_set, log_func=None, label="thread"):
    cpus = _parse_cpu_set(cpu_set)
    if not cpus or not hasattr(os, "sched_setaffinity"):
        return False
    try:
        os.sched_setaffinity(0, cpus)
        if log_func is not None:
            log_func(f"{label} CPU affinity: {sorted(cpus)}")
        return True
    except Exception as exc:
        if log_func is not None:
            log_func(f"{label} CPU affinity skipped: {exc}")
        return False


class DebugStreamServer:
    """Publishes latest rendered HUD frame as a non-blocking MJPEG stream."""

    def __init__(self, params=None, log_func=None):
        self.params = dict(DEBUG_STREAM_DEFAULTS)
        if params:
            self.params.update(params)
        self.log_func = log_func or print
        self.enabled = bool(self.params["ENABLE"])
        self.host = str(self.params["HOST"])
        self.port = int(self.params["PORT"])
        self.path = str(self.params["PATH"])
        self.snapshot_path = str(self.params["SNAPSHOT_PATH"])
        self.fps = max(0.0, float(self.params["FPS"]))
        self.jpeg_quality = int(max(1, min(100, int(self.params["JPEG_QUALITY"]))))
        self.max_width = int(self.params["MAX_WIDTH"])
        self.stale_seconds = max(0.1, float(self.params["STALE_SECONDS"]))
        self.cpu_set = str(self.params.get("CPUSET") or "").strip()

        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._frame = None
        self._frame_ts = 0.0
        self._frame_id = 0
        self._encoded_jpg = None
        self._encoded_frame_id = -1
        self._encoded_ts = 0.0
        self._encoded_count = 0
        self._encode_ms = 0.0
        self._dropped_before_encode = 0
        self._client_count = 0
        self._publish_fps = 0.0
        self._publish_window_ts = time.time()
        self._publish_window_count = 0
        self._encode_fps = 0.0
        self._encode_window_ts = time.time()
        self._encode_window_count = 0
        self._server = None
        self._thread = None
        self._encoder_thread = None

    def start(self):
        if not self.enabled:
            self._log("debug stream disabled")
            return self
        try:
            self._server = ThreadingHTTPServer((self.host, self.port), self._make_handler())
            self.port = int(self._server.server_address[1])
        except Exception as exc:
            self.enabled = False
            self._log(f"debug stream server failed: {exc}")
            return self

        self._encoder_thread = threading.Thread(
            target=self._encode_loop,
            name="debug-stream-encoder",
            daemon=True,
        )
        self._encoder_thread.start()
        self._thread = threading.Thread(
            target=self._serve_forever,
            name="debug-stream-http",
            daemon=True,
        )
        self._thread.start()
        self._log(f"Debug HUD stream: http://{self.host}:{self.port}{self.path}")
        return self

    def stop(self):
        self.enabled = False
        with self._condition:
            self._condition.notify_all()
        if self._server is None:
            return
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass
        self._server = None

    def publish(self, frame):
        if not self.enabled or frame is None:
            return
        try:
            if frame.size == 0:
                return
        except Exception:
            return
        now = time.time()
        with self._condition:
            self._frame = frame
            self._frame_ts = now
            self._frame_id += 1
            self._mark_publish_locked(now)
            self._condition.notify_all()

    def snapshot(self):
        if not self.enabled:
            return {
                "enabled": False,
                "url": None,
                "frame_id": self._frame_id,
                "age": None,
            }
        now = time.time()
        age = None if self._frame_ts <= 0.0 else max(0.0, now - self._frame_ts)
        return {
            "enabled": True,
            "host": self.host,
            "port": self.port,
            "path": self.path,
            "url": f"http://<rk-ip>:{self.port}{self.path}",
            "snapshot_path": self.snapshot_path,
            "frame_id": self._frame_id,
            "age": age,
            "stale": age is None or age > self.stale_seconds,
            "fps_limit": None if self.fps <= 0.0 else self.fps,
            "jpeg_quality": self.jpeg_quality,
            "max_width": self.max_width,
            "publish_fps": self._publish_fps,
            "encode_fps": self._encode_fps,
            "encode_ms": self._encode_ms,
            "encoded_frame_id": self._encoded_frame_id,
            "encoded_age": None if self._encoded_ts <= 0.0 else max(0.0, now - self._encoded_ts),
            "encoded_count": self._encoded_count,
            "drop_before_encode": self._dropped_before_encode,
            "client_count": self._client_count,
            "cpuset": self.cpu_set or None,
        }

    def _latest_frame(self):
        with self._condition:
            frame = self._frame
            ts = self._frame_ts
            frame_id = self._frame_id
        if frame is None:
            return _waiting_frame(), ts, frame_id
        return frame, ts, frame_id

    def _latest_jpeg(self):
        with self._condition:
            jpg = self._encoded_jpg
        if jpg is not None:
            return jpg
        frame, _ts, _frame_id = self._latest_frame()
        return self._encode_jpeg(frame)

    def _wait_encoded_after(self, last_frame_id, timeout=1.0):
        with self._condition:
            self._condition.wait_for(
                lambda: (
                    not self.enabled
                    or (
                        self._encoded_jpg is not None
                        and self._encoded_frame_id != last_frame_id
                    )
                ),
                timeout=timeout,
            )
            if not self.enabled:
                return None, last_frame_id
            if self._encoded_jpg is None or self._encoded_frame_id == last_frame_id:
                return None, last_frame_id
            return self._encoded_jpg, self._encoded_frame_id

    def _encode_jpeg(self, frame):
        if self.max_width > 0 and frame.shape[1] > self.max_width:
            scale = self.max_width / float(frame.shape[1])
            height = max(1, int(round(frame.shape[0] * scale)))
            frame = cv2.resize(frame, (self.max_width, height), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return None
        return encoded.tobytes()

    def _serve_forever(self):
        apply_current_thread_affinity(self.cpu_set, self._log, "debug-stream-http")
        self._server.serve_forever()

    def _encode_loop(self):
        apply_current_thread_affinity(self.cpu_set, self._log, "debug-stream-encoder")
        last_encoded_source_id = -1
        while self.enabled:
            with self._condition:
                self._condition.wait_for(
                    lambda: not self.enabled or self._frame_id != last_encoded_source_id,
                    timeout=0.5,
                )
                if not self.enabled:
                    break
                frame = self._frame
                frame_id = self._frame_id

            if frame is None:
                frame = _waiting_frame()
            if last_encoded_source_id >= 0 and frame_id > last_encoded_source_id + 1:
                with self._condition:
                    self._dropped_before_encode += frame_id - last_encoded_source_id - 1

            encode_start = time.perf_counter()
            jpg = self._encode_jpeg(frame)
            encode_ms = (time.perf_counter() - encode_start) * 1000.0
            if jpg is None:
                time.sleep(0.001)
                last_encoded_source_id = frame_id
                continue

            now = time.time()
            with self._condition:
                self._encoded_jpg = jpg
                self._encoded_frame_id = frame_id
                self._encoded_ts = now
                self._encoded_count += 1
                self._encode_ms = encode_ms
                self._mark_encode_locked(now)
                self._condition.notify_all()
            last_encoded_source_id = frame_id

    def _mark_publish_locked(self, now):
        self._publish_window_count += 1
        dt = now - self._publish_window_ts
        if dt >= 1.0:
            self._publish_fps = self._publish_window_count / max(1e-6, dt)
            self._publish_window_count = 0
            self._publish_window_ts = now

    def _mark_encode_locked(self, now):
        self._encode_window_count += 1
        dt = now - self._encode_window_ts
        if dt >= 1.0:
            self._encode_fps = self._encode_window_count / max(1e-6, dt)
            self._encode_window_count = 0
            self._encode_window_ts = now

    def _make_handler(self):
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    body = _html_page(owner.port, owner.path, owner.snapshot_path)
                    self._send_body(200, body, "text/html; charset=utf-8")
                    return
                if path == owner.snapshot_path:
                    self._send_snapshot()
                    return
                if path == owner.path:
                    self._send_mjpeg()
                    return
                if path == "/health":
                    body = (str(owner.snapshot()) + "\n").encode("utf-8")
                    self._send_body(200, body, "text/plain; charset=utf-8")
                    return
                self._send_body(404, b"not found\n", "text/plain; charset=utf-8")

            def _send_snapshot(self):
                jpg = owner._latest_jpeg()
                if jpg is None:
                    self._send_body(503, b"encode failed\n", "text/plain; charset=utf-8")
                    return
                self._send_body(200, jpg, "image/jpeg")

            def _send_mjpeg(self):
                self.send_response(200)
                self._send_common_headers()
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                last_sent_id = -1
                with owner._condition:
                    owner._client_count += 1
                # No sleep or FPS cap here: browser FPS tracks the latest
                # encoded debug frame, and slow clients naturally drop old frames.
                try:
                    while owner.enabled:
                        jpg, frame_id = owner._wait_encoded_after(last_sent_id)
                        if jpg is None:
                            continue
                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                        except Exception:
                            break
                        last_sent_id = frame_id
                finally:
                    with owner._condition:
                        owner._client_count = max(0, owner._client_count - 1)

            def _send_body(self, status, body, content_type):
                self.send_response(status)
                self._send_common_headers()
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_common_headers(self):
                self.send_header("Access-Control-Allow-Origin", "*")

            def log_message(self, _format, *args):
                return

        return Handler

    def _log(self, message):
        try:
            self.log_func(message)
        except Exception:
            pass
