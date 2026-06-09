import threading
import time
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

    # Stream frame rate cap. Lower values reduce CPU/network cost.
    "FPS": 10.0,

    # JPEG quality. Higher is clearer but costs more bandwidth and CPU.
    "JPEG_QUALITY": 70,

    # Resize the published frame if it is wider than this value.
    # Use 0 or a negative value to disable resizing.
    "MAX_WIDTH": 1280,

    # If no frame arrives for this many seconds, /health reports stale.
    "STALE_SECONDS": 2.0,
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
        self.fps = max(0.5, float(self.params["FPS"]))
        self.jpeg_quality = int(max(1, min(100, int(self.params["JPEG_QUALITY"]))))
        self.max_width = int(self.params["MAX_WIDTH"])
        self.stale_seconds = max(0.1, float(self.params["STALE_SECONDS"]))

        self._lock = threading.Lock()
        self._frame = None
        self._frame_ts = 0.0
        self._frame_id = 0
        self._server = None
        self._thread = None

    def start(self):
        if not self.enabled:
            self._log("debug stream disabled")
            return self
        try:
            self._server = ThreadingHTTPServer((self.host, self.port), self._make_handler())
        except Exception as exc:
            self.enabled = False
            self._log(f"debug stream server failed: {exc}")
            return self

        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="debug-stream-http",
            daemon=True,
        )
        self._thread.start()
        self._log(f"Debug HUD stream: http://{self.host}:{self.port}{self.path}")
        return self

    def stop(self):
        self.enabled = False
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
        with self._lock:
            self._frame = frame
            self._frame_ts = time.time()
            self._frame_id += 1

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
            "fps": self.fps,
            "jpeg_quality": self.jpeg_quality,
            "max_width": self.max_width,
        }

    def _latest_frame(self):
        with self._lock:
            frame = self._frame
            ts = self._frame_ts
            frame_id = self._frame_id
        if frame is None:
            return _waiting_frame(), ts, frame_id
        return frame, ts, frame_id

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
                frame, _ts, _frame_id = owner._latest_frame()
                jpg = owner._encode_jpeg(frame)
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

                frame_interval = 1.0 / owner.fps
                last_sent_id = -1
                while owner.enabled:
                    start_ts = time.time()
                    frame, _ts, frame_id = owner._latest_frame()
                    if frame_id == last_sent_id and frame_id > 0:
                        time.sleep(min(0.02, frame_interval))
                        continue
                    jpg = owner._encode_jpeg(frame)
                    if jpg is None:
                        time.sleep(frame_interval)
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
                    elapsed = time.time() - start_ts
                    if elapsed < frame_interval:
                        time.sleep(frame_interval - elapsed)

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
