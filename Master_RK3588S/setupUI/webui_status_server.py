import json
import mimetypes
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

from pose_ar_bridge import send_local_udp_json
from status_runtime import read_json_file


def write_json_atomic(path, payload):
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp_path, path)


def get_webui_ip():
    override = os.environ.get("AR_WEBUI_IP", "").strip()
    if override:
        return override
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("8.8.8.8", 80))
            ip = sock.getsockname()[0]
        finally:
            sock.close()
        if ip and not ip.startswith("127."):
            return ip
    except Exception:
        pass
    try:
        return socket.gethostbyname(socket.gethostname())
    except Exception:
        return "127.0.0.1"


class WebUIStatusServer:
    """Small HTTP server for pose/control status and lightweight WebUI configuration APIs."""

    def __init__(
        self,
        host,
        port,
        config_path,
        template_path,
        static_dir,
        objects_path,
        status_payload_func,
        pose_input_port,
        gamepad_control_port,
        vision_controls=None,
        log_func=None,
    ):
        self.host = host
        self.port = int(port)
        self.config_path = config_path
        self.template_path = template_path
        self.static_dir = static_dir
        self.objects_path = objects_path
        self.status_payload_func = status_payload_func
        self.pose_input_port = int(pose_input_port)
        self.gamepad_control_port = int(gamepad_control_port)
        self.vision_controls = vision_controls
        self.log_func = log_func or print
        self.server = None
        self.thread = None

    def start(self):
        try:
            self.server = ThreadingHTTPServer((self.host, self.port), self._make_handler())
        except Exception as exc:
            self.log_func(f"pose status HTTP server failed: {exc}")
            return None

        self.thread = threading.Thread(target=self.server.serve_forever, name="pose-status-http", daemon=True)
        self.thread.start()
        self.log_func(
            f"WebUI/status HTTP server: http://{self.host}:{self.port}/ "
            f"(status: /pose_status)"
        )
        return self

    def stop(self):
        if self.server is None:
            return
        try:
            self.server.shutdown()
            self.server.server_close()
        except Exception:
            pass

    def render_index_html(self):
        try:
            with open(self.template_path, "r", encoding="utf-8") as f:
                body = f.read()
        except Exception as exc:
            return f"WebUI template missing: {exc}".encode("utf-8"), "text/plain; charset=utf-8"

        body = body.replace("{{ ip }}", get_webui_ip())
        body = body.replace("{{ http_port }}", str(self.port))
        return body.encode("utf-8"), "text/html; charset=utf-8"

    def resolve_static_path(self, request_path):
        rel_path = unquote(request_path.split("?", 1)[0].removeprefix("/static/"))
        rel_path = os.path.normpath(rel_path).lstrip(os.sep)
        candidate = os.path.abspath(os.path.join(self.static_dir, rel_path))
        static_root = os.path.abspath(self.static_dir)
        if candidate == static_root or not candidate.startswith(static_root + os.sep):
            return None
        return candidate

    def _make_handler(self):
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_OPTIONS(self):
                self.send_response(204)
                self._send_common_headers()
                self.end_headers()

            def do_HEAD(self):
                path = self.path.split("?", 1)[0]

                if path in ("/", "/index.html"):
                    body, content_type = owner.render_index_html()
                    self._send_headers_only(200, len(body), content_type)
                    return

                if path.startswith("/static/"):
                    local_path = owner.resolve_static_path(path)
                    if not local_path or not os.path.isfile(local_path):
                        self._send_headers_only(404, 0, "application/json; charset=utf-8")
                        return
                    content_type = self._static_content_type(local_path)
                    self._send_headers_only(200, os.path.getsize(local_path), content_type)
                    return

                if path in ("/pose_status", "/pose_packet", "/health", "/main_config.json", "/api/config", "/objects.json", "/api/vision_controls"):
                    self._send_headers_only(200, 0, "application/json; charset=utf-8")
                    return

                self._send_headers_only(404, 0, "application/json; charset=utf-8")

            def do_POST(self):
                path = self.path.split("?", 1)[0]
                try:
                    payload = self._read_json_body()
                except Exception as exc:
                    self._send_json({"error": f"invalid JSON body: {exc}"}, status=400)
                    return

                if path == "/api/config":
                    if not isinstance(payload, dict):
                        self._send_json({"error": "config payload must be an object"}, status=400)
                        return
                    try:
                        write_json_atomic(owner.config_path, payload)
                    except Exception as exc:
                        self._send_json({"error": str(exc)}, status=500)
                        return
                    self._send_json({"ok": True, "path": owner.config_path})
                    return

                if path == "/api/manual_pose":
                    try:
                        packet = {
                            "type": "robot_position",
                            "pos": [
                                float(payload.get("x", 0.0)),
                                float(payload.get("y", 0.16)),
                                float(payload.get("z", 0.30)),
                            ],
                            "euler": [0.0, float(payload.get("yaw", 0.0)), 0.0],
                            "timestamp": time.time(),
                            "source": "webui_wasd",
                        }
                        byte_count = send_local_udp_json(owner.pose_input_port, packet)
                    except Exception as exc:
                        self._send_json({"error": str(exc)}, status=500)
                        return
                    self._send_json({"ok": True, "bytes": byte_count, "target": f"127.0.0.1:{owner.pose_input_port}"})
                    return

                if path == "/api/gamepad_control":
                    try:
                        packet = dict(payload)
                        packet.setdefault("type", "gamepad_control")
                        packet.setdefault("source", "webui")
                        byte_count = send_local_udp_json(owner.gamepad_control_port, packet)
                    except Exception as exc:
                        self._send_json({"error": str(exc)}, status=500)
                        return
                    self._send_json({"ok": True, "bytes": byte_count, "target": f"127.0.0.1:{owner.gamepad_control_port}"})
                    return

                if path == "/api/vision_controls":
                    if owner.vision_controls is None:
                        self._send_json({"error": "vision controls unavailable"}, status=503)
                        return
                    try:
                        controls, changed = owner.vision_controls.update(payload)
                    except Exception as exc:
                        self._send_json({"error": str(exc)}, status=400)
                        return
                    self._send_json({"ok": True, "controls": controls, "changed": changed})
                    return

                self._send_json({"error": "not found", "path": path}, status=404)

            def do_GET(self):
                path = self.path.split("?", 1)[0]

                if path in ("/", "/index.html"):
                    body, content_type = owner.render_index_html()
                    self._send_body(200, body, content_type)
                    return

                if path.startswith("/static/"):
                    self._send_static(path)
                    return

                if path in ("/pose_status", "/pose_packet", "/health"):
                    payload = owner.status_payload_func()
                    if path == "/pose_packet":
                        payload = payload.get("packet") or {}
                    self._send_json(payload)
                    return

                if path in ("/main_config.json", "/api/config"):
                    self._send_json(read_json_file(owner.config_path, {}))
                    return

                if path == "/objects.json":
                    self._send_json(read_json_file(owner.objects_path, []))
                    return

                if path == "/api/vision_controls":
                    if owner.vision_controls is None:
                        self._send_json({"error": "vision controls unavailable"}, status=503)
                        return
                    self._send_json({"ok": True, "controls": owner.vision_controls.snapshot()})
                    return

                self._send_json({"error": "not found", "path": path}, status=404)

            def _send_json(self, payload, status=200):
                body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                self._send_body(status, body, "application/json; charset=utf-8")

            def _read_json_body(self):
                length = int(self.headers.get("Content-Length", "0") or "0")
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                return json.loads(raw.decode("utf-8"))

            def _send_static(self, path):
                local_path = owner.resolve_static_path(path)
                if not local_path or not os.path.isfile(local_path):
                    self._send_json({"error": "static file not found", "path": path}, status=404)
                    return

                try:
                    with open(local_path, "rb") as f:
                        body = f.read()
                except Exception as exc:
                    self._send_json({"error": str(exc), "path": path}, status=500)
                    return

                self._send_body(200, body, self._static_content_type(local_path))

            def _static_content_type(self, local_path):
                content_type = mimetypes.guess_type(local_path)[0] or "application/octet-stream"
                if content_type.startswith("text/") or content_type in ("application/javascript", "application/json"):
                    content_type += "; charset=utf-8"
                return content_type

            def _send_body(self, status, body, content_type):
                self.send_response(status)
                self._send_common_headers()
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _send_headers_only(self, status, content_length, content_type):
                self.send_response(status)
                self._send_common_headers()
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(content_length))
                self.end_headers()

            def log_message(self, _format, *args):
                return

            def _send_common_headers(self):
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "*")

        return Handler
