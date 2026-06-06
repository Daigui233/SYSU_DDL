import time
import struct
import json
import atexit
import os
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
import cv2
from multiprocessing import shared_memory, resource_tracker


from infer_wrap import InferWrap, PPSegInfer
from infer_wrap.base.func import draw
from infer_wrap.base.seg_func import extract_centerline

# serial_comm handles the RK3588S -> TC264D control link.
try:
    import serial_comm as _serial_comm
except Exception as exc:
    _serial_comm = None
    print(f"serial_comm unavailable: {exc}; car serial control disabled")

SHM_NAME = "shm_ar_video"
SHM_HEADER_SIZE = 16
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "dist", "main_config.json")
MODEL_DIR = os.environ.get("AR_MODEL_DIR", os.path.join(BASE_DIR, "infer_wrap", "base", "model"))
SEG_RESULT_TTL = 2.0
DET_RESULT_TTL = 1.0
RUNTIME_STATUS_INTERVAL = 0.5
LINE_LOSS_SAFE_STOP_TIMEOUT = float(os.environ.get("AR_LINE_LOSS_SAFE_STOP_TIMEOUT", "3.0"))
LINE_LOSS_COMMAND_REPEAT_INTERVAL = float(os.environ.get("AR_LINE_LOSS_COMMAND_REPEAT_INTERVAL", "0.2"))
CONTROL_SCALE = 0.5
TRACK_SPEED = float(os.environ.get("AR_TRACK_SPEED", "0.5"))
TRACK_FALLBACK_SPEED = float(os.environ.get("AR_TRACK_FALLBACK_SPEED", str(TRACK_SPEED)))
CONTROL_FLAG_USE_TARGET_SPEED = int(os.environ.get("AR_CONTROL_SPEED_FLAG", "1"), 0)
GAMEPAD_CONTROL_HOST = os.environ.get("AR_GAMEPAD_CONTROL_HOST", "0.0.0.0")
GAMEPAD_CONTROL_PORT = int(os.environ.get("AR_GAMEPAD_CONTROL_PORT", "9010"))
GAMEPAD_CONTROL_TTL = float(os.environ.get("AR_GAMEPAD_CONTROL_TTL", "0.45"))
GAMEPAD_MAX_SPEED_MPS = float(os.environ.get("AR_GAMEPAD_MAX_SPEED_MPS", "0.8"))
GAMEPAD_MAX_TRACK_ERROR = float(os.environ.get("AR_GAMEPAD_MAX_TRACK_ERROR", "240.0"))

DEFAULT_AR_UDP_IP = "127.0.0.1"
DEFAULT_AR_UDP_PORT = 9006
POSE_INPUT_HOST = os.environ.get("AR_POSE_INPUT_HOST", "0.0.0.0")
POSE_INPUT_PORT = int(os.environ.get("AR_POSE_INPUT_PORT", "9005"))

# ===== 定位数据终端调试开关 =====
# Windows AprilTag localization sends official robot_position JSON over UDP.
# 稳定后如果嫌终端刷屏，把这里改成 False。
DEBUG_PRINT_POSE = True
DEBUG_PRINT_POSE_EVERY_N = 1
DEBUG_LOG_POSE_TO_FILE = True
DEBUG_LOG_PATH = os.environ.get("AR_POSE_LOG_PATH", os.path.join(BASE_DIR, "ar_pose_debug.log"))
DEBUG_STATUS_PATH = os.environ.get("AR_POSE_STATUS_PATH", os.path.join(BASE_DIR, "ar_pose_status.json"))
DEBUG_PACKET_PATH = os.environ.get("AR_POSE_PACKET_PATH", os.path.join(BASE_DIR, "xverse_control_live.json"))
DEBUG_LOG_MAX_BYTES = 2 * 1024 * 1024
DEBUG_DRAW_POSE_PANEL = True
POSE_STATUS_HTTP_HOST = os.environ.get("AR_POSE_STATUS_HOST", "0.0.0.0")
POSE_STATUS_HTTP_PORT = int(os.environ.get("AR_POSE_STATUS_PORT", "9105"))
POSE_CONVENTION = {
    "field_m": [4.0, 3.0],
    "origin": "bottom-right",
    "positive_x": "left",
    "positive_z": "up",
    "yaw_index": 1,
    "yaw_zero": "+X",
    "yaw_positive_90": "+Z",
    "source": "Windows AprilTag locator filtered robot_position",
}
AR_FORWARD_MAPPING = {
    "position": "AR pos[0] = input pos[2], AR pos[2] = input pos[0]",
    "yaw": "unchanged euler[1]",
    "reason": "Official AR scene axes are X/Z-swapped relative to the Windows locator preview.",
}

# ===== POSE_PATH_DEBUG_START：定位链路分段验证打印，稳定后可整段删除 =====
# The staged debug output identifies UDP input, JSON validation, local mirror, and AR forwarding.
# 删除方法：搜索 POSE_PATH_DEBUG_START 到 POSE_PATH_DEBUG_END，以及代码中的 POSE_PATH_DEBUG 调用点。
POSE_PATH_DEBUG = True
POSE_PATH_DEBUG_EVERY_N = 1
# ===== POSE_PATH_DEBUG_END =====

STATE_TRACK = getattr(_serial_comm, "STATE_TRACK", 1) if _serial_comm else 1
STATE_SAFE_STOP = getattr(_serial_comm, "STATE_SAFE_STOP", 7) if _serial_comm else 7
CarController = getattr(_serial_comm, "CarController", None) if _serial_comm else None
STATUS_WRITE_LOCK = threading.Lock()
LATEST_CONTROL_STATUS = None
CONTROL_COMMAND_LOCK = threading.Lock()
LAST_CONTROL_SEND_OK = None
LAST_CONTROL_SEND_ERROR = ""
LAST_CONTROL_SEND_TS = 0.0


class DisabledCarController:
    def send_cmd(self, *args, **kwargs):
        return False


def create_car_controller():
    if CarController is None:
        print("CarController unavailable; car serial control disabled")
        return DisabledCarController()
    try:
        return CarController(port="/dev/ttyUSB0", baudrate=460800)
    except TypeError:
        try:
            return CarController("/dev/ttyUSB0", 460800)
        except Exception as exc:
            print(f"CarController init failed: {exc}; car serial control disabled")
            return DisabledCarController()
    except Exception as exc:
        print(f"CarController init failed: {exc}; car serial control disabled")
        return DisabledCarController()


def _send_car_cmd_unlocked(track_error, target_speed, state_cmd, flags):
    send_cmd = getattr(car, "send_cmd", None)
    if send_cmd is None:
        return False, "send_cmd unavailable"
    try:
        result = send_cmd(
            track_error=track_error,
            target_speed=target_speed,
            state_cmd=state_cmd,
            flags=flags,
        )
    except TypeError:
        try:
            result = send_cmd(track_error, target_speed, state_cmd, flags)
        except TypeError:
            try:
                result = send_cmd(track_error)
            except Exception as exc:
                print(f"serial send skip: {exc}")
                return False, str(exc)
        except Exception as exc:
            print(f"serial send skip: {exc}")
            return False, str(exc)
    except Exception as exc:
        print(f"serial send skip: {exc}")
        return False, str(exc)
    ok = True if result is None else bool(result)
    return ok, "" if ok else "serial send returned false"


def send_car_cmd(track_error, target_speed, state_cmd, flags=CONTROL_FLAG_USE_TARGET_SPEED, mark_activity=True):
    global LAST_CONTROL_SEND_OK, LAST_CONTROL_SEND_ERROR, LAST_CONTROL_SEND_TS

    with CONTROL_COMMAND_LOCK:
        ok, error = _send_car_cmd_unlocked(track_error, target_speed, state_cmd, flags)
        LAST_CONTROL_SEND_OK = ok
        LAST_CONTROL_SEND_ERROR = error
        LAST_CONTROL_SEND_TS = time.time()
        return ok


def get_control_send_status():
    with CONTROL_COMMAND_LOCK:
        return {
            "ok": LAST_CONTROL_SEND_OK,
            "error": LAST_CONTROL_SEND_ERROR or None,
            "timestamp": LAST_CONTROL_SEND_TS or None,
        }


def get_car_feedback():
    getter = getattr(car, "get_feedback", None)
    if getter is None:
        return {"online": False, "error": "feedback unavailable"}
    try:
        return getter()
    except Exception as exc:
        return {"online": False, "error": str(exc)}


def finite_float(value, default=0.0):
    try:
        result = float(value)
    except Exception:
        return default
    return result if np.isfinite(result) else default


def clamp_float(value, low, high):
    value = finite_float(value, 0.0)
    return max(float(low), min(float(high), value))

print("initializing NPU AI...")
infer_det = InferWrap(model_dir=MODEL_DIR, TPEs=1, core_ids=[0], max_inflight=1)
infer_seg = PPSegInfer(model_dir=MODEL_DIR, TPEs=1, core_ids=[1], max_inflight=1)
print("AI engines loaded")

car = create_car_controller()


def stop_car_on_exit():
    try:
        send_car_cmd(0.0, 0.0, STATE_SAFE_STOP, flags=0, mark_activity=False)
    except Exception:
        pass


atexit.register(stop_car_on_exit)


def remove_shm_from_resource_tracker():
    try:
        resource_tracker.unregister('/' + SHM_NAME, 'shared_memory')
    except Exception:
        pass


def rotate_debug_log_if_needed():
    if not DEBUG_LOG_POSE_TO_FILE:
        return
    try:
        if os.path.exists(DEBUG_LOG_PATH) and os.path.getsize(DEBUG_LOG_PATH) > DEBUG_LOG_MAX_BYTES:
            old_path = DEBUG_LOG_PATH + ".old"
            if os.path.exists(old_path):
                os.remove(old_path)
            os.replace(DEBUG_LOG_PATH, old_path)
    except Exception:
        pass


def write_debug_log(message, echo=True):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}"
    if echo:
        print(line)
    if not DEBUG_LOG_POSE_TO_FILE:
        return
    try:
        rotate_debug_log_if_needed()
        with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception as exc:
        print(f"pose debug log write failed: {exc}")


# ===== POSE_PATH_DEBUG_START：定位链路分段验证打印，稳定后可整段删除 =====
def pose_path_debug(stage, message, count=None, force=False):
    if not POSE_PATH_DEBUG:
        return
    if count is not None and not force and count % max(1, POSE_PATH_DEBUG_EVERY_N) != 0:
        return
    write_debug_log(f"[POSE_PATH_DEBUG] {stage}: {message}")
# ===== POSE_PATH_DEBUG_END =====


def write_pose_status(status, packet=None, raw_line="", packet_count=0, invalid_count=0, target=None, input_packet=None, ar_packet=None):
    info = {
        "status": status,
        "timestamp": time.time(),
        "packet_count": packet_count,
        "invalid_count": invalid_count,
        "raw_line": raw_line,
        "input_packet": input_packet,
        "packet": packet,
        "ar_packet": ar_packet,
        "target": target,
        "debug_log": DEBUG_LOG_PATH,
        "packet_json": DEBUG_PACKET_PATH,
        "pose_convention": POSE_CONVENTION,
        "ar_forward_mapping": AR_FORWARD_MAPPING,
        "car_feedback": get_car_feedback(),
    }
    write_status_json(info, "pose status")


def write_status_json(info, label, control_info=None):
    global LATEST_CONTROL_STATUS

    try:
        with STATUS_WRITE_LOCK:
            if control_info is not None:
                LATEST_CONTROL_STATUS = dict(control_info)
            if LATEST_CONTROL_STATUS is not None:
                info["control"] = dict(LATEST_CONTROL_STATUS)

            tmp_path = DEBUG_STATUS_PATH + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(info, f, ensure_ascii=False, indent=2)
            os.replace(tmp_path, DEBUG_STATUS_PATH)
    except Exception as exc:
        print(f"{label} write failed: {exc}")


def write_runtime_status(pose_bridge, control_state, command_error, command_speed, command_state, command_flags, gamepad_status=None):
    pose_info = pose_bridge.snapshot()
    control_info = {
        "state": control_state,
        "track_error": float(command_error) if command_error is not None and np.isfinite(command_error) else None,
        "target_speed": float(command_speed),
        "state_cmd": int(command_state),
        "flags": int(command_flags),
        "serial_send": get_control_send_status(),
        "timestamp": time.time(),
    }
    if gamepad_status is not None:
        control_info["gamepad"] = gamepad_status
    info = {
        "status": pose_info.get("status", "ar_receiver running"),
        "timestamp": time.time(),
        "packet_count": pose_info.get("packet_count", 0),
        "invalid_count": pose_info.get("invalid_count", 0),
        "input_packet": pose_info.get("last_input_packet"),
        "packet": pose_info.get("last_packet"),
        "ar_packet": pose_info.get("last_ar_packet"),
        "target": pose_info.get("target", f"{DEFAULT_AR_UDP_IP}:{DEFAULT_AR_UDP_PORT}"),
        "udp_send_count": pose_info.get("udp_send_count", 0),
        "udp_fail_count": pose_info.get("udp_fail_count", 0),
        "pose_convention": POSE_CONVENTION,
        "ar_forward_mapping": AR_FORWARD_MAPPING,
        "control": control_info,
        "car_feedback": get_car_feedback(),
        "debug_log": DEBUG_LOG_PATH,
        "packet_json": DEBUG_PACKET_PATH,
    }
    write_status_json(info, "runtime status", control_info=control_info)

def write_live_pose_packet(packet):
    if not packet:
        return False
    try:
        tmp_path = DEBUG_PACKET_PATH + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(packet, f, ensure_ascii=False, indent=2)
        os.replace(tmp_path, DEBUG_PACKET_PATH)
        return True
    except Exception as exc:
        print(f"live pose packet write failed: {exc}")
        return False


def read_json_file(path, default=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def current_pose_http_payload():
    status = read_json_file(DEBUG_STATUS_PATH, {})
    ar_packet = read_json_file(DEBUG_PACKET_PATH, None)
    if ar_packet is not None and not status.get("ar_packet"):
        status["ar_packet"] = ar_packet
    status.setdefault("status", "ar_receiver not ready")
    status.setdefault("packet_count", 0)
    status.setdefault("invalid_count", 0)
    status.setdefault("debug_log", DEBUG_LOG_PATH)
    status.setdefault("packet_json", DEBUG_PACKET_PATH)
    status.setdefault("target", f"{DEFAULT_AR_UDP_IP}:{DEFAULT_AR_UDP_PORT}")
    status.setdefault("pose_convention", POSE_CONVENTION)
    status.setdefault("ar_forward_mapping", AR_FORWARD_MAPPING)
    status["http_port"] = POSE_STATUS_HTTP_PORT
    status["car_feedback"] = get_car_feedback()
    return status


class PoseStatusHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(204)
        self._send_common_headers()
        self.end_headers()

    def do_GET(self):
        if self.path.split("?", 1)[0] not in ("/", "/pose_status", "/pose_packet"):
            self.send_response(404)
            self._send_common_headers()
            self.end_headers()
            return

        payload = current_pose_http_payload()
        if self.path.split("?", 1)[0] == "/pose_packet":
            payload = payload.get("packet") or {}

        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(200)
        self._send_common_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format, *args):
        return

    def _send_common_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")


def start_pose_status_http_server():
    try:
        server = ThreadingHTTPServer((POSE_STATUS_HTTP_HOST, POSE_STATUS_HTTP_PORT), PoseStatusHandler)
    except Exception as exc:
        write_debug_log(f"pose status HTTP server failed: {exc}")
        return None

    thread = threading.Thread(target=server.serve_forever, name="pose-status-http", daemon=True)
    thread.start()
    write_debug_log(f"pose status HTTP server: http://{POSE_STATUS_HTTP_HOST}:{POSE_STATUS_HTTP_PORT}/pose_status")
    return server



def read_frame_from_shm(shm):
    header = bytes(shm.buf[:SHM_HEADER_SIZE])
    fid, w, h = struct.unpack('QII', header)
    size = w * h * 3
    img_view = np.ndarray((h, w, 3), dtype=np.uint8, buffer=shm.buf[SHM_HEADER_SIZE:SHM_HEADER_SIZE + size])
    frame = img_view.copy()
    del img_view
    frame = cv2.flip(frame, 0)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return fid, frame


def load_ar_network_config():
    # The official AR engine consumes robot_position on network.control_port.
    # WebUI udp_target_* should mirror the board-local AR endpoint
    # (127.0.0.1:9006), while Windows sends external pose data to board_ip:9005.
    ip = os.environ.get("AR_UDP_IP", DEFAULT_AR_UDP_IP)
    port = DEFAULT_AR_UDP_PORT
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        network = cfg.get("network", cfg)
        port = int(network.get("control_port", port))
    except Exception as exc:
        print(f"AR config fallback: {exc}")
    port = int(os.environ.get("AR_UDP_PORT", str(port)))
    return ip, port



def build_ar_udp_targets(ip, port):
    targets = []

    def add_target(target_ip, target_port):
        target = (str(target_ip).strip(), int(target_port))
        if target[0] and target not in targets:
            targets.append(target)

    add_target(ip, port)

    extra = os.environ.get("AR_UDP_EXTRA_TARGETS", "")
    for item in extra.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            host, port_text = item.rsplit(":", 1)
            add_target(host, int(port_text))
        else:
            add_target(item, port)

    return targets


def coerce_official_packet(data):
    if not isinstance(data, dict) or data.get("type") != "robot_position":
        return None

    pos = data.get("pos")
    euler = data.get("euler")
    if not isinstance(pos, (list, tuple)) or len(pos) < 3:
        return None
    if not isinstance(euler, (list, tuple)) or len(euler) < 3:
        return None

    try:
        pos_values = [float(pos[0]), float(pos[1]), float(pos[2])]
        euler_values = [float(euler[0]), float(euler[1]), float(euler[2])]
    except (TypeError, ValueError):
        return None

    if not all(np.isfinite(value) for value in pos_values + euler_values):
        return None

    packet = {
        "type": "robot_position",
        "pos": pos_values,
        "euler": euler_values,
    }
    if "seq" in data:
        packet["seq"] = data["seq"]
    if "timestamp" in data:
        packet["timestamp"] = data["timestamp"]
    return packet


def map_pose_packet_for_ar(packet):
    """Map Windows locator preview coordinates to the official AR scene axes."""
    pos = packet["pos"]
    euler = packet["euler"]
    ar_packet = {
        "type": "robot_position",
        "pos": [float(pos[2]), float(pos[1]), float(pos[0])],
        "euler": [float(euler[0]), float(euler[1]), float(euler[2])],
    }
    if "seq" in packet:
        ar_packet["seq"] = packet["seq"]
    if "timestamp" in packet:
        ar_packet["timestamp"] = packet["timestamp"]
    return ar_packet


def parse_official_pose_datagram(data):
    try:
        raw_text = data.decode("utf-8").strip()
    except UnicodeDecodeError as exc:
        return None, "", f"invalid UTF-8: {exc}"

    if not raw_text:
        return None, raw_text, "empty datagram"

    try:
        decoded = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        return None, raw_text, f"invalid JSON: {exc.msg}"

    packet = coerce_official_packet(decoded)
    if packet is None:
        return None, raw_text, "expected official robot_position with finite pos[3] and euler[3]"
    return packet, raw_text, ""


class GamepadControlReceiver:
    """Receive optional manual gamepad control packets on a separate UDP port."""

    def __init__(self, host=GAMEPAD_CONTROL_HOST, port=GAMEPAD_CONTROL_PORT, ttl=GAMEPAD_CONTROL_TTL):
        self.host = host
        self.port = int(port)
        self.ttl = float(ttl)
        self.sock = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.packet_count = 0
        self.invalid_count = 0
        self.last_ts = 0.0
        self.last_packet = None
        self.last_error = ""
        self.status = "gamepad waiting"

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(0.2)
            self.set_status(f"gamepad listening {self.host}:{self.port}")
            write_debug_log(f"Gamepad UDP ready on {self.host}:{self.port}")
            while not self.stop_event.is_set():
                try:
                    data, addr = self.sock.recvfrom(8192)
                except socket.timeout:
                    continue
                except OSError:
                    break
                self.handle_datagram(data, addr)
        except Exception as exc:
            self.set_status(f"gamepad error: {exc}")
            write_debug_log(f"Gamepad UDP receiver stopped: {exc}")

    def set_status(self, status):
        with self.lock:
            self.status = status
            self.last_error = status if "error" in str(status).lower() else ""

    def handle_datagram(self, data, addr):
        now = time.time()
        try:
            raw = data.decode("utf-8", errors="replace").strip()
            packet = json.loads(raw)
            if not isinstance(packet, dict):
                raise ValueError("packet is not an object")
            if packet.get("type") != "gamepad_control":
                raise ValueError("type is not gamepad_control")

            gamepad_mode = bool(packet.get("gamepad_mode", False))
            safe_stop = bool(packet.get("safe_stop", False))
            state_cmd = int(packet.get("state_cmd", STATE_SAFE_STOP if safe_stop else STATE_TRACK))
            if state_cmd not in (STATE_TRACK, STATE_SAFE_STOP):
                state_cmd = STATE_SAFE_STOP if safe_stop else STATE_TRACK

            if not gamepad_mode:
                command = {
                    "gamepad_mode": False,
                    "active": False,
                    "state_cmd": STATE_TRACK,
                    "target_speed": 0.0,
                    "track_error": 0.0,
                    "flags": 0,
                    "safe_stop": False,
                    "source": packet.get("source", "unknown"),
                    "seq": packet.get("seq"),
                    "inputs": packet.get("inputs", {}),
                }
                status = "gamepad disabled by sender"
            else:
                if safe_stop or state_cmd == STATE_SAFE_STOP:
                    state_cmd = STATE_SAFE_STOP
                    target_speed = 0.0
                    track_error = 0.0
                    flags = 0
                    status = "gamepad safe_stop"
                else:
                    state_cmd = STATE_TRACK
                    target_speed = clamp_float(packet.get("target_speed", 0.0), 0.0, GAMEPAD_MAX_SPEED_MPS)
                    track_error = clamp_float(packet.get("track_error", 0.0), -GAMEPAD_MAX_TRACK_ERROR, GAMEPAD_MAX_TRACK_ERROR)
                    flags = int(packet.get("flags", CONTROL_FLAG_USE_TARGET_SPEED)) | CONTROL_FLAG_USE_TARGET_SPEED
                    status = "gamepad manual track"

                command = {
                    "gamepad_mode": True,
                    "active": True,
                    "state_cmd": state_cmd,
                    "target_speed": target_speed,
                    "track_error": track_error,
                    "flags": flags,
                    "safe_stop": state_cmd == STATE_SAFE_STOP,
                    "source": packet.get("source", "unknown"),
                    "seq": packet.get("seq"),
                    "inputs": packet.get("inputs", {}),
                }

            with self.lock:
                self.packet_count += 1
                self.last_ts = now
                self.last_packet = command
                self.status = status
                self.last_error = ""
        except Exception as exc:
            with self.lock:
                self.invalid_count += 1
                self.last_error = str(exc)
                self.status = f"gamepad invalid: {exc}"
            write_debug_log(f"Bad gamepad UDP from {addr}: {exc}")

    def snapshot(self):
        now = time.time()
        with self.lock:
            packet = dict(self.last_packet) if isinstance(self.last_packet, dict) else None
            age = now - self.last_ts if self.last_ts else None
            active = bool(packet and packet.get("gamepad_mode") and age is not None and age <= self.ttl)
            snap = {
                "status": self.status,
                "active": active,
                "age": age,
                "ttl": self.ttl,
                "packet_count": self.packet_count,
                "invalid_count": self.invalid_count,
                "last_error": self.last_error,
                "last_packet": packet,
                "target": f"{self.host}:{self.port}",
            }
            if packet is not None:
                snap.update({
                    "state_cmd": int(packet.get("state_cmd", STATE_TRACK)),
                    "target_speed": float(packet.get("target_speed", 0.0)),
                    "track_error": float(packet.get("track_error", 0.0)),
                    "flags": int(packet.get("flags", 0)),
                    "safe_stop": bool(packet.get("safe_stop", False)),
                    "seq": packet.get("seq"),
                    "inputs": packet.get("inputs", {}),
                })
            return snap

    def active_command(self):
        snap = self.snapshot()
        if not snap.get("active"):
            return None, snap
        return {
            "track_error": snap.get("track_error", 0.0),
            "target_speed": snap.get("target_speed", 0.0),
            "state_cmd": snap.get("state_cmd", STATE_TRACK),
            "flags": snap.get("flags", CONTROL_FLAG_USE_TARGET_SPEED),
            "safe_stop": snap.get("safe_stop", False),
        }, snap


class ARPoseBridge:
    def __init__(self):
        self.target_ip, self.target_port = load_ar_network_config()
        self.targets = build_ar_udp_targets(self.target_ip, self.target_port)
        self.target_text = ",".join(f"{ip}:{port}" for ip, port in self.targets)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.input_sock = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.status = "idle"
        self.last_datagram = ""
        self.last_source = ""
        self.last_input_packet = None
        self.last_packet = None
        self.last_ar_packet = None
        self.last_ts = 0.0
        self.datagram_count = 0
        self.packet_count = 0
        self.invalid_count = 0
        self.udp_send_count = 0
        self.udp_fail_count = 0
        self.live_json_count = 0

    def start(self):
        write_debug_log("=" * 60)
        write_debug_log(
            f"AR pose bridge starting, Windows UDP input={POSE_INPUT_HOST}:{POSE_INPUT_PORT}, "
            f"AR targets: {self.target_text}"
        )
        write_debug_log(f"pose debug log: {DEBUG_LOG_PATH}")
        write_debug_log(f"pose status json: {DEBUG_STATUS_PATH}")
        write_debug_log(f"live official packet json: {DEBUG_PACKET_PATH}")
        write_pose_status(
            "starting",
            packet_count=self.packet_count,
            invalid_count=self.invalid_count,
            target=self.target_text,
        )
        self.set_status(
            "udp listening",
            f"Waiting for official robot_position JSON on {POSE_INPUT_HOST}:{POSE_INPUT_PORT}",
        )
        self.thread = threading.Thread(target=self._udp_loop, name="ar-pose-udp", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        try:
            if self.input_sock is not None:
                self.input_sock.close()
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "last_datagram": self.last_datagram,
                "last_source": self.last_source,
                "last_input_packet": self.last_input_packet,
                "last_packet": self.last_packet,
                "last_ar_packet": self.last_ar_packet,
                "last_ts": self.last_ts,
                "datagram_count": self.datagram_count,
                "packet_count": self.packet_count,
                "invalid_count": self.invalid_count,
                "target": self.target_text,
                "input": f"{POSE_INPUT_HOST}:{POSE_INPUT_PORT}",
                "protocol": "udp/robot_position",
                "udp_send_count": self.udp_send_count,
                "udp_fail_count": self.udp_fail_count,
                "live_json_count": self.live_json_count,
                "log_path": DEBUG_LOG_PATH,
                "status_path": DEBUG_STATUS_PATH,
                "packet_path": DEBUG_PACKET_PATH,
                "ar_forward_mapping": AR_FORWARD_MAPPING,
            }

    def set_status(self, status, log_message=None):
        with self.lock:
            self.status = status
            input_packet = self.last_input_packet
            packet = self.last_packet
            ar_packet = self.last_ar_packet
            raw_line = self.last_datagram
            packet_count = self.packet_count
            invalid_count = self.invalid_count
        if log_message:
            write_debug_log(log_message)
        write_pose_status(
            status,
            packet=packet,
            raw_line=raw_line,
            packet_count=packet_count,
            invalid_count=invalid_count,
            target=self.target_text,
            input_packet=input_packet,
            ar_packet=ar_packet,
        )

    def _udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.5)
        self.input_sock = sock
        try:
            sock.bind((POSE_INPUT_HOST, POSE_INPUT_PORT))
            self.set_status(
                "udp waiting pose",
                f"Windows localization UDP ready on {POSE_INPUT_HOST}:{POSE_INPUT_PORT}",
            )
            while not self.stop_event.is_set():
                try:
                    data, source = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    if self.stop_event.is_set():
                        break
                    raise

                source_text = f"{source[0]}:{source[1]}"
                packet, raw_text, error = parse_official_pose_datagram(data)
                with self.lock:
                    self.datagram_count += 1
                    self.last_datagram = raw_text
                    self.last_source = source_text
                    datagram_count = self.datagram_count
                pose_path_debug(
                    "UDP_RAW",
                    f"#{datagram_count} from={source_text} bytes={len(data)} text={repr(raw_text)}",
                    count=datagram_count,
                )
                self.handle_packet(packet, raw_text, source_text, error)
        except Exception as exc:
            if not self.stop_event.is_set():
                self.set_status(f"udp error: {exc}", f"UDP pose receiver stopped: {exc}")
        finally:
            try:
                sock.close()
            except Exception:
                pass
            if self.input_sock is sock:
                self.input_sock = None

    def handle_packet(self, packet, raw_text, source_text, error=""):
        now = time.time()
        if packet is None:
            with self.lock:
                self.invalid_count += 1
                invalid_count = self.invalid_count
                packet_count = self.packet_count
                self.status = "invalid pose datagram"
            write_pose_status(
                "invalid pose datagram",
                raw_line=raw_text,
                packet_count=packet_count,
                invalid_count=invalid_count,
                target=self.target_text,
            )
            pose_path_debug(
                "PARSE_FAIL",
                f"from={source_text} reason={error} raw={repr(raw_text)} invalid_count={invalid_count}",
                force=True,
            )
            write_debug_log(f"[AR_POSE_BAD] from={source_text} reason={error} raw={repr(raw_text)}")
            return

        ar_packet = map_pose_packet_for_ar(packet)

        with self.lock:
            self.packet_count += 1
            packet.setdefault("seq", self.packet_count)
            packet.setdefault("timestamp", now)
            ar_packet.setdefault("seq", packet["seq"])
            ar_packet.setdefault("timestamp", packet["timestamp"])
            self.last_input_packet = packet
            self.last_packet = packet
            self.last_ar_packet = ar_packet
            self.last_ts = now
            self.status = "receiving"
            count = self.packet_count

        payload = json.dumps(ar_packet, separators=(",", ":")).encode("utf-8")
        pose_path_debug(
            "PARSE_OK",
            f"#{count} from={source_text} input={json.dumps(packet, separators=(',', ':'))} ar={payload.decode('utf-8')}",
            count=count,
        )
        udp_ok, udp_fail = self.send_pose_payload(payload)
        json_ok = write_live_pose_packet(ar_packet)
        with self.lock:
            self.udp_send_count += udp_ok
            self.udp_fail_count += udp_fail
            if json_ok:
                self.live_json_count += 1
            live_json_count = self.live_json_count
            udp_send_count = self.udp_send_count
            udp_fail_count = self.udp_fail_count
            invalid_count = self.invalid_count
        pose_path_debug(
            "LIVE_JSON_OK" if json_ok else "LIVE_JSON_FAIL",
            f"path={DEBUG_PACKET_PATH} live_json_count={live_json_count}",
            count=count,
            force=not json_ok,
        )
        pose_path_debug(
            "UDP_SEND",
            f"ok_targets={udp_ok} fail_targets={udp_fail} total_ok={udp_send_count} total_fail={udp_fail_count} targets={self.target_text}",
            count=count,
            force=udp_fail > 0,
        )
        write_pose_status(
            "receiving",
            packet=packet,
            raw_line=raw_text,
            packet_count=count,
            invalid_count=invalid_count,
            target=self.target_text,
            input_packet=packet,
            ar_packet=ar_packet,
        )

        if DEBUG_PRINT_POSE and count % max(1, DEBUG_PRINT_POSE_EVERY_N) == 0:
            pos = packet["pos"]
            ar_pos = ar_packet["pos"]
            euler = packet["euler"]
            write_debug_log(
                "[AR_POSE_DEBUG] "
                f"#{count} from={source_text} "
                f"input_pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                f"ar_pos=({ar_pos[0]:.3f},{ar_pos[1]:.3f},{ar_pos[2]:.3f}) "
                f"yaw={euler[1]:.2f} "
                f"udp={self.target_text} "
                f"json={payload.decode('utf-8')}"
            )
        elif count == 1 or count % 10 == 0:
            write_debug_log(f"AR pose forwarded #{count}: {payload.decode('utf-8')}")

    def send_pose_payload(self, payload):
        ok_count = 0
        fail_count = 0
        for target in self.targets:
            try:
                self.sock.sendto(payload, target)
                ok_count += 1
            except Exception as exc:
                fail_count += 1
                write_debug_log(f"UDP pose send failed to {target[0]}:{target[1]}: {exc}")
        return ok_count, fail_count

def draw_waiting(frame):
    cv2.putText(frame, "Road seg: waiting", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Midline: waiting", (10, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
    cv2.putText(frame, "Track err: N/A", (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)


def short_text(text, max_len=74):
    text = str(text or "")
    if len(text) <= max_len:
        return text
    return text[:max_len - 3] + "..."


def pose_status_hint(status, packet_count, invalid_count):
    status_l = str(status).lower()
    if "udp listening" in status_l or "udp waiting pose" in status_l:
        return f"Waiting for official robot_position JSON on board UDP port {POSE_INPUT_PORT}."
    if "udp error" in status_l:
        return f"Cannot receive Windows localization on UDP {POSE_INPUT_PORT}. Check port usage."
    if "invalid" in status_l or invalid_count > 0 and packet_count == 0:
        return "UDP arrived, but it was not valid official robot_position JSON. Check the raw datagram."
    if "receiving" in status_l and packet_count > 0:
        return "Pose received and forwarded to official AR engine UDP control port."
    return "Waiting for Windows AprilTag localization."

def draw_pose_status(frame, pose_bridge, fps=None, track_error=None, control_state="N/A", gamepad_status=None):
    if not DEBUG_DRAW_POSE_PANEL:
        return frame

    info = pose_bridge.snapshot()
    age = time.time() - info["last_ts"] if info["last_ts"] else None
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
                f"RT={inputs.get('rt', 0.0):.2f} LT={inputs.get('lt', 0.0):.2f}"
            )

    feedback = get_car_feedback()
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
        car_lines.append(f"TC264 waiting fb={feedback.get('count', 0)} bad={feedback.get('bad', 0)}")
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

    return np.hstack([frame, panel])


def main():
    pose_status_server = start_pose_status_http_server()
    pose_bridge = ARPoseBridge()
    pose_bridge.start()
    gamepad_receiver = GamepadControlReceiver()
    gamepad_receiver.start()
    print("vision client ready, waiting for camera shared memory...")
    last_seg_res = None
    last_seg_ts = 0.0
    last_det_res = None
    last_det_ts = 0.0
    line_missing_since_ts = None
    last_line_loss_command_ts = 0.0

    while True:
        shm = None
        try:
            try:
                shm = shared_memory.SharedMemory(name=SHM_NAME)
                remove_shm_from_resource_tracker()
                print("connected to camera shared memory")
            except FileNotFoundError:
                time.sleep(1.0)
                continue

            last_fid = 0
            have_frame = False
            fps_t = time.time()
            fps_n = 0
            cur_fps = 0.0
            last_runtime_status_ts = 0.0

            while True:
                try:
                    fid, frame = read_frame_from_shm(shm)
                    if fid == last_fid:
                        now = time.time()
                        if have_frame and now - last_line_loss_command_ts >= LINE_LOSS_COMMAND_REPEAT_INTERVAL:
                            if line_missing_since_ts is None:
                                line_missing_since_ts = now
                            line_loss_age = now - line_missing_since_ts
                            if line_loss_age >= LINE_LOSS_SAFE_STOP_TIMEOUT:
                                command_error = 0.0
                                command_speed = 0.0
                                command_state = STATE_SAFE_STOP
                                command_flags = 0
                                control_state_text = "LINE_LOSS_SAFE_STOP"
                            else:
                                command_error = 0.0
                                command_speed = TRACK_FALLBACK_SPEED
                                command_state = STATE_TRACK
                                command_flags = CONTROL_FLAG_USE_TARGET_SPEED
                                control_state_text = "TRACK_FALLBACK"
                            gamepad_cmd, gamepad_status = gamepad_receiver.active_command()
                            if gamepad_cmd is not None:
                                command_error = gamepad_cmd["track_error"]
                                command_speed = gamepad_cmd["target_speed"]
                                command_state = gamepad_cmd["state_cmd"]
                                command_flags = gamepad_cmd["flags"]
                                control_state_text = "GAMEPAD_SAFE_STOP" if gamepad_cmd.get("safe_stop") else "GAMEPAD_TRACK"

                            send_car_cmd(
                                track_error=command_error,
                                target_speed=command_speed,
                                state_cmd=command_state,
                                flags=command_flags,
                            )
                            if now - last_runtime_status_ts >= RUNTIME_STATUS_INTERVAL:
                                write_runtime_status(
                                    pose_bridge,
                                    control_state_text,
                                    command_error,
                                    command_speed,
                                    command_state,
                                    command_flags,
                                    gamepad_status=gamepad_status,
                                )
                                last_runtime_status_ts = now
                            last_line_loss_command_ts = now
                        time.sleep(0.002)
                        if cv2.waitKey(1) == 27:
                            raise KeyboardInterrupt
                        continue
                    last_fid = fid
                    have_frame = True

                    now = time.time()
                    final_frame = frame.copy()
                    track_error = None
                    control_state_text = "WAIT"

                    try:
                        seg_res, seg_flag = infer_seg.infer(frame.copy())
                        if seg_flag and seg_res is not None:
                            last_seg_res = seg_res
                            last_seg_ts = now
                    except Exception as exc:
                        print(f"seg infer skip: {exc}")

                    try:
                        det_res, det_flag = infer_det.infer(frame.copy())
                        if det_flag and det_res is not None:
                            last_det_res = det_res
                            last_det_ts = now
                    except Exception as exc:
                        print(f"det infer skip: {exc}")

                    if last_seg_res is not None and (now - last_seg_ts) <= SEG_RESULT_TTL:
                        try:
                            final_frame, track_error = extract_centerline(last_seg_res, final_frame)
                        except Exception as exc:
                            print(f"seg draw skip: {exc}")
                            draw_waiting(final_frame)
                    else:
                        draw_waiting(final_frame)

                    if last_det_res is not None and (now - last_det_ts) <= DET_RESULT_TTL:
                        try:
                            boxes, scores, classes = last_det_res
                            draw(final_frame, boxes, scores, classes)
                        except Exception as exc:
                            print(f"det draw skip: {exc}")

                    command_state = STATE_TRACK
                    command_flags = CONTROL_FLAG_USE_TARGET_SPEED
                    if track_error is not None and np.isfinite(track_error):
                        command_error = track_error * CONTROL_SCALE
                        command_speed = TRACK_SPEED
                        control_state_text = "VISION"
                        line_missing_since_ts = None
                    else:
                        if line_missing_since_ts is None:
                            line_missing_since_ts = now
                        line_loss_age = now - line_missing_since_ts
                        command_error = 0.0
                        if line_loss_age >= LINE_LOSS_SAFE_STOP_TIMEOUT:
                            command_speed = 0.0
                            command_state = STATE_SAFE_STOP
                            command_flags = 0
                            control_state_text = "LINE_LOSS_SAFE_STOP"
                        else:
                            command_speed = TRACK_FALLBACK_SPEED
                            control_state_text = "TRACK_FALLBACK"

                    gamepad_cmd, gamepad_status = gamepad_receiver.active_command()
                    if gamepad_cmd is not None:
                        command_error = gamepad_cmd["track_error"]
                        command_speed = gamepad_cmd["target_speed"]
                        command_state = gamepad_cmd["state_cmd"]
                        command_flags = gamepad_cmd["flags"]
                        control_state_text = "GAMEPAD_SAFE_STOP" if gamepad_cmd.get("safe_stop") else "GAMEPAD_TRACK"

                    send_car_cmd(
                        track_error=command_error,
                        target_speed=command_speed,
                        state_cmd=command_state,
                        flags=command_flags,
                    )
                    last_line_loss_command_ts = now

                    if now - last_runtime_status_ts >= RUNTIME_STATUS_INTERVAL:
                        write_runtime_status(
                            pose_bridge,
                            control_state_text,
                            command_error,
                            command_speed,
                            command_state,
                            command_flags,
                            gamepad_status=gamepad_status,
                        )
                        last_runtime_status_ts = now

                    fps_n += 1
                    if now - fps_t >= 1.0:
                        cur_fps = fps_n / max(1e-6, now - fps_t)
                        fps_n = 0
                        fps_t = now

                    final_frame = draw_pose_status(
                        final_frame,
                        pose_bridge,
                        fps=cur_fps,
                        track_error=command_error,
                        control_state=control_state_text,
                        gamepad_status=gamepad_status,
                    )
                    if final_frame is None or final_frame.size == 0:
                        final_frame = frame
                    cv2.imshow("ret", final_frame)
                    if cv2.waitKey(1) == 27:
                        raise KeyboardInterrupt

                except (ValueError, struct.error, BufferError):
                    raise FileNotFoundError

        except KeyboardInterrupt:
            print("\nstopped by user")
            break
        except FileNotFoundError:
            print("signal lost, waiting for recovery...")
            if shm:
                try:
                    shm.close()
                except Exception:
                    pass
            cv2.destroyAllWindows()
            time.sleep(1.0)
        finally:
            if shm:
                try:
                    shm.close()
                except Exception:
                    pass

    send_car_cmd(0.0, 0.0, STATE_SAFE_STOP, flags=0, mark_activity=False)
    write_runtime_status(
        pose_bridge,
        "STOPPED_SAFE_STOP",
        0.0,
        0.0,
        STATE_SAFE_STOP,
        0,
    )
    pose_bridge.stop()
    gamepad_receiver.stop()
    if pose_status_server is not None:
        try:
            pose_status_server.shutdown()
            pose_status_server.server_close()
        except Exception:
            pass
    infer_seg.release()
    infer_det.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()



