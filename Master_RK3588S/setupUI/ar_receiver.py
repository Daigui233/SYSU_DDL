import time
import struct
import asyncio
import json
import math
import os
import re
import socket
import threading
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
import cv2
from multiprocessing import shared_memory, resource_tracker

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

from infer_wrap import InferWrap, PPSegInfer
from infer_wrap.base.func import draw
from infer_wrap.base.seg_func import extract_centerline

# serial_comm 是 RK3588S -> TC264D 的控车串口，和 ESP32 定位 BLE 链路无关。
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
CONTROL_SCALE = 0.5
TRACK_SPEED = 120.0
CONTROL_MODE = os.environ.get("AR_CONTROL_MODE", "pose").strip().lower()
CONTROL_FLAG_USE_TARGET_SPEED = int(os.environ.get("AR_CONTROL_SPEED_FLAG", "1"), 0)
PATH_WAYPOINTS_PATH = os.environ.get("AR_WAYPOINT_PATH", os.path.join(BASE_DIR, "path_waypoints.json"))
POSE_CONTROL_SPEED = float(os.environ.get("AR_POSE_CONTROL_SPEED", "80.0"))
POSE_FRESH_TTL = float(os.environ.get("AR_POSE_FRESH_TTL", "0.8"))
POSE_WAYPOINT_REACH_DIST = float(os.environ.get("AR_POSE_WAYPOINT_REACH_DIST", "0.25"))
POSE_HEADING_GAIN = float(os.environ.get("AR_POSE_HEADING_GAIN", "1.2"))
POSE_CROSSTRACK_GAIN = float(os.environ.get("AR_POSE_CROSSTRACK_GAIN", "80.0"))
POSE_CONTROL_SIGN = float(os.environ.get("AR_POSE_CONTROL_SIGN", "1.0"))
POSE_CONTROL_LIMIT = float(os.environ.get("AR_POSE_CONTROL_LIMIT", "120.0"))

BLE_DEVICE_NAME = os.environ.get("AR_BLE_DEVICE_NAME", "ESP32_BLE_Safe")
BLE_DEVICE_ADDRESS = os.environ.get("AR_BLE_ADDRESS")
UART_SERVICE_UUID = os.environ.get("AR_BLE_SERVICE_UUID", "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
UART_TX_CHAR_UUID = os.environ.get("AR_BLE_TX_UUID", "6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
BLE_BOOTSTRAP_WITH_BLUETOOTHCTL = os.environ.get("AR_BLE_BOOTSTRAP", "1") != "0"
BLE_USE_BLUETOOTHCTL_CACHE = os.environ.get("AR_BLE_USE_CACHE", "0") == "1"
BLE_SCAN_TIMEOUT = float(os.environ.get("AR_BLE_SCAN_TIMEOUT", "8.0"))
DEFAULT_AR_UDP_IP = "127.0.0.1"
DEFAULT_AR_UDP_PORT = 9005
POSE_SCALE = float(os.environ.get("AR_POSE_SCALE", "0.01"))
POSE_HEIGHT = float(os.environ.get("AR_POSE_HEIGHT", "0.16"))
POSE_X_SIGN = float(os.environ.get("AR_POSE_X_SIGN", "1.0"))
POSE_Z_SIGN = float(os.environ.get("AR_POSE_Z_SIGN", "1.0"))
POSE_YAW_SIGN = float(os.environ.get("AR_POSE_YAW_SIGN", "1.0"))
POSE_YAW_OFFSET = float(os.environ.get("AR_POSE_YAW_OFFSET", "0.0"))

# ===== 定位数据终端调试开关 =====
# 联调 ESP32 -> RK3588S -> 官方 AR 引擎链路时保持 True，会在终端打印原始定位行和转发后的官方 JSON。
# 稳定后如果嫌终端刷屏，把这里改成 False；也可以注释掉 handle_line() 里的 AR_POSE_DEBUG 打印块。
DEBUG_PRINT_POSE = True
DEBUG_PRINT_POSE_EVERY_N = 1
DEBUG_LOG_POSE_TO_FILE = True
DEBUG_LOG_PATH = os.environ.get("AR_POSE_LOG_PATH", os.path.join(BASE_DIR, "ar_pose_debug.log"))
DEBUG_STATUS_PATH = os.environ.get("AR_POSE_STATUS_PATH", os.path.join(BASE_DIR, "ar_pose_status.json"))
DEBUG_PACKET_PATH = os.environ.get("AR_POSE_PACKET_PATH", os.path.join(BASE_DIR, "xverse_control_live.json"))
DEBUG_LOG_MAX_BYTES = 2 * 1024 * 1024
DEBUG_DRAW_POSE_PANEL = True
DEBUG_PRINT_RAW_BLE = True
DEBUG_PRINT_RAW_EVERY_N = 1
POSE_STATUS_HTTP_HOST = os.environ.get("AR_POSE_STATUS_HOST", "0.0.0.0")
POSE_STATUS_HTTP_PORT = int(os.environ.get("AR_POSE_STATUS_PORT", "9105"))
NO_NL_BUFFER_LIMIT = int(os.environ.get("AR_POSE_NO_NL_BUFFER_LIMIT", "256"))

# ===== POSE_PATH_DEBUG_START：定位链路分段验证打印，稳定后可整段删除 =====
# 作用：在不改 WebUI 的情况下，判断定位数据卡在 BLE、解析、JSON 镜像、UDP 转发还是官方 AR 引擎消费阶段。
# 删除方法：搜索 POSE_PATH_DEBUG_START 到 POSE_PATH_DEBUG_END，以及代码中的 POSE_PATH_DEBUG 调用点。
POSE_PATH_DEBUG = True
POSE_PATH_DEBUG_EVERY_N = 1
# ===== POSE_PATH_DEBUG_END =====

STATE_TRACK = getattr(_serial_comm, "STATE_TRACK", 1) if _serial_comm else 1
STATE_SAFE_STOP = getattr(_serial_comm, "STATE_SAFE_STOP", 7) if _serial_comm else 7
CarController = getattr(_serial_comm, "CarController", None) if _serial_comm else None


class DisabledCarController:
    def send_cmd(self, *args, **kwargs):
        return None


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


def send_car_cmd(track_error, target_speed, state_cmd, flags=CONTROL_FLAG_USE_TARGET_SPEED):
    send_cmd = getattr(car, "send_cmd", None)
    if send_cmd is None:
        return
    try:
        send_cmd(
            track_error=track_error,
            target_speed=target_speed,
            state_cmd=state_cmd,
            flags=flags,
        )
    except TypeError:
        try:
            send_cmd(track_error, target_speed, state_cmd, flags)
        except TypeError:
            try:
                send_cmd(track_error)
            except Exception as exc:
                print(f"serial send skip: {exc}")
        except Exception as exc:
            print(f"serial send skip: {exc}")
    except Exception as exc:
        print(f"serial send skip: {exc}")


def get_car_feedback():
    getter = getattr(car, "get_feedback", None)
    if getter is None:
        return {"online": False, "error": "feedback unavailable"}
    try:
        return getter()
    except Exception as exc:
        return {"online": False, "error": str(exc)}

print("initializing NPU AI...")
infer_det = InferWrap(model_dir=MODEL_DIR, TPEs=1, core_ids=[0], max_inflight=1)
infer_seg = PPSegInfer(model_dir=MODEL_DIR, TPEs=1, core_ids=[1], max_inflight=1)
print("AI engines loaded")

car = create_car_controller()


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


def write_pose_status(status, packet=None, raw_line="", packet_count=0, invalid_count=0, target=None):
    info = {
        "status": status,
        "timestamp": time.time(),
        "packet_count": packet_count,
        "invalid_count": invalid_count,
        "raw_line": raw_line,
        "packet": packet,
        "target": target,
        "debug_log": DEBUG_LOG_PATH,
        "packet_json": DEBUG_PACKET_PATH,
        "car_feedback": get_car_feedback(),
    }
    try:
        tmp_path = DEBUG_STATUS_PATH + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(info, f, ensure_ascii=False, indent=2)
        os.replace(tmp_path, DEBUG_STATUS_PATH)
    except Exception as exc:
        print(f"pose status write failed: {exc}")


def write_runtime_status(pose_bridge, control_state, command_error, command_speed, command_flags, planner_status):
    pose_info = pose_bridge.snapshot()
    control_info = {
        "state": control_state,
        "track_error": float(command_error) if command_error is not None and np.isfinite(command_error) else None,
        "target_speed": float(command_speed),
        "flags": int(command_flags),
        "planner": planner_status,
        "timestamp": time.time(),
    }
    info = {
        "status": pose_info.get("status", "ar_receiver running"),
        "timestamp": time.time(),
        "packet_count": pose_info.get("packet_count", 0),
        "invalid_count": pose_info.get("invalid_count", 0),
        "packet": pose_info.get("last_packet"),
        "target": pose_info.get("target", f"{DEFAULT_AR_UDP_IP}:{DEFAULT_AR_UDP_PORT}"),
        "udp_send_count": pose_info.get("udp_send_count", 0),
        "udp_fail_count": pose_info.get("udp_fail_count", 0),
        "control": control_info,
        "car_feedback": get_car_feedback(),
        "debug_log": DEBUG_LOG_PATH,
        "packet_json": DEBUG_PACKET_PATH,
    }
    try:
        tmp_path = DEBUG_STATUS_PATH + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(info, f, ensure_ascii=False, indent=2)
        os.replace(tmp_path, DEBUG_STATUS_PATH)
    except Exception as exc:
        print(f"runtime status write failed: {exc}")

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
    packet = read_json_file(DEBUG_PACKET_PATH, None)
    if packet is not None and not status.get("packet"):
        status["packet"] = packet
    status.setdefault("status", "ar_receiver not ready")
    status.setdefault("packet_count", 0)
    status.setdefault("invalid_count", 0)
    status.setdefault("debug_log", DEBUG_LOG_PATH)
    status.setdefault("packet_json", DEBUG_PACKET_PATH)
    status.setdefault("target", f"{DEFAULT_AR_UDP_IP}:{DEFAULT_AR_UDP_PORT}")
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


def parse_bluetoothctl_devices(output):
    devices = []
    for line in (output or "").splitlines():
        parts = line.strip().split(maxsplit=2)
        if len(parts) >= 2 and parts[0] == "Device":
            address = parts[1]
            name = parts[2] if len(parts) >= 3 else ""
            devices.append((address, name))
    return devices


def is_bluez_connected(address):
    ok, output = run_bluetoothctl_command(("info", address), timeout=3.0)
    if not ok:
        return False
    return "Connected: yes" in output


def find_cached_ble_address():
    outputs = []
    ok_connected, connected_output = run_bluetoothctl_command(("devices", "Connected"), timeout=3.0)
    if ok_connected and connected_output:
        outputs.append((connected_output, True))

    ok, output = run_bluetoothctl_command(("devices",), timeout=3.0)
    if ok and output:
        outputs.append((output, False))
    elif not outputs:
        write_debug_log(f"bluetoothctl devices failed: {output}")
        return None, []

    seen = []
    seen_addresses = set()
    target_address = (BLE_DEVICE_ADDRESS or "").lower()
    for output_text, listed_connected in outputs:
        for address, name in parse_bluetoothctl_devices(output_text):
            if address.lower() in seen_addresses:
                continue
            seen_addresses.add(address.lower())
            connected = listed_connected or is_bluez_connected(address)
            suffix = " [connected]" if connected else ""
            seen.append(f"{name or '(no name)'}@{address}{suffix}")
            if target_address and address.lower() == target_address:
                return address, seen
            if BLE_DEVICE_NAME and BLE_DEVICE_NAME in name:
                return address, seen
    return None, seen


def run_bluetoothctl_command(args, timeout=6.0):
    try:
        result = subprocess.run(
            ["bluetoothctl"] + list(args),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
        )
        return result.returncode == 0, result.stdout.strip()
    except FileNotFoundError:
        return False, "bluetoothctl not found"
    except subprocess.TimeoutExpired:
        return False, "bluetoothctl timeout"
    except Exception as exc:
        return False, str(exc)


def bootstrap_bluetooth_adapter():
    if not BLE_BOOTSTRAP_WITH_BLUETOOTHCTL:
        return

    commands = [
        (("power", "on"), 6.0),
        (("agent", "on"), 6.0),
        (("default-agent",), 6.0),
    ]
    for args, timeout in commands:
        ok, output = run_bluetoothctl_command(args, timeout=timeout)
        label = " ".join(args)
        if ok:
            write_debug_log(f"bluetoothctl {label}: ok", echo=False)
        else:
            write_debug_log(f"bluetoothctl {label}: {output}")

    ok, output = run_bluetoothctl_command(("scan", "on"), timeout=3.0)
    write_debug_log(f"bluetoothctl scan on: {'ok' if ok else output}", echo=False)
    time.sleep(3.0)
    ok, output = run_bluetoothctl_command(("devices",), timeout=3.0)
    if ok and output:
        write_debug_log("bluetoothctl cached devices: " + output.replace("\n", "; "), echo=False)
    run_bluetoothctl_command(("scan", "off"), timeout=3.0)


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
    # The WebUI udp_target_* fields describe the external pose source and are not
    # the port this local BLE bridge should send to.
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


def discover_local_ips():
    ips = set()
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ips.add(info[4][0])
    except Exception:
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        ips.add(probe.getsockname()[0])
        probe.close()
    except Exception:
        pass
    return [ip for ip in sorted(ips) if ip and not ip.startswith("127.")]


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


def pick_float(data, names, default=None):
    for name in names:
        if isinstance(data, dict) and name in data and data[name] is not None:
            try:
                return float(data[name])
            except (TypeError, ValueError):
                pass
    return default


def normalize_yaw(yaw):
    if yaw is None:
        return 0.0
    if os.environ.get("AR_YAW_UNIT", "deg").lower().startswith("rad"):
        return float(np.degrees(yaw))
    return float(yaw)


def make_official_ar_packet(x, z, yaw_deg=0.0, height=None, pitch=0.0, roll=0.0):
    y = POSE_HEIGHT if height is None else height
    yaw = normalize_yaw(yaw_deg)
    return {
        "type": "robot_position",
        "pos": [
            float(x) * POSE_SCALE * POSE_X_SIGN,
            float(y),
            float(z) * POSE_SCALE * POSE_Z_SIGN,
        ],
        # Match the local WASD/dataset_collector path: yaw is carried in euler[2].
        "euler": [
            float(pitch),
            float(roll),
            yaw * POSE_YAW_SIGN + POSE_YAW_OFFSET,
        ],
    }


def coerce_official_packet(data):
    pos = data.get("pos")
    euler = data.get("euler", [0.0, 0.0, 0.0])
    if not isinstance(pos, (list, tuple)) or len(pos) < 3:
        return None
    if not isinstance(euler, (list, tuple)):
        euler = [0.0, euler, 0.0]
    euler = list(euler) + [0.0, 0.0, 0.0]
    packet = {
        "type": "robot_position",
        "pos": [float(pos[0]), float(pos[1]), float(pos[2])],
        "euler": [float(euler[0]), float(euler[1]), float(euler[2])],
    }
    if "seq" in data:
        packet["seq"] = data["seq"]
    if "timestamp" in data:
        packet["timestamp"] = data["timestamp"]
    return packet


def packet_from_mapping(data):
    official = coerce_official_packet(data) if "pos" in data else None
    if official is not None:
        return official

    x = pick_float(data, ("x", "pos_x", "px"))
    z = pick_float(data, ("z", "pos_z", "pz"))
    if z is None:
        z = pick_float(data, ("y", "pos_y", "py"))
    height = pick_float(data, ("height", "h", "camera_y"), POSE_HEIGHT)
    yaw = pick_float(data, ("yaw_deg", "yaw", "theta", "heading", "angle"), 0.0)
    pitch = pick_float(data, ("pitch",), 0.0)
    roll = pick_float(data, ("roll",), 0.0)

    if x is None or z is None:
        return None
    return make_official_ar_packet(x, z, yaw, height=height, pitch=pitch, roll=roll)


def packet_from_numbers(nums):
    if len(nums) >= 4:
        return make_official_ar_packet(nums[0], nums[2], nums[3], height=nums[1])
    if len(nums) == 3:
        return make_official_ar_packet(nums[0], nums[1], nums[2])
    if len(nums) == 2:
        return make_official_ar_packet(nums[0], nums[1], 0.0)
    return None


def parse_pose_line(line):
    text = line.strip()
    if not text:
        return None

    if "{" in text and "}" in text:
        json_text = text[text.find("{"):text.rfind("}") + 1]
        try:
            data = json.loads(json_text)
            if isinstance(data, dict):
                return packet_from_mapping(data)
        except json.JSONDecodeError:
            pass

    pairs = dict(
        (key.lower(), float(value))
        for key, value in re.findall(
            r"([A-Za-z_][A-Za-z0-9_]*)\s*[:=]\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)",
            text,
        )
    )
    if pairs:
        packet = packet_from_mapping(pairs)
        if packet is not None:
            return packet

    nums = [float(x) for x in re.findall(r"-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?", text)]
    return packet_from_numbers(nums)


def looks_like_complete_pose_chunk(text):
    text = text.strip()
    if not text:
        return False
    if text.startswith("{") and text.endswith("}"):
        return True
    num = r"-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?"
    sep = r"\s*[,| ]\s*"
    if re.fullmatch(num + f"(?:{sep}{num})" + r"{1,3}", text):
        return True
    return bool(re.search(r"\bx\s*[:=]", text) and re.search(r"\b(y|z)\s*[:=]", text))



def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _read_pose_xyz_yaw(packet):
    if not isinstance(packet, dict):
        return None
    pos = packet.get("pos")
    euler = packet.get("euler")
    if not isinstance(pos, (list, tuple)) or len(pos) < 3:
        return None
    if not isinstance(euler, (list, tuple)) or len(euler) < 3:
        return None
    try:
        x = float(pos[0])
        z = float(pos[2])
        yaw = float(euler[2])
    except (TypeError, ValueError):
        return None
    if not (np.isfinite(x) and np.isfinite(z) and np.isfinite(yaw)):
        return None
    return x, z, yaw


def _normalize_waypoint(item):
    try:
        if isinstance(item, dict):
            if "pos" in item and isinstance(item["pos"], (list, tuple)) and len(item["pos"]) >= 3:
                return float(item["pos"][0]), float(item["pos"][2])
            if "x" in item and "z" in item:
                return float(item["x"]), float(item["z"])
            if "x" in item and "y" in item:
                return float(item["x"]), float(item["y"])
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            return float(item[0]), float(item[1])
    except (TypeError, ValueError):
        return None
    return None


def load_path_waypoints(path):
    if not path or not os.path.exists(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as fp:
            data = json.load(fp)
    except Exception as exc:
        write_debug_log(f"path waypoint load failed: {path} {exc}")
        return []
    raw_points = data.get("waypoints", data.get("points", data)) if isinstance(data, dict) else data
    if not isinstance(raw_points, list):
        return []
    points = []
    for item in raw_points:
        point = _normalize_waypoint(item)
        if point is not None and np.isfinite(point[0]) and np.isfinite(point[1]):
            points.append(point)
    return points


class PosePathPlanner:
    def __init__(self, waypoint_path):
        self.waypoint_path = waypoint_path
        self.waypoints = load_path_waypoints(waypoint_path)
        self.index = 1 if len(self.waypoints) > 1 else 0
        if self.waypoints:
            self.last_status = f"loaded {len(self.waypoints)} pts"
        else:
            self.last_status = "no waypoint file"

    @property
    def enabled(self):
        return len(self.waypoints) >= 2

    def compute(self, pose_packet):
        pose = _read_pose_xyz_yaw(pose_packet)
        if pose is None:
            self.last_status = "pose invalid"
            return None
        if not self.enabled:
            self.last_status = "path disabled"
            return None

        x, z, yaw = pose
        target_x, target_z = self.waypoints[self.index]
        dx = target_x - x
        dz = target_z - z
        dist = math.hypot(dx, dz)

        hops = 0
        while dist <= POSE_WAYPOINT_REACH_DIST and hops < len(self.waypoints):
            self.index = (self.index + 1) % len(self.waypoints)
            target_x, target_z = self.waypoints[self.index]
            dx = target_x - x
            dz = target_z - z
            dist = math.hypot(dx, dz)
            hops += 1

        prev_x, prev_z = self.waypoints[(self.index - 1) % len(self.waypoints)]
        seg_x = target_x - prev_x
        seg_z = target_z - prev_z
        seg_len = max(1e-6, math.hypot(seg_x, seg_z))
        lateral = ((x - prev_x) * seg_z - (z - prev_z) * seg_x) / seg_len

        # X/Z plane: x is lateral, z is forward in the official robot_position payload.
        desired_yaw = math.degrees(math.atan2(dx, dz))
        heading_error = wrap_degrees(desired_yaw - yaw)
        control_error = POSE_CONTROL_SIGN * (
            POSE_HEADING_GAIN * heading_error + POSE_CROSSTRACK_GAIN * lateral
        )
        control_error = clamp(control_error, -POSE_CONTROL_LIMIT, POSE_CONTROL_LIMIT)
        self.last_status = (
            f"wp {self.index + 1}/{len(self.waypoints)} "
            f"dist={dist:.2f} head={heading_error:.1f} lat={lateral:.2f}"
        )
        return control_error, POSE_CONTROL_SPEED, self.last_status

class ARPoseBridge:
    def __init__(self):
        self.target_ip, self.target_port = load_ar_network_config()
        self.targets = build_ar_udp_targets(self.target_ip, self.target_port)
        self.target_text = ",".join(f"{ip}:{port}" for ip, port in self.targets)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.buffer = ""
        self.status = "idle"
        self.last_line = ""
        self.last_packet = None
        self.last_ts = 0.0
        self.last_scan_devices = []
        self.raw_chunk_count = 0
        self.last_raw_chunk = ""
        self.last_raw_ts = 0.0
        self.packet_count = 0
        self.invalid_count = 0
        self.udp_send_count = 0
        self.udp_fail_count = 0
        self.live_json_count = 0

    def start(self):
        write_debug_log("=" * 60)
        write_debug_log(f"AR pose bridge starting, UDP targets: {self.target_text}")
        write_debug_log(f"pose debug log: {DEBUG_LOG_PATH}")
        write_debug_log(f"pose status json: {DEBUG_STATUS_PATH}")
        write_debug_log(f"live official packet json: {DEBUG_PACKET_PATH}")
        write_pose_status(
            "starting",
            packet_count=self.packet_count,
            invalid_count=self.invalid_count,
            target=self.target_text,
        )
        if BleakClient is None or BleakScanner is None:
            self.status = "bleak missing"
            write_debug_log("BLE bridge disabled: install bleak on RK3588S to receive ESP32 BLE pose data")
            write_pose_status(
                self.status,
                packet_count=self.packet_count,
                invalid_count=self.invalid_count,
                target=self.target_text,
            )
            return
        self.set_status("bluetooth init", "Preparing board Bluetooth adapter before BLE scan")
        bootstrap_bluetooth_adapter()
        self.thread = threading.Thread(target=self._thread_main, name="ar-pose-ble", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        try:
            self.sock.close()
        except Exception:
            pass

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "last_line": self.last_line,
                "last_packet": self.last_packet,
                "last_ts": self.last_ts,
                "packet_count": self.packet_count,
                "invalid_count": self.invalid_count,
                "target": self.target_text,
                "device": BLE_DEVICE_ADDRESS or BLE_DEVICE_NAME,
                "seen_devices": self.last_scan_devices,
                "raw_chunk_count": self.raw_chunk_count,
                "last_raw_chunk": self.last_raw_chunk,
                "line_buffer_len": len(self.buffer),
                "udp_send_count": self.udp_send_count,
                "udp_fail_count": self.udp_fail_count,
                "live_json_count": self.live_json_count,
                "log_path": DEBUG_LOG_PATH,
                "status_path": DEBUG_STATUS_PATH,
                "packet_path": DEBUG_PACKET_PATH,
            }

    def set_status(self, status, log_message=None):
        with self.lock:
            self.status = status
            packet = self.last_packet
            raw_line = self.last_line
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
        )

    @staticmethod
    def device_label(device):
        if isinstance(device, str):
            return device
        return f"{device.name or '(no name)'}@{device.address}"

    def _thread_main(self):
        try:
            asyncio.run(self._ble_loop())
        except Exception as exc:
            self.set_status(f"BLE error: {exc}", f"BLE bridge stopped: {exc}")

    async def _ble_loop(self):
        while not self.stop_event.is_set():
            try:
                self.set_status("scanning", f"BLE scanning for {BLE_DEVICE_ADDRESS or BLE_DEVICE_NAME}")
                device = await self._find_device()
                if device is None:
                    self.set_status("scan no device", "BLE scan finished: ESP32 pose device not found")
                    await asyncio.sleep(2.0)
                    continue

                self.set_status("connecting", f"BLE pose device found, connecting: {self.device_label(device)}")
                async with BleakClient(device, timeout=10.0) as client:
                    self.set_status("connected", f"BLE connected: {self.device_label(device)}")
                    notify_target = await self._resolve_notify_characteristic(client)
                    notify_target = await self._start_notify(client, notify_target)
                    target_label = getattr(notify_target, "uuid", notify_target)
                    self.set_status("waiting pose", f"BLE notify enabled on {target_label}, waiting pose lines")
                    while not self.stop_event.is_set() and self._client_connected(client):
                        await asyncio.sleep(0.2)
                    try:
                        await client.stop_notify(notify_target)
                    except Exception:
                        pass
            except Exception as exc:
                self.set_status(f"reconnect: {exc}", f"BLE reconnect needed: {exc}")
                await asyncio.sleep(2.0)

    async def _resolve_notify_characteristic(self, client):
        target_char = None
        try:
            services = getattr(client, "services", None)
            if services is None and hasattr(client, "get_services"):
                services = await client.get_services()

            lines = []
            if services is not None:
                for svc in services:
                    lines.append(f"service {svc.uuid}")
                    for ch in svc.characteristics:
                        props = ",".join(ch.properties)
                        lines.append(f"  char {ch.uuid} [{props}]")
                        if ch.uuid.lower() == UART_TX_CHAR_UUID.lower():
                            target_char = ch
                if lines:
                    write_debug_log("BLE services:\n" + "\n".join(lines), echo=False)
        except Exception as exc:
            write_debug_log(f"BLE service discovery skip: {exc}")

        if target_char is not None:
            write_debug_log(f"BLE notify target characteristic found: {target_char.uuid} [{','.join(target_char.properties)}]")
            return target_char
        write_debug_log(f"BLE notify target fallback to UUID: {UART_TX_CHAR_UUID}")
        return UART_TX_CHAR_UUID

    async def _start_notify(self, client, notify_target):
        try:
            await client.start_notify(UART_TX_CHAR_UUID, self._on_ble_data)
            write_debug_log("BLE start_notify succeeded with UUID")
            return UART_TX_CHAR_UUID
        except Exception as exc:
            write_debug_log(f"BLE start_notify failed on UUID: {exc}")

        if notify_target != UART_TX_CHAR_UUID:
            await client.start_notify(notify_target, self._on_ble_data)
            write_debug_log("BLE start_notify succeeded with discovered characteristic")
            return notify_target
        raise RuntimeError(f"BLE start_notify failed for {UART_TX_CHAR_UUID}")

    async def _find_device(self):
        seen = []
        target_address = (BLE_DEVICE_ADDRESS or "").lower()
        target_service = UART_SERVICE_UUID.lower()

        if not target_address:
            try:
                device = await BleakScanner.find_device_by_filter(
                    lambda d, ad: (
                        (d.name == BLE_DEVICE_NAME)
                        or (getattr(ad, "local_name", None) == BLE_DEVICE_NAME)
                    ),
                    timeout=BLE_SCAN_TIMEOUT,
                )
                if device is not None:
                    self._update_scan_devices([f"{device.name or BLE_DEVICE_NAME}@{device.address}"])
                    write_debug_log(f"BLE find_device_by_filter found: {device.name or BLE_DEVICE_NAME}@{device.address}")
                    return device
            except Exception as exc:
                write_debug_log(f"BLE find_device_by_filter skip: {exc}")

        if BLE_USE_BLUETOOTHCTL_CACHE:
            cached_address, cached_seen = find_cached_ble_address()
            if cached_seen:
                self._update_scan_devices(cached_seen)
                write_debug_log("bluetoothctl cache saw: " + "; ".join(cached_seen), echo=False)
            if cached_address:
                write_debug_log(f"Using cached ESP32 BLE address from bluetoothctl: {cached_address}")
                return cached_address

        try:
            discovered = await BleakScanner.discover(timeout=BLE_SCAN_TIMEOUT, return_adv=True)
            items = []
            for item in discovered.values():
                if isinstance(item, tuple) and len(item) == 2:
                    items.append(item)
            if not items:
                items = [(device, None) for device in discovered.values()]
        except TypeError:
            devices = await BleakScanner.discover(timeout=BLE_SCAN_TIMEOUT)
            items = [(device, None) for device in devices]

        for device, adv in items:
            dev_name = device.name or ""
            adv_name = getattr(adv, "local_name", "") or ""
            name = adv_name or dev_name
            address = (device.address or "").lower()
            service_uuids = [s.lower() for s in (getattr(adv, "service_uuids", None) or [])]
            display_name = name or "(no name)"
            seen.append(f"{display_name}@{device.address}")

            if target_address and address == target_address:
                self._update_scan_devices(seen)
                return device
            if BLE_DEVICE_NAME and (BLE_DEVICE_NAME in dev_name or BLE_DEVICE_NAME in adv_name):
                self._update_scan_devices(seen)
                return device
            if not target_address and target_service in service_uuids and len(service_uuids) > 0:
                self._update_scan_devices(seen)
                return device

        self._update_scan_devices(seen)
        if seen:
            write_debug_log("BLE scan saw: " + "; ".join(seen), echo=False)
        if BLE_USE_BLUETOOTHCTL_CACHE:
            cached_address, cached_seen = find_cached_ble_address()
            if cached_seen:
                self._update_scan_devices(cached_seen)
                write_debug_log("bluetoothctl cache saw: " + "; ".join(cached_seen), echo=False)
            if cached_address:
                write_debug_log(f"Using cached ESP32 BLE address from bluetoothctl: {cached_address}")
                return cached_address
        return None

    def _update_scan_devices(self, seen):
        with self.lock:
            self.last_scan_devices = seen[-8:]

    @staticmethod
    def _client_connected(client):
        connected = getattr(client, "is_connected", False)
        return connected() if callable(connected) else bool(connected)

    def _on_ble_data(self, _sender, data):
        chunk = data.decode("utf-8", errors="ignore")
        normalized_chunk = chunk.replace("\r\n", "\n").replace("\r", "\n")
        now = time.time()
        with self.lock:
            self.raw_chunk_count += 1
            self.last_raw_chunk = chunk
            self.last_raw_ts = now
            raw_count = self.raw_chunk_count
            status = self.status
            packet = self.last_packet
            packet_count = self.packet_count
            invalid_count = self.invalid_count

        if DEBUG_PRINT_RAW_BLE and raw_count % max(1, DEBUG_PRINT_RAW_EVERY_N) == 0:
            write_debug_log(f"[BLE_RAW] #{raw_count} {data.hex()} -> {repr(chunk)}")
        pose_path_debug("BLE_RAW", f"#{raw_count} bytes={len(data)} text={repr(chunk)}", count=raw_count)
        if status == "waiting pose":
            write_pose_status(
                "raw ble data",
                packet=packet,
                raw_line=chunk,
                packet_count=packet_count,
                invalid_count=invalid_count,
                target=self.target_text,
            )

        self.buffer += normalized_chunk
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            pose_path_debug("LINE_READY", f"line={repr(line)} remain_buf={len(self.buffer)}", force=True)
            self.handle_line(line)

        pending = self.buffer.strip()
        if pending and looks_like_complete_pose_chunk(pending):
            self.buffer = ""
            pose_path_debug("LINE_READY_NO_NL", f"line={repr(pending)}", force=True)
            self.handle_line(pending)
        elif len(self.buffer) > NO_NL_BUFFER_LIMIT:
            overflow = self.buffer.strip()
            self.buffer = ""
            pose_path_debug("LINE_READY_NO_NL", f"buffer_overflow line={repr(overflow)}", force=True)
            self.handle_line(overflow)

    def handle_line(self, line):
        packet = parse_pose_line(line)
        now = time.time()
        if packet is None:
            with self.lock:
                self.invalid_count += 1
                self.last_line = line.strip()
                invalid_count = self.invalid_count
                packet_count = self.packet_count
            write_pose_status(
                "invalid pose line",
                raw_line=line.strip(),
                packet_count=packet_count,
                invalid_count=invalid_count,
                target=self.target_text,
            )
            pose_path_debug("PARSE_FAIL", f"raw={repr(line.strip())} invalid_count={invalid_count}", force=True)
            write_debug_log(f"[AR_POSE_BAD] raw='{line.strip()}'")
            return

        with self.lock:
            self.packet_count += 1
            packet.setdefault("seq", self.packet_count)
            packet.setdefault("timestamp", now)
            self.last_line = line.strip()
            self.last_packet = packet
            self.last_ts = now
            self.status = "receiving"
            count = self.packet_count

        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        pose_path_debug("PARSE_OK", f"#{count} packet={payload.decode('utf-8')}", count=count)
        udp_ok, udp_fail = self.send_pose_payload(payload)
        json_ok = write_live_pose_packet(packet)
        with self.lock:
            self.udp_send_count += udp_ok
            self.udp_fail_count += udp_fail
            if json_ok:
                self.live_json_count += 1
            live_json_count = self.live_json_count
            udp_send_count = self.udp_send_count
            udp_fail_count = self.udp_fail_count
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
            raw_line=line.strip(),
            packet_count=count,
            invalid_count=self.invalid_count,
            target=self.target_text,
        )

        # ===== AR_POSE_DEBUG：定位数据终端调试打印 =====
        # 需要检查 BLE 是否收到、JSON 是否转换正确、UDP 是否发到 127.0.0.1:9005 时，保留这一段。
        # 如果后面正式跑车时不想刷屏，可以把这一整个 if 块注释掉，或把文件顶部 DEBUG_PRINT_POSE 改成 False。
        if DEBUG_PRINT_POSE and count % max(1, DEBUG_PRINT_POSE_EVERY_N) == 0:
            pos = packet.get("pos", [0.0, 0.0, 0.0])
            euler = packet.get("euler", [0.0, 0.0, 0.0])
            write_debug_log(
                "[AR_POSE_DEBUG] "
                f"#{count} raw='{line.strip()}' "
                f"pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                f"yaw={euler[2]:.2f} "
                f"udp={self.target_text} "
                f"json={payload.decode('utf-8')}"
            )
        # ===== AR_POSE_DEBUG 结束 =====

        if not DEBUG_PRINT_POSE and (count == 1 or count % 10 == 0):
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
    if "bleak missing" in status_l:
        return "Install python bleak first: pip install bleak"
    if "bluetooth init" in status_l:
        return "Turning board Bluetooth on with bluetoothctl, then auto scanning ESP32."
    if "scanning" in status_l:
        return "Scanning BLE. If stuck here, power ESP32 and check BLE name/adapter."
    if "connecting" in status_l:
        return "ESP32 found. Connecting BLE GATT and enabling notify."
    if "scan no device" in status_l:
        return "No ESP32_BLE_Safe found. Check ESP32 power, BLE name, and board Bluetooth."
    if "connected" in status_l or "waiting pose" in status_l:
        return "BLE connected. If ok stays 0, check whether ESP32 is notifying pose lines."
    if "invalid" in status_l or invalid_count > 0 and packet_count == 0:
        return "BLE has data, but parser rejected it. Check raw line format in log."
    if "receiving" in status_l and packet_count > 0:
        return "Pose received and forwarded to official AR engine UDP control port."
    if "reconnect" in status_l or "error" in status_l:
        return "BLE connection error. Watch log and retry ESP32 power/Bluetooth."
    return "Waiting for BLE pose pipeline."


def draw_pose_status(frame, pose_bridge, fps=None, track_error=None, control_state="N/A", planner_status=None):
    if not DEBUG_DRAW_POSE_PANEL:
        return

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
            f"yaw={euler[2]:.1f} age={age_text}"
        )

    ble_line = (
        f"ESP32 {short_text(info['status'], 18)} "
        f"ok={info['packet_count']} bad={info['invalid_count']}"
    )
    udp_line = f"UDP ok={info['udp_send_count']} fail={info['udp_fail_count']}"
    if track_error is not None and np.isfinite(track_error):
        plan_line = f"PLAN {control_state} err={track_error:.1f}"
    else:
        plan_line = f"PLAN {control_state} err=N/A"
    path_line = f"PATH {short_text(planner_status, 34)}" if planner_status else None
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
            f"SPD in={feedback.get('input_target_speed', 0.0):.0f} "
            f"tgt={feedback.get('motor_target', 0.0):.0f} "
            f"act={feedback.get('actual_speed', 0.0):.1f}"
        )
        car_lines.append(
            f"OUT m={feedback.get('motor_output', 0)} s={feedback.get('servo_output', 0)} "
            f"PID {feedback.get('motor_kp', 0.0):.1f}/{feedback.get('motor_ki', 0.0):.1f}/{feedback.get('motor_kd', 0.0):.1f} "
            f"SV {feedback.get('servo_kp', 0.0):.1f}/{feedback.get('servo_kd', 0.0):.1f}"
        )
    else:
        err = short_text(feedback.get("error", "waiting"), 28)
        car_lines.append(f"TC264 waiting fb={feedback.get('count', 0)} bad={feedback.get('bad', 0)} {err}")
    fps_line = f"FPS {fps:.1f}" if fps is not None else "FPS N/A"

    lines = [
        ble_line,
        pose_line,
        udp_line,
        plan_line,
    ]
    if path_line:
        lines.append(path_line)
    lines.extend(car_lines)
    lines.append(fps_line)

    x0, y0 = 8, 8
    line_h = 16
    panel_w = min(frame.shape[1] - 16, 470)
    panel_h = 10 + line_h * len(lines)
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.38, frame, 0.62, 0, frame)
    cv2.rectangle(frame, (x0, y0), (x0 + panel_w, y0 + panel_h), color, 1)

    for i, line in enumerate(lines):
        y = y0 + 15 + i * line_h
        text_color = color if i == 0 else (230, 245, 245)
        cv2.putText(frame, line, (x0 + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, text_color, 1, cv2.LINE_AA)


def main():
    pose_status_server = start_pose_status_http_server()
    pose_bridge = ARPoseBridge()
    pose_bridge.start()
    pose_planner = PosePathPlanner(PATH_WAYPOINTS_PATH)
    print(f"control mode: {CONTROL_MODE}, waypoint path: {PATH_WAYPOINTS_PATH}, points: {len(pose_planner.waypoints)}")
    print("vision client ready, waiting for camera shared memory...")
    last_seg_res = None
    last_seg_ts = 0.0
    last_det_res = None
    last_det_ts = 0.0

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
            fps_t = time.time()
            fps_n = 0
            cur_fps = 0.0
            last_runtime_status_ts = 0.0

            while True:
                try:
                    fid, frame = read_frame_from_shm(shm)
                    if fid == last_fid:
                        time.sleep(0.002)
                        if cv2.waitKey(1) == 27:
                            raise KeyboardInterrupt
                        continue
                    last_fid = fid

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

                    command_error = None
                    command_speed = 0.0
                    command_state = STATE_SAFE_STOP
                    command_flags = 0
                    planner_status = pose_planner.last_status
                    pose_info = pose_bridge.snapshot()
                    pose_age = now - pose_info["last_ts"] if pose_info["last_ts"] else None
                    pose_packet = pose_info["last_packet"]

                    if CONTROL_MODE in ("pose", "path", "planner") and pose_planner.enabled:
                        if pose_packet is not None and pose_age is not None and pose_age <= POSE_FRESH_TTL:
                            pose_cmd = pose_planner.compute(pose_packet)
                            planner_status = pose_planner.last_status
                            if pose_cmd is not None:
                                command_error, command_speed, planner_status = pose_cmd
                                command_state = STATE_TRACK
                                command_flags = CONTROL_FLAG_USE_TARGET_SPEED
                                control_state_text = "POSE"
                        else:
                            if pose_age is None:
                                planner_status = "pose waiting"
                            else:
                                planner_status = f"pose stale {pose_age:.1f}s"

                    if command_error is None and track_error is not None and np.isfinite(track_error):
                        command_error = track_error * CONTROL_SCALE
                        command_speed = TRACK_SPEED
                        command_state = STATE_TRACK
                        command_flags = CONTROL_FLAG_USE_TARGET_SPEED
                        control_state_text = "VISION"
                    elif command_error is None:
                        command_error = 0.0
                        command_speed = 0.0
                        command_state = STATE_SAFE_STOP
                        command_flags = 0
                        control_state_text = "SAFE_STOP"

                    send_car_cmd(
                        track_error=command_error,
                        target_speed=command_speed,
                        state_cmd=command_state,
                        flags=command_flags,
                    )

                    fps_n += 1
                    if now - fps_t >= 1.0:
                        cur_fps = fps_n / max(1e-6, now - fps_t)
                        fps_n = 0
                        fps_t = now

                    draw_pose_status(final_frame, pose_bridge, fps=cur_fps, track_error=command_error, control_state=control_state_text, planner_status=planner_status)
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

    pose_bridge.stop()
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



