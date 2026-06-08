#!/usr/bin/env python3
import atexit
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial_comm as _serial_comm
except Exception as exc:
    _serial_comm = None
    print(f"serial_comm unavailable: {exc}; car serial control disabled")


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "dist", "main_config.json")

DEFAULT_AR_UDP_IP = "127.0.0.1"
DEFAULT_AR_UDP_PORT = 9006
POSE_INPUT_HOST = os.environ.get("AR_POSE_INPUT_HOST", "0.0.0.0")
POSE_INPUT_PORT = int(os.environ.get("AR_POSE_INPUT_PORT", "9005"))

GAMEPAD_CONTROL_HOST = os.environ.get("AR_GAMEPAD_CONTROL_HOST", "0.0.0.0")
GAMEPAD_CONTROL_PORT = int(os.environ.get("AR_GAMEPAD_CONTROL_PORT", "9010"))
GAMEPAD_CONTROL_TTL = float(os.environ.get("AR_GAMEPAD_CONTROL_TTL", "3.0"))
GAMEPAD_MAX_SPEED_MPS = float(os.environ.get("AR_GAMEPAD_MAX_SPEED_MPS", "1.0"))
GAMEPAD_MAX_TRACK_ERROR = float(os.environ.get("AR_GAMEPAD_MAX_TRACK_ERROR", "240.0"))

CONTROL_REPEAT_INTERVAL = float(os.environ.get("AR_BRIDGE_CONTROL_REPEAT_INTERVAL", "0.05"))
IDLE_SAFE_STOP_INTERVAL = float(os.environ.get("AR_BRIDGE_IDLE_SAFE_STOP_INTERVAL", "0.2"))
RUNTIME_STATUS_INTERVAL = float(os.environ.get("AR_BRIDGE_STATUS_INTERVAL", "0.5"))

LOG_PATH = os.environ.get("AR_UDP_CONTROL_LOG_PATH", os.path.join(BASE_DIR, "ar_udp_control_bridge.log"))
DEBUG_STATUS_PATH = os.environ.get("AR_POSE_STATUS_PATH", os.path.join(BASE_DIR, "ar_pose_status.json"))
DEBUG_PACKET_PATH = os.environ.get("AR_POSE_PACKET_PATH", os.path.join(BASE_DIR, "xverse_control_live.json"))
LOG_MAX_BYTES = 2 * 1024 * 1024

POSE_STATUS_HTTP_HOST = os.environ.get("AR_POSE_STATUS_HOST", "0.0.0.0")
POSE_STATUS_HTTP_PORT = int(os.environ.get("AR_POSE_STATUS_PORT", "9105"))

STATE_TRACK = getattr(_serial_comm, "STATE_TRACK", 1) if _serial_comm else 1
STATE_SAFE_STOP = getattr(_serial_comm, "STATE_SAFE_STOP", 7) if _serial_comm else 7
CONTROL_FLAG_USE_TARGET_SPEED = getattr(_serial_comm, "CONTROL_FLAG_USE_TARGET_SPEED", 0x01) if _serial_comm else 0x01
CarController = getattr(_serial_comm, "CarController", None) if _serial_comm else None

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

STATUS_LOCK = threading.Lock()
LATEST_STATUS = {}

CONTROL_COMMAND_LOCK = threading.Lock()
LAST_CONTROL_SEND_OK = None
LAST_CONTROL_SEND_ERROR = ""
LAST_CONTROL_SEND_TS = 0.0


class DisabledCarController:
    def send_cmd(self, *args, **kwargs):
        return False

    def get_feedback(self):
        return {"online": False, "error": "car serial control disabled"}


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


car = create_car_controller()


def rotate_log_if_needed():
    try:
        if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > LOG_MAX_BYTES:
            old_path = LOG_PATH + ".old"
            if os.path.exists(old_path):
                os.remove(old_path)
            os.replace(LOG_PATH, old_path)
    except Exception:
        pass


def write_log(message, echo=True):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}"
    if echo:
        print(line)
    try:
        rotate_log_if_needed()
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception as exc:
        print(f"bridge log write failed: {exc}")


def write_json_atomic(path, payload):
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp_path, path)


def read_json_file(path, default=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def finite_float(value, default=0.0):
    try:
        result = float(value)
    except Exception:
        return default
    return result if math.isfinite(result) else default


def clamp_float(value, low, high):
    value = finite_float(value, 0.0)
    return max(float(low), min(float(high), value))


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
                return False, str(exc)
        except Exception as exc:
            return False, str(exc)
    except Exception as exc:
        return False, str(exc)
    ok = True if result is None else bool(result)
    return ok, "" if ok else "serial send returned false"


def send_car_cmd(track_error, target_speed, state_cmd, flags=CONTROL_FLAG_USE_TARGET_SPEED):
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


def stop_car_on_exit():
    try:
        send_car_cmd(0.0, 0.0, STATE_SAFE_STOP, flags=0)
    except Exception:
        pass


atexit.register(stop_car_on_exit)


def load_ar_network_config():
    ip = os.environ.get("AR_UDP_IP", DEFAULT_AR_UDP_IP)
    port = DEFAULT_AR_UDP_PORT
    try:
        cfg = read_json_file(CONFIG_PATH, {}) or {}
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

    if not all(math.isfinite(value) for value in pos_values + euler_values):
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


def write_live_pose_packet(packet):
    if not packet:
        return False
    try:
        write_json_atomic(DEBUG_PACKET_PATH, packet)
        return True
    except Exception as exc:
        write_log(f"live pose packet write failed: {exc}")
        return False


def send_local_udp_json(port, payload):
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(data, ("127.0.0.1", int(port)))
    finally:
        sock.close()
    return len(data)


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
        self.last_error = ""

    def start(self):
        write_log("=" * 60)
        write_log(
            f"clean AR UDP/control bridge starting, pose input={POSE_INPUT_HOST}:{POSE_INPUT_PORT}, "
            f"AR targets={self.target_text}, gamepad input={GAMEPAD_CONTROL_HOST}:{GAMEPAD_CONTROL_PORT}"
        )
        self.set_status("udp listening")
        self.thread = threading.Thread(target=self._udp_loop, name="clean-ar-pose-udp", daemon=True)
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
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def set_status(self, status, error=""):
        with self.lock:
            self.status = status
            self.last_error = error

    def snapshot(self):
        with self.lock:
            return {
                "status": self.status,
                "last_error": self.last_error,
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
                "log_path": LOG_PATH,
                "status_path": DEBUG_STATUS_PATH,
                "packet_path": DEBUG_PACKET_PATH,
                "ar_forward_mapping": AR_FORWARD_MAPPING,
            }

    def _udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.5)
        self.input_sock = sock
        try:
            sock.bind((POSE_INPUT_HOST, POSE_INPUT_PORT))
            self.set_status("udp waiting pose")
            write_log(f"Windows localization UDP ready on {POSE_INPUT_HOST}:{POSE_INPUT_PORT}")
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
                self.handle_packet(packet, raw_text, source_text, error)
        except Exception as exc:
            if not self.stop_event.is_set():
                self.set_status(f"udp error: {exc}", str(exc))
                write_log(f"UDP pose receiver stopped: {exc}")
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
                self.status = "invalid pose datagram"
                self.last_error = error
            write_log(f"[AR_POSE_BAD] from={source_text} reason={error} raw={repr(raw_text)}")
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
            self.last_error = ""
            count = self.packet_count

        payload = json.dumps(ar_packet, separators=(",", ":")).encode("utf-8")
        udp_ok, udp_fail = self.send_pose_payload(payload)
        json_ok = write_live_pose_packet(ar_packet)

        with self.lock:
            self.udp_send_count += udp_ok
            self.udp_fail_count += udp_fail
            if json_ok:
                self.live_json_count += 1

        if count == 1 or count % 30 == 0 or udp_fail > 0:
            pos = packet["pos"]
            ar_pos = ar_packet["pos"]
            euler = packet["euler"]
            write_log(
                "[AR_POSE_BRIDGE] "
                f"#{count} from={source_text} "
                f"input_pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                f"ar_pos=({ar_pos[0]:.3f},{ar_pos[1]:.3f},{ar_pos[2]:.3f}) "
                f"yaw={euler[1]:.2f} udp_ok={udp_ok} udp_fail={udp_fail}"
            )

    def send_pose_payload(self, payload):
        ok_count = 0
        fail_count = 0
        for target in self.targets:
            try:
                self.sock.sendto(payload, target)
                ok_count += 1
            except Exception as exc:
                fail_count += 1
                write_log(f"UDP pose send failed to {target[0]}:{target[1]}: {exc}")
        return ok_count, fail_count


class GamepadControlReceiver:
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
        self.thread = threading.Thread(target=self.run, name="clean-ar-gamepad-udp", daemon=True)
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
            write_log(f"Gamepad UDP ready on {self.host}:{self.port}")
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
            write_log(f"Gamepad UDP receiver stopped: {exc}")

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
                    "state_cmd": STATE_SAFE_STOP,
                    "target_speed": 0.0,
                    "track_error": 0.0,
                    "flags": 0,
                    "safe_stop": True,
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
                    target_speed = clamp_float(packet.get("target_speed", 0.0), -GAMEPAD_MAX_SPEED_MPS, GAMEPAD_MAX_SPEED_MPS)
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
            write_log(f"Bad gamepad UDP from {addr}: {exc}")

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
                    "state_cmd": int(packet.get("state_cmd", STATE_SAFE_STOP)),
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


def write_runtime_status(pose_bridge, gamepad_status, control_state, command_error, command_speed, command_state, command_flags):
    global LATEST_STATUS

    pose_info = pose_bridge.snapshot()
    control_info = {
        "state": control_state,
        "track_error": float(command_error) if command_error is not None and math.isfinite(float(command_error)) else None,
        "target_speed": float(command_speed),
        "state_cmd": int(command_state),
        "flags": int(command_flags),
        "serial_send": get_control_send_status(),
        "gamepad": gamepad_status,
        "timestamp": time.time(),
        "source": "clean_ar_udp_control_bridge",
    }
    info = {
        "status": pose_info.get("status", "clean AR UDP/control bridge running"),
        "mode": "clean_ar_udp_control_bridge",
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
        "debug_log": LOG_PATH,
        "packet_json": DEBUG_PACKET_PATH,
        "http_port": POSE_STATUS_HTTP_PORT,
    }
    with STATUS_LOCK:
        LATEST_STATUS = dict(info)
    try:
        write_json_atomic(DEBUG_STATUS_PATH, info)
    except Exception as exc:
        write_log(f"runtime status write failed: {exc}")


def current_http_payload():
    with STATUS_LOCK:
        status = dict(LATEST_STATUS)
    if not status:
        status = read_json_file(DEBUG_STATUS_PATH, {}) or {}
    status.setdefault("status", "clean AR UDP/control bridge not ready")
    status.setdefault("mode", "clean_ar_udp_control_bridge")
    status.setdefault("packet_count", 0)
    status.setdefault("invalid_count", 0)
    status.setdefault("debug_log", LOG_PATH)
    status.setdefault("packet_json", DEBUG_PACKET_PATH)
    status.setdefault("target", f"{DEFAULT_AR_UDP_IP}:{DEFAULT_AR_UDP_PORT}")
    status.setdefault("pose_convention", POSE_CONVENTION)
    status.setdefault("ar_forward_mapping", AR_FORWARD_MAPPING)
    status["http_port"] = POSE_STATUS_HTTP_PORT
    return status


class PoseStatusHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(204)
        self._send_common_headers()
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/pose_status", "/health"):
            self._send_json(current_http_payload())
            return
        if path == "/pose_packet":
            self._send_json(current_http_payload().get("packet") or {})
            return
        self._send_json({"error": "not found", "path": path}, status=404)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            payload = self._read_json_body()
        except Exception as exc:
            self._send_json({"error": f"invalid JSON body: {exc}"}, status=400)
            return

        if path == "/api/manual_pose":
            try:
                packet = {
                    "type": "robot_position",
                    "pos": [
                        float(payload.get("x", 0.0)),
                        float(payload.get("y", 0.16)),
                        float(payload.get("z", 0.0)),
                    ],
                    "euler": [0.0, float(payload.get("yaw", 0.0)), 0.0],
                    "timestamp": time.time(),
                    "source": "bridge_http",
                }
                byte_count = send_local_udp_json(POSE_INPUT_PORT, packet)
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=500)
                return
            self._send_json({"ok": True, "bytes": byte_count, "target": f"127.0.0.1:{POSE_INPUT_PORT}"})
            return

        if path == "/api/gamepad_control":
            try:
                packet = dict(payload)
                packet.setdefault("type", "gamepad_control")
                packet.setdefault("source", "bridge_http")
                byte_count = send_local_udp_json(GAMEPAD_CONTROL_PORT, packet)
            except Exception as exc:
                self._send_json({"error": str(exc)}, status=500)
                return
            self._send_json({"ok": True, "bytes": byte_count, "target": f"127.0.0.1:{GAMEPAD_CONTROL_PORT}"})
            return

        self._send_json({"error": "not found", "path": path}, status=404)

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))

    def _send_json(self, payload, status=200):
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self._send_common_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_common_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")

    def log_message(self, _format, *args):
        return


def start_pose_status_http_server():
    try:
        server = ThreadingHTTPServer((POSE_STATUS_HTTP_HOST, POSE_STATUS_HTTP_PORT), PoseStatusHandler)
    except Exception as exc:
        write_log(f"pose status HTTP server failed: {exc}")
        return None

    thread = threading.Thread(target=server.serve_forever, name="clean-ar-status-http", daemon=True)
    thread.start()
    write_log(
        f"Bridge status HTTP server: http://{POSE_STATUS_HTTP_HOST}:{POSE_STATUS_HTTP_PORT}/pose_status"
    )
    return server


def run_control_loop(pose_bridge, gamepad_receiver, stop_event):
    last_status_ts = 0.0
    last_control_ts = 0.0
    last_idle_stop_ts = 0.0
    last_state_text = "IDLE_SAFE_STOP"
    last_error = 0.0
    last_speed = 0.0
    last_state = STATE_SAFE_STOP
    last_flags = 0

    while not stop_event.is_set():
        now = time.time()
        gamepad_cmd, gamepad_status = gamepad_receiver.active_command()

        if gamepad_cmd is not None:
            command_error = gamepad_cmd["track_error"]
            command_speed = gamepad_cmd["target_speed"]
            command_state = gamepad_cmd["state_cmd"]
            command_flags = gamepad_cmd["flags"]
            control_state_text = "GAMEPAD_SAFE_STOP" if gamepad_cmd.get("safe_stop") else "GAMEPAD_TRACK"
            should_send = now - last_control_ts >= CONTROL_REPEAT_INTERVAL
        else:
            command_error = 0.0
            command_speed = 0.0
            command_state = STATE_SAFE_STOP
            command_flags = 0
            control_state_text = "IDLE_SAFE_STOP"
            should_send = now - last_idle_stop_ts >= IDLE_SAFE_STOP_INTERVAL

        if should_send:
            send_car_cmd(
                track_error=command_error,
                target_speed=command_speed,
                state_cmd=command_state,
                flags=command_flags,
            )
            last_control_ts = now
            if gamepad_cmd is None:
                last_idle_stop_ts = now

        last_state_text = control_state_text
        last_error = command_error
        last_speed = command_speed
        last_state = command_state
        last_flags = command_flags

        if now - last_status_ts >= RUNTIME_STATUS_INTERVAL:
            write_runtime_status(
                pose_bridge,
                gamepad_status,
                last_state_text,
                last_error,
                last_speed,
                last_state,
                last_flags,
            )
            last_status_ts = now

        stop_event.wait(0.01)


def main():
    stop_event = threading.Event()
    pose_status_server = start_pose_status_http_server()
    pose_bridge = ARPoseBridge()
    gamepad_receiver = GamepadControlReceiver()

    pose_bridge.start()
    gamepad_receiver.start()

    try:
        run_control_loop(pose_bridge, gamepad_receiver, stop_event)
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        stop_event.set()
        send_car_cmd(0.0, 0.0, STATE_SAFE_STOP, flags=0)
        write_runtime_status(
            pose_bridge,
            gamepad_receiver.snapshot(),
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


if __name__ == "__main__":
    main()
