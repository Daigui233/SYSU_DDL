import json
import os
import socket
import threading
import time


GAMEPAD_CONTROL_HOST = os.environ.get("AR_GAMEPAD_CONTROL_HOST", "0.0.0.0")
GAMEPAD_CONTROL_PORT = int(os.environ.get("AR_GAMEPAD_CONTROL_PORT", "9010"))
GAMEPAD_CONTROL_TTL = float(os.environ.get("AR_GAMEPAD_CONTROL_TTL", "3.0"))
GAMEPAD_MAX_SPEED_MPS = float(os.environ.get("AR_GAMEPAD_MAX_SPEED_MPS", "1.0"))
GAMEPAD_MAX_TRACK_ERROR = float(os.environ.get("AR_GAMEPAD_MAX_TRACK_ERROR", "240.0"))


def finite_float(value, default=0.0):
    try:
        result = float(value)
    except Exception:
        return default
    return result if result == result and abs(result) != float("inf") else default


def clamp_float(value, low, high):
    value = finite_float(value, 0.0)
    return max(float(low), min(float(high), value))


class GamepadControlReceiver:
    """Receive optional manual gamepad control packets on a separate UDP port."""

    def __init__(
        self,
        host=GAMEPAD_CONTROL_HOST,
        port=GAMEPAD_CONTROL_PORT,
        ttl=GAMEPAD_CONTROL_TTL,
        max_speed_mps=GAMEPAD_MAX_SPEED_MPS,
        max_track_error=GAMEPAD_MAX_TRACK_ERROR,
        state_track=1,
        state_safe_stop=7,
        control_flag_use_target_speed=0x01,
        log_func=None,
    ):
        self.host = host
        self.port = int(port)
        self.ttl = float(ttl)
        self.max_speed_mps = float(max_speed_mps)
        self.max_track_error = float(max_track_error)
        self.state_track = int(state_track)
        self.state_safe_stop = int(state_safe_stop)
        self.control_flag_use_target_speed = int(control_flag_use_target_speed)
        self.log_func = log_func

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
        self.thread = threading.Thread(target=self.run, name="gamepad-control-udp", daemon=True)
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

    def log(self, message):
        if self.log_func is not None:
            try:
                self.log_func(message)
                return
            except TypeError:
                self.log_func(str(message))
                return
        print(message)

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(0.2)
            self.set_status(f"gamepad listening {self.host}:{self.port}")
            self.log(f"Gamepad UDP ready on {self.host}:{self.port}")
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
            self.log(f"Gamepad UDP receiver stopped: {exc}")

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
            state_cmd = int(packet.get("state_cmd", self.state_safe_stop if safe_stop else self.state_track))
            if state_cmd not in (self.state_track, self.state_safe_stop):
                state_cmd = self.state_safe_stop if safe_stop else self.state_track

            if not gamepad_mode:
                command = {
                    "gamepad_mode": False,
                    "active": False,
                    "state_cmd": self.state_track,
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
                if safe_stop or state_cmd == self.state_safe_stop:
                    state_cmd = self.state_safe_stop
                    target_speed = 0.0
                    track_error = 0.0
                    flags = 0
                    status = "gamepad safe_stop"
                else:
                    state_cmd = self.state_track
                    target_speed = clamp_float(packet.get("target_speed", 0.0), -self.max_speed_mps, self.max_speed_mps)
                    track_error = clamp_float(packet.get("track_error", 0.0), -self.max_track_error, self.max_track_error)
                    flags = int(packet.get("flags", self.control_flag_use_target_speed)) | self.control_flag_use_target_speed
                    status = "gamepad manual track"

                command = {
                    "gamepad_mode": True,
                    "active": True,
                    "state_cmd": state_cmd,
                    "target_speed": target_speed,
                    "track_error": track_error,
                    "flags": flags,
                    "safe_stop": state_cmd == self.state_safe_stop,
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
            self.log(f"Bad gamepad UDP from {addr}: {exc}")

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
                    "state_cmd": int(packet.get("state_cmd", self.state_track)),
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
            "state_cmd": snap.get("state_cmd", self.state_track),
            "flags": snap.get("flags", self.control_flag_use_target_speed),
            "safe_stop": snap.get("safe_stop", False),
        }, snap
