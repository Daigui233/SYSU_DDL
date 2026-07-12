import json
import math
import os
import socket
import threading
import time


GAMEPAD_HOST = os.environ.get("AR_GAMEPAD_CONTROL_HOST", "0.0.0.0")
GAMEPAD_PORT = int(os.environ.get("AR_GAMEPAD_CONTROL_PORT", "9010"))
GAMEPAD_PACKET_TTL_SECONDS = float(os.environ.get("AR_GAMEPAD_CONTROL_TTL", "3.0"))
GAMEPAD_MAX_SPEED_MPS = float(os.environ.get("AR_GAMEPAD_MAX_SPEED_MPS", "1.0"))
GAMEPAD_MAX_TRACK_ERROR = float(os.environ.get("AR_GAMEPAD_MAX_TRACK_ERROR", "240.0"))

STATE_TRACK = 1
STATE_SAFE_STOP = 7
CONTROL_FLAG_USE_TARGET_SPEED = 0x01


def _finite_float(value, default=0.0):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), _finite_float(value)))


class GamepadControlReceiver:
    """Receive explicit manual-control packets from the Windows locator."""

    def __init__(
        self,
        host=GAMEPAD_HOST,
        port=GAMEPAD_PORT,
        ttl=GAMEPAD_PACKET_TTL_SECONDS,
        max_speed_mps=GAMEPAD_MAX_SPEED_MPS,
        max_track_error=GAMEPAD_MAX_TRACK_ERROR,
        log_func=print,
    ):
        self.host = str(host)
        self.port = int(port)
        self.ttl = float(ttl)
        self.max_speed_mps = float(max_speed_mps)
        self.max_track_error = float(max_track_error)
        self.log_func = log_func

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._socket = None

        self.packet_count = 0
        self.invalid_count = 0
        self.last_timestamp = 0.0
        self.last_command = None
        self.last_error = ""

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="gamepad-control", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def snapshot(self):
        with self._lock:
            command = dict(self.last_command) if self.last_command else None
            age = time.time() - self.last_timestamp if self.last_timestamp else None
            active = bool(command and command["gamepad_mode"] and age is not None and age <= self.ttl)
            return {
                "target": f"{self.host}:{self.port}",
                "active": active,
                "age": age,
                "ttl": self.ttl,
                "packet_count": self.packet_count,
                "invalid_count": self.invalid_count,
                "command": command,
                "error": self.last_error,
            }

    def active_command(self):
        status = self.snapshot()
        if not status["active"]:
            return None, status
        command = status["command"]
        return {
            "track_error": command["track_error"],
            "target_speed": command["target_speed"],
            "state_cmd": command["state_cmd"],
            "flags": command["flags"],
            "safe_stop": command["safe_stop"],
        }, status

    def handle_datagram(self, data):
        try:
            packet = json.loads(data.decode("utf-8").strip())
            if not isinstance(packet, dict) or packet.get("type") != "gamepad_control":
                raise ValueError("expected gamepad_control packet")

            enabled = bool(packet.get("gamepad_mode", False))
            safe_stop = bool(packet.get("safe_stop", False))
            if not enabled:
                command = {
                    "gamepad_mode": False,
                    "track_error": 0.0,
                    "target_speed": 0.0,
                    "state_cmd": STATE_TRACK,
                    "flags": 0,
                    "safe_stop": False,
                }
            elif safe_stop or int(packet.get("state_cmd", STATE_TRACK)) == STATE_SAFE_STOP:
                command = {
                    "gamepad_mode": True,
                    "track_error": 0.0,
                    "target_speed": 0.0,
                    "state_cmd": STATE_SAFE_STOP,
                    "flags": 0,
                    "safe_stop": True,
                }
            else:
                command = {
                    "gamepad_mode": True,
                    "track_error": _clamp(
                        packet.get("track_error", 0.0),
                        -self.max_track_error,
                        self.max_track_error,
                    ),
                    "target_speed": _clamp(
                        packet.get("target_speed", 0.0),
                        -self.max_speed_mps,
                        self.max_speed_mps,
                    ),
                    "state_cmd": STATE_TRACK,
                    "flags": CONTROL_FLAG_USE_TARGET_SPEED,
                    "safe_stop": False,
                }

            command["seq"] = packet.get("seq")
            command["source"] = packet.get("source", "windows_locator")
            with self._lock:
                self.packet_count += 1
                self.last_timestamp = time.time()
                self.last_command = command
                self.last_error = ""
            return True
        except Exception as exc:
            with self._lock:
                self.invalid_count += 1
                self.last_error = str(exc)
            return False

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.settimeout(0.5)
            if self.log_func is not None:
                self.log_func(f"Gamepad receiver ready: {self.host}:{self.port}")
            while not self._stop_event.is_set():
                try:
                    data, _source = sock.recvfrom(8192)
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    raise
                self.handle_datagram(data)
        except Exception as exc:
            with self._lock:
                self.last_error = str(exc)
            if self.log_func is not None:
                self.log_func(f"Gamepad receiver stopped: {exc}")
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._socket = None
