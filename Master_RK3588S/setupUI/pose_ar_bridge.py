import json
import math
import os
import socket
import threading
import time


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "dist", "main_config.json")

POSE_INPUT_HOST = "0.0.0.0"
POSE_INPUT_PORT = 9005
AR_TARGET_HOST = "127.0.0.1"
AR_TARGET_PORT = 9006
POSE_RX_BUFFER_BYTES = 65536


def _load_ar_port(config_path):
    try:
        with open(config_path, "r", encoding="utf-8") as file:
            config = json.load(file)
        return int(config.get("network", {}).get("control_port", AR_TARGET_PORT))
    except Exception:
        return AR_TARGET_PORT


def parse_pose_datagram(data):
    try:
        packet = json.loads(data.decode("utf-8").strip())
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None

    if not isinstance(packet, dict) or packet.get("type") != "robot_position":
        return None

    pos = packet.get("pos")
    euler = packet.get("euler")
    if not isinstance(pos, (list, tuple)) or len(pos) < 3:
        return None
    if not isinstance(euler, (list, tuple)) or len(euler) < 3:
        return None

    try:
        pos = [float(pos[index]) for index in range(3)]
        euler = [float(euler[index]) for index in range(3)]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in pos + euler):
        return None

    result = {"type": "robot_position", "pos": pos, "euler": euler}
    for key in ("seq", "timestamp"):
        if key in packet:
            result[key] = packet[key]
    return result


def copy_official_pose(packet):
    """Copy an already mapped official AR pose packet without axis changes."""
    copied = {
        "type": "robot_position",
        "pos": list(packet["pos"]),
        "euler": list(packet["euler"]),
    }
    for key in ("seq", "timestamp"):
        if key in packet:
            copied[key] = packet[key]
    return copied


class ARPoseBridge:
    """Forward Windows AprilTag pose packets to the local X-Verse AR engine."""

    def __init__(
        self,
        input_host=POSE_INPUT_HOST,
        input_port=POSE_INPUT_PORT,
        target_host=AR_TARGET_HOST,
        target_port=None,
        config_path=CONFIG_PATH,
        log_func=print,
    ):
        self.input_host = str(input_host)
        self.input_port = int(input_port)
        self.target_host = str(target_host)
        self.target_port = int(_load_ar_port(config_path) if target_port is None else target_port)
        self.log_func = log_func

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._input_socket = None
        self._output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.packet_count = 0
        self.invalid_count = 0
        self.drop_count = 0
        self.send_fail_count = 0
        self.last_packet = None
        self.last_ar_packet = None
        self.last_source = None
        self.last_timestamp = 0.0
        self.last_error = ""

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="ar-pose-bridge", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._input_socket is not None:
            try:
                self._input_socket.close()
            except OSError:
                pass
        try:
            self._output_socket.close()
        except OSError:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def snapshot(self):
        with self._lock:
            age = time.time() - self.last_timestamp if self.last_timestamp else None
            return {
                "input": f"{self.input_host}:{self.input_port}",
                "target": f"{self.target_host}:{self.target_port}",
                "packet_count": self.packet_count,
                "invalid_count": self.invalid_count,
                "drop_count": self.drop_count,
                "send_fail_count": self.send_fail_count,
                "last_packet": dict(self.last_packet) if self.last_packet else None,
                "last_ar_packet": dict(self.last_ar_packet) if self.last_ar_packet else None,
                "last_source": self.last_source,
                "age": age,
                "error": self.last_error,
            }

    def _log(self, message):
        if self.log_func is not None:
            self.log_func(message)

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._input_socket = sock
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, POSE_RX_BUFFER_BYTES)
            sock.bind((self.input_host, self.input_port))
            sock.settimeout(0.5)
            self._log(
                f"Pose bridge ready: {self.input_host}:{self.input_port} -> "
                f"{self.target_host}:{self.target_port}"
            )

            while not self._stop_event.is_set():
                try:
                    data, source = sock.recvfrom(65535)
                    data, source, dropped = self._drain_latest(sock, data, source)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if self._stop_event.is_set():
                        break
                    raise exc

                packet = parse_pose_datagram(data)
                if packet is None:
                    with self._lock:
                        self.invalid_count += 1
                        self.drop_count += dropped
                    continue

                ar_packet = copy_official_pose(packet)
                payload = json.dumps(ar_packet, separators=(",", ":")).encode("utf-8")
                try:
                    self._output_socket.sendto(payload, (self.target_host, self.target_port))
                    error = ""
                except OSError as exc:
                    error = str(exc)
                    with self._lock:
                        self.send_fail_count += 1

                with self._lock:
                    self.packet_count += 1
                    self.drop_count += dropped
                    self.last_packet = packet
                    self.last_ar_packet = ar_packet
                    self.last_source = f"{source[0]}:{source[1]}"
                    self.last_timestamp = time.time()
                    self.last_error = error
        except Exception as exc:
            with self._lock:
                self.last_error = str(exc)
            self._log(f"Pose bridge stopped: {exc}")
        finally:
            try:
                sock.close()
            except OSError:
                pass
            self._input_socket = None

    @staticmethod
    def _drain_latest(sock, data, source):
        dropped = 0
        timeout = sock.gettimeout()
        try:
            sock.setblocking(False)
            while True:
                data, source = sock.recvfrom(65535)
                dropped += 1
        except (BlockingIOError, socket.timeout):
            pass
        finally:
            sock.settimeout(timeout)
        return data, source, dropped
