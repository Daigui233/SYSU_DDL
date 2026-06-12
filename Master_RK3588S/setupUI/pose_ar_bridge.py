import json
import math
import os
import socket
import threading
import time


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "dist", "main_config.json")
DEFAULT_AR_UDP_IP = "127.0.0.1"
DEFAULT_AR_UDP_PORT = 9006
POSE_INPUT_HOST = os.environ.get("AR_POSE_INPUT_HOST", "0.0.0.0")
POSE_INPUT_PORT = int(os.environ.get("AR_POSE_INPUT_PORT", "9005"))
DEBUG_PACKET_PATH = os.environ.get("AR_POSE_PACKET_PATH", os.path.join(BASE_DIR, "xverse_control_live.json"))
POSE_STATUS_WRITE_INTERVAL = float(os.environ.get("AR_POSE_STATUS_INTERVAL", "0.20"))
POSE_LIVE_JSON_WRITE_INTERVAL = float(os.environ.get("AR_POSE_LIVE_JSON_INTERVAL", "0.20"))
POSE_RX_DRAIN_LATEST = os.environ.get("AR_POSE_RX_DRAIN_LATEST", "1").strip().lower() not in ("0", "false", "no", "off")
POSE_RX_BUFFER_BYTES = int(os.environ.get("AR_POSE_RX_BUFFER_BYTES", "65536"))

POSE_CONVENTION = {
    "field_m": [4.0, 3.0],
    "field_bottom_right_xz_m": [0.0, 0.30],
    "field_z_range_m": [0.30, 3.30],
    "origin": "30cm below field bottom-right reference",
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


def read_json_file(path, default=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def write_json_atomic(path, payload):
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp_path, path)


def send_local_udp_json(port, payload):
    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(data, ("127.0.0.1", int(port)))
    finally:
        sock.close()
    return len(data)


def load_ar_network_config(config_path=CONFIG_PATH):
    # Official AR consumes robot_position on network.control_port.
    ip = os.environ.get("AR_UDP_IP", DEFAULT_AR_UDP_IP)
    port = DEFAULT_AR_UDP_PORT
    try:
        cfg = read_json_file(config_path, {}) or {}
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


class ARPoseBridge:
    """Receive Windows robot_position UDP and forward it to the official AR engine."""

    def __init__(
        self,
        config_path=CONFIG_PATH,
        input_host=POSE_INPUT_HOST,
        input_port=POSE_INPUT_PORT,
        packet_path=DEBUG_PACKET_PATH,
        log_path=None,
        status_path=None,
        log_func=None,
        pose_status_writer=None,
        pose_path_debug=None,
        debug_print_pose=True,
        debug_print_every_n=1,
    ):
        self.input_host = input_host
        self.input_port = int(input_port)
        self.packet_path = packet_path
        self.log_path = log_path
        self.status_path = status_path
        self.log_func = log_func
        self.pose_status_writer = pose_status_writer
        self.pose_path_debug = pose_path_debug
        self.debug_print_pose = bool(debug_print_pose)
        self.debug_print_every_n = max(1, int(debug_print_every_n))

        self.target_ip, self.target_port = load_ar_network_config(config_path)
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
        self.input_drop_count = 0
        self.last_recv_ts = 0.0
        self.last_forward_ts = 0.0
        self.last_forward_ms = 0.0
        self.last_handle_ms = 0.0
        self._next_pose_status_ts = 0.0
        self._next_live_json_ts = 0.0

    def start(self):
        self.log("=" * 60)
        self.log(
            f"AR pose bridge starting, Windows UDP input={self.input_host}:{self.input_port}, "
            f"AR targets: {self.target_text}"
        )
        if self.log_path:
            self.log(f"pose debug log: {self.log_path}")
        if self.status_path:
            self.log(f"pose status json: {self.status_path}")
        self.log(f"live official packet json: {self.packet_path}")
        self.write_pose_status("starting")
        self.set_status(
            "udp listening",
            f"Waiting for official robot_position JSON on {self.input_host}:{self.input_port}",
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
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def log(self, message):
        if self.log_func is not None:
            self.log_func(message)
        else:
            print(message)

    def debug(self, stage, message, count=None, force=False):
        if self.pose_path_debug is None:
            return
        self.pose_path_debug(stage, message, count=count, force=force)

    def should_debug(self, count=None, force=False):
        printer = self.pose_path_debug
        if printer is None or not getattr(printer, "enabled", False):
            return False
        if force:
            return True
        if count is None:
            return True
        every_n = max(1, int(getattr(printer, "every_n", 1)))
        return count % every_n == 0

    def write_pose_status(self, status, packet=None, raw_line="", input_packet=None, ar_packet=None):
        if self.pose_status_writer is None:
            return
        self.pose_status_writer(
            status,
            packet=packet,
            raw_line=raw_line,
            packet_count=self.packet_count,
            invalid_count=self.invalid_count,
            target=self.target_text,
            input_packet=input_packet,
            ar_packet=ar_packet,
        )

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
                "input": f"{self.input_host}:{self.input_port}",
                "protocol": "udp/robot_position",
                "udp_send_count": self.udp_send_count,
                "udp_fail_count": self.udp_fail_count,
                "live_json_count": self.live_json_count,
                "input_drop_count": self.input_drop_count,
                "last_recv_ts": self.last_recv_ts,
                "last_forward_ts": self.last_forward_ts,
                "last_forward_ms": self.last_forward_ms,
                "last_handle_ms": self.last_handle_ms,
                "log_path": self.log_path,
                "status_path": self.status_path,
                "packet_path": self.packet_path,
                "ar_forward_mapping": AR_FORWARD_MAPPING,
            }

    def set_status(self, status, log_message=None):
        with self.lock:
            self.status = status
            input_packet = self.last_input_packet
            packet = self.last_packet
            ar_packet = self.last_ar_packet
            raw_line = self.last_datagram
        if log_message:
            self.log(log_message)
        self.write_pose_status(
            status,
            packet=packet,
            raw_line=raw_line,
            input_packet=input_packet,
            ar_packet=ar_packet,
        )

    def _udp_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, POSE_RX_BUFFER_BYTES)
        except Exception:
            pass
        sock.settimeout(0.5)
        self.input_sock = sock
        try:
            sock.bind((self.input_host, self.input_port))
            self.set_status(
                "udp waiting pose",
                f"Windows localization UDP ready on {self.input_host}:{self.input_port}",
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

                data, source, dropped = self._drain_latest_datagram(sock, data, source)
                recv_ts = time.time()
                source_text = f"{source[0]}:{source[1]}"
                packet, raw_text, error = parse_official_pose_datagram(data)
                with self.lock:
                    self.datagram_count += 1 + dropped
                    self.input_drop_count += dropped
                    self.last_datagram = raw_text
                    self.last_source = source_text
                    self.last_recv_ts = recv_ts
                    datagram_count = self.datagram_count
                if self.should_debug(datagram_count):
                    self.debug(
                        "UDP_RAW",
                        f"#{datagram_count} from={source_text} bytes={len(data)} dropped={dropped} text={repr(raw_text)}",
                        count=datagram_count,
                    )
                self.handle_packet(packet, raw_text, source_text, error, recv_ts=recv_ts)
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

    def _drain_latest_datagram(self, sock, data, source):
        if not POSE_RX_DRAIN_LATEST:
            return data, source, 0
        dropped = 0
        original_timeout = sock.gettimeout()
        try:
            sock.setblocking(False)
            while True:
                latest_data, latest_source = sock.recvfrom(65535)
                data, source = latest_data, latest_source
                dropped += 1
        except (BlockingIOError, socket.timeout):
            pass
        finally:
            sock.settimeout(original_timeout)
        return data, source, dropped

    def handle_packet(self, packet, raw_text, source_text, error="", recv_ts=None):
        start_perf = time.perf_counter()
        now = time.time() if recv_ts is None else float(recv_ts)
        if packet is None:
            with self.lock:
                self.invalid_count += 1
                self.status = "invalid pose datagram"
                invalid_count = self.invalid_count
            self.write_pose_status_throttled("invalid pose datagram", now, raw_line=raw_text, force=True)
            self.debug(
                "PARSE_FAIL",
                f"from={source_text} reason={error} raw={repr(raw_text)} invalid_count={invalid_count}",
                force=True,
            )
            self.log(f"[AR_POSE_BAD] from={source_text} reason={error} raw={repr(raw_text)}")
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
        forward_start = time.perf_counter()
        udp_ok, udp_fail = self.send_pose_payload(payload)
        forward_ms = (time.perf_counter() - forward_start) * 1000.0
        forward_ts = time.time()
        with self.lock:
            self.udp_send_count += udp_ok
            self.udp_fail_count += udp_fail
            self.last_forward_ts = forward_ts
            self.last_forward_ms = forward_ms
            udp_send_count = self.udp_send_count
            udp_fail_count = self.udp_fail_count
            invalid_count = self.invalid_count

        if self.should_debug(count):
            self.debug(
                "PARSE_OK",
                f"#{count} from={source_text} input={json.dumps(packet, separators=(',', ':'))} ar={payload.decode('utf-8')}",
                count=count,
            )

        json_ok = None
        live_json_count = self.live_json_count
        if self.should_write_live_json(now, force=count == 1):
            json_ok = self.write_live_pose_packet(ar_packet)
            with self.lock:
                if json_ok:
                    self.live_json_count += 1
                live_json_count = self.live_json_count
            if self.should_debug(count, force=not json_ok):
                self.debug(
                    "LIVE_JSON_OK" if json_ok else "LIVE_JSON_FAIL",
                    f"path={self.packet_path} live_json_count={live_json_count}",
                    count=count,
                    force=not json_ok,
                )

        if self.should_debug(count, force=udp_fail > 0):
            self.debug(
                "UDP_SEND",
                f"ok_targets={udp_ok} fail_targets={udp_fail} total_ok={udp_send_count} total_fail={udp_fail_count} forward_ms={forward_ms:.3f} targets={self.target_text}",
                count=count,
                force=udp_fail > 0,
            )
        self.write_pose_status_throttled(
            "receiving",
            now,
            packet=packet,
            raw_line=raw_text,
            input_packet=packet,
            ar_packet=ar_packet,
        )

        if self.debug_print_pose and count % self.debug_print_every_n == 0:
            pos = packet["pos"]
            ar_pos = ar_packet["pos"]
            euler = packet["euler"]
            self.log(
                "[AR_POSE_DEBUG] "
                f"#{count} from={source_text} "
                f"input_pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                f"ar_pos=({ar_pos[0]:.3f},{ar_pos[1]:.3f},{ar_pos[2]:.3f}) "
                f"yaw={euler[1]:.2f} "
                f"udp={self.target_text} "
                f"json={payload.decode('utf-8')}"
            )
        elif count == 1 or count % 60 == 0:
            self.log(f"AR pose forwarded #{count}: {payload.decode('utf-8')}")

        with self.lock:
            self.last_handle_ms = (time.perf_counter() - start_perf) * 1000.0

    def should_write_live_json(self, now, force=False):
        if force:
            self._next_live_json_ts = float(now) + POSE_LIVE_JSON_WRITE_INTERVAL
            return True
        if now < self._next_live_json_ts:
            return False
        self._next_live_json_ts = float(now) + POSE_LIVE_JSON_WRITE_INTERVAL
        return True

    def write_pose_status_throttled(self, status, now, force=False, **kwargs):
        if force:
            self._next_pose_status_ts = float(now) + POSE_STATUS_WRITE_INTERVAL
            self.write_pose_status(status, **kwargs)
            return True
        if now < self._next_pose_status_ts:
            return False
        self._next_pose_status_ts = float(now) + POSE_STATUS_WRITE_INTERVAL
        self.write_pose_status(status, **kwargs)
        return True

    def write_live_pose_packet(self, packet):
        if not packet:
            return False
        try:
            write_json_atomic(self.packet_path, packet)
            return True
        except Exception as exc:
            self.log(f"live pose packet write failed: {exc}")
            return False

    def send_pose_payload(self, payload):
        ok_count = 0
        fail_count = 0
        for target in self.targets:
            try:
                self.sock.sendto(payload, target)
                ok_count += 1
            except Exception as exc:
                fail_count += 1
                self.log(f"UDP pose send failed to {target[0]}:{target[1]}: {exc}")
        return ok_count, fail_count
