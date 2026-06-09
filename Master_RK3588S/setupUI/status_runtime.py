import json
import math
import os
import threading
import time


def read_json_file(path, default=None):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


class RuntimeStatusStore:
    """Writes and serves the shared pose/control status JSON used by WebUI and HUD debug tools."""

    def __init__(
        self,
        status_path,
        packet_path,
        debug_log_path,
        pose_convention,
        ar_forward_mapping,
        default_target,
        http_port,
        get_car_feedback,
        get_control_send_status,
        ai_status_provider=None,
    ):
        self.status_path = status_path
        self.packet_path = packet_path
        self.debug_log_path = debug_log_path
        self.pose_convention = pose_convention
        self.ar_forward_mapping = ar_forward_mapping
        self.default_target = default_target
        self.http_port = int(http_port)
        self.get_car_feedback = get_car_feedback
        self.get_control_send_status = get_control_send_status
        self.ai_status_provider = ai_status_provider or self._default_ai_status
        self.lock = threading.Lock()
        self.latest_control_status = None

    def _default_ai_status(self):
        return {
            "ok": False,
            "detector": "not initialized",
            "segmenter": "not initialized",
            "error": None,
        }

    def write_pose_status(
        self,
        status,
        packet=None,
        raw_line="",
        packet_count=0,
        invalid_count=0,
        target=None,
        input_packet=None,
        ar_packet=None,
    ):
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
            "debug_log": self.debug_log_path,
            "packet_json": self.packet_path,
            "pose_convention": self.pose_convention,
            "ar_forward_mapping": self.ar_forward_mapping,
            "car_feedback": self.get_car_feedback(),
        }
        self.write_status_json(info, "pose status")

    def write_status_json(self, info, label, control_info=None):
        try:
            with self.lock:
                if control_info is not None:
                    self.latest_control_status = dict(control_info)
                if self.latest_control_status is not None:
                    info["control"] = dict(self.latest_control_status)

                tmp_path = self.status_path + ".tmp"
                with open(tmp_path, "w", encoding="utf-8") as f:
                    json.dump(info, f, ensure_ascii=False, indent=2)
                os.replace(tmp_path, self.status_path)
        except Exception as exc:
            print(f"{label} write failed: {exc}")

    def write_runtime_status(
        self,
        pose_bridge,
        control_state,
        command_error,
        command_speed,
        command_state,
        command_flags,
        gamepad_status=None,
    ):
        pose_info = pose_bridge.snapshot()
        control_info = {
            "state": control_state,
            "track_error": float(command_error) if command_error is not None and math.isfinite(float(command_error)) else None,
            "target_speed": float(command_speed),
            "state_cmd": int(command_state),
            "flags": int(command_flags),
            "serial_send": self.get_control_send_status(),
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
            "target": pose_info.get("target", self.default_target),
            "udp_send_count": pose_info.get("udp_send_count", 0),
            "udp_fail_count": pose_info.get("udp_fail_count", 0),
            "pose_convention": self.pose_convention,
            "ar_forward_mapping": self.ar_forward_mapping,
            "control": control_info,
            "car_feedback": self.get_car_feedback(),
            "debug_log": self.debug_log_path,
            "packet_json": self.packet_path,
        }
        self.write_status_json(info, "runtime status", control_info=control_info)

    def current_payload(self):
        status = read_json_file(self.status_path, {})
        ar_packet = read_json_file(self.packet_path, None)
        if ar_packet is not None and not status.get("ar_packet"):
            status["ar_packet"] = ar_packet
        status.setdefault("status", "ar_receiver not ready")
        status.setdefault("packet_count", 0)
        status.setdefault("invalid_count", 0)
        status.setdefault("debug_log", self.debug_log_path)
        status.setdefault("packet_json", self.packet_path)
        status.setdefault("target", self.default_target)
        status.setdefault("pose_convention", self.pose_convention)
        status.setdefault("ar_forward_mapping", self.ar_forward_mapping)
        status["http_port"] = self.http_port
        status["ai"] = self.ai_status_provider()
        status["car_feedback"] = self.get_car_feedback()
        return status
