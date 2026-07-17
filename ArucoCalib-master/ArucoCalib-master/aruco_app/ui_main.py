"""
PyQt main interface for ArUco coordinate system
"""
import sys
import ctypes
from datetime import datetime
from pathlib import Path
import time
import cv2
import numpy as np
import math
import yaml
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QFileDialog,
                             QRadioButton, QButtonGroup, QTextEdit, QMessageBox,
                             QSpinBox, QCheckBox, QLineEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QPoint
from PyQt5.QtGui import QImage, QPixmap, QMouseEvent
from aruco_core import ArUcoDetector, CoordinateTransformer, get_config
from aruco_core.video_recorder import VideoRecorder
from aruco_core.udp_sender import UdpPoseSender, UdpGamepadControlSender


def make_run_ts():
    now = datetime.now()
    return now.strftime("%Y%m%d_%H%M%S") + f"_{now.microsecond // 1000:03d}"


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", ctypes.c_ushort),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", ctypes.c_ulong),
        ("Gamepad", XINPUT_GAMEPAD),
    ]


class XInputGamepadReader:
    """Minimal Windows XInput reader for RT/LT/LX manual-control capture."""

    BUTTON_BITS = {
        "A": 0x1000,
        "B": 0x2000,
        "X": 0x4000,
        "Y": 0x8000,
        "LB": 0x0100,
        "RB": 0x0200,
        "BACK": 0x0020,
        "START": 0x0010,
    }

    def __init__(self):
        self.dll = None
        self.dll_name = ""
        self.last_error = ""
        for name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
            try:
                self.dll = ctypes.windll.LoadLibrary(name)  # type: ignore[attr-defined]
                self.dll_name = name
                break
            except Exception as exc:
                self.last_error = str(exc)

    @staticmethod
    def _norm_thumb(value):
        if value >= 0:
            return max(-1.0, min(1.0, float(value) / 32767.0))
        return max(-1.0, min(1.0, float(value) / 32768.0))

    def read(self):
        if self.dll is None:
            return {
                "connected": False,
                "rt": 0.0,
                "lt": 0.0,
                "lx": 0.0,
                "buttons": {},
                "error": "XInput DLL unavailable",
            }

        for index in range(4):
            state = XINPUT_STATE()
            result = self.dll.XInputGetState(index, ctypes.byref(state))
            if result == 0:
                gp = state.Gamepad
                return {
                    "connected": True,
                    "index": index,
                    "backend": self.dll_name,
                    "rt": float(gp.bRightTrigger) / 255.0,
                    "lt": float(gp.bLeftTrigger) / 255.0,
                    "lx": self._norm_thumb(gp.sThumbLX),
                    "buttons": {
                        name: bool(gp.wButtons & bit)
                        for name, bit in self.BUTTON_BITS.items()
                    },
                    "error": "",
                }

        return {
            "connected": False,
            "rt": 0.0,
            "lt": 0.0,
            "lx": 0.0,
            "buttons": {},
            "error": "No XInput controller",
        }


class PygameGamepadReader:
    """Optional DirectInput/HID fallback for Bluetooth controllers."""

    BUTTON_NAMES = {
        0: "A",
        1: "B",
        2: "X",
        3: "Y",
        4: "LB",
        5: "RB",
        6: "BACK",
        7: "START",
    }

    def __init__(self):
        self.pg = None
        self.joystick = None
        self.last_error = ""
        try:
            import pygame  # type: ignore

            self.pg = pygame
            pygame.init()
            pygame.joystick.init()
        except Exception as exc:
            self.last_error = str(exc)

    @staticmethod
    def _clamp(value, lo=-1.0, hi=1.0):
        return max(lo, min(hi, float(value)))

    @staticmethod
    def _trigger_from_axis(value):
        value = PygameGamepadReader._clamp(value)
        return max(0.0, min(1.0, (value + 1.0) * 0.5))

    def refresh(self):
        self.joystick = None
        if self.pg is None:
            return
        try:
            self.pg.joystick.quit()
            self.pg.joystick.init()
            if self.pg.joystick.get_count() > 0:
                self.joystick = self.pg.joystick.Joystick(0)
                self.joystick.init()
        except Exception as exc:
            self.last_error = str(exc)
            self.joystick = None

    def read(self):
        if self.pg is None:
            return {
                "connected": False,
                "rt": 0.0,
                "lt": 0.0,
                "lx": 0.0,
                "buttons": {},
                "error": f"pygame unavailable: {self.last_error}",
            }
        try:
            self.pg.event.pump()
            if self.joystick is None:
                self.refresh()
            if self.joystick is None:
                return {
                    "connected": False,
                    "rt": 0.0,
                    "lt": 0.0,
                    "lx": 0.0,
                    "buttons": {},
                    "error": "No pygame controller",
                }

            axes = [
                self._clamp(self.joystick.get_axis(i))
                for i in range(self.joystick.get_numaxes())
            ]
            lx = axes[0] if len(axes) > 0 else 0.0
            lt = 0.0
            rt = 0.0
            if len(axes) > 5:
                lt = self._trigger_from_axis(axes[4])
                rt = self._trigger_from_axis(axes[5])
            elif len(axes) > 4:
                lt = self._trigger_from_axis(axes[2])
                rt = self._trigger_from_axis(axes[4])
            elif len(axes) > 2:
                combined = axes[2]
                rt = max(0.0, combined)
                lt = max(0.0, -combined)

            buttons = {}
            for i in range(self.joystick.get_numbuttons()):
                pressed = bool(self.joystick.get_button(i))
                buttons[f"BTN_{i}"] = pressed
                name = self.BUTTON_NAMES.get(i)
                if name:
                    buttons[name] = pressed
            return {
                "connected": True,
                "index": 0,
                "backend": "pygame",
                "name": self.joystick.get_name(),
                "rt": rt,
                "lt": lt,
                "lx": lx,
                "buttons": buttons,
                "error": "",
            }
        except Exception as exc:
            self.last_error = str(exc)
            self.joystick = None
            return {
                "connected": False,
                "rt": 0.0,
                "lt": 0.0,
                "lx": 0.0,
                "buttons": {},
                "error": f"pygame error: {exc}",
            }


class GamepadReader:
    """Read XInput first, then pygame for Bluetooth/DirectInput controllers."""

    def __init__(self):
        self.xinput = XInputGamepadReader()
        self.pygame = PygameGamepadReader()

    def read(self):
        x_state = self.xinput.read()
        if x_state.get("connected"):
            x_state["reader"] = "xinput"
            return x_state
        p_state = self.pygame.read()
        if p_state.get("connected"):
            p_state["reader"] = "pygame"
            return p_state
        p_state["error"] = f"{x_state.get('error', '')}; {p_state.get('error', '')}"
        return p_state


class ImageLabel(QLabel):
    """Custom QLabel for image display with mouse click handling"""

    clicked = pyqtSignal(int, int)  # Signal emitted when image is clicked (x, y)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(640, 480)
        self.setStyleSheet("border: 1px solid gray;")
        self.original_image = None
        self.scale_factor = 1.0
        self.offset_x = 0
        self.offset_y = 0

    def set_image(self, image):
        """Set the image to display"""
        self.original_image = image.copy()
        self._update_display()

    def _update_display(self):
        """Update the displayed image"""
        if self.original_image is None:
            return

        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w

        # Create QImage
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

        # Scale to fit label while maintaining aspect ratio
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(
            self.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )

        self.setPixmap(scaled_pixmap)

        # Calculate scale factor and offset
        pixmap_rect = scaled_pixmap.rect()
        label_rect = self.rect()

        # Calculate scale factor (ratio between scaled and original)
        self.scale_factor_x = scaled_pixmap.width() / w
        self.scale_factor_y = scaled_pixmap.height() / h

        self.offset_x = (label_rect.width() - pixmap_rect.width()) // 2
        self.offset_y = (label_rect.height() - pixmap_rect.height()) // 2

    def mousePressEvent(self, event: QMouseEvent):
        """Handle mouse click events"""
        if self.original_image is None:
            return

        # Get click position relative to label
        label_x = event.x()
        label_y = event.y()

        # Convert to image coordinates
        pixmap = self.pixmap()
        if pixmap is None:
            return

        pixmap_rect = pixmap.rect()
        pixmap_rect.moveTopLeft(
            QPoint(self.offset_x, self.offset_y)
        )

        if pixmap_rect.contains(label_x, label_y):
            # Calculate pixel coordinates in original image
            rel_x = label_x - self.offset_x
            rel_y = label_y - self.offset_y

            # Convert from scaled pixmap coordinates to original image coordinates
            img_x = int(rel_x / self.scale_factor_x)
            img_y = int(rel_y / self.scale_factor_y)

            # Emit signal with pixel coordinates
            self.clicked.emit(img_x, img_y)

    def resizeEvent(self, event):
        """Handle resize events"""
        super().resizeEvent(event)
        self._update_display()


class MainWindow(QMainWindow):
    """Main application window"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ArucoCalib")
        self.setGeometry(80, 60, 1500, 900)

        # Initialize components
        self.detector = ArUcoDetector()
        self.transformer = CoordinateTransformer()
        self.cfg = get_config()
        self.settings_path = self._editable_config_path()

        # Video capture
        self.cap = None
        self.timer = QTimer()
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.update_frame)
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self._send_gamepad_control)
        self.is_detecting = False

        # Session-level timestamp for optional MP4 recordings.
        self.script_start_ts = make_run_ts()
        app_root = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parents[1]
        self.runs_root_dir = app_root / "runs"

        # Raw video recording (mp4)
        self.is_recording = False
        self.video_recorder = None
        self.source_fps = 30.0

        # mp4 file path for "video detection" mode
        self.video_file_path = None

        # Current image
        self.current_image = None
        self.last_clicked_pixel = None
        self.last_clicked_world = None

        self.enable_trace = False
        # Trajectory history window (ms). Configure in config.yaml -> ui.trace_window_ms.
        self.trace_window_ms = int(getattr(self.cfg, "TRACE_WINDOW_MS", 500))
        self.trace_max_len = 15
        self.marker_traces = {}  # marker_id -> list[(x,y)]

        # Vehicle marker info (vehicle_id from config)
        self.vehicle_id = int(getattr(self.cfg, "VEHICLE_ID", 0))
        self.last_vehicle_center_px = None  # (cx, cy)
        self.last_vehicle_center_world = None  # (wx, wy) mm
        self.last_vehicle_yaw_deg = None  # float deg
        self.last_vehicle_corners_px = None
        self.vehicle_track_center_px = None  # persistent center used only to disambiguate candidates
        self.last_output_pose = None  # (x_m, z_m, yaw_deg)
        self.last_output_pose_ts = 0.0
        self.last_pose_live = False
        self.pose_hold_timeout_s = 0.5
        self.filtered_output_pose = None  # (x_m, z_m, yaw_deg)
        self.filtered_output_pose_ts = 0.0
        # Keep physical detections as a list: fixed and vehicle tags may share an ID.
        self.locked_fixed_markers = []  # [{id, center, corners, match_radius}, ...]
        self.rectified_detection_geometry = None  # (image->rectified, rectified->image, size)
        self.last_detection_rectified = False
        self.last_udp_ok = False
        self.pose_seq = 0
        self.pose_history = []

        self.pose_sender = UdpPoseSender(
            target_ip=getattr(self.cfg, "UDP_TARGET_IP", "127.0.0.1"),
            target_port=getattr(self.cfg, "UDP_TARGET_PORT", 9005),
            enabled=getattr(self.cfg, "UDP_ENABLED", False),
        )
        self.gamepad_reader = GamepadReader()
        self.gamepad_sender = UdpGamepadControlSender(
            target_ip=getattr(self.cfg, "UDP_TARGET_IP", "127.0.0.1"),
            target_port=getattr(self.cfg, "GAMEPAD_TARGET_PORT", 9010),
        )
        self.last_gamepad_status = "Gamepad: off"
        self.last_gamepad_ok = False
        self.last_gamepad_packet_ts = 0.0

        # Setup UI
        self.setup_ui()

    @staticmethod
    def _norm_angle_deg(a: float) -> float:
        """Normalize to (-180, 180]."""
        x = (float(a) + 180.0) % 360.0 - 180.0
        if x <= -180.0:
            x += 360.0
        return x

    @staticmethod
    def _clamp_float(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(value)))

    def _reset_pose_filter(self):
        self.filtered_output_pose = None
        self.filtered_output_pose_ts = 0.0

    @staticmethod
    def _editable_config_path():
        if getattr(sys, "frozen", False):
            return Path(sys.executable).resolve().parent / "config.yaml"
        return Path(__file__).resolve().parents[1] / "config.yaml"

    def _read_editable_config(self):
        source_path = self.settings_path
        if not source_path.exists():
            source_path = Path(getattr(self.cfg, "CONFIG_PATH", source_path))
        if not source_path.exists():
            return {}
        with open(source_path, "r", encoding="utf-8") as config_file:
            return yaml.safe_load(config_file) or {}

    def _save_runtime_settings(self):
        try:
            data = self._read_editable_config()
            camera = data.setdefault("camera", {})
            camera["index"] = int(self.camera_index_spin.value())
            camera["backend"] = "DSHOW"
            camera["width"] = 1920
            camera["height"] = 1200
            camera["fps"] = 60
            camera["fourcc"] = "YUY2"
            camera["mirror"] = bool(self.camera_mirror_chk.isChecked())

            data["udp"] = {
                "enabled": bool(self.udp_enabled_chk.isChecked()),
                "target_ip": self.udp_ip_edit.text().strip(),
                "target_port": 9005,
            }
            data["gamepad_control"] = {
                "target_port": int(getattr(self.cfg, "GAMEPAD_TARGET_PORT", 9010)),
                "max_speed_mps": float(getattr(self.cfg, "GAMEPAD_MAX_SPEED_MPS", 1.0)),
                "steer_error_scale": float(getattr(self.cfg, "GAMEPAD_STEER_ERROR_SCALE", 210.0)),
            }

            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            temp_path = self.settings_path.with_name(self.settings_path.name + ".tmp")
            with open(temp_path, "w", encoding="utf-8") as config_file:
                yaml.safe_dump(data, config_file, sort_keys=False, allow_unicode=True)
            temp_path.replace(self.settings_path)
            return True, f"saved {self.settings_path.name}"
        except Exception as exc:
            return False, f"SAVE FAILED: {exc}"

    def _update_vehicle_panel(self, detected: bool):
        vid = self.vehicle_id
        if not detected or self.last_vehicle_center_px is None:
            self.vehicle_text.setText(f"vehicle_id: {vid}\nStatus: Not detected")
            return

        cx, cy = self.last_vehicle_center_px
        lines = [f"vehicle_id: {vid}", f"center_px: ({cx:.1f}, {cy:.1f})"]
        lines.append(f"detection: {'rectified' if self.last_detection_rectified else 'direct'}")

        output_pose = self.last_output_pose if self.last_pose_live else None
        if output_pose is None:
            lines.append("center_world(m): Not calibrated")
            lines.append("yaw_deg: Not calibrated")
        else:
            x_m, z_m, yaw_deg = output_pose
            pose_label = "filtered_pose(m)" if bool(getattr(self.cfg, "POSE_FILTER_ENABLED", True)) else "output_pose(m)"
            lines.append(f"{pose_label}: ({x_m:.3f}, {z_m:.3f})")
            lines.append(f"yaw_deg: {yaw_deg:.2f}")

        self.vehicle_text.setText("\n".join(lines))

    def _update_udp_status(self, suffix=""):
        state = "enabled" if self.pose_sender.enabled else "disabled"
        text = (
            f"UDP {state} -> {self.pose_sender.target_ip}:9005 "
            f"sent={self.pose_sender.sent_count} fail={self.pose_sender.fail_count}"
        )
        if suffix:
            text += f" | {suffix}"
        self.udp_status_label.setText(text)

    def apply_udp_settings(self, persist=False):
        self.pose_sender.configure(
            target_ip=self.udp_ip_edit.text().strip(),
            target_port=9005,
            enabled=self.udp_enabled_chk.isChecked(),
        )
        self.gamepad_sender.configure(
            target_ip=self.udp_ip_edit.text().strip(),
            target_port=int(getattr(self.cfg, "GAMEPAD_TARGET_PORT", 9010)),
        )
        save_message = ""
        if persist:
            _, save_message = self._save_runtime_settings()
        self._update_udp_status(save_message)

    @staticmethod
    def _apply_deadzone(value, deadzone=0.08):
        value = max(-1.0, min(1.0, float(value)))
        deadzone = max(0.0, min(0.95, float(deadzone)))
        if abs(value) <= deadzone:
            return 0.0
        scaled = (abs(value) - deadzone) / (1.0 - deadzone)
        return math.copysign(scaled, value)

    def _send_gamepad_disabled(self):
        ok = self.gamepad_sender.send_control(
            gamepad_mode=False,
            target_speed=0.0,
            track_error=0.0,
            state_cmd=1,
            flags=0,
            safe_stop=False,
            rt=0.0,
            lt=0.0,
            lx=0.0,
            connected=False,
        )
        self.last_gamepad_ok = ok
        self.last_gamepad_status = "Gamepad: disabled -> vision"
        self.last_gamepad_packet_ts = time.monotonic()
        self._update_gamepad_status()

    def on_gamepad_mode_changed(self, _state):
        if not self.gamepad_mode_chk.isChecked():
            self.gamepad_timer.stop()
            self._send_gamepad_disabled()
            return
        self._sync_gamepad_timer()
        self._send_gamepad_control(force=True)
        self.gamepad_timer.start()

    def _gamepad_interval_ms(self):
        fps = float(self.source_fps) if self.source_fps else float(getattr(self.cfg, "CAMERA_FPS", 60.0))
        if fps <= 0.0:
            fps = 60.0
        return max(1, int(round(1000.0 / fps)))

    def _sync_gamepad_timer(self):
        interval_ms = self._gamepad_interval_ms()
        if self.gamepad_timer.interval() != interval_ms:
            self.gamepad_timer.setInterval(interval_ms)

    def _send_gamepad_control(self, force=False):
        if not hasattr(self, "gamepad_mode_chk") or not self.gamepad_mode_chk.isChecked():
            return False

        state = self.gamepad_reader.read()
        connected = bool(state.get("connected"))
        rt = float(state.get("rt", 0.0))
        lt = float(state.get("lt", 0.0))
        lx_raw = float(state.get("lx", 0.0))
        lx = self._apply_deadzone(lx_raw, 0.08)
        buttons = state.get("buttons", {}) or {}
        b_stop = bool(buttons.get("B", False))
        max_speed = max(0.0, float(getattr(self.cfg, "GAMEPAD_MAX_SPEED_MPS", 1.0)))
        steer_scale = float(getattr(self.cfg, "GAMEPAD_STEER_ERROR_SCALE", 210.0))
        safe_stop = (not connected) or b_stop

        if safe_stop:
            target_speed = 0.0
            track_error = 0.0
            state_cmd = 7
            flags = 0
        else:
            target_speed = (max(0.0, min(1.0, rt)) - max(0.0, min(1.0, lt))) * max_speed
            track_error = lx * steer_scale
            state_cmd = 1
            flags = 0x01

        ok = self.gamepad_sender.send_control(
            gamepad_mode=True,
            target_speed=target_speed,
            track_error=track_error,
            state_cmd=state_cmd,
            flags=flags,
            safe_stop=safe_stop,
            rt=rt,
            lt=lt,
            lx=lx,
            b=b_stop,
            connected=connected,
        )
        self.last_gamepad_ok = ok
        if not connected:
            error_text = str(state.get("error", ""))[:36]
            self.last_gamepad_status = f"Gamepad: ON no controller -> SAFE_STOP fail={self.gamepad_sender.fail_count} {error_text}"
        elif safe_stop:
            backend = state.get("reader") or state.get("backend", "?")
            self.last_gamepad_status = f"Gamepad: SAFE_STOP {backend} B={int(b_stop)} sent={self.gamepad_sender.sent_count}"
        else:
            backend = state.get("reader") or state.get("backend", "?")
            self.last_gamepad_status = (
                f"Gamepad: TRACK {backend} v={target_speed:.2f} err={track_error:.0f} "
                f"RT={rt:.2f} LT={lt:.2f} LX={lx:.2f} sent={self.gamepad_sender.sent_count}"
            )
        if self.gamepad_sender.last_error:
            self.last_gamepad_status += f" err={self.gamepad_sender.last_error[:40]}"
        self.last_gamepad_packet_ts = time.monotonic()
        self._update_gamepad_status()
        return ok

    def _update_gamepad_status(self):
        if hasattr(self, "gamepad_status_label"):
            self.gamepad_status_label.setText(self.last_gamepad_status)

    def apply_camera_settings(self, persist=False):
        save_message = ""
        if persist:
            _, save_message = self._save_runtime_settings()

        if self.cap is not None:
            self.stop_camera()
        self.start_camera()
        if save_message:
            self.camera_status_label.setText(f"{self.camera_status_label.text()} | {save_message}")

    def on_mirror_changed(self, _state):
        _, message = self._save_runtime_settings()
        if hasattr(self, "camera_status_label"):
            state = "on" if self.camera_mirror_chk.isChecked() else "off"
            self.camera_status_label.setText(f"Camera mirror: {state} | {message}")

    @staticmethod
    def _camera_backend_flag():
        return cv2.CAP_DSHOW

    def _set_cap_prop(self, prop, value):
        if self.cap is None:
            return
        try:
            self.cap.set(prop, float(value))
        except Exception:
            pass

    def _apply_capture_properties(self):
        if self.cap is None:
            return
        self._set_cap_prop(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUY2"))
        self._set_cap_prop(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self._set_cap_prop(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
        self._set_cap_prop(cv2.CAP_PROP_FPS, 60)
        # Prefer the newest camera frame if processing ever falls behind.
        self._set_cap_prop(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _current_output_pose(self):
        if self.last_vehicle_center_world is None or self.last_vehicle_yaw_deg is None:
            return None
        wx_mm, wy_mm = self.last_vehicle_center_world
        x_m = float(wx_mm) * float(getattr(self.cfg, "OUTPUT_COORD_SCALE", 0.001)) * float(getattr(self.cfg, "OUTPUT_X_SIGN", 1.0))
        z_m = float(wy_mm) * float(getattr(self.cfg, "OUTPUT_COORD_SCALE", 0.001)) * float(getattr(self.cfg, "OUTPUT_Z_SIGN", 1.0))
        yaw_deg = self._norm_angle_deg(
            float(self.last_vehicle_yaw_deg) * float(getattr(self.cfg, "OUTPUT_YAW_SIGN", 1.0))
            + float(getattr(self.cfg, "OUTPUT_YAW_OFFSET_DEG", 0.0))
        )
        return x_m, z_m, yaw_deg

    def _apply_pose_filter(self, raw_pose, now_s: float):
        if raw_pose is None:
            return None

        if not bool(getattr(self.cfg, "POSE_FILTER_ENABLED", True)):
            self.filtered_output_pose = raw_pose
            self.filtered_output_pose_ts = now_s
            return raw_pose

        pos_alpha = self._clamp_float(getattr(self.cfg, "POSE_FILTER_POSITION_ALPHA", 0.35), 0.01, 1.0)
        yaw_alpha = self._clamp_float(getattr(self.cfg, "POSE_FILTER_YAW_ALPHA", 0.35), 0.01, 1.0)
        reset_gap_s = max(0.0, float(getattr(self.cfg, "POSE_FILTER_RESET_GAP_S", 0.35)))

        if (
            self.filtered_output_pose is None
            or self.filtered_output_pose_ts <= 0.0
            or (reset_gap_s > 0.0 and now_s - self.filtered_output_pose_ts > reset_gap_s)
        ):
            self.filtered_output_pose = raw_pose
            self.filtered_output_pose_ts = now_s
            return raw_pose

        prev_x, prev_z, prev_yaw = self.filtered_output_pose
        raw_x, raw_z, raw_yaw = raw_pose
        x_m = prev_x + pos_alpha * (raw_x - prev_x)
        z_m = prev_z + pos_alpha * (raw_z - prev_z)
        yaw_delta = self._norm_angle_deg(raw_yaw - prev_yaw)
        yaw_deg = self._norm_angle_deg(prev_yaw + yaw_alpha * yaw_delta)

        self.filtered_output_pose = (x_m, z_m, yaw_deg)
        self.filtered_output_pose_ts = now_s
        return self.filtered_output_pose

    def _send_current_pose(self):
        raw_pose = self._current_output_pose()
        if raw_pose is None:
            self.last_pose_live = False
            self.last_udp_ok = False
            return
        now_s = time.monotonic()
        pose = self._apply_pose_filter(raw_pose, now_s)
        self.last_output_pose = pose
        self.last_output_pose_ts = now_s
        self.last_pose_live = True
        x_m, z_m, yaw_deg = pose
        self.pose_seq += 1
        self.last_udp_ok = self.pose_sender.send_pose(
            x_m,
            z_m,
            yaw_deg,
            height_m=float(getattr(self.cfg, "OUTPUT_HEIGHT_M", 0.0)),
        )
        self.pose_history.append((x_m, z_m, yaw_deg))
        if len(self.pose_history) > 3000:
            self.pose_history = self.pose_history[-3000:]
        self._update_udp_status()

    def _display_pose_state(self):
        if self.last_output_pose is None or self.last_output_pose_ts <= 0.0:
            return None, "WAIT", None
        age_s = time.monotonic() - self.last_output_pose_ts
        if self.last_pose_live:
            return self.last_output_pose, "LIVE", age_s
        if age_s <= self.pose_hold_timeout_s:
            return self.last_output_pose, "HOLD", age_s
        return None, "WAIT", age_s

    def _draw_pose_hud(self, image_bgr):
        pose, pose_state, age_s = self._display_pose_state()
        sent_now = bool(self.last_pose_live and self.last_udp_ok)
        if pose is not None:
            x_m, z_m, yaw_deg = pose
            state_text = pose_state if age_s is None else f"{pose_state} {age_s:.2f}s"
            filter_text = "on" if bool(getattr(self.cfg, "POSE_FILTER_ENABLED", True)) else "off"
            lines = [
                f"seq={self.pose_seq} sent={1 if sent_now else 0} pose={state_text} filt={filter_text}",
                f"x={x_m:.3f}m z={z_m:.3f}m yaw={yaw_deg:.2f}deg",
            ]
        else:
            lines = [f"seq={self.pose_seq} sent=0", "pose: waiting"]

        if self.pose_sender.last_error:
            lines.append(f"udp error: {self.pose_sender.last_error[:50]}")
        if hasattr(self, "gamepad_mode_chk") and self.gamepad_mode_chk.isChecked():
            lines.append(self.last_gamepad_status[:58])

        y = 30
        for line in lines:
            cv2.putText(image_bgr, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)
            y += 30
        return image_bgr

    def _draw_trajectory_inset(self, image_bgr):
        h, w = image_bgr.shape[:2]
        inset_w = min(360, max(240, w // 4))
        inset_h = min(270, max(190, h // 4))
        x0 = w - inset_w - 16
        y0 = 16
        x1 = w - 16
        y1 = y0 + inset_h

        overlay = image_bgr.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (10, 14, 12), -1)
        cv2.addWeighted(overlay, 0.82, image_bgr, 0.18, 0, image_bgr)
        cv2.rectangle(image_bgr, (x0, y0), (x1, y1), (110, 125, 115), 1)
        cv2.putText(image_bgr, "UDP trajectory", (x0 + 10, y0 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 220, 210), 1, cv2.LINE_AA)

        world_points = list(getattr(self.cfg, "WORLD_COORDINATES", {}).values())
        coord_scale = float(getattr(self.cfg, "OUTPUT_COORD_SCALE", 0.001))
        x_sign = float(getattr(self.cfg, "OUTPUT_X_SIGN", 1.0))
        z_sign = float(getattr(self.cfg, "OUTPUT_Z_SIGN", 1.0))
        field_points = [
            (float(p[0]) * coord_scale * x_sign, float(p[1]) * coord_scale * z_sign)
            for p in world_points
        ]
        if not field_points:
            field_points.append((0.0, 0.30))

        field_min_x = min(p[0] for p in field_points)
        field_max_x = max(p[0] for p in field_points)
        field_min_z = min(p[1] for p in field_points)
        field_max_z = max(p[1] for p in field_points)
        min_x, max_x = field_min_x, field_max_x
        min_z, max_z = field_min_z, field_max_z
        span_x = max(0.5, max_x - min_x)
        span_z = max(0.5, max_z - min_z)
        margin_m = max(0.15, max(span_x, span_z) * 0.06)
        min_x -= margin_m
        max_x += margin_m
        min_z -= margin_m
        max_z += margin_m

        left, right = x0 + 18, x1 - 18
        top, bottom = y0 + 38, y1 - 44
        avail_w = max(1, right - left)
        avail_h = max(1, bottom - top)
        field_aspect = max(1e-9, max_x - min_x) / max(1e-9, max_z - min_z)
        avail_aspect = avail_w / float(avail_h)
        if avail_aspect > field_aspect:
            plot_w = int(round(avail_h * field_aspect))
            left += (avail_w - plot_w) // 2
            right = left + plot_w
        else:
            plot_h = int(round(avail_w / field_aspect))
            top += (avail_h - plot_h) // 2
            bottom = top + plot_h

        def map_point(px, pz):
            # Formal AR field view: bottom-right reference is (X=0, Z=field_min_z), +X left, +Z up.
            sx = right - int((float(px) - min_x) / max(1e-9, max_x - min_x) * (right - left))
            sy = bottom - int((float(pz) - min_z) / max(1e-9, max_z - min_z) * (bottom - top))
            return sx, sy

        axis_color = (150, 165, 155)
        field_rect = np.array(
            [
                map_point(field_min_x, field_min_z),
                map_point(field_max_x, field_min_z),
                map_point(field_max_x, field_max_z),
                map_point(field_min_x, field_max_z),
            ],
            dtype=np.int32,
        ).reshape(-1, 1, 2)
        cv2.polylines(image_bgr, [field_rect], True, (80, 95, 85), 1, cv2.LINE_AA)

        grid_color = (55, 70, 62)
        gx = math.ceil(field_min_x)
        while gx < field_max_x:
            cv2.line(image_bgr, map_point(gx, field_min_z), map_point(gx, field_max_z), grid_color, 1, cv2.LINE_AA)
            gx += 1
        gz = math.ceil(field_min_z)
        while gz < field_max_z:
            cv2.line(image_bgr, map_point(field_min_x, gz), map_point(field_max_x, gz), grid_color, 1, cv2.LINE_AA)
            gz += 1

        axis_origin = map_point(0.0, field_min_z)
        axis_x_end = map_point(field_max_x, field_min_z)
        axis_z_end = map_point(0.0, field_max_z)
        cv2.arrowedLine(image_bgr, axis_origin, axis_x_end, axis_color, 1, cv2.LINE_AA, tipLength=0.06)
        cv2.arrowedLine(image_bgr, axis_origin, axis_z_end, axis_color, 1, cv2.LINE_AA, tipLength=0.08)
        cv2.circle(image_bgr, axis_origin, 3, axis_color, -1, cv2.LINE_AA)
        cv2.putText(image_bgr, "+X", (axis_x_end[0] + 2, axis_x_end[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, axis_color, 1, cv2.LINE_AA)
        cv2.putText(image_bgr, "+Z", (axis_z_end[0] - 30, axis_z_end[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, axis_color, 1, cv2.LINE_AA)
        cv2.putText(image_bgr, "BR", (axis_origin[0] - 10, axis_origin[1] + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.38, axis_color, 1, cv2.LINE_AA)
        field_label = f"{field_max_x - field_min_x:.1f}m x {field_max_z - field_min_z:.1f}m"
        cv2.putText(image_bgr, field_label, (x0 + 10, y0 + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (170, 185, 175), 1, cv2.LINE_AA)

        if len(self.pose_history) >= 2:
            pts = np.array([map_point(px, pz) for px, pz, _ in self.pose_history], dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(image_bgr, [pts], False, (0, 220, 255), 2, cv2.LINE_AA)

        display_pose, pose_state, _age_s = self._display_pose_state()
        if display_pose is not None:
            px, pz, yaw = display_pose
            center = map_point(px, pz)
            length = max(14, inset_w // 16)
            angle = math.radians(yaw)
            tip = (center[0] - int(math.cos(angle) * length), center[1] - int(math.sin(angle) * length))
            cv2.circle(image_bgr, center, 4, (0, 80, 255), -1)
            cv2.arrowedLine(image_bgr, center, tip, (0, 80, 255), 2, cv2.LINE_AA, tipLength=0.35)
            footer = f"seq={self.pose_seq} {pose_state} x={px:.3f} z={pz:.3f} yaw={yaw:.1f}deg"
            cv2.putText(image_bgr, footer, (x0 + 10, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (210, 220, 210), 1, cv2.LINE_AA)

        return image_bgr

    @staticmethod
    def _fixed_marker_match_radius(marker_corners):
        """Pixel gate used to recognize a still-visible locked physical tag."""
        marker_corners = np.asarray(marker_corners, dtype=np.float32)
        sides = np.linalg.norm(marker_corners - np.roll(marker_corners, -1, axis=0), axis=1)
        mean_side = float(np.mean(sides)) if sides.size else 0.0
        return max(10.0, min(50.0, mean_side * 0.6))

    def _select_vehicle_marker(self, marker_instances):
        """Select the moving vehicle instance even when a fixed tag has the same ID."""
        candidates = [
            marker for marker in marker_instances
            if int(marker["id"]) == int(self.vehicle_id)
        ]
        if not candidates:
            return None

        locked_same_id = [
            marker for marker in self.locked_fixed_markers
            if int(marker["id"]) == int(self.vehicle_id)
        ]
        unmatched_indices = set(range(len(candidates)))

        if locked_same_id:
            pair_distances = []
            for locked_index, locked in enumerate(locked_same_id):
                locked_center = np.asarray(locked["center"], dtype=np.float32)
                for candidate_index, candidate in enumerate(candidates):
                    distance = float(
                        np.linalg.norm(np.asarray(candidate["center"], dtype=np.float32) - locked_center)
                    )
                    pair_distances.append((distance, locked_index, candidate_index))

            matched_locked = set()
            matched_candidates = set()
            always_reserve_fixed = len(candidates) > len(locked_same_id)
            for distance, locked_index, candidate_index in sorted(pair_distances):
                if locked_index in matched_locked or candidate_index in matched_candidates:
                    continue
                radius = float(locked_same_id[locked_index]["match_radius"])
                if not always_reserve_fixed and distance > radius:
                    continue
                matched_locked.add(locked_index)
                matched_candidates.add(candidate_index)
            unmatched_indices.difference_update(matched_candidates)

        remaining = [candidates[index] for index in sorted(unmatched_indices)]
        if not remaining:
            return None
        if self.vehicle_track_center_px is None or len(remaining) == 1:
            return remaining[0]

        previous = np.asarray(self.vehicle_track_center_px, dtype=np.float32)
        return min(
            remaining,
            key=lambda marker: float(
                np.linalg.norm(np.asarray(marker["center"], dtype=np.float32) - previous)
            ),
        )

    def _prepare_rectified_detection(self):
        """Cache a uniform-scale field view derived from the four locked corners."""
        self.rectified_detection_geometry = None
        if not bool(getattr(self.cfg, "VEHICLE_RECTIFICATION_ENABLED", True)):
            return False
        if not self.transformer.get_calibration_status():
            return False
        image_to_world = self.transformer.homography_matrix
        if image_to_world is None:
            return False

        world_points = np.asarray(
            list(getattr(self.cfg, "WORLD_COORDINATES", {}).values()),
            dtype=np.float64,
        )
        if world_points.shape != (4, 2) or not np.all(np.isfinite(world_points)):
            return False

        min_x, min_z = np.min(world_points, axis=0)
        max_x, max_z = np.max(world_points, axis=0)
        margin_mm = float(getattr(self.cfg, "VEHICLE_RECTIFICATION_MARGIN_MM", 150.0))
        scale = float(getattr(self.cfg, "VEHICLE_RECTIFICATION_SCALE_PX_PER_MM", 0.30))
        span_x = max(1.0, float(max_x - min_x + 2.0 * margin_mm))
        span_z = max(1.0, float(max_z - min_z + 2.0 * margin_mm))
        max_dimension = int(getattr(self.cfg, "VEHICLE_RECTIFICATION_MAX_DIMENSION", 1600))
        scale = min(scale, max_dimension / max(span_x, span_z))
        output_size = (
            max(64, int(round(span_x * scale))),
            max(64, int(round(span_z * scale))),
        )

        # Existing axes: +X points image-left and +Z points image-up.
        world_to_rectified = np.asarray(
            [
                [-scale, 0.0, (max_x + margin_mm) * scale],
                [0.0, -scale, (max_z + margin_mm) * scale],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        image_to_rectified = world_to_rectified @ np.asarray(image_to_world, dtype=np.float64)
        try:
            rectified_to_image = np.linalg.inv(image_to_rectified)
        except np.linalg.LinAlgError:
            return False
        if not np.all(np.isfinite(rectified_to_image)):
            return False

        self.rectified_detection_geometry = (
            image_to_rectified,
            rectified_to_image,
            output_size,
        )
        return True

    def _detect_rectified_marker_instances(self, image):
        """Run one synchronous detection pass on the uniform-scale field image."""
        if self.rectified_detection_geometry is None and not self._prepare_rectified_detection():
            return None
        image_to_rectified, rectified_to_image, output_size = self.rectified_detection_geometry
        rectified = cv2.warpPerspective(
            image,
            image_to_rectified,
            output_size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255),
        )
        corners, ids, _ = self.detector.detect_markers(rectified)
        rectified_instances = self.detector.get_marker_instances(corners, ids)

        mapped_instances = []
        for marker in rectified_instances:
            marker_corners = cv2.perspectiveTransform(
                np.asarray(marker["corners"], dtype=np.float32).reshape(-1, 1, 2),
                rectified_to_image,
            ).reshape(4, 2)
            if not np.all(np.isfinite(marker_corners)):
                continue
            mapped_instances.append(
                {
                    "id": int(marker["id"]),
                    "index": int(marker["index"]),
                    "corners": marker_corners.astype(np.float32),
                    "center": np.mean(marker_corners, axis=0).astype(np.float32),
                }
            )
        return mapped_instances

    @staticmethod
    def _draw_marker_box(image_bgr, marker_corners, marker_id, color, suffix=""):
        corners = np.array(marker_corners, dtype=np.float32)
        if corners.shape != (4, 2):
            return image_bgr
        pts = np.round(corners).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(image_bgr, [pts], True, color, 2, cv2.LINE_AA)
        center = np.mean(corners, axis=0)
        c0 = corners[0]
        cv2.circle(image_bgr, (int(round(c0[0])), int(round(c0[1]))), 3, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(image_bgr, (int(round(center[0])), int(round(center[1]))), 3, color, -1, cv2.LINE_AA)
        label = f"id={int(marker_id)}{suffix}"
        label_pos = (int(round(center[0])) + 4, int(round(center[1])) - 4)
        cv2.putText(image_bgr, label, label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return image_bgr

    def _draw_marker_overlays(self, image_bgr, corners, ids):
        marker_instances = self.detector.get_marker_instances(corners, ids)

        if self.transformer.get_calibration_status() and self.locked_fixed_markers:
            for marker in self.locked_fixed_markers:
                self._draw_marker_box(
                    image_bgr,
                    marker["corners"],
                    marker["id"],
                    (0, 190, 0),
                    " lock",
                )
        else:
            for marker in marker_instances:
                self._draw_marker_box(
                    image_bgr,
                    marker["corners"],
                    marker["id"],
                    (0, 190, 0),
                )

        if self.last_vehicle_corners_px is not None:
            self._draw_marker_box(
                image_bgr,
                self.last_vehicle_corners_px,
                self.vehicle_id,
                (255, 120, 0),
                " vehicle",
            )

        return image_bgr

    def _draw_vehicle_arrow(self, image_bgr):
        if self.last_vehicle_corners_px is None or self.last_vehicle_center_px is None:
            return image_bgr
        center = np.array(self.last_vehicle_center_px, dtype=np.float32)
        front = None
        if (
            self.transformer.get_calibration_status()
            and self.last_vehicle_center_world is not None
            and self.last_output_pose is not None
        ):
            wx_mm, wy_mm = self.last_vehicle_center_world
            yaw_deg = float(self.last_output_pose[2])
            yaw_rad = math.radians(yaw_deg)
            x_sign = float(getattr(self.cfg, "OUTPUT_X_SIGN", 1.0))
            z_sign = float(getattr(self.cfg, "OUTPUT_Z_SIGN", 1.0))
            heading_len_mm = 260.0
            tip_world = (
                float(wx_mm) + math.cos(yaw_rad) / max(1e-9, x_sign) * heading_len_mm,
                float(wy_mm) + math.sin(yaw_rad) / max(1e-9, z_sign) * heading_len_mm,
            )
            tip_px = self.transformer.world_to_pixel(tip_world[0], tip_world[1])
            if tip_px is not None:
                front = np.array(tip_px, dtype=np.float32)

        if front is None:
            corners = np.array(self.last_vehicle_corners_px, dtype=np.float32)
            front = (corners[1] + corners[2]) * 0.5

        cv2.circle(image_bgr, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
        cv2.arrowedLine(
            image_bgr,
            (int(center[0]), int(center[1])),
            (int(front[0]), int(front[1])),
            (0, 0, 255),
            3,
            tipLength=0.25,
        )
        return image_bgr

    def setup_ui(self):
        """Build a video-first runtime interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(5)

        self.image_label = ImageLabel()
        self.image_label.clicked.connect(self.on_image_clicked)
        main_layout.addWidget(self.image_label, 1)

        source_row = QHBoxLayout()
        source_row.addWidget(QLabel("Camera"))
        self.camera_index_spin = QSpinBox()
        self.camera_index_spin.setRange(0, 16)
        self.camera_index_spin.setValue(int(getattr(self.cfg, "CAMERA_INDEX", 0)))
        self.camera_index_spin.setFixedWidth(58)
        source_row.addWidget(self.camera_index_spin)
        self.btn_apply_camera = QPushButton("Reconnect & Save")
        self.btn_apply_camera.clicked.connect(lambda _checked=False: self.apply_camera_settings(persist=True))
        source_row.addWidget(self.btn_apply_camera)
        self.camera_mirror_chk = QCheckBox("Mirror")
        self.camera_mirror_chk.setChecked(bool(getattr(self.cfg, "CAMERA_MIRROR", True)))
        self.camera_mirror_chk.stateChanged.connect(self.on_mirror_changed)
        source_row.addWidget(self.camera_mirror_chk)
        source_row.addWidget(QLabel("AprilTag 36h11 | 1920x1200 YUY2 60FPS"))
        source_row.addSpacing(18)
        source_row.addWidget(QLabel("RK3588S IP"))
        self.udp_ip_edit = QLineEdit(str(getattr(self.cfg, "UDP_TARGET_IP", "127.0.0.1")))
        self.udp_ip_edit.setFixedWidth(150)
        source_row.addWidget(self.udp_ip_edit)
        self.udp_enabled_chk = QCheckBox("UDP Enable")
        self.udp_enabled_chk.setChecked(bool(getattr(self.cfg, "UDP_ENABLED", False)))
        source_row.addWidget(self.udp_enabled_chk)
        self.gamepad_mode_chk = QCheckBox("Gamepad Mode")
        self.gamepad_mode_chk.setChecked(False)
        self.gamepad_mode_chk.stateChanged.connect(self.on_gamepad_mode_changed)
        self.gamepad_mode_chk.setToolTip("Explicit manual-control override. Localization UDP still goes to 9005; gamepad control uses 9010.")
        source_row.addWidget(self.gamepad_mode_chk)
        self.btn_apply_udp = QPushButton("Apply & Save")
        self.btn_apply_udp.clicked.connect(lambda _checked=False: self.apply_udp_settings(persist=True))
        source_row.addWidget(self.btn_apply_udp)
        source_row.addStretch()
        main_layout.addLayout(source_row)

        action_row = QHBoxLayout()
        self.btn_start = QPushButton("Start Detection")
        self.btn_start.clicked.connect(self.start_detection)
        action_row.addWidget(self.btn_start)
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.clicked.connect(self.stop_detection)
        self.btn_stop.setEnabled(False)
        action_row.addWidget(self.btn_stop)
        self.btn_reset_calibration = QPushButton("Reset Calibration")
        self.btn_reset_calibration.clicked.connect(self.reset_calibration)
        action_row.addWidget(self.btn_reset_calibration)
        self.btn_clear_trajectory = QPushButton("Clear Trajectory")
        self.btn_clear_trajectory.clicked.connect(self.clear_trajectory)
        action_row.addWidget(self.btn_clear_trajectory)
        self.calib_label = QLabel("Calibration: Not calibrated")
        action_row.addWidget(self.calib_label)
        action_row.addStretch()
        self.status_label = QLabel("Status: Idle")
        action_row.addWidget(self.status_label)
        main_layout.addLayout(action_row)

        state_row = QHBoxLayout()
        self.camera_status_label = QLabel("Camera: not opened")
        state_row.addWidget(self.camera_status_label)
        state_row.addSpacing(18)
        self.udp_status_label = QLabel("UDP: idle")
        state_row.addWidget(self.udp_status_label)
        state_row.addSpacing(18)
        self.gamepad_status_label = QLabel("Gamepad: off")
        state_row.addWidget(self.gamepad_status_label)
        state_row.addStretch()
        main_layout.addLayout(state_row)

        # Legacy helpers retained internally for image/video utilities without occupying the runtime UI.
        self.input_group = QButtonGroup()
        self.radio_image = QRadioButton()
        self.radio_camera = QRadioButton()
        self.radio_video = QRadioButton()
        self.radio_camera.setChecked(True)
        self.input_group.addButton(self.radio_image, 0)
        self.input_group.addButton(self.radio_camera, 1)
        self.input_group.addButton(self.radio_video, 2)
        self.btn_load_image = QPushButton()
        self.btn_choose_video = QPushButton()
        self.btn_record_toggle = QPushButton()
        self.coord_text = QTextEdit()
        self.vehicle_text = QTextEdit()
        self.tag_status_text = QTextEdit()
        self.chk_lock_calibration = QCheckBox()
        self.chk_lock_calibration.setChecked(True)
        self.chk_lock_calibration.setEnabled(False)
        self.grid_size_spin = QSpinBox()
        self.grid_size_spin.setRange(1, 5000)
        self.grid_size_spin.setValue(500)
        self.chk_trace = QCheckBox()

        self.apply_udp_settings()

    def on_input_source_changed(self, button):
        """Handle input source change"""
        if button == self.radio_camera:
            self.btn_load_image.setEnabled(False)
            self.btn_choose_video.setEnabled(False)
            self.btn_record_toggle.setEnabled(True)

            self.stop_recording()
            if not self.is_detecting:
                self.stop_camera()
                self.start_camera()

        elif button == self.radio_video:
            self.btn_load_image.setEnabled(False)
            self.btn_choose_video.setEnabled(True)
            self.btn_record_toggle.setEnabled(False)

            self.stop_recording()
            self.stop_camera()
            if self.video_file_path is not None:
                self.start_video(self.video_file_path)

        else:
            # self.radio_image
            self.btn_load_image.setEnabled(True)
            self.btn_choose_video.setEnabled(False)
            self.btn_record_toggle.setEnabled(False)

            self.stop_recording()
            self.stop_camera()

    def reset_calibration(self):
        if self.transformer.get_calibration_status():
            reply = QMessageBox.question(
                self,
                "Confirm Recalibration",
                "Clear the locked calibration and recalibrate when any 4 corner tags are visible again?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                self.status_label.setText("Status: Recalibration cancelled")
                return

        self.transformer.reset_calibration()
        self.locked_fixed_markers = []
        self.vehicle_track_center_px = None
        self.rectified_detection_geometry = None
        self.last_detection_rectified = False
        self.last_output_pose = None
        self._reset_pose_filter()
        self.pose_seq = 0
        self.pose_history = []
        self.calib_label.setText("Calibration: Not calibrated")
        self.status_label.setText("Status: Waiting for any 4 corner tags to recalibrate")

    def clear_trajectory(self):
        self.pose_history = []
        self.marker_traces = {}
        self.status_label.setText("Status: Trajectory cleared")

    def on_grid_size_changed(self, value: int):
        """
        Update grid overlay when grid size changes.
        Only triggers a redraw for static-image mode to avoid extra computation on camera frames.
        """
        _ = value
        if self.current_image is None:
            return
        if self.radio_image.isChecked() and self.transformer.get_calibration_status():
            # Redetect markers so the overlay matches current frame.
            corners, ids, image_with_markers = self.detector.detect_markers(self.current_image)
            overlay_img = self.draw_grid_and_axes(image_with_markers)
            # Preserve the last clicked point (if any)
            if self.last_clicked_pixel is not None:
                x, y = self.last_clicked_pixel
                cv2.circle(overlay_img, (x, y), 5, (0, 255, 0), -1)
                cv2.circle(overlay_img, (x, y), 10, (0, 255, 0), 2)
            self.image_label.set_image(overlay_img)

    def on_trace_toggled(self, state: int):
        self.enable_trace = state == Qt.Checked
        if not self.enable_trace:
            self.marker_traces = {}

    def load_image(self):
        """Load image from file"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Image",
            "",
            "Image Files (*.png *.jpg *.jpeg *.bmp)"
        )

        if file_path:
            image = cv2.imread(file_path)
            if image is not None:
                self.current_image = image
                self.image_label.set_image(image)
                self.status_label.setText(f"Status: Image loaded - {file_path}")
                # Auto detect markers
                self.detect_and_calibrate(image)
            else:
                QMessageBox.warning(self, "Error", "Failed to load image file")

    def start_camera(self):
        """Start camera capture"""
        self.cap = cv2.VideoCapture(self.camera_index_spin.value(), self._camera_backend_flag())
        if not self.cap.isOpened():
            QMessageBox.warning(self, "Error", "Failed to open camera")
            return

        self.radio_camera.setChecked(True)
        self._apply_capture_properties()

        # Determine FPS for recording/video pacing.
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        try:
            fps = float(fps)
        except Exception:
            fps = 0.0
        if fps <= 0.0:
            fps = 30.0
        self.source_fps = fps

        self._update_trace_max_len()

        # At 60 FPS, floor to 16 ms instead of rounding to 17 ms (58.8 FPS).
        # VideoCapture blocks briefly until the camera's next frame is ready.
        interval_ms = max(1, int(1000.0 / self.source_fps))
        self.timer.start(interval_ms)
        self._sync_gamepad_timer()
        actual_w = int(round(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_h = int(round(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.camera_status_label.setText(
            f"Camera: {actual_w}x{actual_h} FPS={self.source_fps:.1f} YUY2"
        )
        self.status_label.setText(f"Status: Camera started (FPS={self.source_fps:.1f})")

    def stop_camera(self):
        """Stop current capture (camera/video)."""
        if self.cap is not None:
            self.timer.stop()
            self.cap.release()
            self.cap = None

        self.stop_recording()

        if self.radio_video.isChecked():
            self.status_label.setText("Status: Video stopped")
        elif self.radio_camera.isChecked():
            self.status_label.setText("Status: Camera stopped")
        else:
            self.status_label.setText("Status: Stopped")

    def start_video(self, file_path: str):
        """Start playing an mp4 (or other supported) file for detection."""
        self.cap = cv2.VideoCapture(file_path)
        if not self.cap.isOpened():
            QMessageBox.warning(self, "Error", f"Failed to open video file: {file_path}")
            self.radio_image.setChecked(True)
            self.video_file_path = None
            return

        fps = self.cap.get(cv2.CAP_PROP_FPS)
        try:
            fps = float(fps)
        except Exception:
            fps = 0.0
        if fps <= 0.0:
            fps = 30.0
        self.source_fps = fps

        self._update_trace_max_len()

        interval_ms = max(1, int(round(1000.0 / self.source_fps)))
        self.timer.start(interval_ms)
        self._sync_gamepad_timer()
        self.status_label.setText(f"Status: Video started - {file_path} (FPS={self.source_fps:.1f})")

    def _update_trace_max_len(self):
        """Update trajectory max length from the desired time window and current FPS."""
        fps = float(self.source_fps) if self.source_fps else 30.0
        window_s = max(0.0, float(self.trace_window_ms) / 1000.0)
        # At least 2 points so a line can be drawn.
        self.trace_max_len = max(2, int(round(fps * window_s)))

    def choose_video(self):
        """Select an mp4 file for "video detection" mode."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Video",
            "",
            "Video Files (*.mp4 *.avi *.mkv *.mov);;All Files (*)",
        )
        if not file_path:
            return

        self.video_file_path = file_path
        self.status_label.setText(f"Status: Video selected - {file_path}")

        if self.radio_video.isChecked():
            self.stop_camera()
            self.start_video(self.video_file_path)

    def stop_recording(self):
        """Stop mp4 recording if active."""
        if self.is_recording:
            self.is_recording = False
            if self.video_recorder is not None:
                self.video_recorder.stop()
            self.video_recorder = None
            self.btn_record_toggle.setText("Start Recording")
            # Keep status label intact if detection is running; otherwise set a neutral message.
            if not self.is_detecting:
                self.status_label.setText("Status: Recording stopped")

    def toggle_recording(self):
        """Toggle mp4 recording (raw frames only)."""
        if not self.radio_camera.isChecked():
            QMessageBox.information(self, "Info", "Recording is available only in camera mode")
            return

        if self.is_recording:
            self.stop_recording()
            return

        # Ensure capture is running so we can write the first frame soon.
        if self.cap is None:
            self.start_camera()

        record_start_ts = make_run_ts()
        record_dir = self.runs_root_dir / self.script_start_ts / "videos"
        mp4_path = record_dir / f"record_{record_start_ts}.mp4"
        self.video_recorder = VideoRecorder(mp4_path, fps=self.source_fps)
        self.is_recording = True
        self.btn_record_toggle.setText("Stop Recording")
        self.status_label.setText(f"Status: Recording -> {mp4_path.name}")

    def start_detection(self):
        """Start detection"""
        self.is_detecting = True
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)

        if self.radio_image.isChecked():
            if self.current_image is not None:
                self.detect_and_calibrate(self.current_image)
            return

        # Camera / video detection mode: ensure capture is running.
        if self.radio_camera.isChecked():
            if self.cap is None:
                self.start_camera()
        elif self.radio_video.isChecked():
            if self.video_file_path is None:
                QMessageBox.warning(self, "Error", "Please select a video file first")
                self.is_detecting = False
                self.btn_start.setEnabled(True)
                self.btn_stop.setEnabled(False)
                return
            if self.cap is None:
                self.start_video(self.video_file_path)

        self.status_label.setText("Status: Detection started")

    def stop_detection(self):
        """Stop detection"""
        self.is_detecting = False
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

        if self.radio_image.isChecked() and self.current_image is not None:
            # For static image mode, remove any overlay by showing raw image again.
            self.image_label.set_image(self.current_image)
        else:
            self.status_label.setText("Status: Detection stopped")

    def update_frame(self):
        """Update frame from camera"""
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                if self.camera_mirror_chk.isChecked():
                    frame = cv2.flip(frame, 1)
                self.current_image = frame

                # Optional raw frame recording (mp4), independent from inference.
                if self.is_recording and self.video_recorder is not None:
                    self.video_recorder.write_frame(frame)

                # Run inference without persisting every frame.
                if self.is_detecting:
                    self.detect_and_calibrate(frame)
                else:
                    self.image_label.set_image(frame)
                return

            # End of stream (video file) or read error.
            self.timer.stop()
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None

            if self.radio_video.isChecked():
                self.is_detecting = False
                self.btn_start.setEnabled(True)
                self.btn_stop.setEnabled(False)
                self.status_label.setText("Status: Video playback finished")
            else:
                self.status_label.setText("Status: Capture failed/stopped")

    def detect_and_calibrate(self, image):
        """Detect ArUco markers and calibrate coordinate system"""
        # Before lock, detect the four calibration tags in the source image.
        # After lock, run exactly one detection pass on a uniform-scale field
        # view so far/top tags gain pixels without adding a second-pass delay.
        corners = None
        ids = None
        image_with_markers = image.copy()
        self.last_detection_rectified = False
        marker_instances = None
        if self.transformer.get_calibration_status():
            marker_instances = self._detect_rectified_marker_instances(image)
            self.last_detection_rectified = marker_instances is not None
        if marker_instances is None:
            corners, ids, image_with_markers = self.detector.detect_markers(image)
            marker_instances = self.detector.get_marker_instances(corners, ids)

        # Update every physical detection separately; duplicate IDs are valid.
        self.update_tag_status(marker_instances)

        # IDs do not participate in calibration. Select the four widest corner instances.
        calibration_markers = None
        if not self.transformer.get_calibration_status():
            calibration_markers = self.detector.select_calibration_markers(
                marker_instances,
                required_count=self.cfg.MIN_MARKER_COUNT,
            )

        detected_count = len(marker_instances)
        if self.transformer.get_calibration_status():
            self.calib_label.setText("Calibration: Calibrated (locked)")
            self.status_label.setText("Status: Calibration locked; fixed tags held")
        elif calibration_markers is not None:
            success = self.transformer.calibrate(calibration_markers)
            if success:
                self.locked_fixed_markers = [
                    {
                        "id": int(marker["id"]),
                        "center": np.asarray(marker["center"], dtype=np.float32).copy(),
                        "corners": np.asarray(marker["corners"], dtype=np.float32).copy(),
                        "match_radius": self._fixed_marker_match_radius(marker["corners"]),
                    }
                    for marker in calibration_markers
                ]
                self._prepare_rectified_detection()
                self.calib_label.setText("Calibration: Calibrated (locked)")
                locked_ids = ", ".join(str(marker["id"]) for marker in self.locked_fixed_markers)
                self.status_label.setText(f"Status: Calibration locked (IDs: {locked_ids})")
            else:
                self.calib_label.setText("Calibration: Failed")
                self.status_label.setText("Status: Calibration failed")
                self.transformer.reset_calibration()
                self.locked_fixed_markers = []
        else:
            self.calib_label.setText("Calibration: Not calibrated")
            self.status_label.setText(
                f"Status: {detected_count} tag instances detected, {self.cfg.MIN_MARKER_COUNT} required for calibration"
            )

        # Vehicle marker (single ID) pose display (based on current calibration status)
        vehicle_detected = False
        self.last_pose_live = False
        self.last_vehicle_center_px = None
        self.last_vehicle_center_world = None
        self.last_vehicle_yaw_deg = None
        self.last_vehicle_corners_px = None
        vehicle_marker = None
        if self.transformer.get_calibration_status():
            vehicle_marker = self._select_vehicle_marker(marker_instances)
        if vehicle_marker is not None:
            try:
                marker_corners_px = np.asarray(vehicle_marker["corners"], dtype=np.float32)
                center_px = np.asarray(vehicle_marker["center"], dtype=np.float32)
                cx, cy = float(center_px[0]), float(center_px[1])
                self.last_vehicle_center_px = (cx, cy)
                self.vehicle_track_center_px = (cx, cy)
                self.last_vehicle_corners_px = marker_corners_px
                vehicle_detected = True

                center_world = self.transformer.pixel_to_world(cx, cy, z=0.0)
                # Heading direction stays unchanged: ArUco corner 1 -> corner 2.
                c1 = marker_corners_px[1]
                c2 = marker_corners_px[2]
                p1w = self.transformer.pixel_to_world(float(c1[0]), float(c1[1]), z=0.0)
                p2w = self.transformer.pixel_to_world(float(c2[0]), float(c2[1]), z=0.0)
                if center_world is not None and p1w is not None and p2w is not None:
                    dx = float(p2w[0] - p1w[0])
                    dz = float(p2w[1] - p1w[1])
                    # Official AR convention used here: yaw=0 points +X; +90 points +Z.
                    yaw_deg = math.degrees(math.atan2(dz, dx))
                    self.last_vehicle_center_world = center_world
                    self.last_vehicle_yaw_deg = self._norm_angle_deg(yaw_deg)
            except Exception:
                vehicle_detected = False

        if vehicle_detected and self.transformer.get_calibration_status():
            self._send_current_pose()
        self._update_vehicle_panel(vehicle_detected)


        # Trajectory overlay (pixel space). Does not affect H calculation.
        if self.enable_trace and vehicle_detected and self.last_vehicle_center_px is not None:
            mid = int(self.vehicle_id)
            center = self.last_vehicle_center_px
            pt = (int(round(float(center[0]))), int(round(float(center[1]))))
            trace = self.marker_traces.get(mid, [])
            trace.append(pt)
            if len(trace) > self.trace_max_len:
                trace = trace[-self.trace_max_len:]
            self.marker_traces[mid] = trace

            if len(trace) >= 2:
                pts = np.array(trace, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(
                    image_with_markers,
                    [pts],
                    isClosed=False,
                    color=(255, 0, 0),
                    thickness=2,
                )

        image_with_markers = self._draw_marker_overlays(image_with_markers, corners, ids)
        image_with_markers = self._draw_vehicle_arrow(image_with_markers)
        image_with_markers = self._draw_trajectory_inset(image_with_markers)
        image_with_markers = self._draw_pose_hud(image_with_markers)

        # (Removed) YOLO overlay
        self.image_label.set_image(image_with_markers)

    def on_image_clicked(self, x, y):
        """Handle image click event"""
        if self.current_image is None:
            return

        # Check if coordinates are valid
        h, w = self.current_image.shape[:2]
        if x < 0 or x >= w or y < 0 or y >= h:
            return

        # Convert to world coordinates
        world_coords = self.transformer.pixel_to_world(x, y, z=0.0)
        self.last_clicked_pixel = (x, y)
        self.last_clicked_world = world_coords

        # Update coordinate display
        coord_info = f"Pixel coordinates: ({x}, {y})\n"
        if world_coords is not None:
            scale = float(getattr(self.cfg, "OUTPUT_COORD_SCALE", 0.001))
            x_m = float(world_coords[0]) * scale * float(getattr(self.cfg, "OUTPUT_X_SIGN", 1.0))
            z_m = float(world_coords[1]) * scale * float(getattr(self.cfg, "OUTPUT_Z_SIGN", 1.0))
            coord_info += f"World coordinates (m): ({x_m:.3f}, {z_m:.3f})\n"
        else:
            coord_info += "World coordinates: Not calibrated, conversion unavailable\n"

        self.coord_text.setText(coord_info)

        # Draw point on image
        image_copy = self.current_image.copy()
        cv2.circle(image_copy, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(image_copy, (x, y), 10, (0, 255, 0), 2)

        # Re-detect markers to keep them visible
        corners, ids, image_with_markers = self.detector.detect_markers(image_copy)
        image_with_markers = self._draw_marker_overlays(image_with_markers, corners, ids)
        image_with_markers = self._draw_vehicle_arrow(image_with_markers)
        image_with_markers = self._draw_trajectory_inset(image_with_markers)
        image_with_markers = self._draw_pose_hud(image_with_markers)
        self.image_label.set_image(image_with_markers)

    def draw_grid_and_axes(self, image_bgr):
        """
        Draw world grid and X/Y axes on the given image using world->pixel mapping.

        World coordinate axes follow `config.WORLD_COORDINATES`:
        - x axis: along increasing world x (ID0 -> ID1)
        - y axis: along increasing world y
        """
        if not self.transformer.get_calibration_status():
            return image_bgr

        step_mm = int(self.grid_size_spin.value())
        margin_mm = step_mm
        # Draw minor grid at half spacing for more visual reference.
        step_minor_mm = max(0.5, step_mm / 2.0)

        # World coordinate drawing range (unit: mm).
        # Your request: -5m ~ 5m.
        range_mm = 5000.0
        x0 = -range_mm
        x1 = range_mm
        y0 = -range_mm
        y1 = range_mm

        # Use darker grid color so it remains visible on bright backgrounds,
        # especially after UI down-scaling.
        grid_color = (90, 90, 90)  # BGR
        minor_grid_color = (140, 140, 140)  # BGR (lighter than major)
        axis_x_color = (0, 0, 255)  # Red (BGR)
        axis_y_color = (0, 200, 0)  # Green (BGR)

        # Draw thickness in *original image space* so it stays visible after QLabel down-scaling.
        img_h, img_w = image_bgr.shape[:2]
        label_w = max(1, self.image_label.width())
        label_h = max(1, self.image_label.height())
        scale = min(label_w / float(img_w), label_h / float(img_h))
        if scale <= 0:
            scale = 1.0

        # Target thickness in display space (roughly in pixels).
        grid_thickness_display_px = 2.0
        axis_thickness_display_px = 4.0
        grid_thickness = max(1, int(round(grid_thickness_display_px / scale)))
        axis_thickness = max(grid_thickness + 1, int(round(axis_thickness_display_px / scale)))
        minor_grid_thickness = max(1, int(round(grid_thickness * 0.6)))

        # Helper to draw a line between two world points.
        def draw_world_line(wx1, wy1, wx2, wy2, color, thickness):
            p1 = self.transformer.world_to_pixel(wx1, wy1)
            p2 = self.transformer.world_to_pixel(wx2, wy2)
            if p1 is None or p2 is None:
                return
            # Draw the portion of the projected *infinite line* that intersects
            # the image rectangle. This avoids the issue where OpenCV may not
            # draw the segment when both endpoints are outside the image.
            x_min, y_min = 0.0, 0.0
            x_max, y_max = float(img_w - 1), float(img_h - 1)

            x1_, y1_ = float(p1[0]), float(p1[1])
            x2_, y2_ = float(p2[0]), float(p2[1])

            dx = x2_ - x1_
            dy = y2_ - y1_

            candidates = []

            def add_candidate(cx, cy):
                if cx < x_min - 1e-6 or cx > x_max + 1e-6:
                    return
                if cy < y_min - 1e-6 or cy > y_max + 1e-6:
                    return
                candidates.append((cx, cy))

            # Intersect with rectangle edges x = x_min/x_max and y = y_min/y_max.
            eps = 1e-12
            if abs(dx) > eps:
                # x = x_min
                t = (x_min - x1_) / dx
                cy = y1_ + t * dy
                add_candidate(x_min, cy)
                # x = x_max
                t = (x_max - x1_) / dx
                cy = y1_ + t * dy
                add_candidate(x_max, cy)
            if abs(dy) > eps:
                # y = y_min
                t = (y_min - y1_) / dy
                cx = x1_ + t * dx
                add_candidate(cx, y_min)
                # y = y_max
                t = (y_max - y1_) / dy
                cx = x1_ + t * dx
                add_candidate(cx, y_max)

            if len(candidates) < 2:
                return

            # Deduplicate very close points.
            unique = []
            tol = 1e-3
            for cx, cy in candidates:
                if all((cx - ux) ** 2 + (cy - uy) ** 2 > tol ** 2 for ux, uy in unique):
                    unique.append((cx, cy))

            if len(unique) < 2:
                return

            # Choose the two farthest points on the rectangle boundary.
            max_d = -1.0
            pA = None
            pB = None
            for i in range(len(unique)):
                for j in range(i + 1, len(unique)):
                    ux, uy = unique[i]
                    vx, vy = unique[j]
                    d = (ux - vx) ** 2 + (uy - vy) ** 2
                    if d > max_d:
                        max_d = d
                        pA = unique[i]
                        pB = unique[j]

            if pA is None or pB is None:
                return

            cx1, cy1 = pA
            cx2, cy2 = pB

            cv2.line(
                image_bgr,
                (int(round(cx1)), int(round(cy1))),
                (int(round(cx2)), int(round(cy2))),
                color,
                thickness,
            )

        # Draw grid lines (minor first, then major so major stays prominent)
        # Minor grid
        start_x_minor = np.floor(x0 / step_minor_mm) * step_minor_mm
        end_x_minor = np.ceil(x1 / step_minor_mm) * step_minor_mm
        start_y_minor = np.floor(y0 / step_minor_mm) * step_minor_mm
        end_y_minor = np.ceil(y1 / step_minor_mm) * step_minor_mm

        for wx in np.arange(start_x_minor, end_x_minor + 0.5 * step_minor_mm, step_minor_mm):
            draw_world_line(wx, y0, wx, y1, minor_grid_color, minor_grid_thickness)
        for wy in np.arange(start_y_minor, end_y_minor + 0.5 * step_minor_mm, step_minor_mm):
            draw_world_line(x0, wy, x1, wy, minor_grid_color, minor_grid_thickness)

        # Major grid
        start_x = np.floor(x0 / step_mm) * step_mm
        end_x = np.ceil(x1 / step_mm) * step_mm
        start_y = np.floor(y0 / step_mm) * step_mm
        end_y = np.ceil(y1 / step_mm) * step_mm

        for wx in np.arange(start_x, end_x + 0.5 * step_mm, step_mm):
            draw_world_line(wx, y0, wx, y1, grid_color, grid_thickness)
        for wy in np.arange(start_y, end_y + 0.5 * step_mm, step_mm):
            draw_world_line(x0, wy, x1, wy, grid_color, grid_thickness)

        # Draw axes (x axis at world_y=0, y axis at world_x=0)
        draw_world_line(x0, 0.0, x1, 0.0, axis_x_color, axis_thickness)
        draw_world_line(0.0, y0, 0.0, y1, axis_y_color, axis_thickness)

        # Put axis labels near positive ends
        px_pos_x = self.transformer.world_to_pixel(x1, 0.0)
        py_pos_y = self.transformer.world_to_pixel(0.0, y1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        if px_pos_x is not None:
            cv2.putText(
                image_bgr,
                "X",
                (int(round(px_pos_x[0])) + 5, int(round(px_pos_x[1])) - 5),
                font,
                0.7,
                axis_x_color,
                2,
                cv2.LINE_AA,
            )
        if py_pos_y is not None:
            cv2.putText(
                image_bgr,
                "Y",
                (int(round(py_pos_y[0])) + 5, int(round(py_pos_y[1])) + 15),
                font,
                0.7,
                axis_y_color,
                2,
                cv2.LINE_AA,
            )

        return image_bgr

    def update_tag_status(self, marker_instances):
        """
        Update right-panel tag status view.

        Args:
            marker_instances: current-frame physical detections; IDs may repeat
        """
        marker_instances = marker_instances or []
        lines = [f"Detected instances: {len(marker_instances)}"]
        for index, marker in enumerate(marker_instances, start=1):
            center = marker["center"]
            lines.append(
                f"#{index} ID {int(marker['id'])}: "
                f"center_px=({float(center[0]):.1f}, {float(center[1]):.1f})"
            )

        if self.locked_fixed_markers:
            locked_ids = ", ".join(str(marker["id"]) for marker in self.locked_fixed_markers)
            lines.append(f"Locked fixed instances: [{locked_ids}]")
        lines.append(f"Vehicle ID: {int(self.vehicle_id)} (duplicate ID supported)")

        self.tag_status_text.setText("\n".join(lines))

    def closeEvent(self, event):
        """Handle window close event"""
        if hasattr(self, "gamepad_mode_chk") and self.gamepad_mode_chk.isChecked():
            self.gamepad_timer.stop()
            self._send_gamepad_disabled()
        self.stop_camera()
        event.accept()


def main():
    """Main entry point"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

