# -*- coding: utf-8 -*-
"""SYSU_DDL Windows gamepad mapping tester.

This tool is intentionally input-only: it does not send UDP control packets.
Use it to confirm joystick axes, buttons, deadzone, inversion, and future
manual-control mapping before connecting it to the car control chain.
"""

from __future__ import annotations

import ctypes
import json
import math
import sys
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Dict, List, Optional


APP_TITLE = "SYSU_DDL Gamepad Mapper"
APP_DIR = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "gamepad_mapping.json"
MAPPING_VERSION = 2


DEFAULT_MAPPING = {
    "version": MAPPING_VERSION,
    "steer_axis": "LX",
    "steer_invert": False,
    "throttle_source": "RT",
    "throttle_invert": False,
    "safe_stop_axis": "LT",
    "safe_stop_threshold": 0.90,
    "deadzone": 0.08,
    "enable_button": "RB",
    "stop_button": "B",
    "max_speed_mps": 0.50,
    "steer_error_scale": 210.0,
}


@dataclass
class GamepadState:
    connected: bool = False
    backend: str = "none"
    name: str = "No controller"
    axes: Dict[str, float] = field(default_factory=dict)
    buttons: Dict[str, bool] = field(default_factory=dict)
    timestamp: float = 0.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    deadzone = clamp(deadzone, 0.0, 0.95)
    value = clamp(value, -1.0, 1.0)
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


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


class XInputBackend:
    BUTTON_BITS = {
        "DPAD_UP": 0x0001,
        "DPAD_DOWN": 0x0002,
        "DPAD_LEFT": 0x0004,
        "DPAD_RIGHT": 0x0008,
        "START": 0x0010,
        "BACK": 0x0020,
        "LS": 0x0040,
        "RS": 0x0080,
        "LB": 0x0100,
        "RB": 0x0200,
        "A": 0x1000,
        "B": 0x2000,
        "X": 0x4000,
        "Y": 0x8000,
    }

    def __init__(self) -> None:
        self.dll = None
        self.dll_name = ""
        for name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
            try:
                self.dll = ctypes.windll.LoadLibrary(name)  # type: ignore[attr-defined]
                self.dll_name = name
                break
            except Exception:
                continue

    @property
    def available(self) -> bool:
        return self.dll is not None

    @staticmethod
    def _norm_thumb(value: int) -> float:
        if value >= 0:
            return clamp(value / 32767.0, -1.0, 1.0)
        return clamp(value / 32768.0, -1.0, 1.0)

    def poll(self) -> GamepadState:
        if self.dll is None:
            return GamepadState(backend="XInput", name="XInput DLL not available")

        for index in range(4):
            state = XINPUT_STATE()
            result = self.dll.XInputGetState(index, ctypes.byref(state))
            if result == 0:
                gp = state.Gamepad
                axes = {
                    "LX": self._norm_thumb(gp.sThumbLX),
                    "LY": self._norm_thumb(gp.sThumbLY),
                    "RX": self._norm_thumb(gp.sThumbRX),
                    "RY": self._norm_thumb(gp.sThumbRY),
                    "LT": gp.bLeftTrigger / 255.0,
                    "RT": gp.bRightTrigger / 255.0,
                }
                buttons = {
                    name: bool(gp.wButtons & bit)
                    for name, bit in self.BUTTON_BITS.items()
                }
                return GamepadState(
                    connected=True,
                    backend="XInput",
                    name=f"XInput controller {index} ({self.dll_name})",
                    axes=axes,
                    buttons=buttons,
                    timestamp=time.time(),
                )

        return GamepadState(backend="XInput", name="No XInput controller")


class PygameBackend:
    def __init__(self) -> None:
        self.pg = None
        self.joystick = None
        self.error = ""
        try:
            import pygame  # type: ignore

            self.pg = pygame
            pygame.init()
            pygame.joystick.init()
        except Exception as exc:
            self.error = str(exc)

    @property
    def available(self) -> bool:
        return self.pg is not None

    def refresh(self) -> None:
        self.joystick = None
        if self.pg is None:
            return
        self.pg.joystick.quit()
        self.pg.joystick.init()
        if self.pg.joystick.get_count() > 0:
            self.joystick = self.pg.joystick.Joystick(0)
            self.joystick.init()

    def poll(self) -> GamepadState:
        if self.pg is None:
            return GamepadState(backend="pygame", name=f"pygame unavailable: {self.error}")

        self.pg.event.pump()
        if self.joystick is None:
            self.refresh()
        if self.joystick is None:
            return GamepadState(backend="pygame", name="No pygame controller")

        axes = {
            f"AXIS_{i}": clamp(float(self.joystick.get_axis(i)), -1.0, 1.0)
            for i in range(self.joystick.get_numaxes())
        }
        buttons = {
            f"BTN_{i}": bool(self.joystick.get_button(i))
            for i in range(self.joystick.get_numbuttons())
        }
        hats = {}
        for i in range(self.joystick.get_numhats()):
            x, y = self.joystick.get_hat(i)
            hats[f"HAT_{i}_UP"] = y > 0
            hats[f"HAT_{i}_DOWN"] = y < 0
            hats[f"HAT_{i}_LEFT"] = x < 0
            hats[f"HAT_{i}_RIGHT"] = x > 0
        buttons.update(hats)
        return GamepadState(
            connected=True,
            backend="pygame",
            name=self.joystick.get_name(),
            axes=axes,
            buttons=buttons,
            timestamp=time.time(),
        )


class BackendManager:
    def __init__(self) -> None:
        self.xinput = XInputBackend()
        self.pygame = PygameBackend()
        self.prefer_pygame = False

    def refresh(self) -> None:
        self.pygame.refresh()

    def poll(self) -> GamepadState:
        if not self.prefer_pygame:
            state = self.xinput.poll()
            if state.connected:
                return state
        state = self.pygame.poll()
        if state.connected:
            return state
        if self.prefer_pygame:
            return self.xinput.poll()
        return state


class AxisBar(ttk.Frame):
    def __init__(self, master: tk.Misc, name: str) -> None:
        super().__init__(master)
        self.name = name
        self.label = ttk.Label(self, text=name, width=10)
        self.value_label = ttk.Label(self, text="+0.000", width=8, anchor="e")
        self.canvas = tk.Canvas(self, width=260, height=18, bg="#171717", highlightthickness=1)
        self.label.pack(side=tk.LEFT)
        self.canvas.pack(side=tk.LEFT, padx=4)
        self.value_label.pack(side=tk.LEFT)
        self.set_value(0.0)

    def set_value(self, value: float) -> None:
        value = clamp(value, -1.0, 1.0)
        self.value_label.configure(text=f"{value:+.3f}")
        self.canvas.delete("all")
        width = int(self.canvas["width"])
        height = int(self.canvas["height"])
        mid = width // 2
        self.canvas.create_line(mid, 0, mid, height, fill="#777")
        if value >= 0:
            self.canvas.create_rectangle(mid, 3, mid + int(value * mid), height - 3, fill="#37d95b", outline="")
        else:
            self.canvas.create_rectangle(mid + int(value * mid), 3, mid, height - 3, fill="#ffb02e", outline="")


class MapperApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("860x680")
        self.root.minsize(800, 600)

        self.backend = BackendManager()
        self.mapping = self.load_mapping()
        self.axis_bars: Dict[str, AxisBar] = {}
        self.button_labels: Dict[str, ttk.Label] = {}
        self.vehicle_labels: Dict[str, ttk.Label] = {}
        self.last_axis_names: List[str] = []
        self.last_button_names: List[str] = []

        self.vars = {
            "steer_axis": tk.StringVar(value=str(self.mapping["steer_axis"])),
            "steer_invert": tk.BooleanVar(value=bool(self.mapping["steer_invert"])),
            "throttle_source": tk.StringVar(value=str(self.mapping["throttle_source"])),
            "throttle_invert": tk.BooleanVar(value=bool(self.mapping["throttle_invert"])),
            "safe_stop_axis": tk.StringVar(value=str(self.mapping["safe_stop_axis"])),
            "safe_stop_threshold": tk.DoubleVar(value=float(self.mapping["safe_stop_threshold"])),
            "deadzone": tk.DoubleVar(value=float(self.mapping["deadzone"])),
            "enable_button": tk.StringVar(value=str(self.mapping["enable_button"])),
            "stop_button": tk.StringVar(value=str(self.mapping["stop_button"])),
            "max_speed_mps": tk.DoubleVar(value=float(self.mapping["max_speed_mps"])),
            "steer_error_scale": tk.DoubleVar(value=float(self.mapping["steer_error_scale"])),
            "prefer_pygame": tk.BooleanVar(value=False),
        }

        self._build_ui()
        self._schedule_poll()

    def load_mapping(self) -> Dict[str, object]:
        mapping = dict(DEFAULT_MAPPING)
        if CONFIG_PATH.exists():
            try:
                data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
                if isinstance(data, dict) and int(data.get("version", 0)) == MAPPING_VERSION:
                    mapping.update(data)
            except Exception:
                pass
        return mapping

    def save_mapping(self) -> None:
        data = {
            "version": MAPPING_VERSION,
            "steer_axis": self.vars["steer_axis"].get(),
            "steer_invert": self.vars["steer_invert"].get(),
            "throttle_source": self.vars["throttle_source"].get(),
            "throttle_invert": self.vars["throttle_invert"].get(),
            "safe_stop_axis": self.vars["safe_stop_axis"].get(),
            "safe_stop_threshold": round(float(self.vars["safe_stop_threshold"].get()), 3),
            "deadzone": round(float(self.vars["deadzone"].get()), 3),
            "enable_button": self.vars["enable_button"].get(),
            "stop_button": self.vars["stop_button"].get(),
            "max_speed_mps": round(float(self.vars["max_speed_mps"].get()), 3),
            "steer_error_scale": round(float(self.vars["steer_error_scale"].get()), 1),
        }
        CONFIG_PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        messagebox.showinfo(APP_TITLE, f"映射已保存：\n{CONFIG_PATH}")

    def _build_ui(self) -> None:
        root = ttk.Frame(self.root, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)
        self.status_label = ttk.Label(top, text="正在检测手柄...", font=("Microsoft YaHei UI", 11, "bold"))
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Checkbutton(
            top,
            text="优先 pygame",
            variable=self.vars["prefer_pygame"],
            command=self._toggle_backend,
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="刷新手柄", command=self._refresh).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="保存映射", command=self.save_mapping).pack(side=tk.LEFT, padx=4)

        warning = ttk.Label(
            root,
            text="当前工具只显示手柄输入与未来控车映射预览，不发送 UDP，不会控制小车。",
            foreground="#9a5d00",
        )
        warning.pack(fill=tk.X, pady=(6, 10))

        paned = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(paned, padding=(0, 0, 8, 0))
        right = ttk.Frame(paned, padding=(8, 0, 0, 0))
        paned.add(left, weight=3)
        paned.add(right, weight=2)

        axes_box = ttk.LabelFrame(left, text="轴输入")
        axes_box.pack(fill=tk.BOTH, expand=False)
        self.axes_frame = ttk.Frame(axes_box, padding=6)
        self.axes_frame.pack(fill=tk.X)

        buttons_box = ttk.LabelFrame(left, text="按键输入")
        buttons_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.buttons_frame = ttk.Frame(buttons_box, padding=6)
        self.buttons_frame.pack(fill=tk.BOTH, expand=True)

        mapping_box = ttk.LabelFrame(right, text="映射设置")
        mapping_box.pack(fill=tk.X)
        self._build_mapping_grid(mapping_box)

        preview_box = ttk.LabelFrame(right, text="未来控车预览")
        preview_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.command_label = tk.Label(
            preview_box,
            text="NO CONTROLLER",
            bg="#666666",
            fg="#ffffff",
            font=("Microsoft YaHei UI", 16, "bold"),
            pady=8,
        )
        self.command_label.pack(fill=tk.X, padx=6, pady=(6, 4))

        info = ttk.Frame(preview_box, padding=(6, 0, 6, 4))
        info.pack(fill=tk.X)
        rows = [
            ("enabled", "使能状态"),
            ("rt", "右扳机 RT"),
            ("lt", "左扳机 LT"),
            ("target_speed", "目标速度"),
            ("track_error", "转向误差"),
            ("future_frame", "未来控制帧"),
        ]
        for row, (key, label) in enumerate(rows):
            ttk.Label(info, text=label, width=12).grid(row=row, column=0, sticky="w", pady=2)
            value = ttk.Label(info, text="-", font=("Consolas", 10, "bold"))
            value.grid(row=row, column=1, sticky="ew", pady=2)
            self.vehicle_labels[key] = value
        info.columnconfigure(1, weight=1)

        self.preview_text = tk.Text(preview_box, height=10, wrap=tk.WORD, font=("Consolas", 10))
        self.preview_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.preview_text.configure(state=tk.DISABLED)

    def _build_mapping_grid(self, parent: ttk.LabelFrame) -> None:
        frame = ttk.Frame(parent, padding=8)
        frame.pack(fill=tk.X)
        self.steer_combo = self._combo(frame, 0, "转向轴", "steer_axis", ["LX", "RX", "AXIS_0", "AXIS_2"])
        ttk.Checkbutton(frame, text="转向反向", variable=self.vars["steer_invert"]).grid(row=1, column=1, sticky="w", pady=3)
        self.throttle_combo = self._combo(frame, 2, "速度扳机", "throttle_source", ["RT", "AXIS_5", "AXIS_2"])
        ttk.Checkbutton(frame, text="油门反向", variable=self.vars["throttle_invert"]).grid(row=3, column=1, sticky="w", pady=3)
        self.safe_stop_combo = self._combo(frame, 4, "停车扳机", "safe_stop_axis", ["LT", "AXIS_4", "AXIS_2"])
        ttk.Label(frame, text="停车阈值").grid(row=5, column=0, sticky="w", pady=3)
        ttk.Spinbox(
            frame,
            from_=0.50,
            to=1.00,
            increment=0.05,
            textvariable=self.vars["safe_stop_threshold"],
            width=8,
        ).grid(row=5, column=1, sticky="w", pady=3)
        self.enable_combo = self._combo(frame, 6, "使能键", "enable_button", ["RB", "LB", "A", "START", "BTN_5"])
        self.stop_combo = self._combo(frame, 7, "备用急停键", "stop_button", ["B", "BACK", "BTN_1", "BTN_6"])

        ttk.Label(frame, text="死区").grid(row=8, column=0, sticky="w", pady=3)
        ttk.Scale(frame, from_=0.0, to=0.40, variable=self.vars["deadzone"], orient=tk.HORIZONTAL).grid(
            row=8, column=1, sticky="ew", pady=3
        )
        self.deadzone_label = ttk.Label(frame, text="")
        self.deadzone_label.grid(row=8, column=2, sticky="e", padx=(6, 0))

        ttk.Label(frame, text="最大速度 m/s").grid(row=9, column=0, sticky="w", pady=3)
        ttk.Spinbox(frame, from_=0.05, to=2.00, increment=0.05, textvariable=self.vars["max_speed_mps"], width=8).grid(
            row=9, column=1, sticky="w", pady=3
        )
        ttk.Label(frame, text="转向误差比例").grid(row=10, column=0, sticky="w", pady=3)
        ttk.Spinbox(frame, from_=50.0, to=800.0, increment=10.0, textvariable=self.vars["steer_error_scale"], width=8).grid(
            row=10, column=1, sticky="w", pady=3
        )
        frame.columnconfigure(1, weight=1)

    def _combo(self, parent: ttk.Frame, row: int, label: str, var_name: str, values: List[str]) -> ttk.Combobox:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=3)
        combo = ttk.Combobox(parent, textvariable=self.vars[var_name], values=values, state="readonly", width=14)
        combo.grid(row=row, column=1, sticky="ew", pady=3)
        return combo

    def _toggle_backend(self) -> None:
        self.backend.prefer_pygame = self.vars["prefer_pygame"].get()
        self._refresh()

    def _refresh(self) -> None:
        self.backend.refresh()

    def _schedule_poll(self) -> None:
        state = self.backend.poll()
        self._update_view(state)
        self.root.after(33, self._schedule_poll)

    def _update_view(self, state: GamepadState) -> None:
        self.status_label.configure(
            text=f"{'已连接' if state.connected else '未连接'} | {state.backend} | {state.name}"
        )
        self._update_axis_widgets(state)
        self._update_button_widgets(state)
        self._update_mapping_options(state)
        self._update_preview(state)

    def _update_axis_widgets(self, state: GamepadState) -> None:
        names = list(state.axes.keys())
        if names != self.last_axis_names:
            for child in self.axes_frame.winfo_children():
                child.destroy()
            self.axis_bars.clear()
            for name in names:
                bar = AxisBar(self.axes_frame, name)
                bar.pack(fill=tk.X, pady=2)
                self.axis_bars[name] = bar
            self.last_axis_names = names
        for name, bar in self.axis_bars.items():
            bar.set_value(state.axes.get(name, 0.0))

    def _update_button_widgets(self, state: GamepadState) -> None:
        names = list(state.buttons.keys())
        if names != self.last_button_names:
            for child in self.buttons_frame.winfo_children():
                child.destroy()
            self.button_labels.clear()
            for i, name in enumerate(names):
                label = ttk.Label(self.buttons_frame, text=name, width=12, relief=tk.GROOVE, anchor="center")
                label.grid(row=i // 4, column=i % 4, sticky="ew", padx=3, pady=3)
                self.button_labels[name] = label
            for col in range(4):
                self.buttons_frame.columnconfigure(col, weight=1)
            self.last_button_names = names
        for name, label in self.button_labels.items():
            pressed = state.buttons.get(name, False)
            label.configure(text=f"{name} {'ON' if pressed else ''}")
            label.configure(foreground="#008a22" if pressed else "#555")

    def _update_mapping_options(self, state: GamepadState) -> None:
        axis_names = list(state.axes.keys())
        button_names = list(state.buttons.keys())

        throttle_values = list(dict.fromkeys(["RT", *axis_names]))
        safe_stop_values = list(dict.fromkeys(["LT", *axis_names]))
        for combo, current, values in (
            (self.steer_combo, self.vars["steer_axis"].get(), axis_names),
            (self.throttle_combo, self.vars["throttle_source"].get(), throttle_values),
            (self.safe_stop_combo, self.vars["safe_stop_axis"].get(), safe_stop_values),
            (self.enable_combo, self.vars["enable_button"].get(), button_names),
            (self.stop_combo, self.vars["stop_button"].get(), button_names),
        ):
            if values:
                combo.configure(values=values)
                if current not in values:
                    combo.set(values[0])
        self.deadzone_label.configure(text=f"{float(self.vars['deadzone'].get()):.2f}")

    def _axis_value(self, axes: Dict[str, float], source: str) -> float:
        if source == "RT-LT":
            return axes.get("RT", 0.0) - axes.get("LT", 0.0)
        if source == "LT-RT":
            return axes.get("LT", 0.0) - axes.get("RT", 0.0)
        return axes.get(source, 0.0)

    def _trigger_opening(self, axes: Dict[str, float], source: str, deadzone: float) -> tuple[float, float]:
        raw = self._axis_value(axes, source)
        if source in ("LT", "RT"):
            opening = clamp(raw, 0.0, 1.0)
        elif source.startswith("AXIS_"):
            opening = clamp((raw + 1.0) * 0.5, 0.0, 1.0)
        else:
            opening = clamp(raw, 0.0, 1.0)
        if opening <= deadzone:
            opening = 0.0
        else:
            opening = (opening - deadzone) / (1.0 - deadzone)
        return raw, clamp(opening, 0.0, 1.0)

    def _update_preview(self, state: GamepadState) -> None:
        deadzone = float(self.vars["deadzone"].get())
        steer_raw = self._axis_value(state.axes, self.vars["steer_axis"].get())
        throttle_raw, throttle = self._trigger_opening(state.axes, self.vars["throttle_source"].get(), deadzone)
        safe_stop_raw, safe_stop_opening = self._trigger_opening(state.axes, self.vars["safe_stop_axis"].get(), 0.0)
        steer = apply_deadzone(steer_raw, deadzone)
        if self.vars["steer_invert"].get():
            steer = -steer
        if self.vars["throttle_invert"].get():
            throttle = 1.0 - throttle

        enable_button = self.vars["enable_button"].get()
        stop_button = self.vars["stop_button"].get()
        enabled = bool(state.buttons.get(enable_button, False))
        button_stop = bool(state.buttons.get(stop_button, False))
        safe_stop_threshold = clamp(float(self.vars["safe_stop_threshold"].get()), 0.0, 1.0)
        trigger_stop = safe_stop_opening >= safe_stop_threshold
        stop = button_stop or trigger_stop
        max_speed = float(self.vars["max_speed_mps"].get())
        steer_scale = float(self.vars["steer_error_scale"].get())
        requested_speed = throttle * max_speed
        requested_error = steer * steer_scale

        if not state.connected:
            command_state = "NO_CONTROLLER"
            command_color = "#666666"
            effective_speed = 0.0
            effective_error = 0.0
            future_frame = "no send"
        elif stop:
            command_state = "STATE_SAFE_STOP"
            command_color = "#cc2f2f"
            effective_speed = 0.0
            effective_error = 0.0
            future_frame = "state=SAFE_STOP speed=0 err=0 flags=0x00"
        elif enabled:
            command_state = "STATE_TRACK"
            command_color = "#168a35"
            effective_speed = requested_speed
            effective_error = requested_error
            future_frame = f"state=TRACK speed={effective_speed:.3f} err={effective_error:.1f} flags=0x01"
        else:
            command_state = "MANUAL_DISABLED"
            command_color = "#7a6a00"
            effective_speed = 0.0
            effective_error = 0.0
            future_frame = "manual off; no car command preview"

        self.command_label.configure(text=command_state, bg=command_color)
        self.vehicle_labels["enabled"].configure(text=f"{'ON' if enabled else 'OFF'}  button={enable_button}")
        self.vehicle_labels["rt"].configure(text=f"raw={throttle_raw:+.3f} open={throttle:.3f}")
        self.vehicle_labels["lt"].configure(
            text=f"raw={safe_stop_raw:+.3f} open={safe_stop_opening:.3f} threshold={safe_stop_threshold:.2f}"
        )
        self.vehicle_labels["target_speed"].configure(
            text=f"{effective_speed:.3f} m/s  (request {requested_speed:.3f})"
        )
        self.vehicle_labels["track_error"].configure(
            text=f"{effective_error:.1f}  (request {requested_error:.1f})"
        )
        self.vehicle_labels["future_frame"].configure(text=future_frame)

        preview = {
            "type": "manual_control_preview",
            "note": "preview only, UDP disabled",
            "connected": state.connected,
            "command_state": command_state,
            "enabled_button_pressed": enabled,
            "stop_button_pressed": button_stop,
            "safe_stop_trigger_pressed": trigger_stop,
            "raw": {
                "steer": round(steer_raw, 4),
                "throttle": round(throttle_raw, 4),
                "safe_stop_trigger": round(safe_stop_raw, 4),
            },
            "mapped": {
                "steer": round(steer, 4),
                "throttle": round(throttle, 4),
                "safe_stop_trigger": round(safe_stop_opening, 4),
                "requested_target_speed_mps": round(requested_speed, 3),
                "requested_track_error": round(requested_error, 1),
                "effective_target_speed_mps": round(effective_speed, 3),
                "effective_track_error": round(effective_error, 1),
            },
            "default_mapping": "右扳机 RT 控制 target_speed，左摇杆 LX 控制 track_error，左扳机 LT >= 90% 进入 SAFE_STOP。",
            "future_logic": "按住使能键才允许 STATE_TRACK；LT 急停、备用急停键或断联应停车。",
        }
        text = json.dumps(preview, ensure_ascii=False, indent=2)
        self.preview_text.configure(state=tk.NORMAL)
        self.preview_text.delete("1.0", tk.END)
        self.preview_text.insert(tk.END, text)
        self.preview_text.configure(state=tk.DISABLED)


def main() -> int:
    if sys.platform != "win32":
        print("This tester is intended for Windows gamepad input.")
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except Exception:
        pass
    MapperApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
