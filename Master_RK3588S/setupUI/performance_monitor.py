import csv
import math
import os
import re
import time


BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ===== Performance monitor parameter area =====
# Master switch for runtime performance probing. Disable this if the probe itself
# is suspected to affect timing during a race.
PERF_MONITOR_DEFAULTS = {
    # Enable timing, HUD summary, and CSV output.
    "ENABLE": True,

    # Show a short performance summary in the OpenCV HUD panel.
    "HUD_SUMMARY": True,

    # Write detailed samples to CSV for offline analysis.
    "LOG_TO_CSV": True,

    # CSV path. The file is created when ar_receiver.py is running.
    "LOG_PATH": os.path.join(BASE_DIR, "performance_debug.csv"),

    # Minimum seconds between CSV rows. Lower values give finer logs but add
    # more disk writes.
    "LOG_INTERVAL": 0.2,

    # Minimum seconds between HUD summary refreshes. The video can still render
    # faster; this only limits text refresh cost.
    "HUD_UPDATE_INTERVAL": 0.5,

    # Minimum seconds between Linux sysfs/procfs hardware probes.
    "SYSTEM_PROBE_INTERVAL": 1.0,

    # RK NPU load path. Some kernels require root/debugfs access; missing files
    # are handled as N/A.
    "NPU_LOAD_PATH": "/sys/kernel/debug/rknpu/load",
    # Non-root processes usually cannot read debugfs. This devfreq node exposes
    # aggregate load as "load@frequency" and is readable on the target board.
    "NPU_LOAD_PATHS": [
        "/sys/kernel/debug/rknpu/load",
        "/sys/class/devfreq/fdab0000.npu/load",
    ],

    # Common RK3588/RK3588S NPU devfreq path. Missing files are handled as N/A.
    "NPU_FREQ_PATH": "/sys/class/devfreq/fdab0000.npu/cur_freq",

    # GPU is not used by the Python RKNN vision pipeline. These optional nodes
    # help reveal load from the official AR compositor or desktop rendering.
    "GPU_LOAD_PATHS": [
        "/sys/class/devfreq/fb000000.gpu/load",
        "/sys/devices/platform/fb000000.gpu/devfreq/fb000000.gpu/load",
    ],
    "GPU_FREQ_PATHS": [
        "/sys/class/devfreq/fb000000.gpu/cur_freq",
        "/sys/devices/platform/fb000000.gpu/devfreq/fb000000.gpu/cur_freq",
    ],

    # Thermal paths checked in order. Values are usually millidegree Celsius.
    "TEMP_PATHS": [
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/thermal/thermal_zone1/temp",
        "/sys/class/thermal/thermal_zone2/temp",
    ],
}

CSV_FIELDS = [
    "timestamp",
    "fid",
    "raw_fps",
    "loop_fps",
    "total_ms",
    "read_ms",
    "vision_ms",
    "vision_total_ms",
    "seg_infer_ms",
    "seg_post_ms",
    "det_infer_ms",
    "det_post_ms",
    "command_ms",
    "planner_draw_ms",
    "serial_ms",
    "runtime_status_ms",
    "hud_ms",
    "display_ms",
    "npu_load",
    "npu_freq_mhz",
    "cpu_usage_percent",
    "cpu_max_core_percent",
    "gpu_load_percent",
    "gpu_freq_mhz",
    "cpu_temp_c",
    "loadavg_1m",
]


def _safe_float(value, default=None):
    try:
        value = float(value)
        if math.isfinite(value):
            return value
    except Exception:
        pass
    return default


def _format_float(value, digits=2):
    value = _safe_float(value)
    if value is None:
        return ""
    return f"{value:.{digits}f}"


def _read_text(path):
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            return f.read().strip()
    except Exception:
        return None


def _read_first_text(paths):
    for path in paths or []:
        text = _read_text(path)
        if text:
            return text
    return None


def _parse_npu_load(text):
    if not text:
        return None
    devfreq_match = re.search(r"([-+]?\d+(?:\.\d+)?)\s*@\s*\d+", text)
    if devfreq_match:
        return [_safe_float(devfreq_match.group(1), 0.0)]
    percent_values = re.findall(r"([-+]?\d+(?:\.\d+)?)\s*%", text)
    if percent_values:
        return [_safe_float(v, 0.0) for v in percent_values[:3]]

    # Fallback for kernels that expose plain numeric load values.
    values = re.findall(r"[-+]?\d+(?:\.\d+)?", text)
    if not values:
        return None
    parsed = [_safe_float(v, 0.0) for v in values]
    return parsed[:3] if parsed else None


def _read_freq_mhz(path):
    text = _read_text(path)
    value = _safe_float(text)
    if value is None:
        return None
    if value > 1_000_000:
        return value / 1_000_000.0
    if value > 1_000:
        return value / 1_000.0
    return value


def _read_first_freq_mhz(paths):
    for path in paths or []:
        value = _read_freq_mhz(path)
        if value is not None:
            return value
    return None


def _parse_device_load(text):
    if not text:
        return None
    match = re.search(r"([-+]?\d+(?:\.\d+)?)\s*%", text)
    if match is None:
        match = re.search(r"[-+]?\d+(?:\.\d+)?", text)
    value = _safe_float(match.group(0).rstrip("%")) if match is not None else None
    if value is None:
        return None
    return max(0.0, min(100.0, value))


def _parse_cpu_stat(text):
    samples = {}
    for line in (text or "").splitlines():
        parts = line.split()
        if not parts or not re.fullmatch(r"cpu\d*", parts[0]):
            continue
        values = []
        for item in parts[1:]:
            try:
                values.append(int(item))
            except Exception:
                break
        if len(values) < 4:
            continue
        total = sum(values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        samples[parts[0]] = (total, idle)
    return samples


def _cpu_usage_percent(previous, current):
    usage = {}
    for name, (total, idle) in (current or {}).items():
        if name not in (previous or {}):
            continue
        prev_total, prev_idle = previous[name]
        total_delta = total - prev_total
        idle_delta = idle - prev_idle
        if total_delta <= 0:
            continue
        usage[name] = max(0.0, min(100.0, 100.0 * (total_delta - idle_delta) / total_delta))
    return usage


def _read_temp_c(paths):
    for path in paths:
        text = _read_text(path)
        value = _safe_float(text)
        if value is None:
            continue
        if value > 1000.0:
            value /= 1000.0
        return value
    return None


def _read_loadavg_1m():
    text = _read_text("/proc/loadavg")
    if not text:
        return None
    parts = text.split()
    return _safe_float(parts[0]) if parts else None


def _join_npu_load(values):
    if not values:
        return ""
    return "/".join(_format_float(v, 0) for v in values)


class PerformanceMonitor:
    """Lightweight timing and hardware probe for the ar_receiver.py main loop."""

    def __init__(self, params=None, log_func=None):
        self.params = dict(PERF_MONITOR_DEFAULTS)
        if params:
            self.params.update(params)
        self.log_func = log_func or print
        self.enabled = bool(self.params["ENABLE"])
        self.csv_enabled = self.enabled and bool(self.params["LOG_TO_CSV"])
        self.hud_enabled = self.enabled and bool(self.params["HUD_SUMMARY"])

        self.stage_ms = {}
        self.system_status = {
            "npu_load": None,
            "npu_freq_mhz": None,
            "cpu_usage_percent": None,
            "cpu_max_core_percent": None,
            "gpu_load_percent": None,
            "gpu_freq_mhz": None,
            "cpu_temp_c": None,
            "loadavg_1m": None,
        }
        self.raw_fps = 0.0
        self.loop_fps = 0.0
        self.current_fid = None
        self.total_ms = 0.0

        self._last_raw_fid = None
        self._last_raw_ts = None
        self._loop_window_ts = time.time()
        self._loop_window_count = 0
        self._frame_start_perf = None
        self._last_log_ts = 0.0
        self._next_system_probe_ts = 0.0
        self._next_hud_ts = 0.0
        self._hud_cache = None
        self._csv_file = None
        self._csv_writer = None
        self._csv_error_logged = False
        self._previous_cpu_stat = None

    def close(self):
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
            self._csv_writer = None

    def start(self):
        if not self.enabled:
            return None
        return time.perf_counter()

    def stop(self, name, token):
        if not self.enabled or token is None:
            return 0.0
        elapsed_ms = (time.perf_counter() - token) * 1000.0
        self.stage_ms[str(name)] = elapsed_ms
        return elapsed_ms

    def record_stages(self, values):
        if not self.enabled or not isinstance(values, dict):
            return
        for name, value in values.items():
            self.stage_ms[str(name)] = _safe_float(value)

    def begin_frame(self, fid, now=None):
        if not self.enabled:
            return
        now = time.time() if now is None else float(now)
        self.current_fid = int(fid)
        self._frame_start_perf = time.perf_counter()
        self._mark_raw_frame(int(fid), now)
        self._probe_system_if_needed(now)

    def finish_frame(self, now=None):
        if not self.enabled:
            return
        now = time.time() if now is None else float(now)
        if self._frame_start_perf is not None:
            self.total_ms = (time.perf_counter() - self._frame_start_perf) * 1000.0
            self._frame_start_perf = None
        self._mark_loop_frame(now)
        self._write_csv_if_needed(now)

    def snapshot(self):
        if not self.enabled:
            return {"enabled": False}
        data = {
            "enabled": True,
            "timestamp": time.time(),
            "fid": self.current_fid,
            "raw_fps": self.raw_fps,
            "loop_fps": self.loop_fps,
            "total_ms": self.total_ms,
            "stages_ms": dict(self.stage_ms),
            "system": dict(self.system_status),
            "log_path": self.params["LOG_PATH"] if self.csv_enabled else None,
        }
        return data

    def hud_snapshot(self):
        if not self.hud_enabled:
            return None
        now = time.time()
        if self._hud_cache is not None and now < self._next_hud_ts:
            return self._hud_cache
        self._hud_cache = self.snapshot()
        self._next_hud_ts = now + float(self.params["HUD_UPDATE_INTERVAL"])
        return self._hud_cache

    def _mark_raw_frame(self, fid, now):
        if self._last_raw_fid is not None and self._last_raw_ts is not None:
            dt = max(1e-6, now - self._last_raw_ts)
            df = fid - self._last_raw_fid
            if df > 0:
                self.raw_fps = df / dt
        self._last_raw_fid = fid
        self._last_raw_ts = now

    def _mark_loop_frame(self, now):
        self._loop_window_count += 1
        dt = now - self._loop_window_ts
        if dt >= 1.0:
            self.loop_fps = self._loop_window_count / max(1e-6, dt)
            self._loop_window_count = 0
            self._loop_window_ts = now

    def _probe_system_if_needed(self, now):
        if now < self._next_system_probe_ts:
            return
        self._next_system_probe_ts = now + float(self.params["SYSTEM_PROBE_INTERVAL"])
        current_cpu_stat = _parse_cpu_stat(_read_text("/proc/stat"))
        cpu_usage = _cpu_usage_percent(self._previous_cpu_stat, current_cpu_stat)
        self._previous_cpu_stat = current_cpu_stat
        core_usage = [value for name, value in cpu_usage.items() if name != "cpu"]
        self.system_status = {
            "npu_load": _parse_npu_load(
                _read_first_text(
                    self.params.get("NPU_LOAD_PATHS")
                    or [self.params.get("NPU_LOAD_PATH")]
                )
            ),
            "npu_freq_mhz": _read_freq_mhz(self.params["NPU_FREQ_PATH"]),
            "cpu_usage_percent": cpu_usage.get("cpu"),
            "cpu_max_core_percent": max(core_usage) if core_usage else None,
            "gpu_load_percent": _parse_device_load(_read_first_text(self.params["GPU_LOAD_PATHS"])),
            "gpu_freq_mhz": _read_first_freq_mhz(self.params["GPU_FREQ_PATHS"]),
            "cpu_temp_c": _read_temp_c(self.params["TEMP_PATHS"]),
            "loadavg_1m": _read_loadavg_1m(),
        }

    def _open_csv(self):
        if self._csv_writer is not None:
            return self._csv_writer
        path = self.params["LOG_PATH"]
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        file_exists = os.path.exists(path) and os.path.getsize(path) > 0
        self._csv_file = open(path, "a", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        if not file_exists:
            self._csv_writer.writeheader()
            self._csv_file.flush()
        return self._csv_writer

    def _write_csv_if_needed(self, now):
        if not self.csv_enabled:
            return
        if now - self._last_log_ts < float(self.params["LOG_INTERVAL"]):
            return
        self._last_log_ts = now

        stages = self.stage_ms
        system = self.system_status
        row = {
            "timestamp": f"{now:.6f}",
            "fid": "" if self.current_fid is None else int(self.current_fid),
            "raw_fps": _format_float(self.raw_fps, 2),
            "loop_fps": _format_float(self.loop_fps, 2),
            "total_ms": _format_float(self.total_ms, 2),
            "read_ms": _format_float(stages.get("read_ms"), 2),
            "vision_ms": _format_float(stages.get("vision_ms"), 2),
            "vision_total_ms": _format_float(stages.get("vision_total_ms"), 2),
            "seg_infer_ms": _format_float(stages.get("seg_infer_ms"), 2),
            "seg_post_ms": _format_float(stages.get("seg_post_ms"), 2),
            "det_infer_ms": _format_float(stages.get("det_infer_ms"), 2),
            "det_post_ms": _format_float(stages.get("det_post_ms"), 2),
            "command_ms": _format_float(stages.get("command_ms"), 2),
            "planner_draw_ms": _format_float(stages.get("planner_draw_ms"), 2),
            "serial_ms": _format_float(stages.get("serial_ms"), 2),
            "runtime_status_ms": _format_float(stages.get("runtime_status_ms"), 2),
            "hud_ms": _format_float(stages.get("hud_ms"), 2),
            "display_ms": _format_float(stages.get("display_ms"), 2),
            "npu_load": _join_npu_load(system.get("npu_load")),
            "npu_freq_mhz": _format_float(system.get("npu_freq_mhz"), 1),
            "cpu_usage_percent": _format_float(system.get("cpu_usage_percent"), 1),
            "cpu_max_core_percent": _format_float(system.get("cpu_max_core_percent"), 1),
            "gpu_load_percent": _format_float(system.get("gpu_load_percent"), 1),
            "gpu_freq_mhz": _format_float(system.get("gpu_freq_mhz"), 1),
            "cpu_temp_c": _format_float(system.get("cpu_temp_c"), 1),
            "loadavg_1m": _format_float(system.get("loadavg_1m"), 2),
        }

        try:
            writer = self._open_csv()
            writer.writerow(row)
            self._csv_file.flush()
        except Exception as exc:
            if not self._csv_error_logged:
                self._csv_error_logged = True
                try:
                    self.log_func(f"performance CSV write failed: {exc}")
                except Exception:
                    pass
