import os
import threading
import time

import cv2
import numpy as np

from ui_debug_stream_server import apply_current_thread_affinity
from ui_hud_renderer import draw_pose_status

try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:
    Image = None
    ImageDraw = None
    ImageFont = None

# 中文字体路径（与 ui_hud_renderer 保持一致）
_OCR_FONT_PATHS = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
]
_ocr_font = None

def _get_ocr_font():
    global _ocr_font
    if _ocr_font is not None:
        return _ocr_font
    if ImageFont is None:
        return None
    for path in _OCR_FONT_PATHS:
        if not path:
            continue
        try:
            if __import__('os').path.exists(path):
                _ocr_font = ImageFont.truetype(path, 16)
                return _ocr_font
        except Exception:
            continue
    return None


def _draw_ocr_on_debug_frame(final_frame, ocr_result):
    """直接在 debug_feed 帧上绘制 OCR 信息（cv2 英文 + PIL 中文双保险）。"""
    if ocr_result is None:
        return final_frame

    active = ocr_result.get("active", False)
    status = ocr_result.get("status", "?")
    instruction = ocr_result.get("instruction") or {}
    instruction_current = bool(ocr_result.get("instruction_current", False))
    latest_instruction = ocr_result.get("latest_instruction") or {}
    ocr_data = ocr_result.get("ocr") or {}
    ocr_text = ocr_data.get("text", "")
    ocr_conf = ocr_data.get("confidence", 0.0)
    stable = ocr_result.get("stable", False)

    h, w = final_frame.shape[:2]
    left_panel_w = 300
    right_panel_w = 390
    x_start = left_panel_w + 10
    x_end = w - right_panel_w - 10
    if x_end <= x_start:
        return final_frame

    # === 第一行：cv2 英文状态（100% 可靠） ===
    parts = []
    if active:
        parts.append(f"OCR:{status}")
    else:
        parts.append("OCR:WAITING")
    if ocr_text:
        parts.append(f"conf={ocr_conf:.2f}")
        parts.append(f"stable={'Y' if stable else 'N'}")
    if instruction_current and instruction.get("valid"):
        parts.append(f"API:{instruction.get('action')}")
        parts.append(f"pref:{instruction.get('preferred_branch')}")
        avoid = instruction.get("avoid_branches") or []
        if avoid:
            parts.append(f"avoid:{','.join(avoid)}")
        parts.append(f"api_c={instruction.get('confidence',0):.2f}")
    elif status in ("api_pending", "ocr_pending", "ocr_submitted", "worker_starting"):
        parts.append("API:pending")
    elif latest_instruction.get("valid"):
        parts.append(f"API:last:{latest_instruction.get('action')}")

    cv2_line = " | ".join(parts)

    bar_h = 22
    y_start = h - bar_h - 6

    # 背景条
    bar = final_frame[y_start:y_start + bar_h, x_start:x_end].copy()
    overlay = np.zeros_like(bar)
    overlay[:, :] = (8, 12, 16)
    cv2.addWeighted(overlay, 0.85, bar, 0.15, 0.0, bar)
    final_frame[y_start:y_start + bar_h, x_start:x_end] = bar

    # cv2 英文行（永远能渲染）
    cv2.putText(final_frame, cv2_line, (x_start + 6, y_start + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1, cv2.LINE_AA)

    # === 第二行：PIL 中文文字（如果有） ===
    if ocr_text:
        font = _get_ocr_font()
        bar2_h = 22
        y2_start = y_start - bar2_h - 2
        if y2_start >= 0 and font is not None:
            bar2 = final_frame[y2_start:y2_start + bar2_h, x_start:x_end].copy()
            overlay2 = np.zeros_like(bar2)
            overlay2[:, :] = (8, 12, 16)
            cv2.addWeighted(overlay2, 0.85, bar2, 0.15, 0.0, bar2)

            pil_img = Image.fromarray(cv2.cvtColor(bar2, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_img)
            draw.text((6, 3), f"TXT: {ocr_text}", font=font, fill=(240, 235, 0))
            bar2 = cv2.cvtColor(np.asarray(pil_img), cv2.COLOR_RGB2BGR)
            final_frame[y2_start:y2_start + bar2_h, x_start:x_end] = bar2

    return final_frame


class DebugRenderWorker:
    """Renders visual debug frames off the control loop.

    The worker keeps only the newest control snapshot. If rendering or JPEG
    publishing falls behind, older debug frames are dropped instead of slowing
    down command generation.
    """

    def __init__(
        self,
        vision_pipeline,
        local_planner,
        debug_stream_server,
        pose_bridge,
        get_car_feedback,
        performance_provider,
        pose_input_port,
        draw_pose_panel=True,
        local_preview=False,
        cpu_set="",
        log_func=None,
    ):
        self.vision_pipeline = vision_pipeline
        self.local_planner = local_planner
        self.debug_stream_server = debug_stream_server
        self.pose_bridge = pose_bridge
        self.get_car_feedback = get_car_feedback
        self.performance_provider = performance_provider
        self.pose_input_port = pose_input_port
        self.draw_pose_panel = bool(draw_pose_panel)
        self.local_preview = bool(local_preview)
        self.cpu_set = str(cpu_set or "").strip()
        self.log_func = log_func or print
        self.max_render_fps = float(os.environ.get("AR_DEBUG_RENDER_FPS", "4"))
        self._last_publish_accept_ts = 0.0

        self._condition = threading.Condition()
        self._latest = None
        self._latest_id = 0
        self._enabled = False
        self._thread = None

        self._render_fps = 0.0
        self._render_ms = 0.0
        self._render_count = 0
        self._render_window_ts = time.time()
        self._render_window_count = 0
        self._dropped_before_render = 0
        self._last_rendered_id = -1
        self._last_error = None

    def start(self):
        if self._enabled:
            return self
        self._enabled = True
        self._thread = threading.Thread(
            target=self._run,
            name="debug-render-worker",
            daemon=True,
        )
        self._thread.start()
        return self

    def stop(self):
        self._enabled = False
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.local_preview:
            try:
                cv2.destroyWindow("ret")
            except Exception:
                pass

    def publish(
        self,
        frame,
        frame_id,
        perception,
        task_decision,
        plan_result,
        command,
        gamepad_status,
        control_fps,
        ocr_result=None,
    ):
        if not self._enabled or frame is None:
            return
        now = time.time()
        if self.max_render_fps > 0.0:
            min_interval = 1.0 / max(0.1, self.max_render_fps)
            if now - self._last_publish_accept_ts < min_interval:
                with self._condition:
                    self._dropped_before_render += 1
                return
            self._last_publish_accept_ts = now
        try:
            if frame.size == 0:
                return
        except Exception:
            return
        item = {
            "frame": frame,
            "frame_id": int(frame_id),
            "perception": perception,
            "task_decision": task_decision,
            "plan_result": plan_result,
            "command": command,
            "gamepad_status": gamepad_status,
            "control_fps": control_fps,
            "ocr_result": ocr_result,
            "timestamp": now,
        }
        with self._condition:
            self._latest = item
            self._latest_id += 1
            self._condition.notify_all()

    def snapshot(self):
        with self._condition:
            latest_age = None
            if self._latest is not None:
                latest_age = max(0.0, time.time() - float(self._latest.get("timestamp") or time.time()))
            return {
                "enabled": self._enabled,
                "render_fps": self._render_fps,
                "render_ms": self._render_ms,
                "render_count": self._render_count,
                "latest_id": self._latest_id,
                "latest_age": latest_age,
                "rendered_id": self._last_rendered_id,
                "drop_before_render": self._dropped_before_render,
                "last_error": self._last_error,
                "cpuset": self.cpu_set or None,
            }

    def _run(self):
        apply_current_thread_affinity(self.cpu_set, self.log_func, "debug-render-worker")
        last_seen_id = -1
        while self._enabled:
            with self._condition:
                self._condition.wait_for(
                    lambda: not self._enabled or self._latest_id != last_seen_id,
                    timeout=0.5,
                )
                if not self._enabled:
                    break
                item = self._latest
                latest_id = self._latest_id

            if item is None:
                last_seen_id = latest_id
                continue
            if last_seen_id >= 0 and latest_id > last_seen_id + 1:
                with self._condition:
                    self._dropped_before_render += latest_id - last_seen_id - 1

            render_start = time.perf_counter()
            try:
                final_frame = self.vision_pipeline.draw_debug_frame(
                    item["frame"],
                    item.get("perception"),
                )
                final_frame = self.local_planner.draw_debug(
                    final_frame,
                    item.get("task_decision"),
                    item.get("plan_result"),
                    draw_text=False,
                )

                command = item.get("command") or {}
                performance_status = self._combined_performance_status()
                final_frame = draw_pose_status(
                    final_frame,
                    self.pose_bridge,
                    fps=item.get("control_fps"),
                    track_error=command.get("track_error"),
                    control_state=command.get("state_text", "N/A"),
                    gamepad_status=item.get("gamepad_status"),
                    get_car_feedback=self.get_car_feedback,
                    pose_input_port=self.pose_input_port,
                    performance_status=performance_status,
                    target_speed=command.get("target_speed"),
                    segmentation_status=(item.get("perception") or {}).get("segmentation"),
                    detection_status=(item.get("perception") or {}).get("detection_status"),
                    task_decision=item.get("task_decision"),
                    plan_result=item.get("plan_result"),
                    ocr_result=item.get("ocr_result"),
                    enabled=self.draw_pose_panel,
                )
                if final_frame is None or final_frame.size == 0:
                    final_frame = item["frame"]

                # ---- OCR 信息叠加到 debug_feed 底部 ----
                final_frame = _draw_ocr_on_debug_frame(final_frame, item.get("ocr_result"))

                self.debug_stream_server.publish(final_frame)
                if self.local_preview:
                    cv2.imshow("ret", final_frame)
                    cv2.waitKey(1)
                self._last_error = None
            except Exception as exc:
                self._last_error = str(exc)
                try:
                    self.log_func(f"debug render skipped: {exc}")
                except Exception:
                    pass

            render_ms = (time.perf_counter() - render_start) * 1000.0
            now = time.time()
            with self._condition:
                self._render_ms = render_ms
                self._render_count += 1
                self._last_rendered_id = latest_id
                self._render_window_count += 1
                dt = now - self._render_window_ts
                if dt >= 1.0:
                    self._render_fps = self._render_window_count / max(1e-6, dt)
                    self._render_window_count = 0
                    self._render_window_ts = now
            last_seen_id = latest_id

    def _combined_performance_status(self):
        try:
            performance_status = self.performance_provider() if self.performance_provider else None
        except Exception:
            performance_status = None
        if isinstance(performance_status, dict):
            data = dict(performance_status)
        else:
            data = {"enabled": False}
        data["debug_render"] = self.snapshot()
        try:
            data["debug_stream"] = self.debug_stream_server.snapshot()
        except Exception:
            data["debug_stream"] = None
        return data
