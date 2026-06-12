import threading
import time

import cv2

from ui_debug_stream_server import apply_current_thread_affinity
from ui_hud_renderer import draw_pose_status


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
    ):
        if not self._enabled or frame is None:
            return
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
            "timestamp": time.time(),
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
                    enabled=self.draw_pose_panel,
                )
                if final_frame is None or final_frame.size == 0:
                    final_frame = item["frame"]

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
