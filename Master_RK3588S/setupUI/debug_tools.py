import os
import time


class RotatingDebugLogger:
    """Small callable logger with optional file rotation."""

    def __init__(self, log_path, max_bytes=2 * 1024 * 1024, enabled=True):
        self.log_path = log_path
        self.max_bytes = int(max_bytes)
        self.enabled = bool(enabled)

    def __call__(self, message, echo=True):
        line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}"
        if echo:
            print(line)
        if not self.enabled:
            return
        try:
            self.rotate_if_needed()
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception as exc:
            print(f"pose debug log write failed: {exc}")

    def rotate_if_needed(self):
        try:
            if os.path.exists(self.log_path) and os.path.getsize(self.log_path) > self.max_bytes:
                old_path = self.log_path + ".old"
                if os.path.exists(old_path):
                    os.remove(old_path)
                os.replace(self.log_path, old_path)
        except Exception:
            pass


class PosePathDebugPrinter:
    """Rate-limited pose bridge debug hook."""

    def __init__(self, logger, enabled=True, every_n=1):
        self.logger = logger
        self.enabled = bool(enabled)
        self.every_n = max(1, int(every_n))

    def __call__(self, stage, message, count=None, force=False):
        if not self.enabled:
            return
        if count is not None and not force and count % self.every_n != 0:
            return
        self.logger(f"[POSE_PATH_DEBUG] {stage}: {message}")
