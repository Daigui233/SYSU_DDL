import threading


DEFAULT_VISION_CONTROLS = {
    "enable_detection": True,
    "enable_segmentation": True,
    "enable_model_overlay": True,
}


class VisionRuntimeControls:
    """Thread-safe runtime switches for vision inference and debug overlay."""

    def __init__(self, initial=None):
        self._lock = threading.Lock()
        self._values = dict(DEFAULT_VISION_CONTROLS)
        if isinstance(initial, dict):
            self.update(initial)

    def snapshot(self):
        with self._lock:
            return dict(self._values)

    def update(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("vision controls payload must be an object")
        allowed = set(DEFAULT_VISION_CONTROLS)
        changed = {}
        with self._lock:
            for key, value in payload.items():
                if key not in allowed:
                    continue
                next_value = _coerce_bool(value)
                if self._values.get(key) != next_value:
                    changed[key] = next_value
                self._values[key] = next_value
            return dict(self._values), changed


def _coerce_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() not in ("0", "false", "no", "off", "disabled")
    return bool(value)
