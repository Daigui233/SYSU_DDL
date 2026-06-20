import threading
import time

from control_serial_comm import (
    CONTROL_FLAG_USE_TARGET_SPEED,
    STATE_SAFE_STOP,
    STATE_TRACK,
    CarController,
)


SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 460800


class CarControlLink:
    """Small thread-safe facade for the RK3588S to TC264D serial link."""

    def __init__(self, port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE):
        self.controller = CarController(port=port, baudrate=baudrate)
        self._lock = threading.Lock()
        self.last_send_ok = None
        self.last_send_timestamp = 0.0

    def send_cmd(
        self,
        track_error,
        target_speed,
        state_cmd=STATE_TRACK,
        flags=CONTROL_FLAG_USE_TARGET_SPEED,
    ):
        with self._lock:
            self.last_send_ok = self.controller.send_cmd(
                track_error=track_error,
                target_speed=target_speed,
                state_cmd=state_cmd,
                flags=flags,
            )
            self.last_send_timestamp = time.time()
            return self.last_send_ok

    def safe_stop(self):
        return self.send_cmd(0.0, 0.0, state_cmd=STATE_SAFE_STOP, flags=0)

    def get_feedback(self):
        return self.controller.get_feedback()

    def get_send_status(self):
        with self._lock:
            return {
                "ok": self.last_send_ok,
                "timestamp": self.last_send_timestamp or None,
            }

    def close(self):
        self.controller.close()
