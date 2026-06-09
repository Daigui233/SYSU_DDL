import threading
import time


try:
    import serial_comm as _serial_comm
except Exception as exc:
    _serial_comm = None
    print(f"serial_comm unavailable: {exc}; car serial control disabled")


STATE_TRACK = getattr(_serial_comm, "STATE_TRACK", 1) if _serial_comm else 1
STATE_SAFE_STOP = getattr(_serial_comm, "STATE_SAFE_STOP", 7) if _serial_comm else 7
CONTROL_FLAG_USE_TARGET_SPEED = (
    getattr(_serial_comm, "CONTROL_FLAG_USE_TARGET_SPEED", 0x01) if _serial_comm else 0x01
)
CarController = getattr(_serial_comm, "CarController", None) if _serial_comm else None


class DisabledCarController:
    def send_cmd(self, *args, **kwargs):
        return False

    def get_feedback(self):
        return {"online": False, "error": "car serial control disabled"}


class CarControlLink:
    """Thin wrapper around serial_comm for RK3588S -> TC264D control frames."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=460800, log_func=None):
        self.port = port
        self.baudrate = int(baudrate)
        self.log_func = log_func
        self.lock = threading.Lock()
        self.last_send_ok = None
        self.last_send_error = ""
        self.last_send_ts = 0.0
        self.car = self._create_controller()

    def _log(self, message):
        if self.log_func is not None:
            self.log_func(message)
        else:
            print(message)

    def _create_controller(self):
        if CarController is None:
            self._log("CarController unavailable; car serial control disabled")
            return DisabledCarController()
        try:
            return CarController(port=self.port, baudrate=self.baudrate)
        except TypeError:
            try:
                return CarController(self.port, self.baudrate)
            except Exception as exc:
                self._log(f"CarController init failed: {exc}; car serial control disabled")
                return DisabledCarController()
        except Exception as exc:
            self._log(f"CarController init failed: {exc}; car serial control disabled")
            return DisabledCarController()

    def _send_unlocked(self, track_error, target_speed, state_cmd, flags):
        send_cmd = getattr(self.car, "send_cmd", None)
        if send_cmd is None:
            return False, "send_cmd unavailable"
        try:
            result = send_cmd(
                track_error=track_error,
                target_speed=target_speed,
                state_cmd=state_cmd,
                flags=flags,
            )
        except TypeError:
            try:
                result = send_cmd(track_error, target_speed, state_cmd, flags)
            except TypeError:
                try:
                    result = send_cmd(track_error)
                except Exception as exc:
                    self._log(f"serial send skip: {exc}")
                    return False, str(exc)
            except Exception as exc:
                self._log(f"serial send skip: {exc}")
                return False, str(exc)
        except Exception as exc:
            self._log(f"serial send skip: {exc}")
            return False, str(exc)
        ok = True if result is None else bool(result)
        return ok, "" if ok else "serial send returned false"

    def send_cmd(self, track_error, target_speed, state_cmd, flags=CONTROL_FLAG_USE_TARGET_SPEED):
        with self.lock:
            ok, error = self._send_unlocked(track_error, target_speed, state_cmd, flags)
            self.last_send_ok = ok
            self.last_send_error = error
            self.last_send_ts = time.time()
            return ok

    def safe_stop(self):
        return self.send_cmd(0.0, 0.0, STATE_SAFE_STOP, flags=0)

    def get_send_status(self):
        with self.lock:
            return {
                "ok": self.last_send_ok,
                "error": self.last_send_error or None,
                "timestamp": self.last_send_ts or None,
            }

    def get_feedback(self):
        getter = getattr(self.car, "get_feedback", None)
        if getter is None:
            return {"online": False, "error": "feedback unavailable"}
        try:
            return getter()
        except Exception as exc:
            return {"online": False, "error": str(exc)}
