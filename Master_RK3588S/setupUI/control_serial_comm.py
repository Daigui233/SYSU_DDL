import struct
import threading
import time

try:
    import serial
except ImportError:
    serial = None


STATE_TRACK = 1
STATE_SAFE_STOP = 7
CONTROL_FLAG_USE_TARGET_SPEED = 0x01

FRAME_HEAD = 0x42
RX_ADDR = 0x10
RX_PAYLOAD_LEN = 10
TX_ADDR = 0x90
FEEDBACK_V3_PAYLOAD_LEN = 70
FEEDBACK_V2_PAYLOAD_LEN = 46
FEEDBACK_V1_PAYLOAD_LEN = 11


class CarController:
    """TC264D serial transport; control policy remains outside this module."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=460800, reconnect_interval=1.0):
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.reconnect_interval = float(reconnect_interval)

        self.ser = None
        self._rx_buffer = bytearray()
        self._lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._open_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader = None
        self._last_open_attempt = 0.0
        self._last_open_error = ""

        self.feedback_count = 0
        self.feedback_bad_count = 0
        self.raw_rx_count = 0
        self.raw_drop_count = 0
        self.last_feedback = None
        self.last_feedback_ts = 0.0
        self.last_error = ""

        if serial is None:
            self.last_error = "pyserial is unavailable"
            return

        self._ensure_open(force=True)
        self._reader = threading.Thread(target=self._read_loop, name="tc264-feedback", daemon=True)
        self._reader.start()

    def close(self):
        self._stop_event.set()
        with self._lock:
            self._drop_serial_locked()
        if self._reader is not None:
            self._reader.join(timeout=1.0)

    def send_cmd(
        self,
        track_error,
        target_speed,
        state_cmd=STATE_TRACK,
        flags=CONTROL_FLAG_USE_TARGET_SPEED,
    ):
        if not self._ensure_open():
            return False

        payload = struct.pack(
            "<BBBffBB",
            FRAME_HEAD,
            RX_ADDR,
            RX_PAYLOAD_LEN,
            float(track_error),
            float(target_speed),
            int(state_cmd),
            int(flags),
        )
        frame = payload + bytes([sum(payload) & 0xFF])
        try:
            with self._lock:
                if self.ser is None or not self.ser.is_open:
                    return False
                written = self.ser.write(frame)
                if written != len(frame):
                    self.last_error = f"serial write incomplete: {written}/{len(frame)}"
                    self._drop_serial_locked()
                    return False
                self.last_error = ""
            return True
        except Exception as exc:
            self._drop_serial(exc)
            return False

    def get_feedback(self):
        with self._lock:
            feedback = dict(self.last_feedback) if self.last_feedback else None
            timestamp = self.last_feedback_ts
            result = {
                "online": bool(self.ser is not None and getattr(self.ser, "is_open", False)),
                "port": self.port,
                "count": self.feedback_count,
                "bad": self.feedback_bad_count,
                "raw_rx": self.raw_rx_count,
                "raw_drop": self.raw_drop_count,
                "age": time.time() - timestamp if timestamp else None,
                "error": self.last_error,
            }
        if feedback:
            result.update(feedback)
        return result

    def _read_loop(self):
        while not self._stop_event.is_set():
            if not self._ensure_open():
                self._stop_event.wait(0.05)
                continue
            try:
                with self._lock:
                    ser_instance = self.ser
                if ser_instance is None or not ser_instance.is_open:
                    self._stop_event.wait(0.05)
                    continue
                waiting = int(getattr(ser_instance, "in_waiting", 0))
                data = ser_instance.read(waiting or 1)
                if not data:
                    self._stop_event.wait(0.005)
                    continue
                with self._lock:
                    self.raw_rx_count += len(data)
                with self._rx_lock:
                    self._rx_buffer.extend(data)
                    self._parse_feedback_buffer()
            except Exception as exc:
                self._drop_serial(exc)
                self._stop_event.wait(0.05)

    def _ensure_open(self, force=False):
        if serial is None or self._stop_event.is_set():
            return False
        now = time.time()
        with self._lock:
            if self.ser is not None and getattr(self.ser, "is_open", False):
                return True
            if not force and now - self._last_open_attempt < self.reconnect_interval:
                return False
            self._last_open_attempt = now

        if not self._open_lock.acquire(blocking=False):
            return False
        try:
            new_serial = serial.Serial(self.port, self.baudrate, timeout=0.02)
            with self._rx_lock:
                self._rx_buffer.clear()
            with self._lock:
                self.ser = new_serial
                self.last_error = ""
                self._last_open_error = ""
            return True
        except Exception as exc:
            message = str(exc)
            with self._lock:
                self.last_error = message
                self._last_open_error = message
            return False
        finally:
            self._open_lock.release()

    def _drop_serial_locked(self):
        ser_instance = self.ser
        self.ser = None
        if ser_instance is not None:
            try:
                ser_instance.close()
            except Exception:
                pass

    def _drop_serial(self, exc):
        with self._lock:
            self.last_error = str(exc)
            self._drop_serial_locked()

    def _parse_feedback_buffer(self):
        while self._rx_buffer:
            head_index = self._rx_buffer.find(bytes([FRAME_HEAD]))
            if head_index < 0:
                with self._lock:
                    self.raw_drop_count += len(self._rx_buffer)
                self._rx_buffer.clear()
                return
            if head_index:
                with self._lock:
                    self.raw_drop_count += head_index
                del self._rx_buffer[:head_index]
            if len(self._rx_buffer) < 4:
                return
            if self._rx_buffer[1] != TX_ADDR:
                with self._lock:
                    self.raw_drop_count += 1
                del self._rx_buffer[0]
                continue

            payload_len = self._rx_buffer[2]
            frame_len = payload_len + 4
            if len(self._rx_buffer) < frame_len:
                return
            frame = bytes(self._rx_buffer[:frame_len])
            del self._rx_buffer[:frame_len]
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                with self._lock:
                    self.feedback_bad_count += 1
                continue

            feedback = self._decode_feedback(frame, payload_len)
            if feedback is None:
                with self._lock:
                    self.feedback_bad_count += 1
                continue
            with self._lock:
                self.last_feedback = feedback
                self.last_feedback_ts = time.time()
                self.feedback_count += 1

    @staticmethod
    def _decode_feedback(frame, payload_len):
        try:
            if payload_len in (FEEDBACK_V2_PAYLOAD_LEN, FEEDBACK_V3_PAYLOAD_LEN):
                feedback = {
                    "actual_speed": struct.unpack_from("<f", frame, 3)[0],
                    "motor_target": struct.unpack_from("<f", frame, 7)[0],
                    "input_target_speed": struct.unpack_from("<f", frame, 11)[0],
                    "input_track_error": struct.unpack_from("<f", frame, 15)[0],
                    "motor_output": struct.unpack_from("<i", frame, 19)[0],
                    "servo_output": struct.unpack_from("<i", frame, 23)[0],
                    "state": frame[47],
                    "flags": frame[48],
                    "format": "v3" if payload_len == FEEDBACK_V3_PAYLOAD_LEN else "v2",
                }
                if payload_len == FEEDBACK_V3_PAYLOAD_LEN:
                    age_ms = struct.unpack_from("<H", frame, 49)[0]
                    feedback.update(
                        {
                            "input_age_ms": None if age_ms == 0xFFFF else int(age_ms),
                            "safety_flags": struct.unpack_from("<H", frame, 51)[0],
                            "servo_raw_output": struct.unpack_from("<f", frame, 53)[0],
                            "servo_limited_output": struct.unpack_from("<f", frame, 57)[0],
                            "motor_feedforward": struct.unpack_from("<f", frame, 61)[0],
                            "motor_pid_correction": struct.unpack_from("<f", frame, 65)[0],
                            "feedback_seq": struct.unpack_from("<I", frame, 69)[0],
                        }
                    )
                return feedback
            if payload_len == FEEDBACK_V1_PAYLOAD_LEN:
                return {
                    "actual_speed": struct.unpack_from("<f", frame, 3)[0],
                    "motor_output": struct.unpack_from("<i", frame, 7)[0],
                    "servo_output": struct.unpack_from("<H", frame, 11)[0],
                    "state": frame[13],
                    "flags": None,
                    "format": "v1",
                }
        except (IndexError, struct.error):
            return None
        return None
