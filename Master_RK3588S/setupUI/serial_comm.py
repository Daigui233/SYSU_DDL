import struct
import threading
import time

try:
    import serial
except Exception:
    serial = None


STATE_IDLE = 0
STATE_TRACK = 1
STATE_SAFE_STOP = 7
CONTROL_FLAG_USE_TARGET_SPEED = 0x01

FRAME_HEAD = 0x42
RX_ADDR = 0x10
RX_PAYLOAD_LEN = 10
TX_ADDR = 0x90
FEEDBACK_PAYLOAD_LEN = 46
FEEDBACK_FRAME_LEN = FEEDBACK_PAYLOAD_LEN + 4
OLD_FEEDBACK_PAYLOAD_LEN = 11


class CarController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=460800):
        self.ser = None
        self._rx_buffer = bytearray()
        self._lock = threading.Lock()
        self._stop = False
        self.feedback_count = 0
        self.feedback_bad_count = 0
        self.last_feedback = None
        self.last_feedback_ts = 0.0
        self.last_error = ""
        self.port = port
        self.baudrate = baudrate

        if serial is None:
            self.last_error = "serial module unavailable"
            print("serial module unavailable; vision continues")
            return
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.02)
            print(f"serial {port} opened")
            self._reader = threading.Thread(target=self._read_loop, name="tc264-feedback", daemon=True)
            self._reader.start()
        except Exception as exc:
            self.last_error = str(exc)
            print(f"serial open failed: {exc}; vision continues")
            self.ser = None

    def send_cmd(self, track_error: float, target_speed: float, state_cmd: int = STATE_TRACK, flags: int = CONTROL_FLAG_USE_TARGET_SPEED):
        if self.ser is None or not getattr(self.ser, "is_open", False):
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
        checksum = sum(payload) & 0xFF
        try:
            frame = payload + struct.pack("<B", checksum)
            with self._lock:
                written = self.ser.write(frame)
                if written != len(frame):
                    self.last_error = f"serial write incomplete: {written}/{len(frame)}"
                    return False
                self.last_error = ""
                return True
        except Exception as exc:
            with self._lock:
                self.last_error = str(exc)
            return False

    def get_feedback(self):
        with self._lock:
            data = dict(self.last_feedback) if self.last_feedback else None
            ts = self.last_feedback_ts
            count = self.feedback_count
            bad = self.feedback_bad_count
            err = self.last_error
        if data is None:
            return {
                "online": False,
                "port": self.port,
                "count": count,
                "bad": bad,
                "age": None,
                "error": err,
            }
        data["online"] = True
        data["port"] = self.port
        data["count"] = count
        data["bad"] = bad
        data["age"] = time.time() - ts if ts else None
        data["error"] = err
        return data

    def _read_loop(self):
        while not self._stop:
            try:
                if self.ser is None or not getattr(self.ser, "is_open", False):
                    time.sleep(0.05)
                    continue
                waiting = getattr(self.ser, "in_waiting", 0)
                data = self.ser.read(waiting or 1)
                if data:
                    self._rx_buffer.extend(data)
                    self._parse_feedback_buffer()
                else:
                    time.sleep(0.005)
            except Exception as exc:
                with self._lock:
                    self.last_error = str(exc)
                time.sleep(0.05)

    def _parse_feedback_buffer(self):
        while self._rx_buffer:
            head_index = self._rx_buffer.find(bytes([FRAME_HEAD]))
            if head_index < 0:
                del self._rx_buffer[:]
                return
            if head_index > 0:
                del self._rx_buffer[:head_index]
            if len(self._rx_buffer) < 4:
                return
            if self._rx_buffer[1] != TX_ADDR:
                del self._rx_buffer[0]
                continue
            payload_len = self._rx_buffer[2]
            frame_len = payload_len + 4
            if len(self._rx_buffer) < frame_len:
                return
            frame = bytes(self._rx_buffer[:frame_len])
            del self._rx_buffer[:frame_len]
            checksum = sum(frame[:-1]) & 0xFF
            if checksum != frame[-1]:
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

    def _decode_feedback(self, frame, payload_len):
        try:
            if payload_len == FEEDBACK_PAYLOAD_LEN:
                return {
                    "actual_speed": struct.unpack_from("<f", frame, 3)[0],
                    "motor_target": struct.unpack_from("<f", frame, 7)[0],
                    "input_target_speed": struct.unpack_from("<f", frame, 11)[0],
                    "input_track_error": struct.unpack_from("<f", frame, 15)[0],
                    "motor_output": struct.unpack_from("<i", frame, 19)[0],
                    "servo_output": struct.unpack_from("<i", frame, 23)[0],
                    "motor_kp": struct.unpack_from("<f", frame, 27)[0],
                    "motor_ki": struct.unpack_from("<f", frame, 31)[0],
                    "motor_kd": struct.unpack_from("<f", frame, 35)[0],
                    "servo_kp": struct.unpack_from("<f", frame, 39)[0],
                    "servo_kd": struct.unpack_from("<f", frame, 43)[0],
                    "state": frame[47],
                    "flags": frame[48],
                    "format": "v2",
                }
            if payload_len == OLD_FEEDBACK_PAYLOAD_LEN:
                return {
                    "actual_speed": struct.unpack_from("<f", frame, 3)[0],
                    "motor_output": struct.unpack_from("<i", frame, 7)[0],
                    "servo_output": struct.unpack_from("<H", frame, 11)[0],
                    "state": frame[13],
                    "flags": None,
                    "format": "v1",
                }
        except Exception as exc:
            with self._lock:
                self.last_error = str(exc)
        return None
