import struct
import threading
import time

try:
    import serial
except Exception:
    serial = None


STATE_IDLE = 0
STATE_TRACK = 1
STATE_AVOID_CAR = 2
STATE_AVOID_HUMAN = 3
STATE_COLLECT_GOLD = 4
STATE_RECOVER_LINE = 5
STATE_LINE_LOSS_SAFE_STOP = 6
STATE_SAFE_STOP = 7
STATE_AVOID_STONE = 8
STATE_TRAFFIC_LIGHT_STOP = 9
STATE_ENDSIGN_STOP = 10
CONTROL_FLAG_USE_TARGET_SPEED = 0x01

FRAME_HEAD = 0x42
RX_ADDR = 0x10
RX_PAYLOAD_LEN = 10
TX_ADDR = 0x90
FEEDBACK_PAYLOAD_LEN = 46
FEEDBACK_FRAME_LEN = FEEDBACK_PAYLOAD_LEN + 4
OLD_FEEDBACK_PAYLOAD_LEN = 11


class CarController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=460800, reconnect_interval=1.0):
        self.ser = None
        self._rx_buffer = bytearray()
        self._lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._open_lock = threading.Lock()
        self._stop = False
        self.feedback_count = 0
        self.feedback_bad_count = 0
        self.raw_rx_count = 0
        self.raw_drop_count = 0
        self.last_rx_hex = ""
        self.last_feedback = None
        self.last_feedback_ts = 0.0
        self.last_error = ""
        self.port = port
        self.baudrate = baudrate
        self.reconnect_interval = float(reconnect_interval)
        self._last_open_attempt = 0.0
        self._last_open_error = ""

        if serial is None:
            self.last_error = "serial module unavailable"
            print("serial module unavailable; vision continues")
            return

        self._ensure_open(force=True)
        self._reader = threading.Thread(target=self._read_loop, name="tc264-feedback", daemon=True)
        self._reader.start()

    def send_cmd(self, track_error: float, target_speed: float, state_cmd: int = STATE_TRACK, flags: int = CONTROL_FLAG_USE_TARGET_SPEED):
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
        checksum = sum(payload) & 0xFF
        try:
            frame = payload + struct.pack("<B", checksum)
            with self._lock:
                ser = self.ser
                if ser is None or not getattr(ser, "is_open", False):
                    return False
                written = ser.write(frame)
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
            data = dict(self.last_feedback) if self.last_feedback else None
            ts = self.last_feedback_ts
            count = self.feedback_count
            bad = self.feedback_bad_count
            raw_rx = self.raw_rx_count
            raw_drop = self.raw_drop_count
            last_rx = self.last_rx_hex
            err = self.last_error
        if data is None:
            return {
                "online": False,
                "port": self.port,
                "count": count,
                "bad": bad,
                "raw_rx": raw_rx,
                "raw_drop": raw_drop,
                "last_rx": last_rx,
                "age": None,
                "error": err,
            }
        data["online"] = True
        data["port"] = self.port
        data["count"] = count
        data["bad"] = bad
        data["raw_rx"] = raw_rx
        data["raw_drop"] = raw_drop
        data["last_rx"] = last_rx
        data["age"] = time.time() - ts if ts else None
        data["error"] = err
        return data

    def _read_loop(self):
        while not self._stop:
            if not self._ensure_open():
                time.sleep(0.05)
                continue
            try:
                with self._lock:
                    ser = self.ser
                if ser is None or not getattr(ser, "is_open", False):
                    time.sleep(0.05)
                    continue
                waiting = getattr(ser, "in_waiting", 0)
                data = ser.read(waiting or 1)
                if data:
                    with self._lock:
                        self.raw_rx_count += len(data)
                        self.last_rx_hex = data[-12:].hex(" ")
                    with self._rx_lock:
                        self._rx_buffer.extend(data)
                        self._parse_feedback_buffer()
                else:
                    time.sleep(0.005)
            except Exception as exc:
                self._drop_serial(exc)
                time.sleep(0.05)

    def _ensure_open(self, force=False):
        if serial is None:
            return False
        now = time.time()
        with self._lock:
            ser = self.ser
            if ser is not None and getattr(ser, "is_open", False):
                return True
            if not force and now - self._last_open_attempt < self.reconnect_interval:
                return False
            self._last_open_attempt = now
        if not self._open_lock.acquire(blocking=False):
            return False
        try:
            with self._lock:
                ser = self.ser
                if ser is not None and getattr(ser, "is_open", False):
                    return True
            new_ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
            with self._rx_lock:
                self._rx_buffer.clear()
            with self._lock:
                self.ser = new_ser
                self.last_error = ""
                self._last_open_error = ""
            print(f"serial {self.port} opened")
            return True
        except Exception as exc:
            msg = str(exc)
            should_print = False
            with self._lock:
                self.last_error = msg
                if msg != self._last_open_error:
                    self._last_open_error = msg
                    should_print = True
            if should_print:
                print(f"serial open failed: {msg}; retrying")
            return False
        finally:
            self._open_lock.release()

    def _drop_serial_locked(self):
        ser = self.ser
        self.ser = None
        if ser is not None:
            try:
                ser.close()
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
                del self._rx_buffer[:]
                return
            if head_index > 0:
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
