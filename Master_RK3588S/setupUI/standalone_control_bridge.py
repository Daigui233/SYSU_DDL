import os
import signal
import threading
import time

from ar_pose_bridge import (
    AR_FORWARD_MAPPING,
    ARPoseBridge,
    DEFAULT_AR_UDP_IP,
    DEFAULT_AR_UDP_PORT,
    DEBUG_PACKET_PATH,
    POSE_CONVENTION,
    write_json_atomic,
)
from car_control_link import (
    CONTROL_FLAG_USE_TARGET_SPEED,
    STATE_SAFE_STOP,
    STATE_TRACK,
    CarControlLink,
)
from gamepad_control_receiver import GamepadControlReceiver


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATUS_PATH = os.environ.get(
    "AR_STANDALONE_STATUS_PATH",
    os.path.join(BASE_DIR, "standalone_control_bridge_status.json"),
)
LOG_PATH = os.environ.get(
    "AR_STANDALONE_LOG_PATH",
    os.path.join(BASE_DIR, "standalone_control_bridge.log"),
)
CONTROL_INTERVAL = float(os.environ.get("AR_STANDALONE_CONTROL_INTERVAL", "0.05"))
STATUS_INTERVAL = float(os.environ.get("AR_STANDALONE_STATUS_INTERVAL", "0.5"))
SERIAL_PORT = os.environ.get("AR_CAR_SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUDRATE = int(os.environ.get("AR_CAR_SERIAL_BAUDRATE", "460800"))
SAFE_STOP_ON_EXIT = os.environ.get("AR_STANDALONE_SAFE_STOP_ON_EXIT", "1") != "0"


def log(message):
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}"
    print(line)
    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line + "\n")
    except Exception:
        pass


def build_status(pose_bridge, gamepad_receiver, car_link, control_state, command):
    pose_info = pose_bridge.snapshot()
    gamepad_status = gamepad_receiver.snapshot()
    return {
        "status": "standalone_control_bridge running",
        "timestamp": time.time(),
        "mode": "manual backup bridge",
        "note": "Use this only when ar_receiver.py is not running.",
        "pose": {
            "status": pose_info.get("status"),
            "input": pose_info.get("input"),
            "target": pose_info.get("target", f"{DEFAULT_AR_UDP_IP}:{DEFAULT_AR_UDP_PORT}"),
            "packet_count": pose_info.get("packet_count", 0),
            "invalid_count": pose_info.get("invalid_count", 0),
            "udp_send_count": pose_info.get("udp_send_count", 0),
            "udp_fail_count": pose_info.get("udp_fail_count", 0),
            "last_input_packet": pose_info.get("last_input_packet"),
            "last_ar_packet": pose_info.get("last_ar_packet"),
        },
        "pose_convention": POSE_CONVENTION,
        "ar_forward_mapping": AR_FORWARD_MAPPING,
        "gamepad": gamepad_status,
        "control": {
            "state": control_state,
            "command": command,
            "serial_send": car_link.get_send_status(),
        },
        "car_feedback": car_link.get_feedback(),
        "debug_log": LOG_PATH,
        "packet_json": DEBUG_PACKET_PATH,
    }


def write_status(pose_bridge, gamepad_receiver, car_link, control_state, command):
    try:
        write_json_atomic(STATUS_PATH, build_status(pose_bridge, gamepad_receiver, car_link, control_state, command))
    except Exception as exc:
        log(f"standalone status write failed: {exc}")


def run_control_loop(pose_bridge, gamepad_receiver, car_link, stop_event):
    last_status_ts = 0.0
    was_gamepad_active = False
    sent_timeout_stop = False
    control_state = "IDLE_NO_COMMAND"
    last_command = None

    while not stop_event.is_set():
        now = time.time()
        gamepad_cmd, _gamepad_status = gamepad_receiver.active_command()

        if gamepad_cmd is not None:
            control_state = "GAMEPAD_SAFE_STOP" if gamepad_cmd.get("safe_stop") else "GAMEPAD_TRACK"
            last_command = {
                "track_error": float(gamepad_cmd["track_error"]),
                "target_speed": float(gamepad_cmd["target_speed"]),
                "state_cmd": int(gamepad_cmd["state_cmd"]),
                "flags": int(gamepad_cmd["flags"]),
                "safe_stop": bool(gamepad_cmd.get("safe_stop", False)),
            }
            car_link.send_cmd(
                track_error=last_command["track_error"],
                target_speed=last_command["target_speed"],
                state_cmd=last_command["state_cmd"],
                flags=last_command["flags"],
            )
            was_gamepad_active = True
            sent_timeout_stop = False
        elif was_gamepad_active and not sent_timeout_stop:
            control_state = "GAMEPAD_TIMEOUT_SAFE_STOP"
            last_command = {
                "track_error": 0.0,
                "target_speed": 0.0,
                "state_cmd": STATE_SAFE_STOP,
                "flags": 0,
                "safe_stop": True,
            }
            car_link.safe_stop()
            sent_timeout_stop = True
        else:
            control_state = "IDLE_NO_COMMAND"

        if now - last_status_ts >= STATUS_INTERVAL:
            write_status(pose_bridge, gamepad_receiver, car_link, control_state, last_command)
            last_status_ts = now

        time.sleep(max(0.01, CONTROL_INTERVAL))


def main():
    stop_event = threading.Event()

    def request_stop(_signum=None, _frame=None):
        stop_event.set()

    for sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None)):
        if sig is not None:
            try:
                signal.signal(sig, request_stop)
            except Exception:
                pass

    car_link = CarControlLink(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, log_func=log)
    pose_bridge = ARPoseBridge(
        log_path=LOG_PATH,
        status_path=STATUS_PATH,
        log_func=log,
        debug_print_pose=False,
    )
    gamepad_receiver = GamepadControlReceiver(
        state_track=STATE_TRACK,
        state_safe_stop=STATE_SAFE_STOP,
        control_flag_use_target_speed=CONTROL_FLAG_USE_TARGET_SPEED,
        log_func=log,
    )

    try:
        log("=" * 60)
        log("standalone_control_bridge starting")
        log("This is a manual backup bridge. Do not run it together with ar_receiver.py.")
        log("Functions: Windows pose UDP -> official AR, gamepad UDP -> TC264D serial.")
        pose_bridge.start()
        gamepad_receiver.start()
        run_control_loop(pose_bridge, gamepad_receiver, car_link, stop_event)
    finally:
        if SAFE_STOP_ON_EXIT:
            car_link.safe_stop()
        write_status(
            pose_bridge,
            gamepad_receiver,
            car_link,
            "STOPPED",
            {
                "track_error": 0.0,
                "target_speed": 0.0,
                "state_cmd": STATE_SAFE_STOP,
                "flags": 0,
                "safe_stop": True,
            },
        )
        gamepad_receiver.stop()
        pose_bridge.stop()
        log("standalone_control_bridge stopped")


if __name__ == "__main__":
    main()
