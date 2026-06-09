import time
import atexit
import os
import cv2
from struct import error as StructError
from multiprocessing import shared_memory


from pose_ar_bridge import (
    AR_FORWARD_MAPPING,
    ARPoseBridge,
    DEFAULT_AR_UDP_IP,
    DEFAULT_AR_UDP_PORT,
    POSE_CONVENTION,
    POSE_INPUT_PORT,
)
from control_car_link import (
    CONTROL_FLAG_USE_TARGET_SPEED,
    STATE_SAFE_STOP,
    CarControlLink,
)
from control_arbitrator import ControlArbitrator
from control_states import TaskState, task_state_to_tc264_state
from debug_tools import PosePathDebugPrinter, RotatingDebugLogger
from control_gamepad_receiver import GAMEPAD_CONTROL_PORT, GamepadControlReceiver
from ui_hud_renderer import draw_pose_status
from control_local_planner import LocalPlanner
from status_runtime import RuntimeStatusStore
from control_task_state_machine import TaskStateMachine
from performance_monitor import PerformanceMonitor
from vision_frame_source import read_frame_from_shm, remove_shm_from_resource_tracker
from vision_pipeline import VisionPipeline
from webui_status_server import WebUIStatusServer

SHM_NAME = "shm_ar_video"
SHM_HEADER_SIZE = 16
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "dist", "main_config.json")
TEMPLATE_PATH = os.path.join(BASE_DIR, "templates", "index.html")
STATIC_DIR = os.path.join(BASE_DIR, "static")
RUNTIME_STATUS_INTERVAL = 0.5

# ===== 定位数据终端调试开关 =====
# Windows AprilTag localization sends official robot_position JSON over UDP.
# 稳定后如果嫌终端刷屏，把这里改成 False。
DEBUG_PRINT_POSE = True
DEBUG_PRINT_POSE_EVERY_N = 1
DEBUG_LOG_POSE_TO_FILE = True
DEBUG_LOG_PATH = os.environ.get("AR_POSE_LOG_PATH", os.path.join(BASE_DIR, "ar_pose_debug.log"))
DEBUG_STATUS_PATH = os.environ.get("AR_POSE_STATUS_PATH", os.path.join(BASE_DIR, "ar_pose_status.json"))
DEBUG_PACKET_PATH = os.environ.get("AR_POSE_PACKET_PATH", os.path.join(BASE_DIR, "xverse_control_live.json"))
DEBUG_LOG_MAX_BYTES = 2 * 1024 * 1024
DEBUG_DRAW_POSE_PANEL = True
POSE_STATUS_HTTP_HOST = os.environ.get("AR_POSE_STATUS_HOST", "0.0.0.0")
POSE_STATUS_HTTP_PORT = int(os.environ.get("AR_POSE_STATUS_PORT", "9105"))

# ===== POSE_PATH_DEBUG_START：定位链路分段验证打印，稳定后可整段删除 =====
# The staged debug output identifies UDP input, JSON validation, local mirror, and AR forwarding.
# 删除方法：搜索 POSE_PATH_DEBUG_START 到 POSE_PATH_DEBUG_END，以及代码中的 POSE_PATH_DEBUG 调用点。
POSE_PATH_DEBUG = True
POSE_PATH_DEBUG_EVERY_N = 1
# ===== POSE_PATH_DEBUG_END =====

def send_car_cmd(track_error, target_speed, state_cmd, flags=CONTROL_FLAG_USE_TARGET_SPEED, mark_activity=True):
    return car_link.send_cmd(track_error, target_speed, state_cmd, flags)


def get_control_send_status():
    return car_link.get_send_status()


def get_car_feedback():
    return car_link.get_feedback()

car_link = CarControlLink()
vision_pipeline = None


def get_ai_status():
    if vision_pipeline is None:
        return {
            "ok": False,
            "detector": "not initialized",
            "segmenter": "not initialized",
            "error": None,
        }
    return vision_pipeline.status()


runtime_status = RuntimeStatusStore(
    status_path=DEBUG_STATUS_PATH,
    packet_path=DEBUG_PACKET_PATH,
    debug_log_path=DEBUG_LOG_PATH,
    pose_convention=POSE_CONVENTION,
    ar_forward_mapping=AR_FORWARD_MAPPING,
    default_target=f"{DEFAULT_AR_UDP_IP}:{DEFAULT_AR_UDP_PORT}",
    http_port=POSE_STATUS_HTTP_PORT,
    get_car_feedback=get_car_feedback,
    get_control_send_status=get_control_send_status,
    ai_status_provider=get_ai_status,
)
write_debug_log = RotatingDebugLogger(
    log_path=DEBUG_LOG_PATH,
    max_bytes=DEBUG_LOG_MAX_BYTES,
    enabled=DEBUG_LOG_POSE_TO_FILE,
)
pose_path_debug = PosePathDebugPrinter(
    logger=write_debug_log,
    enabled=POSE_PATH_DEBUG,
    every_n=POSE_PATH_DEBUG_EVERY_N,
)


def stop_car_on_exit():
    try:
        send_car_cmd(0.0, 0.0, STATE_SAFE_STOP, flags=0, mark_activity=False)
    except Exception:
        pass


atexit.register(stop_car_on_exit)


def main():
    global vision_pipeline

    pose_status_server = WebUIStatusServer(
        host=POSE_STATUS_HTTP_HOST,
        port=POSE_STATUS_HTTP_PORT,
        config_path=CONFIG_PATH,
        template_path=TEMPLATE_PATH,
        static_dir=STATIC_DIR,
        objects_path=os.path.join(BASE_DIR, "dist", "objects.json"),
        status_payload_func=runtime_status.current_payload,
        pose_input_port=POSE_INPUT_PORT,
        gamepad_control_port=GAMEPAD_CONTROL_PORT,
        log_func=write_debug_log,
    ).start()
    pose_bridge = ARPoseBridge(
        config_path=CONFIG_PATH,
        packet_path=DEBUG_PACKET_PATH,
        log_path=DEBUG_LOG_PATH,
        status_path=DEBUG_STATUS_PATH,
        log_func=write_debug_log,
        pose_status_writer=runtime_status.write_pose_status,
        pose_path_debug=pose_path_debug,
        debug_print_pose=DEBUG_PRINT_POSE,
        debug_print_every_n=DEBUG_PRINT_POSE_EVERY_N,
    )
    pose_bridge.start()
    gamepad_receiver = GamepadControlReceiver(
        log_func=write_debug_log,
    )
    gamepad_receiver.start()
    control_arbitrator = ControlArbitrator()
    task_state_machine = TaskStateMachine()
    local_planner = LocalPlanner()
    vision_pipeline = VisionPipeline(
        log_func=write_debug_log,
    )
    performance_monitor = PerformanceMonitor(log_func=write_debug_log)

    def build_autonomy_command(now, perception):
        task_decision = task_state_machine.update(perception, now)
        plan_result = local_planner.plan(perception, task_decision, now)
        final_track_error = plan_result["final_track_error"]
        desired_speed = task_decision["desired_speed"]
        task_state = task_decision["task_state"]
        task_state_cmd = int(task_state_to_tc264_state(task_state))
        task_safe_stop = task_state == TaskState.LINE_LOSS_SAFE_STOP.value

        gamepad_cmd, gamepad_status = gamepad_receiver.active_command()
        command = control_arbitrator.decide(
            now,
            final_track_error,
            gamepad_cmd=gamepad_cmd,
            desired_speed=desired_speed,
            state_text=task_state,
            safe_stop=task_safe_stop,
            state_cmd=task_state_cmd,
            line_loss_age=task_decision.get("line_loss_age", 0.0),
        )
        return command, gamepad_status, task_decision, plan_result

    print("vision client ready, waiting for camera shared memory...")
    last_line_loss_command_ts = 0.0

    while True:
        shm = None
        try:
            try:
                shm = shared_memory.SharedMemory(name=SHM_NAME)
                remove_shm_from_resource_tracker(SHM_NAME)
                print("connected to camera shared memory")
            except FileNotFoundError:
                time.sleep(1.0)
                continue

            last_fid = 0
            have_frame = False
            fps_t = time.time()
            fps_n = 0
            cur_fps = 0.0
            last_runtime_status_ts = 0.0

            while True:
                try:
                    perf_token = performance_monitor.start()
                    fid, frame = read_frame_from_shm(shm, SHM_HEADER_SIZE)
                    performance_monitor.stop("read_ms", perf_token)
                    if fid == last_fid:
                        now = time.time()
                        if have_frame and control_arbitrator.should_repeat_command(now, last_line_loss_command_ts):
                            command, gamepad_status, _, _ = build_autonomy_command(now, None)
                            command_error = command["track_error"]
                            command_speed = command["target_speed"]
                            command_state = command["state_cmd"]
                            command_flags = command["flags"]
                            control_state_text = command["state_text"]

                            send_car_cmd(
                                track_error=command_error,
                                target_speed=command_speed,
                                state_cmd=command_state,
                                flags=command_flags,
                            )
                            if now - last_runtime_status_ts >= RUNTIME_STATUS_INTERVAL:
                                runtime_status.write_runtime_status(
                                    pose_bridge,
                                    control_state_text,
                                    command_error,
                                    command_speed,
                                    command_state,
                                    command_flags,
                                    gamepad_status=gamepad_status,
                                    performance_status=performance_monitor.snapshot(),
                                )
                                last_runtime_status_ts = now
                            last_line_loss_command_ts = now
                        time.sleep(0.002)
                        if cv2.waitKey(1) == 27:
                            raise KeyboardInterrupt
                        continue
                    last_fid = fid
                    have_frame = True

                    now = time.time()
                    performance_monitor.begin_frame(fid, now)
                    perf_token = performance_monitor.start()
                    final_frame, perception = vision_pipeline.process(frame, now)
                    performance_monitor.stop("vision_ms", perf_token)
                    perf_token = performance_monitor.start()
                    command, gamepad_status, task_decision, plan_result = build_autonomy_command(now, perception)
                    performance_monitor.stop("command_ms", perf_token)
                    perf_token = performance_monitor.start()
                    final_frame = local_planner.draw_debug(final_frame, task_decision, plan_result)
                    performance_monitor.stop("planner_draw_ms", perf_token)
                    command_error = command["track_error"]
                    command_speed = command["target_speed"]
                    command_state = command["state_cmd"]
                    command_flags = command["flags"]
                    control_state_text = command["state_text"]

                    perf_token = performance_monitor.start()
                    send_car_cmd(
                        track_error=command_error,
                        target_speed=command_speed,
                        state_cmd=command_state,
                        flags=command_flags,
                    )
                    performance_monitor.stop("serial_ms", perf_token)
                    last_line_loss_command_ts = now

                    if now - last_runtime_status_ts >= RUNTIME_STATUS_INTERVAL:
                        perf_token = performance_monitor.start()
                        runtime_status.write_runtime_status(
                            pose_bridge,
                            control_state_text,
                            command_error,
                            command_speed,
                            command_state,
                            command_flags,
                            gamepad_status=gamepad_status,
                            performance_status=performance_monitor.snapshot(),
                        )
                        performance_monitor.stop("runtime_status_ms", perf_token)
                        last_runtime_status_ts = now

                    fps_n += 1
                    if now - fps_t >= 1.0:
                        cur_fps = fps_n / max(1e-6, now - fps_t)
                        fps_n = 0
                        fps_t = now

                    perf_token = performance_monitor.start()
                    final_frame = draw_pose_status(
                        final_frame,
                        pose_bridge,
                        fps=cur_fps,
                        track_error=command_error,
                        control_state=control_state_text,
                        gamepad_status=gamepad_status,
                        get_car_feedback=get_car_feedback,
                        pose_input_port=POSE_INPUT_PORT,
                        performance_status=performance_monitor.hud_snapshot(),
                        enabled=DEBUG_DRAW_POSE_PANEL,
                    )
                    performance_monitor.stop("hud_ms", perf_token)
                    if final_frame is None or final_frame.size == 0:
                        final_frame = frame
                    perf_token = performance_monitor.start()
                    cv2.imshow("ret", final_frame)
                    key = cv2.waitKey(1)
                    performance_monitor.stop("display_ms", perf_token)
                    performance_monitor.finish_frame(time.time())
                    if key == 27:
                        raise KeyboardInterrupt

                except (ValueError, StructError, BufferError):
                    raise FileNotFoundError

        except KeyboardInterrupt:
            print("\nstopped by user")
            break
        except FileNotFoundError:
            print("signal lost, waiting for recovery...")
            if shm:
                try:
                    shm.close()
                except Exception:
                    pass
            cv2.destroyAllWindows()
            time.sleep(1.0)
        finally:
            if shm:
                try:
                    shm.close()
                except Exception:
                    pass

    send_car_cmd(0.0, 0.0, STATE_SAFE_STOP, flags=0, mark_activity=False)
    runtime_status.write_runtime_status(
        pose_bridge,
        "STOPPED_SAFE_STOP",
        0.0,
        0.0,
        STATE_SAFE_STOP,
        0,
        performance_status=performance_monitor.snapshot(),
    )
    pose_bridge.stop()
    gamepad_receiver.stop()
    if pose_status_server is not None:
        pose_status_server.stop()
    if vision_pipeline is not None:
        vision_pipeline.release()
    performance_monitor.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
