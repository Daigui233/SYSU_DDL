import time
import atexit
import os
import math
import re
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
from control_local_planner import LocalPlanner
from status_runtime import RuntimeStatusStore
from control_task_state_machine import TaskStateMachine
from cpu_affinity import ProcessAffinityGuard, apply_current_thread_affinity
from performance_monitor import PerformanceMonitor
from ui_debug_stream_server import DebugStreamServer
from ui_debug_render_worker import DebugRenderWorker
from vision_frame_source import FrameSnapshotChanged, read_frame_from_shm, remove_shm_from_resource_tracker
from vision_pipeline import VisionPipeline
from vision_runtime_controls import VisionRuntimeControls
from webui_status_server import WebUIStatusServer
from turnsign_ocr_api import AsyncTurnSignOcrApiProcessor, TurnSignOcrApiProcessor

SHM_NAME = "shm_ar_video"
SHM_HEADER_SIZE = 16
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "dist", "main_config.json")
TEMPLATE_PATH = os.path.join(BASE_DIR, "templates", "index.html")
STATIC_DIR = os.path.join(BASE_DIR, "static")
RUNTIME_STATUS_INTERVAL = 0.5


def _load_ocr_env_file():
    """Load the token from the local OCR env file without executing shell code."""
    if os.environ.get("AISTUDIO_ACCESS_TOKEN"):
        return
    path = os.path.join(BASE_DIR, "ocr_env.sh")
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                match = re.match(r"\s*export\s+AISTUDIO_ACCESS_TOKEN\s*=\s*(.*?)\s*$", raw_line)
                if not match:
                    continue
                value = match.group(1).strip().strip("\"'")
                if value and "your-token" not in value and "你的token" not in value:
                    os.environ["AISTUDIO_ACCESS_TOKEN"] = value
                    print("[OCR] loaded AI Studio token from ocr_env.sh")
                return
    except FileNotFoundError:
        return
    except OSError as exc:
        print(f"[OCR] could not read ocr_env.sh: {exc}")


_load_ocr_env_file()


def _env_flag(name, default):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() not in ("0", "false", "no", "off")


def _env_int(name, default):
    try:
        return int(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return int(default)

# ===== 定位数据终端调试开关 =====
# Windows AprilTag localization sends official robot_position JSON over UDP.
# 稳定后如果嫌终端刷屏，把这里改成 False。
DEBUG_PRINT_POSE = _env_flag("AR_DEBUG_PRINT_POSE", False)
DEBUG_PRINT_POSE_EVERY_N = max(1, _env_int("AR_DEBUG_PRINT_POSE_EVERY_N", 60))
DEBUG_LOG_POSE_TO_FILE = _env_flag("AR_POSE_LOG_TO_FILE", False)
DEBUG_LOG_PATH = os.environ.get("AR_POSE_LOG_PATH", os.path.join(BASE_DIR, "ar_pose_debug.log"))
DEBUG_STATUS_PATH = os.environ.get("AR_POSE_STATUS_PATH", os.path.join(BASE_DIR, "ar_pose_status.json"))
DEBUG_PACKET_PATH = os.environ.get("AR_POSE_PACKET_PATH", os.path.join(BASE_DIR, "xverse_control_live.json"))
DEBUG_LOG_MAX_BYTES = 2 * 1024 * 1024
DEBUG_DRAW_POSE_PANEL = True
DEBUG_LOCAL_PREVIEW = os.environ.get("AR_LOCAL_PREVIEW", "0").strip().lower() not in ("0", "false", "no", "off")
# Keep latency-sensitive inference/path extraction on the two fastest RK3588S
# cores. Debug drawing/JPEG use the little cores so they cannot evict control
# work. Both defaults remain overridable for other boards.
MAIN_CPUSET = os.environ.get("AR_MAIN_CPUSET", "6-7").strip()
DEBUG_RENDER_CPUSET = os.environ.get("AR_DEBUG_RENDER_CPUSET", "0-3").strip()
OCR_CPUSET = os.environ.get("AR_TURNSIGN_OCR_CPUSET", "0-3").strip()
EXTERNAL_APP_CPUSET = os.environ.get("AR_EXTERNAL_APP_CPUSET", "0-5").strip()
EXTERNAL_APP_NAMES = os.environ.get(
    "AR_EXTERNAL_APP_NAMES",
    "app,setup_webui,chromium-browse,code,codex,Lingma,Xorg,xfwm4,mihomo",
).strip()
POSE_STATUS_HTTP_HOST = os.environ.get("AR_POSE_STATUS_HOST", "0.0.0.0")
POSE_STATUS_HTTP_PORT = int(os.environ.get("AR_POSE_STATUS_PORT", "9105"))

# ===== POSE_PATH_DEBUG_START：定位链路分段验证打印，稳定后可整段删除 =====
# The staged debug output identifies UDP input, JSON validation, local mirror, and AR forwarding.
# 删除方法：搜索 POSE_PATH_DEBUG_START 到 POSE_PATH_DEBUG_END，以及代码中的 POSE_PATH_DEBUG 调用点。
POSE_PATH_DEBUG = _env_flag("AR_POSE_PATH_DEBUG", False)
POSE_PATH_DEBUG_EVERY_N = max(1, _env_int("AR_POSE_PATH_DEBUG_EVERY_N", 60))
# ===== POSE_PATH_DEBUG_END =====

def send_car_cmd(track_error, target_speed, state_cmd, flags=CONTROL_FLAG_USE_TARGET_SPEED, mark_activity=True):
    return car_link.send_cmd(track_error, target_speed, state_cmd, flags)


def get_control_send_status():
    return car_link.get_send_status()


def get_car_feedback():
    return car_link.get_feedback()

# ===== TurnSign OCR 接入（仅读取，暂不写入控制） =====
# 默认开启。如需关闭：export AR_TURNSIGN_OCR_ENABLED=0
# AI Studio token（必须配置才能调 API，OCR 本地识别不需要）：
#   方式一（推荐）：export AISTUDIO_ACCESS_TOKEN="你的token"
#   方式二：export EB_ACCESS_TOKEN="你的token"
#   获取地址：https://aistudio.baidu.com/account/accessToken
TURNSIGN_OCR_ENABLED = _env_flag("AR_TURNSIGN_OCR_ENABLED", True)
TURNSIGN_OCR_ASYNC = _env_flag("AR_TURNSIGN_OCR_ASYNC", True)
# 当前优先验证 TurnSign OCR/API，默认进入安全的 OCR-only 模式：
# - 车辆不按巡线路径跑；
# - 语义分割和中线/岔路后处理默认关闭；
# - 只保留目标检测里的 TurnSign -> OCR -> API 链路。
TURNSIGN_OCR_ONLY = _env_flag("AR_TURNSIGN_OCR_ONLY", True)
TURNSIGN_OCR_LOG_EVERY_N = max(1, _env_int("AR_TURNSIGN_OCR_LOG_EVERY_N", 30))
TURNSIGN_MIN_DET_SCORE = float(os.environ.get("AR_TURNSIGN_MIN_DET_SCORE", "0.30"))
TURNSIGN_MIN_OCR_CONFIDENCE = float(os.environ.get("AR_TURNSIGN_MIN_OCR_CONFIDENCE", "0.25"))
TURNSIGN_STABLE_FRAMES = max(1, _env_int("AR_TURNSIGN_STABLE_FRAMES", 1))
TURNSIGN_OCR_INTERVAL = float(os.environ.get("AR_TURNSIGN_OCR_INTERVAL", "0.35"))
TURNSIGN_API_COOLDOWN = float(os.environ.get("AR_TURNSIGN_API_COOLDOWN", "1.0"))
OCR_ONLY_VISION_FPS = float(os.environ.get("AR_OCR_ONLY_VISION_FPS", "6"))
OCR_ONLY_LOOP_FPS = float(os.environ.get("AR_OCR_ONLY_LOOP_FPS", "12"))
VISION_ENABLE_DETECTION = _env_flag("AR_ENABLE_DETECTION", True)
VISION_ENABLE_SEGMENTATION = False if TURNSIGN_OCR_ONLY else _env_flag("AR_ENABLE_SEGMENTATION", False)
VISION_ENABLE_MODEL_OVERLAY = _env_flag("AR_ENABLE_MODEL_OVERLAY", True)

car_link = CarControlLink()
vision_pipeline = None
vision_controls = VisionRuntimeControls(
    {
        "enable_detection": VISION_ENABLE_DETECTION,
        "enable_segmentation": VISION_ENABLE_SEGMENTATION,
        "enable_model_overlay": VISION_ENABLE_MODEL_OVERLAY,
    }
)
ocr_processor = None
ocr_frame_count = 0


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
    apply_current_thread_affinity(MAIN_CPUSET, write_debug_log, "ar-main")
    external_affinity_guard = ProcessAffinityGuard(
        EXTERNAL_APP_NAMES,
        EXTERNAL_APP_CPUSET,
        interval=2.0,
        exclude_pids={os.getpid()},
        log_func=write_debug_log,
        label="external-app",
    ).start()

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
        vision_controls=vision_controls,
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
        runtime_controls=vision_controls,
    )
    # ---- TurnSign OCR 初始化（仅读取链路） ----
    global ocr_processor, ocr_frame_count
    if TURNSIGN_OCR_ENABLED:
        try:
            print("[OCR] initializing TurnSign OCR processor ...")
            processor_cls = AsyncTurnSignOcrApiProcessor if TURNSIGN_OCR_ASYNC else TurnSignOcrApiProcessor
            ocr_kwargs = {
                "min_det_score": TURNSIGN_MIN_DET_SCORE,
                "min_ocr_confidence": TURNSIGN_MIN_OCR_CONFIDENCE,
                "stable_frames": TURNSIGN_STABLE_FRAMES,
                "ocr_interval": TURNSIGN_OCR_INTERVAL,
                "api_cooldown": TURNSIGN_API_COOLDOWN,
                "cache_ttl": 0.0,
                "async_api": True,
                "log_func": write_debug_log,
            }
            if TURNSIGN_OCR_ASYNC:
                ocr_kwargs["worker_cpu_set"] = OCR_CPUSET
            ocr_processor = processor_cls(**ocr_kwargs)
            mode_text = "async-worker" if TURNSIGN_OCR_ASYNC else "sync"
            print(f"[OCR] TurnSign OCR processor initialized ({mode_text}, read-only, not wired to control)")
            write_debug_log(f"turnsign OCR processor initialized ({mode_text}, read-only, not wired to control)")
            write_debug_log(
                "turnsign OCR params "
                f"det>={TURNSIGN_MIN_DET_SCORE:.2f} "
                f"ocr>={TURNSIGN_MIN_OCR_CONFIDENCE:.2f} "
                f"stable={TURNSIGN_STABLE_FRAMES} "
                f"interval={TURNSIGN_OCR_INTERVAL:.2f}s "
                f"api_cooldown={TURNSIGN_API_COOLDOWN:.2f}s"
            )
        except Exception as exc:
            print(f"[OCR] init FAILED: {exc}")
            write_debug_log(f"turnsign OCR processor init failed: {exc}")
            ocr_processor = None
    else:
        print("[OCR] disabled via AR_TURNSIGN_OCR_ENABLED=0")

    performance_monitor = PerformanceMonitor(log_func=write_debug_log)
    debug_stream_server = DebugStreamServer(log_func=write_debug_log).start()
    debug_render_worker = DebugRenderWorker(
        vision_pipeline=vision_pipeline,
        local_planner=local_planner,
        debug_stream_server=debug_stream_server,
        pose_bridge=pose_bridge,
        get_car_feedback=get_car_feedback,
        performance_provider=performance_monitor.hud_snapshot,
        pose_input_port=POSE_INPUT_PORT,
        draw_pose_panel=DEBUG_DRAW_POSE_PANEL,
        local_preview=DEBUG_LOCAL_PREVIEW,
        cpu_set=DEBUG_RENDER_CPUSET,
        log_func=write_debug_log,
    ).start()

    def build_autonomy_command(now, perception):
        if TURNSIGN_OCR_ONLY:
            _gamepad_cmd, gamepad_status = gamepad_receiver.active_command()
            command = {
                "track_error": 0.0,
                "target_speed": 0.0,
                "state_cmd": STATE_SAFE_STOP,
                "flags": 0,
                "state_text": "OCR_ONLY_SAFE_STOP",
            }
            task_decision = {
                "task_state": "OCR_ONLY",
                "desired_speed": 0.0,
                "line_loss_age": 0.0,
            }
            plan_result = {
                "final_track_error": 0.0,
                "speed_override": 0.0,
            }
            return command, gamepad_status, task_decision, plan_result

        car_feedback = get_car_feedback()
        pose_packet = (pose_bridge.snapshot() or {}).get("last_packet")
        task_decision = task_state_machine.update(
            perception,
            now,
            pose_packet=pose_packet,
            car_feedback=car_feedback,
        )
        plan_result = local_planner.plan(perception, task_decision, now)
        final_track_error = plan_result["final_track_error"]
        desired_speed = task_decision["desired_speed"]
        speed_override = plan_result.get("speed_override")
        if speed_override is not None:
            try:
                speed_override = float(speed_override)
            except (TypeError, ValueError):
                speed_override = None
            if speed_override is not None and math.isfinite(speed_override):
                desired_speed = speed_override
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

    def runtime_performance_snapshot():
        data = performance_monitor.snapshot()
        if not isinstance(data, dict):
            data = {"enabled": False}
        else:
            data = dict(data)
        data["debug_render"] = debug_render_worker.snapshot()
        data["debug_stream"] = debug_stream_server.snapshot()
        return data

    print("vision client ready, waiting for camera shared memory...")
    print(
        "[VISION] controls "
        f"det={VISION_ENABLE_DETECTION} seg={VISION_ENABLE_SEGMENTATION} "
        f"overlay={VISION_ENABLE_MODEL_OVERLAY} ocr_only={TURNSIGN_OCR_ONLY} "
        f"ocr_only_fps={OCR_ONLY_VISION_FPS:.1f}/{OCR_ONLY_LOOP_FPS:.1f}"
    )
    write_debug_log(
        "vision controls "
        f"det={VISION_ENABLE_DETECTION} seg={VISION_ENABLE_SEGMENTATION} "
        f"overlay={VISION_ENABLE_MODEL_OVERLAY} ocr_only={TURNSIGN_OCR_ONLY} "
        f"ocr_only_fps={OCR_ONLY_VISION_FPS:.1f}/{OCR_ONLY_LOOP_FPS:.1f}"
    )
    if TURNSIGN_OCR_ONLY:
        print("[OCR] OCR/API-only mode: segmentation and line following are disabled")
        write_debug_log("OCR/API-only mode enabled: segmentation and line following disabled")
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
            last_vision_process_ts = 0.0
            last_perception = None
            latest_ocr_result = None

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
                                    performance_status=runtime_performance_snapshot(),
                                )
                                last_runtime_status_ts = now
                            last_line_loss_command_ts = now
                        time.sleep(0.002)
                        continue
                    last_fid = fid
                    have_frame = True

                    now = time.time()
                    performance_monitor.begin_frame(fid, now)
                    vision_interval = 0.0
                    if TURNSIGN_OCR_ONLY and OCR_ONLY_VISION_FPS > 0.0:
                        vision_interval = 1.0 / max(0.1, OCR_ONLY_VISION_FPS)
                    run_vision = (
                        not TURNSIGN_OCR_ONLY
                        or last_perception is None
                        or vision_interval <= 0.0
                        or now - last_vision_process_ts >= vision_interval
                    )

                    if run_vision:
                        perf_token = performance_monitor.start()
                        _, perception = vision_pipeline.process(frame, now, frame_id=fid, draw_debug=False)
                        performance_monitor.stop("vision_ms", perf_token)
                        performance_monitor.record_stages((perception or {}).get("timings_ms"))
                        last_perception = perception
                        last_vision_process_ts = now

                        # ---- TurnSign OCR 读取（不写入控制决策） ----
                        ocr_result = None
                        if ocr_processor is not None and perception is not None:
                            try:
                                perf_token = performance_monitor.start()
                                detections = perception.get("detections") or []
                                ocr_result = ocr_processor.process(frame, detections, timestamp=now)
                                performance_monitor.stop("ocr_ms", perf_token)
                                latest_ocr_result = ocr_result
                                ocr_frame_count += 1
                                if ocr_result.get("active") and ocr_frame_count % TURNSIGN_OCR_LOG_EVERY_N == 0:
                                    status = ocr_result.get("status", "?")
                                    instr = ocr_result.get("instruction") or {}
                                    if ocr_result.get("instruction_current") and instr.get("valid"):
                                        write_debug_log(
                                            f"turnsign OCR valid | action={instr.get('action')} "
                                            f"preferred={instr.get('preferred_branch')} "
                                            f"avoid={instr.get('avoid_branches')} "
                                            f"text={instr.get('source_text', '')[:40]}"
                                        )
                                    elif status not in ("ocr_throttled", "no_turnsign"):
                                        api_error = instr.get("api_error") or instr.get("api_parse_error") or ocr_result.get("error")
                                        extra = f" error={api_error}" if api_error else ""
                                        write_debug_log(f"turnsign OCR status={status}{extra}")
                            except Exception as exc:
                                write_debug_log(f"turnsign OCR process error: {exc}")
                    else:
                        perception = last_perception
                        ocr_result = latest_ocr_result

                    perf_token = performance_monitor.start()
                    command, gamepad_status, task_decision, plan_result = build_autonomy_command(now, perception)
                    performance_monitor.stop("command_ms", perf_token)
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
                            performance_status=runtime_performance_snapshot(),
                            perception=perception,
                            ocr_result=ocr_result,
                        )
                        performance_monitor.stop("runtime_status_ms", perf_token)
                        last_runtime_status_ts = now

                    fps_n += 1
                    if now - fps_t >= 1.0:
                        cur_fps = fps_n / max(1e-6, now - fps_t)
                        fps_n = 0
                        fps_t = now

                    performance_monitor.finish_frame(time.time())
                    debug_render_worker.publish(
                        frame=frame,
                        frame_id=fid,
                        perception=perception,
                        task_decision=task_decision,
                        plan_result=plan_result,
                        command=command,
                        gamepad_status=gamepad_status,
                        control_fps=cur_fps,
                        ocr_result=ocr_result,
                    )
                    if TURNSIGN_OCR_ONLY and OCR_ONLY_LOOP_FPS > 0.0:
                        elapsed = time.time() - now
                        target_dt = 1.0 / max(0.1, OCR_ONLY_LOOP_FPS)
                        if elapsed < target_dt:
                            time.sleep(target_dt - elapsed)

                except FrameSnapshotChanged:
                    time.sleep(0.001)
                    continue
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
        performance_status=runtime_performance_snapshot(),
    )
    pose_bridge.stop()
    gamepad_receiver.stop()
    if pose_status_server is not None:
        pose_status_server.stop()
    debug_render_worker.stop()
    debug_stream_server.stop()
    if external_affinity_guard is not None:
        external_affinity_guard.stop()
    if ocr_processor is not None and hasattr(ocr_processor, "close"):
        ocr_processor.close()
    if vision_pipeline is not None:
        vision_pipeline.release()
    performance_monitor.close()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == "__main__":
    main()
