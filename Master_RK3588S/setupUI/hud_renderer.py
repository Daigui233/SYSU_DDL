import time

import cv2
import numpy as np


def short_text(text, max_len=74):
    text = str(text or "")
    if len(text) <= max_len:
        return text
    return text[:max_len - 3] + "..."


def draw_pose_status(
    frame,
    pose_bridge,
    fps=None,
    track_error=None,
    control_state="N/A",
    gamepad_status=None,
    get_car_feedback=None,
    pose_input_port=9005,
    enabled=True,
):
    if not enabled:
        return frame

    info = pose_bridge.snapshot()
    age = time.time() - info["last_ts"] if info["last_ts"] else None
    fresh = age is not None and age < 1.0
    status_l = info["status"].lower()
    color = (70, 240, 70) if fresh else (0, 220, 255)
    if "error" in status_l or "missing" in status_l or "no device" in status_l:
        color = (40, 80, 255)

    packet = info["last_packet"]
    pose_line = "POSE: waiting"
    if packet:
        pos = packet.get("pos", [0.0, 0.0, 0.0])
        euler = packet.get("euler", [0.0, 0.0, 0.0])
        age_text = f"{age:.1f}s" if age is not None else "N/A"
        pose_line = (
            f"POSE x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f} "
            f"yaw={euler[1]:.1f} age={age_text}"
        )

    source_line = (
        f"WIN-UDP {short_text(info['status'], 18)} "
        f"ok={info['packet_count']} bad={info['invalid_count']}"
    )
    udp_line = f"AR-FWD ok={info['udp_send_count']} fail={info['udp_fail_count']}"
    if track_error is not None and np.isfinite(track_error):
        control_line = f"CTRL {control_state} err={track_error:.1f}"
    else:
        control_line = f"CTRL {control_state} err=N/A"

    gamepad_lines = []
    if gamepad_status is not None:
        gp_age = gamepad_status.get("age")
        gp_age_text = f"{gp_age:.1f}s" if gp_age is not None else "N/A"
        gp_state = "ACTIVE" if gamepad_status.get("active") else "idle"
        gamepad_lines.append(
            f"GAMEPAD {gp_state} ok={gamepad_status.get('packet_count', 0)} "
            f"bad={gamepad_status.get('invalid_count', 0)} age={gp_age_text}"
        )
        if gamepad_status.get("last_packet"):
            inputs = gamepad_status.get("inputs") or {}
            gamepad_lines.append(
                f"GP v={gamepad_status.get('target_speed', 0.0):.2f} "
                f"err={gamepad_status.get('track_error', 0.0):.0f} "
                f"RT={inputs.get('rt', 0.0):.2f} LT={inputs.get('lt', 0.0):.2f} B={int(bool(inputs.get('b', False)))}"
            )

    feedback = get_car_feedback() if get_car_feedback is not None else {"online": False, "error": "waiting"}
    car_lines = []
    if feedback.get("online"):
        fb_age = feedback.get("age")
        fb_age_text = f"{fb_age:.1f}s" if fb_age is not None else "N/A"
        flags = feedback.get("flags")
        flag_text = "N/A" if flags is None else f"0x{int(flags):02X}"
        car_lines.append(
            f"TC264 fb={feedback.get('count', 0)} age={fb_age_text} "
            f"st={feedback.get('state', 'N/A')} flags={flag_text}"
        )
        car_lines.append(
            f"RX raw={feedback.get('raw_rx', 0)} drop={feedback.get('raw_drop', 0)} "
            f"bad={feedback.get('bad', 0)}"
        )
        car_lines.append(
            f"SPD in={feedback.get('input_target_speed', 0.0):.2f} "
            f"tgt={feedback.get('motor_target', 0.0):.2f} "
            f"act={feedback.get('actual_speed', 0.0):.2f}m/s"
        )
        car_lines.append(
            f"OUT m={feedback.get('motor_output', 0)} s={feedback.get('servo_output', 0)}"
        )
        car_lines.append(
            f"PID {feedback.get('motor_kp', 0.0):.1f}/{feedback.get('motor_ki', 0.0):.1f}/{feedback.get('motor_kd', 0.0):.1f} "
            f"SV {feedback.get('servo_kp', 0.0):.1f}/{feedback.get('servo_kd', 0.0):.1f}"
        )
    else:
        err = short_text(feedback.get("error", "waiting"), 28)
        car_lines.append(
            f"TC264 waiting fb={feedback.get('count', 0)} raw={feedback.get('raw_rx', 0)} "
            f"drop={feedback.get('raw_drop', 0)} bad={feedback.get('bad', 0)}"
        )
        if feedback.get("last_rx"):
            car_lines.append(f"last rx {short_text(feedback.get('last_rx', ''), 28)}")
        car_lines.append(err)
    fps_line = f"FPS {fps:.1f}" if fps is not None else "FPS N/A"

    lines = [
        "DEBUG STATUS",
        source_line,
        pose_line,
        udp_line,
        control_line,
    ]
    lines.extend(gamepad_lines)
    lines.extend(car_lines)
    lines.append(fps_line)

    h = frame.shape[0]
    panel_w = 430
    panel = np.zeros((h, panel_w, 3), dtype=frame.dtype)
    panel[:, :] = (12, 16, 16)
    cv2.rectangle(panel, (0, 0), (panel_w - 1, h - 1), color, 1)

    line_h = 22
    y = 28
    for i, line in enumerate(lines):
        text_color = color if i == 0 else (230, 245, 245)
        cv2.putText(panel, short_text(line, 58), (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, text_color, 1, cv2.LINE_AA)
        y += line_h
        if i in (0, 4):
            cv2.line(panel, (10, y - 8), (panel_w - 12, y - 8), (55, 70, 70), 1, cv2.LINE_AA)

    return np.hstack([frame, panel])
