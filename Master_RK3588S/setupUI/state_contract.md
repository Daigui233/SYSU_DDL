# RK 任务状态与 TC264D 状态契约

## 控制误差契约

控制误差只允许按下面方向传递：

```text
segmentation track_error
  -> task_state_machine
  -> local_planner final_track_error
  -> control_arbitrator 直通下发
  -> TC264D input_track_error
```

- `segmentation track_error`：`seg_func.py` 在当前帧分割中线 `lookahead_y=300` 处取点，计算 `x - center_x`。
- `final_track_error`：`control_local_planner.py` 在当前帧最终目标路径 `lookahead_y=300` 处取点，计算 `x - center_x`。
- `command["track_error"]`：视觉自动驾驶时必须等于 `final_track_error`。
- `TC264D input_track_error`：应等于串口收到的 `command["track_error"]`。

上位机禁止对视觉控制误差做比例缩放、死区、限幅、限步、跨帧平滑或非线性增强。手柄接管、安全停车清零、非法输入保护不属于视觉误差后处理。

## 参数归属

- 上位机负责感知、任务状态机、局部路径规划和串口组帧。
- 下位机负责舵机线性控制、电机速度 PID、PWM 硬限幅、串口超时和 STOP 安全保护。
- 当前前瞻点为 `lookahead_y=300`。
- 当前舵机满量程误差为 `SERVO_FULL_STEER_ERROR_PX=200`。
- 当前 TC264D 舵机映射为 P-only：`servo_pwm = SERVO_DUTY_MID + SERVO_LINEAR_KP * (0 - input_track_error)`，`SERVO_LINEAR_KP=0.8`，再进入 `Servo.h` 硬限幅。

## 状态映射

| RK `task_state` | RK 行为 | TC264D `state_cmd` |
| --- | --- | --- |
| `NORMAL_TRACK` | 使用分割中线生成目标路径，并从 `y=300` 计算误差 | `STATE_TRACK = 1` |
| `AVOID_CAR` | 在 road mask 内生成连续绕车目标线 | `STATE_AVOID_CAR = 2` |
| `AVOID_HUMAN` | 用行人横向速度预测短时占用区，再选择可通行走廊 | `STATE_AVOID_HUMAN = 3` |
| `COLLECT_GOLD` | 无风险目标时将目标线偏向金币 | `STATE_COLLECT_GOLD = 4` |
| `RECOVER_LINE` | 短时衰减上一帧有效规划误差 | `STATE_RECOVER_LINE = 5` |
| `LINE_LOSS_SAFE_STOP` | 连续丢线超过约 `0.8 s` 后停车 | `STATE_LINE_LOSS_SAFE_STOP = 6` |
| `AVOID_STONE` | 在分岔候选路径中选择避开 stone 的分支 | `STATE_AVOID_STONE = 8` |
| `TRAFFIC_LIGHT_STOP` | 近处红灯确认后停车，绿灯恢复 | `STATE_TRAFFIC_LIGHT_STOP = 9` |
| `ENDSIGN_STOP` | 终点标志完成确认后停车 | `STATE_ENDSIGN_STOP = 10` |

`STATE_IDLE = 0` 用于 TC264D 本地空闲。`STATE_SAFE_STOP = 7` 用于手动急停、程序退出清零和 TC264D 本地串口超时。

## 接口形状

`vision_pipeline.py` 输出：

- `segmentation.line_valid`
- `segmentation.track_error`
- `segmentation.road_mask`
- `segmentation.mid_points`
- `segmentation.road_held`，当前固定为 `False`
- `detections[]`

`control_task_state_machine.py` 输出：

- `task_state`
- `desired_speed`
- `planner_intent`
- 丢线计时、状态原因等调试字段

`control_local_planner.py` 输出：

- `final_track_error`
- `raw_final_track_error`
- `target_path`
- `control_lookahead_y`
- `planner_mode`

`control_arbitrator.py` 输出串口控制帧字段：

- `state_cmd`
- `target_speed`
- `track_error`
- `flags`

## 当前仍允许的非控制滤波

- 状态机 TTL/滞回：用于防止检测误触发。
- 行人 bbox 横向速度 EMA：用于短时占用区预测，不直接滤波 CMD error。
- road mask 形态学处理、连通域筛选和同帧中线跳变保护：用于分割几何稳定。
- 当前帧多项式中线拟合：用于降低单帧行扫描噪声。
- `RECOVER_LINE` 短时衰减：只在丢线恢复状态使用，正常 TRACK/AVOID/COLLECT/STONE 不使用。
