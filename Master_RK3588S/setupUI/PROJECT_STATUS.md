# RK3588S 上位机当前状态

## 控制链路轻量化基线

当前正式控制链路：

```text
seg_func.py segmentation track_error
  -> control_task_state_machine.py task_state / desired_speed / planner_intent
  -> control_local_planner.py final_track_error
  -> control_arbitrator.py command["track_error"] = final_track_error
  -> control_serial_comm.py
  -> TC264D input_track_error
  -> Control.c 线性舵机 P / 电机 PID
  -> Servo.h / Motor.h PWM 硬限幅
```

当前核心调车基线：

- 图像尺寸按 `640 x 480` 使用，摄像头中心线为 `x=320`。
- 分割层和规划层统一使用 `lookahead_y=300`。
- `track_error/final_track_error = lookahead 点 x - center_x`，单位为像素。
- 上位机不再做 error 限幅、限步、跨帧平滑或非线性增强。
- 视觉自动驾驶下，`control_arbitrator.py` 直通下发 `final_track_error`。
- TC264D 使用 `SERVO_FULL_STEER_ERROR_PX=320`，即约 `320 px` 对应线性转向输出满量程。
- TC264D 舵机当前为 P-only，`SERVO_LINEAR_KP=0.5`，`KD=0`，再由 `Servo.h` 做硬限幅。
- TC264D 本地控制帧超时为 `0.5 s`。
- TC264D 反馈帧已扩展到 v3，HUD 会显示下位机实际收到的 error、输入年龄、舵机/电机输出、限幅标志、前馈/PID 修正和反馈序号。

## 当前保留的软件处理

- `seg_func.py` 保留 road mask 闭运算/开运算、连通域筛选、同帧行扫描跳变保护和当前帧多项式拟合。
- `seg_func.py` 已关闭旧 mask 帧保持、4 帧 mask 投票、底部锚点跨帧混合、中线点跨帧混合和 `track_error` EMA。当前帧无有效 road mask 时直接输出 LOST，不再沿用上一帧分割结果。
- `control_task_state_machine.py` 保留检测触发阈值、风险排序、状态 TTL/滞回、短时 `RECOVER_LINE` 和安全停车。这些属于状态防误触发，不是 CMD error 滤波。
- `control_local_planner.py` 保留连续目标线生成、road mask 内几何约束、避障侧选择、行人短时运动预测、stone 分支选择、gold 偏移逻辑和短时 `RECOVER_LINE` 衰减。
- `control_arbitrator.py` 只保留手柄覆盖、安全停车清零、非法输入保护、状态/速度/flags 组帧和命令重复间隔。
- TC264D 保留电机速度 PID/PWM 硬限幅、舵机 PWM 硬限幅、串口超时和 STOP 安全保护。
- HUD 右侧 `ERR cmd/tc/d` 用于确认 `CMD err` 与 `TC264D input_track_error` 是否对齐；`SERVO` 和 `MOTOR` 行用于判断下位机输出是否已经足够；`SAFE` 行用于判断是否发生超时、停车、舵机饱和、电机饱和或速度死区。

## 当前路径规划做法

- `NORMAL_TRACK`：使用分割中线作为目标路径，在 `y=300` 取最近目标点计算 `final_track_error`。
- `AVOID_CAR`：在 road mask 内按左右走廊评分选择绕行侧，生成从近处中线连续偏向绕行侧的紫色目标线。
- `AVOID_HUMAN`：估计行人 bbox 横向速度，预测短时占用区，再结合 road mask 左右走廊评分选择绕行侧。行人运动方向参与占用区和打平时倾向，不再只按单帧“哪边更空”决定。
- `COLLECT_GOLD`：在风险池为空且金币足够近时，让目标线向金币方向偏移，但仍受 road mask 约束。
- `AVOID_STONE`：在分岔候选路径中选择避开 stone 的分支，仍属于图像坐标局部策略，不是全局地图规划。
- `RECOVER_LINE`：短时丢线时衰减上一帧有效 `final_track_error`，默认衰减窗口约 `0.5 s`；正常 TRACK/AVOID/COLLECT/STONE 不受这个保持影响。

## 当前风险与不足

- `lookahead_y=300` 和 `FULL_STEER_ERROR_PX=320` 是当前实车调参基线，不保证一次到位。摄像头接近车头，前瞻点不宜过低，也不宜过远，后续应通过 HUD 中紫色目标点和实车响应微调。
- `320 px` 满舵比旧版 `200 px` 更不敏感，用于先验证是否能缓解转向过猛；如果转不过来，可用二分法回调 `SERVO_FULL_STEER_ERROR_PX`。
- 分割层取消帧保持后，旧视频中“车跑偏但分割还保留前几帧”的问题会消失，但分割模型抖动会更直接暴露出来。
- 行人路径规划仍是图像局部启发式预测，不是严格动态避障。若行人横向速度估计不稳，仍可能选边不自然，需要用实车视频继续调 `HUMAN_MOTION_*` 和 `ROAD_SIDE_*`。
- 如果 HUD 中 `Final err / CMD err / TC264D input_track_error` 已经一致，但车仍响应慢，下一步优先看 TC264D 舵机映射、舵机机械速度/角度和供电；如果车身已经能快速给角但路径仍不合理，再回看上位机规划。
- 这次反馈扩展主要提升可观测性，不是帧数优化。串口帧从旧 v2 的 50 字节增加到 v3 的 74 字节，在 `460800 baud` 下开销很小；HUD 多显示几行文字可能略增绘制耗时，但通常不应成为主要 FPS 瓶颈。若后续要专门优化帧数，应优先看视觉推理、HUD 绘制、窗口显示和 MJPEG 发布。
