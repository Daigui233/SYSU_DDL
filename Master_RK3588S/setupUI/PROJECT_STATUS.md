# RK3588S 上位机当前状态

## 正式链路

```text
Windows 顶置相机
  -> AprilTag 定位 robot_position
  -> UDP 板卡IP:9005
  -> ar_receiver.py
  -> 127.0.0.1:9006
  -> 官方 AR 引擎
  -> 融合视频 shm_ar_video
  -> 分割模型生成 track_error
  -> STATE_TRACK + target_speed + flags
  -> /dev/ttyUSB0
  -> TC264D
```

AprilTag 定位只服务官方 AR 融合、坐标显示和记录，不参与 `track_error`、`state_cmd` 或转向控制。检测模型继续运行，但当前只画 `gold / car / human` 检测框，不参与状态机。

## 当前控制

- 正常视觉控制只启用视觉巡线和 `STATE_TRACK`。
- 有有效分割误差时，下发 `STATE_TRACK + TRACK_SPEED + flags=0x01`，默认速度 `0.5 m/s`。
- 无有效分割误差但未超过 `3 s` 时，保持 `TRACK_FALLBACK`：误差 `0`、默认速度 `0.5 m/s`。
- 连续 `3 s` 没有有效 `track_error` 时，上位机进入 `LINE_LOSS_SAFE_STOP` 并重复下发 `STATE_SAFE_STOP`。
- 再次识别到语义分割中线后，下一帧恢复 `STATE_TRACK + VISION`，不会锁死在 `STATE_SAFE_STOP`。
- 中线丢失阈值可通过 `AR_LINE_LOSS_SAFE_STOP_TIMEOUT` 调整；丢线状态下命令重复间隔可通过 `AR_LINE_LOSS_COMMAND_REPEAT_INTERVAL` 调整。
- TC264D 本地连续 `2.5 s` 未收到有效控制帧时，自行进入 `STATE_SAFE_STOP`。
- OCR/API 和多状态任务决策尚未接入；`state_cmd / target_speed / track_error / flags` 接口为后续扩展保留。
- 当前分割和检测模型效果较差，仍需继续训练、调参与实车验证。

## 可选手柄遥控

- 手柄遥控只用于调试和采集分割数据集，默认不启用。
- Windows 定位 EXE 勾选 `Gamepad Mode` 后，额外向板卡 `9010` 发送 `gamepad_control` 包。
- `Gamepad Mode` 由 Windows 定位 EXE 内部独立定时器发送，频率与当前相机/视频帧率一致，不依赖 Tag、标定或 `robot_position` 成功发送。
- RK3588S 只有收到 `gamepad_mode=true` 且未超过 `0.45 s` 的新鲜遥控包时，才临时覆盖视觉控制。
- 取消勾选、遥控包超时、或使用赛方定位模块时，RK3588S 自动回到视觉巡线。
- 遥控模式不影响定位：`robot_position` 仍走 `9005`，并继续转发到 `127.0.0.1:9006`。
- 默认映射：`RT -> target_speed`，`LX -> track_error`，`LT >= 90%` 或手柄断连 -> `STATE_SAFE_STOP`。

## 端口与设备

| 用途 | 当前配置 |
| --- | --- |
| Windows 定位入口 | `0.0.0.0:9005` |
| 官方 AR 转发 | `127.0.0.1:9006` |
| WebUI 定位数据 | `127.0.0.1:9006` |
| 可选手柄遥控入口 | `0.0.0.0:9010` |
| 定位状态 HTTP | `0.0.0.0:9105` |
| 融合视频共享内存 | `shm_ar_video` |
| TC264D 串口 | `/dev/ttyUSB0`，`460800` baud，写超时 `0.05 s` |

当前定位约定以 Windows AprilTag 程序为准：场地 `4 m x 3 m`，定位预览原点在右下角，`+X` 向左、`+Z` 向上；Yaw 使用 `euler[1]`，`0 deg -> +X`、`+90 deg -> +Z`。Windows 端已对输出位姿做轻量滤波。官方 AR 场景资产的 X/Z 平移轴与 Windows 定位预览互换，因此 RK3588S 仅在转发到 `127.0.0.1:9006` 时交换 `pos[0]` 和 `pos[2]`，Yaw 保持不变；定位仍不转换为控车误差。

## 重点文件

| 文件 | 作用 |
| --- | --- |
| `ar_receiver.py` | 定位接收/转发、融合视频读取、双模型推理、视觉巡线控制、可选手柄接管和 HUD |
| `serial_comm.py` | TC264D 控制帧和反馈帧串口通信 |
| `infer_wrap/base/seg_func.py` | 分割后处理、中线提取和 `track_error` 计算 |
| `infer_wrap/base/func.py` | 检测模型后处理与画框 |
| `templates/index.html` | WebUI 页面和右侧链路调试状态栏 |
| `dist/main_config.json` | 官方 AR/WebUI 网络配置 |

## 已知风险

- `TRACK_SPEED` 与 `TRACK_FALLBACK` 默认速度已设为 `0.5 m/s`；首次落地前仍应架空车轮确认方向和制动行为。
- TC264D 电机 PWM 硬限幅为 `±2500` duty；`STATE_TRACK` 当前使用约 `1450` duty 启动前馈 + 小 PI 修正，先解决电机死区和低速卡顿，PID 只做稳速微调。
- `target_speed` 当前按同款 CarDo 车模参数粗换算为 `m/s`；TC264D 的 `actual_speed` 由编码器计数换算得到，后续仍需确认编码器正负号并用实测速度修正比例误差。
- 视觉模型短时丢线会继续按 `TRACK_FALLBACK` 直行；若连续 `3 s` 没有有效 `track_error`，RK3588S 会进入 `LINE_LOSS_SAFE_STOP`。
- TC264D 本地串口输入超时已补入，但硬件断电、舵机/电机驱动异常和机械风险仍需要现场急停手段。
- Windows 定位程序必须把目标 IP 配成 RK3588S 板卡局域网 IP；`127.0.0.1` 只适用于同机测试。
- 当前模型效果不足以直接假定稳定巡线，应先架空轮胎和低速验证误差方向、舵机方向、PID 与反馈。

## 运行

```bash
python3 ar_receiver.py
```
