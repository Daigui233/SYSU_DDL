# Master_RK3588S

RK3588S 上位机负责转发定位给官方 AR、读取融合视频、运行视觉模型、生成控制命令，并与 TC264D 串口通信。

## 正确链路

```text
Windows AprilTag 定位 -> UDP 板卡IP:9005 -> ar_pose_bridge.py -> 127.0.0.1:9006 -> 官方 AR
官方 AR 融合视频 -> shm_ar_video -> 分割模型 -> track_error -> STATE_TRACK -> /dev/ttyUSB0 -> TC264D
官方 AR 融合视频 -> shm_ar_video -> 检测模型 -> 仅画 gold/car/human 检测框
可选手柄遥控 -> UDP 板卡IP:9010 -> gamepad_control_receiver.py -> ar_receiver.py 临时覆盖视觉控制 -> /dev/ttyUSB0 -> TC264D
```

定位只驱动 AR 融合和坐标显示，不直接参与转向控制。默认控车输入来自融合视频上的视觉处理结果；只有 Windows 定位 EXE 显式勾选 `Gamepad Mode` 且 `9010` 收到新鲜遥控包时，手柄才临时覆盖视觉控制。

## 重点文件

| 文件 | 作用 |
| --- | --- |
| `setupUI/ar_receiver.py` | 上位机主入口：启动模块、读取帧、调度视觉/仲裁/发送/显示 |
| `setupUI/ar_pose_bridge.py` | 独立接收 Windows `robot_position`，校验、记录并转发给官方 AR |
| `setupUI/gamepad_control_receiver.py` | 独立接收 `9010` 手柄遥控包，供 `ar_receiver.py` 临时覆盖视觉控制 |
| `setupUI/vision_pipeline.py` | 分割/检测模型初始化、推理、后处理、画中线和检测框 |
| `setupUI/control_arbitrator.py` | 根据视觉中线、丢线计时和可选手柄命令生成最终控制帧 |
| `setupUI/car_control_link.py` | 封装 `/dev/ttyUSB0` 串口控车链路，供主入口或备用桥复用 |
| `setupUI/runtime_status.py` | 写入 `/pose_status` 所需的定位、控制、AI 和 TC264D 状态 |
| `setupUI/webui_status_server.py` | WebUI 状态 HTTP 服务、配置接口和轻量调试 API |
| `setupUI/hud_renderer.py` | OpenCV 预览窗口右侧调试栏绘制 |
| `setupUI/video_frame_source.py` | 从 `shm_ar_video` 共享内存读取并转换帧 |
| `setupUI/debug_tools.py` | 调试日志轮转和定位链路分段打印 |
| `setupUI/standalone_control_bridge.py` | 手动备用桥：不启动 `ar_receiver.py` 时接收定位和手柄，并可用手柄控车 |
| `setupUI/infer_wrap/base/seg_func.py` | 从分割结果提取路线中心和 `track_error` |
| `setupUI/infer_wrap/base/func.py` | 检测模型后处理与画框 |
| `setupUI/serial_comm.py` | TC264D 串口收发 |
| `setupUI/dist/main_config.json` | 官方 AR/WebUI 配置 |
| `setupUI/ar_pose_status.json` | 定位、控制和 TC264D 反馈状态 |

## 当前控制

当前只启用巡线状态：

```text
融合视频 -> 分割模型 -> track_error -> STATE_TRACK + target_speed + flags=0x01 -> TC264D
```

- 分割模型负责生成路线误差。
- 检测模型当前识别 `gold / car / human`，但只用于画框，不参与状态切换。
- OCR/API 尚未接入。
- 当前分割和检测模型效果较差，仍需继续训练和优化。
- 无有效分割误差但未超过 `3 s` 时，当前进入 `TRACK_FALLBACK`，误差 `0`、默认速度 `0.5 m/s`。
- 连续 `3 s` 没有有效 `track_error` 时，RK3588S 进入 `LINE_LOSS_SAFE_STOP` 并重复下发 `STATE_SAFE_STOP`。
- 再次识别到语义分割中线后，下一帧恢复 `STATE_TRACK + VISION`，不会锁死在 `STATE_SAFE_STOP`。
- 中线丢失阈值可通过 `AR_LINE_LOSS_SAFE_STOP_TIMEOUT` 调整；丢线状态下命令重复间隔可通过 `AR_LINE_LOSS_COMMAND_REPEAT_INTERVAL` 调整。
- 默认巡线速度为 `0.5 m/s`，可通过 `AR_TRACK_SPEED` 调整；`TRACK_FALLBACK` 可通过 `AR_TRACK_FALLBACK_SPEED` 单独覆盖。

## 可选手柄遥控

手柄链路用于低速遥控和采集分割数据集，不是比赛默认控制链路：

- RK3588S 监听 `0.0.0.0:9010`，只接收 `type=gamepad_control`。
- 未收到遥控包、遥控包超过 `3.0 s`、或定位 EXE 取消 `Gamepad Mode` 时，控制源自动回到视觉巡线；取消勾选会主动发送关闭包，正常关闭不需要等待超时。
- 遥控有效时，`RT` 前进、`LT` 倒车，合成为 `target_speed=(RT-LT)*1.0 m/s`；`LX` 映射 `track_error`；`B` 键或手柄断连映射 `STATE_SAFE_STOP`。
- 遥控接管只覆盖最终发给 TC264D 的控制帧，不影响 `9005 -> 9006` 定位转发，也不修改 AR 融合链路。
- Windows 定位 EXE 内部用独立定时器发送 `9010` 遥控包；没有固定 Tag、车载 Tag 或有效 `robot_position` 时，遥控仍可工作。
- RK 侧对遥控速度和误差再次限幅：默认最大速度 `±1.0 m/s`，最大 `track_error` 绝对值 `240`，可通过 `AR_GAMEPAD_MAX_SPEED_MPS` / `AR_GAMEPAD_MAX_TRACK_ERROR` 调整。

## 后续扩展接口

上位机新增功能应保持模块化：先新建可复用模块，再由 `ar_receiver.py` 调度。`ar_receiver.py` 不再直接承载大块新增业务逻辑，后续状态机、路径规划、OCR/API、任务记录和调参工具都应独立成文件，避免视频接收、模型推理、控制仲裁和调试界面互相缠在一起。后续任务状态机建议独立为 `task_state_machine.py`，输入分割/检测/OCR/API/定位状态，输出 `state_cmd / target_speed / track_error / flags`。

`setupUI/standalone_control_bridge.py` 是手动备用入口，适合采集数据时只打开纯净 AR 融合流、没有启动 `ar_receiver.py`，但仍需要 `9005` 定位转发和 `9010` 手柄控车的情况。它不是默认入口，不要和 `ar_receiver.py` 同时运行，以免争用 `9005/9010` 端口或形成双控制源。

串口控制帧已包含：

- `track_error`：当前由分割模型产生的转向误差。
- `target_speed`：当前状态目标速度，单位按 TC264D 侧粗换算为 `m/s`。
- `state_cmd`：上位机任务状态。
- `flags`：控制选项。

后续模型和 OCR/API 可通过独立任务决策层生成这些字段。新增状态时应分别定义进入、保持、退出和失效规则，不修改定位链路，也尽量保持串口协议兼容。

## 定位与端口

```json
{"type":"robot_position","pos":[x,0.16,z],"euler":[0.0,yaw,0.0]}
```

- 坐标使用当前 Windows 定位端输出：场地 `4 m × 3 m`，定位预览原点在右下角，`+X` 向左、`+Z` 向上。
- Yaw 位于 `euler[1]`，约定为 `0° -> +X`、`+90° -> +Z`；RK3588S 不重新计算定位。
- 官方 AR 场景资产的 X/Z 平移轴与 Windows 定位预览互换，因此 `ar_receiver.py` 只在转发到 `127.0.0.1:9006` 时交换 `pos[0]` 和 `pos[2]`，即 AR 侧收到 `pos=[z,0.16,x]`，Yaw 保持不变。
- Windows 定位端已对输出位姿做轻量滤波；RK3588S/WebUI/HUD 看到的是同一份官方 `robot_position`。
- 外部定位入口：`0.0.0.0:9005`。
- 官方 AR 转发：`127.0.0.1:9006`。
- WebUI 定位数据地址：`127.0.0.1`。
- WebUI 端口号：`9006`。
- 可选手柄遥控入口：`0.0.0.0:9010`。
- UNITY 同步端口：`9003`。
- 控车串口：固定 `/dev/ttyUSB0`，`460800` baud；启动时打不开或运行中读写异常时，串口层会约每 `1 s` 自动重连。

## 调试重点

WebUI/HUD 优先确认：

- `WIN-UDP ok` 和 `AR-FWD ok` 持续增长，AR 画面随定位变化。
- 分割结果和 `track_error` 是否稳定。
- HUD 控制来源正常应为 `CTRL VISION` 或 `CTRL TRACK_FALLBACK`；手柄接管时应为 `CTRL GAMEPAD_TRACK` 或 `CTRL GAMEPAD_SAFE_STOP`。
- WebUI 右侧调试栏的 `GAMEPAD` 行应显示遥控是否 active、包计数、错误计数和年龄。
- TC264D feedback 持续增长。
- `flags=0x01`，电机和舵机输出随误差变化。

状态接口：

- `http://<RK3588S-IP>:9105/pose_status`
- `http://<RK3588S-IP>:9105/pose_packet`

`/pose_status` 中的 `control` 字段每 `0.5 s` 刷新，并包含控制来源、`track_error`、目标速度（m/s）、`state_cmd`、flags、最近一次串口发送结果和手柄遥控状态。
