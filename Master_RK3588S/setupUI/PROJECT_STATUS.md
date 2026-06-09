# RK3588S 上位机当前状态

## 正式链路

```text
Windows 顶置相机
  -> AprilTag 定位 robot_position
  -> UDP 板卡IP:9005
  -> ar_pose_bridge.py
  -> 127.0.0.1:9006
  -> 官方 AR 引擎
  -> 融合视频 shm_ar_video
  -> ar_receiver.py 读取视频并运行模型
  -> 分割模型生成 track_error
  -> STATE_TRACK + target_speed + flags
  -> /dev/ttyUSB0
  -> TC264D
```

AprilTag 定位只服务官方 AR 融合、坐标显示和记录，不参与 `track_error`、`state_cmd` 或转向控制。检测模型继续运行，但当前只画 `gold / car / human` 检测框，不参与状态机。

## 当前模块边界

| 文件 | 作用 |
| --- | --- |
| `ar_receiver.py` | 上位机主入口：启动模块、读取帧、调度视觉/仲裁/发送/显示 |
| `ar_pose_bridge.py` | 独立接收 Windows `robot_position`，校验、记录并转发到 `127.0.0.1:9006` |
| `gamepad_control_receiver.py` | 独立接收 `9010` 手柄遥控包，只提供候选控制命令，不直接写串口 |
| `vision_pipeline.py` | 分割/检测模型初始化、推理、后处理、画中线和检测框 |
| `control_arbitrator.py` | 根据视觉中线、丢线计时和可选手柄命令生成最终控制帧 |
| `car_control_link.py` | 封装 `/dev/ttyUSB0` 串口控车链路，供主入口或备用桥复用 |
| `runtime_status.py` | 写入 `/pose_status` 所需的定位、控制、AI 和 TC264D 状态 |
| `webui_status_server.py` | WebUI 状态 HTTP 服务、配置接口和轻量调试 API |
| `hud_renderer.py` | OpenCV 预览窗口右侧调试栏绘制 |
| `video_frame_source.py` | 从 `shm_ar_video` 共享内存读取并转换帧 |
| `debug_tools.py` | 调试日志轮转和定位链路分段打印 |
| `standalone_control_bridge.py` | 手动备用桥：不启动 `ar_receiver.py` 时接收定位和手柄，并可用手柄控车 |
| `serial_comm.py` | TC264D 控制帧和反馈帧串口通信，固定 `/dev/ttyUSB0`、`460800` |
| `infer_wrap/base/seg_func.py` | 分割后处理、中线提取和 `track_error` 计算 |
| `infer_wrap/base/func.py` | 检测模型后处理与画框 |
| `templates/index.html` | WebUI 页面和右侧链路调试状态栏 |
| `dist/main_config.json` | 官方 AR/WebUI 网络配置 |

工程约定：后续新增上位机功能时，优先新建可复用模块文件，再由 `ar_receiver.py` 调度。不要把状态机、路径规划、OCR/API、数据记录或大块调参逻辑直接堆进 `ar_receiver.py`。后续任务状态机建议独立为 `task_state_machine.py`，输入分割/检测/OCR/API/定位状态，输出 `state_cmd / target_speed / track_error / flags`。

`standalone_control_bridge.py` 是手动备用入口，适合采集数据时只打开纯净 AR 融合流、没有启动 `ar_receiver.py`，但仍需要 `9005` 定位转发和 `9010` 手柄控车的情况。它不是默认入口，不要和 `ar_receiver.py` 同时运行，否则可能争用 `9005/9010` 端口或形成双控制源。

## 当前控制

- 正常视觉控制只启用视觉巡线和 `STATE_TRACK`。
- 有有效分割误差时，下发 `STATE_TRACK + TRACK_SPEED + flags=0x01`，默认速度 `0.5 m/s`。
- 无有效分割误差但未超过 `3 s` 时，保持 `TRACK_FALLBACK`：误差 `0`、默认速度 `0.5 m/s`。
- 连续 `3 s` 没有有效 `track_error` 时，上位机进入 `LINE_LOSS_SAFE_STOP` 并重复下发 `STATE_SAFE_STOP`。
- 再次识别到语义分割中线后，下一帧恢复 `STATE_TRACK + VISION`，不会锁死在 `STATE_SAFE_STOP`。
- 中线丢失阈值可通过 `AR_LINE_LOSS_SAFE_STOP_TIMEOUT` 调整；丢线状态下命令重复间隔可通过 `AR_LINE_LOSS_COMMAND_REPEAT_INTERVAL` 调整。
- TC264D 本地连续 `2.5 s` 未收到有效控制帧时，自行进入 `STATE_SAFE_STOP`。
- OCR/API 和多状态任务决策尚未接入；`state_cmd / target_speed / track_error / flags` 接口为后续扩展保留。
- 当前分割和检测模型效果较差，仍需继续训练、调参和实车验证。

## 可选手柄遥控

- 手柄遥控只用于调试和采集分割数据集，默认不启用。
- Windows 定位 EXE 勾选 `Gamepad Mode` 后，额外向板卡 `9010` 发送 `gamepad_control` 包。
- `Gamepad Mode` 由 Windows 定位 EXE 内部独立定时器发送，频率与当前相机/视频帧率一致，不依赖 Tag、标定或 `robot_position` 成功发送。
- RK3588S 只有收到 `gamepad_mode=true` 且未超过 `3.0 s` 的新鲜遥控包时，才临时覆盖视觉控制；取消勾选会主动发送关闭包，正常关闭不需要等待超时。
- 取消勾选、遥控包超时、或使用赛方定位模块时，RK3588S 自动回到视觉巡线。
- 遥控模式不影响定位：`robot_position` 仍走 `9005`，并继续转发到 `127.0.0.1:9006`。
- 默认映射：`RT` 前进、`LT` 倒车，合成为 `target_speed=(RT-LT)*1.0 m/s`；`LX -> track_error`；`B` 键或手柄断连 -> `STATE_SAFE_STOP`。

## 端口与设备

| 用途 | 当前配置 |
| --- | --- |
| Windows 定位入口 | `0.0.0.0:9005` |
| 官方 AR 转发 | `127.0.0.1:9006` |
| WebUI 定位数据 | `127.0.0.1:9006` |
| 可选手柄遥控入口 | `0.0.0.0:9010` |
| 定位状态 HTTP | `0.0.0.0:9105` |
| 融合视频共享内存 | `shm_ar_video` |
| TC264D 串口 | 固定 `/dev/ttyUSB0`，`460800` baud；打开失败或读写异常时约每 `1 s` 自动重连 |

当前定位约定以 Windows AprilTag 程序为准：场地 `4 m x 3 m`，定位预览原点在右下角，`+X` 向左、`+Z` 向上，Yaw 使用 `euler[1]`，`0 deg -> +X`、`+90 deg -> +Z`。Windows 端已对输出位姿做轻量滤波。官方 AR 场景资产的 X/Z 平移轴与 Windows 定位预览互换，因此 RK3588S 仅在转发到 `127.0.0.1:9006` 时交换 `pos[0]` 和 `pos[2]`，Yaw 保持不变；定位仍不转换为控车误差。

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

通过 WebUI 启用需要 RK 侧模型、HUD 或控制状态的流时，入口为 `ar_receiver.py`；`ar_receiver.py` 会自动启动定位接收转发和手柄遥控接收后台线程。

如果当前只运行纯净 AR 融合流、没有启动 `ar_receiver.py`，但需要定位和手柄遥控，可手动运行备用桥：

```bash
python3 standalone_control_bridge.py
```

备用桥只负责 `9005 -> 9006` 定位转发和 `9010 -> TC264D` 手柄控车，不运行分割、检测或视觉巡线。
