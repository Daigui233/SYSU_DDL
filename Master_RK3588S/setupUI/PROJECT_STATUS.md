# RK3588S 上位机当前状态

## 正式链路

```text
Windows 顶置相机
  -> AprilTag 定位 robot_position
  -> UDP 板卡IP:9005
  -> pose_ar_bridge.py
  -> 127.0.0.1:9006
  -> 官方 AR 引擎
  -> 融合视频 shm_ar_video
  -> ar_receiver.py 读取视频并运行模型
  -> 结构化感知 segmentation + detections
  -> control_race_state_machine.py 更新圈数、红绿灯和终点事件
  -> control_task_state_machine.py 生成 task_state / desired_speed / planner_intent
  -> control_local_planner.py 生成 final_track_error
  -> state_cmd + target_speed + final_track_error + flags
  -> /dev/ttyUSB0
  -> TC264D
```

AprilTag 定位只服务官方 AR 融合、坐标显示和记录，不参与 `track_error`、`state_cmd` 或转向控制。检测模型输出 `gold / car / human / stone / traffic_light / door / begin_sign / end_sign` 等结构化感知结果，进入上位机状态机；检测结果不直接控车。

## 当前模块边界

| 文件 | 作用 |
| --- | --- |
| `ar_receiver.py` | 上位机主入口：启动模块、读取帧、调度视觉/仲裁/发送/显示 |
| `pose_ar_bridge.py` | 独立接收 Windows `robot_position`，校验、记录并转发到 `127.0.0.1:9006` |
| `control_gamepad_receiver.py` | 独立接收 `9010` 手柄遥控包，只提供候选控制命令，不直接写串口 |
| `vision_pipeline.py` | 分割/检测模型初始化、推理、后处理，输出结构化 `segmentation + detections` 并绘制基础视觉结果 |
| `vision_traffic_light.py` | 在 `TrafficLight` 检测框内用 OpenCV HSV 判断红/绿/未知 |
| `control_states.py` | 状态契约：定义 RK 任务状态、TC264D 状态码、局部规划模式和映射关系 |
| `control_race_state_machine.py` | 比赛事件状态机：只基于图像检测框跟踪 Door、BeginSign、EndSign、红绿灯和圈数 |
| `control_task_state_machine.py` | 上位机任务状态机：维护状态速度表、掉线恢复、避人滞回和任务触发规则 |
| `control_local_planner.py` | 图像坐标局部规划，生成连续最终目标线，输出 `final_track_error` 并绘制规划结果 |
| `control_arbitrator.py` | 最终控制仲裁：合成状态机/局部规划结果和可选手柄覆盖，生成串口控制帧 |
| `control_car_link.py` | 封装 `/dev/ttyUSB0` 串口控车链路，供主入口或备用桥复用 |
| `status_runtime.py` | 写入 `/pose_status` 所需的定位、控制、AI 和 TC264D 状态 |
| `webui_status_server.py` | WebUI 状态 HTTP 服务、配置接口和轻量调试 API |
| `ui_hud_renderer.py` | OpenCV 预览窗口 HUD 绘制：左侧裁判事件提示，中间 AR 视频，右侧链路调试栏 |
| `vision_frame_source.py` | 从 `shm_ar_video` 共享内存读取并转换帧 |
| `debug_tools.py` | 调试日志轮转和定位链路分段打印 |
| `standalone_control_bridge.py` | 手动备用桥：不启动 `ar_receiver.py` 时接收定位和手柄，并可用手柄控车 |
| `control_serial_comm.py` | TC264D 控制帧和反馈帧串口通信，固定 `/dev/ttyUSB0`、`460800` |
| `infer_wrap/base/seg_func.py` | 分割后处理、中线提取和 `track_error` 计算 |
| `infer_wrap/base/func.py` | 检测模型后处理与画框 |
| `templates/index.html` | WebUI 页面和右侧链路调试状态栏 |
| `dist/main_config.json` | 官方 AR/WebUI 网络配置 |

工程约定：后续新增上位机功能时，优先新建可复用模块文件，再由 `ar_receiver.py` 调度。不要把状态机、路径规划、OCR/API、数据记录或大块调参逻辑直接堆进 `ar_receiver.py`。状态机放在 `control_task_state_machine.py`，局部规划放在 `control_local_planner.py`。

`standalone_control_bridge.py` 是手动备用入口，适合采集数据时只打开纯净 AR 融合流、没有启动 `ar_receiver.py`，但仍需要 `9005` 定位转发和 `9010` 手柄控车的情况。它不是默认入口，不要和 `ar_receiver.py` 同时运行，否则可能争用 `9005/9010` 端口或形成双控制源。

## 当前控制

- 当前以 RK 上位机状态机为准，默认状态为 `NORMAL_TRACK`，下发 `STATE_TRACK + target_speed + flags=0x01`。
- 普通检测目标已拆成风险池和奖励池：`human / car / stone` 平级进入风险池，按目标底边距离等级、相对当前路径的横向遮挡等级和短时状态保持奖励计算 `risk_score`；`gold` 属于奖励池，只在风险池为空且金币足够近、路径代价不大时触发 `COLLECT_GOLD`。
- 状态机速度由 `control_task_state_machine.py` 开头的 `TASK_SPEED_DEFAULTS` 管理，当前所有非停车状态默认 `0.05 m/s`。
- 无有效分割误差但未超过 `3 s` 时，进入 `RECOVER_LINE`，短暂保持/衰减上一帧有效目标线，速度由 `TASK_SPEED_DEFAULTS[TaskState.RECOVER_LINE]` 管理。
- 避车/避人不再把红色中线整体平移；`control_local_planner.py` 会让紫色规划线从近处红线连续延伸，中远处逐渐偏向绕行侧，并从规划线前瞻点计算 `final_track_error`。`AVOID_STONE` 使用独立状态，在分岔候选中默认走外圈，若 stone 命中默认外圈候选路径则选择内圈。
- `final_track_error` 使用目标线相对摄像头视觉中心线的实际像素偏差；误差上限默认从 `dist/main_config.json` 的 `width` 自动取半幅宽，当前 `640x480` 为 `±320 px`。
- 连续 `3 s` 没有有效 `track_error` 时，上位机进入 `LINE_LOSS_SAFE_STOP`，并按状态契约下发 `STATE_LINE_LOSS_SAFE_STOP`。
- 再次识别到语义分割中线后，下一帧恢复 `STATE_TRACK + NORMAL_TRACK`，不会锁死在停机状态。
- 比赛事件由 `control_race_state_machine.py` 管理：远处红灯只记录为 `red_far`，红灯进入近处停车区后才进入 `TRAFFIC_LIGHT_STOP`，绿灯继续通行；`Door + BeginSign` 开始第 1 圈，之后每次有效经过 Door 更新圈数；看到 `EndSign` 后继续循迹，直到 EndSign 消失超过短 TTL 后进入 `ENDSIGN_STOP`。
- 中线丢失阈值和状态保持时间由 `control_task_state_machine.py` 开头的 `TASK_TIMING_DEFAULTS` 管理；命令重复间隔由 `control_arbitrator.py` 开头参数管理。
- TC264D 本地连续 `2.5 s` 未收到有效控制帧时，自行进入 `STATE_SAFE_STOP`。
- OCR/API 和更多任务决策尚未接入；`state_cmd / target_speed / track_error / flags` 接口为后续扩展保留。
- 检测模型标签包含 `Door / SpeedSign / TurnSign / Stone / BeginSign / EndSign / Crosswalk / TrafficLight / Coin / Human / Car`；当前进入控车/比赛事件逻辑的是 `gold / car / human / stone / Door / BeginSign / EndSign / TrafficLight`，`SpeedSign / TurnSign / Crosswalk` 暂不参与控车。
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

当前定位约定以 Windows AprilTag 程序为准：场地 `4 m x 3 m`，定位预览右下角基准点为 `(x=0, z=0.30 m)`，底边 `z=0.30 m`、上边 `z=3.30 m`，`+X` 向左、`+Z` 向上，Yaw 使用 `euler[1]`，`0 deg -> +X`、`+90 deg -> +Z`。Windows 端已对输出位姿做轻量滤波。官方 AR 场景资产的 X/Z 平移轴与 Windows 定位预览互换，因此 RK3588S 仅在转发到 `127.0.0.1:9006` 时交换 `pos[0]` 和 `pos[2]`，Yaw 保持不变；定位仍不转换为控车误差。

## 已知风险

- `NORMAL_TRACK`、避车、避人、stone、金币和丢线恢复默认速度均为 `0.05 m/s`；`TRAFFIC_LIGHT_STOP / ENDSIGN_STOP / LINE_LOSS_SAFE_STOP` 为 `0.0 m/s`。速度在 `control_task_state_machine.py` 开头统一维护，首次落地前仍应架空车轮确认方向和制动行为。
- TC264D 电机 PWM 硬限幅为 `±2500` duty；`STATE_TRACK` 当前使用约 `1450` duty 启动前馈 + 小 PI 修正，先解决电机死区和低速卡顿，PID 只做稳速微调。
- `target_speed` 当前按同款 CarDo 车模参数粗换算为 `m/s`；TC264D 的 `actual_speed` 由编码器计数换算得到，后续仍需确认编码器正负号并用实测速度修正比例误差。
- 视觉模型短时丢线会进入 `RECOVER_LINE` 并保持/衰减上一帧有效目标线；若连续 `3 s` 没有有效 `track_error`，RK3588S 会进入 `LINE_LOSS_SAFE_STOP`。
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
