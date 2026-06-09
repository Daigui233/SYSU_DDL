# SYSU_DDL

RK3588S 上位机 + TC264D 下位机智能车工程。当前阶段已搭建视觉巡线、结构化感知、上位机任务状态机、图像坐标局部规划和 TC264D 串口控制链路；默认仍以低速 `NORMAL_TRACK` 巡线为主，`gold / car / human` 检测先接入第一阶段局部任务逻辑。

## 正确系统架构

```text
Windows 顶置相机
  -> AprilTag 定位 robot_position
  -> UDP 板卡IP:9005 -> pose_ar_bridge.py -> 127.0.0.1:9006
  -> 官方 AR 引擎根据定位更新融合画面
  -> 融合视频流 shm_ar_video
  -> RK3588S 视觉模型与上位机决策
       - 分割 -> 基础中线 track_error / line_valid
       - 检测 -> gold / car / human 感知输入
       - control_task_state_machine.py -> task_state / desired_speed / planner_intent
       - control_local_planner.py -> final_track_error
  -> /dev/ttyUSB0 -> TC264D
  -> 电机、舵机和状态执行

可选数据采集遥控链路：
Windows 定位 EXE 勾选 Gamepad Mode
  -> UDP 板卡IP:9010
  -> control_gamepad_receiver.py -> ar_receiver.py 临时覆盖视觉控制
  -> /dev/ttyUSB0 -> TC264D
```

AprilTag 定位只服务于 AR 融合、坐标显示和记录，不直接规划路线，也不直接生成舵机误差。默认情况下小车如何行驶，由融合视频上的视觉结果和上位机决策决定；只有 Windows 定位 EXE 中显式勾选 `Gamepad Mode` 且 RK3588S 收到新鲜遥控包时，才临时进入手柄遥控。

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `ArucoCalib-master/ArucoCalib-master/` | Windows AprilTag 定位，发送官方 `robot_position`；可选发送手柄遥控包 |
| 官方 AR 引擎/WebUI | 根据定位更新 AR 世界并产生融合视频流 |
| `Master_RK3588S/setupUI/ar_receiver.py` | 上位机主入口：启动模块、读取帧、调度视觉/仲裁/发送/显示 |
| `Master_RK3588S/setupUI/pose_ar_bridge.py` | 接收 Windows `robot_position`，校验、记录并转发到官方 AR `127.0.0.1:9006` |
| `Master_RK3588S/setupUI/control_gamepad_receiver.py` | 接收可选 `9010` 手柄遥控包，提供给 `ar_receiver.py` 做控制源仲裁 |
| `Master_RK3588S/setupUI/vision_pipeline.py` | 分割/检测模型初始化、推理、后处理，输出结构化 `segmentation + detections` 并绘制基础视觉结果 |
| `Master_RK3588S/setupUI/control_states.py` | 上位机任务状态、TC264D 状态码、局部规划模式和状态映射契约 |
| `Master_RK3588S/setupUI/control_task_state_machine.py` | 上位机任务状态机：根据分割、检测和掉线计时输出 `task_state / desired_speed / planner_intent` |
| `Master_RK3588S/setupUI/control_local_planner.py` | 图像坐标局部规划：根据状态机意图输出最终 `final_track_error`，并绘制最终目标线 |
| `Master_RK3588S/setupUI/control_arbitrator.py` | 最终控制仲裁：合成状态机/局部规划结果和可选手柄覆盖，生成串口控制帧 |
| `Master_RK3588S/setupUI/control_car_link.py` | 封装 RK3588S 到 TC264D 的串口控制链路 |
| `Master_RK3588S/setupUI/status_runtime.py` | 写入 `/pose_status` 所需的定位、控制、AI 和 TC264D 状态 |
| `Master_RK3588S/setupUI/performance_monitor.py` | 运行时性能探针：统计 AR 帧率、主循环耗时、HUD/显示耗时、NPU/温度信息，并写入 CSV |
| `Master_RK3588S/setupUI/webui_status_server.py` | WebUI 状态 HTTP 服务、配置接口和轻量调试 API |
| `Master_RK3588S/setupUI/ui_hud_renderer.py` | OpenCV 预览窗口 HUD 绘制：左侧裁判事件提示，中间 AR 视频，右侧链路调试栏 |
| `Master_RK3588S/setupUI/vision_frame_source.py` | 从 `shm_ar_video` 共享内存读取并转换帧 |
| `Master_RK3588S/setupUI/debug_tools.py` | 调试日志轮转和定位链路分段打印 |
| `Master_RK3588S/setupUI/standalone_control_bridge.py` | 手动备用桥：不启动 `ar_receiver.py` 时接收定位和手柄，并可用手柄控车 |
| `Master_RK3588S/setupUI/control_serial_comm.py` | 向 TC264D 下发控制帧并接收反馈 |
| `Slave_TC264D/` | 执行上位机给出的状态、速度和转向误差 |
| `Tools_Windows/gamepad_mapper.py` | 手柄输入和映射测试工具，不直接控车 |

命名前缀用于表达大模块归属：`control_*` 属于控车、状态机、串口和手柄接管；`vision_*` 属于视频帧和模型感知；`pose_*` 属于定位与 AR 桥接；`ui_* / webui_* / status_*` 属于显示和状态服务；`performance_*` 属于性能监控与事后分析；`debug_*` 属于调试辅助。`ar_receiver.py` 保持主入口名称不变，只负责调度这些模块。

## 当前阶段

- 工程约定：后续新增上位机功能时，优先新建可复用模块文件，再由 `ar_receiver.py` 调度；不要把状态机、路径规划、OCR/API、数据记录等大块逻辑直接堆进 `ar_receiver.py`。核心调车参数、阈值和速度表由对应模块维护，`ar_receiver.py` 只负责实例化和连接模块。
- 分割模型从融合视频生成基础 `track_error`；目标检测作为感知输入进入 `control_task_state_machine.py`，由状态机输出 `task_state / desired_speed / planner_intent`。
- `control_local_planner.py` 根据任务状态输出最终 `final_track_error`，上位机再以对应状态码、目标速度和 `flags=0x01` 下发给 TC264D。
- 当前分割和检测模型效果较差，仍需优化；`gold / car / human` 已接入第一阶段局部任务逻辑，但需要低速实车验证。
- OCR 和外部 API 尚未接入；当前 `gold / car / human` 只接入第一阶段图像坐标局部策略，红绿灯、复杂任务和全局规划暂不接入。
- 控制协议中的 `state_cmd / target_speed / track_error / flags` 保持不变；当前状态契约见 `Master_RK3588S/setupUI/state_contract.md`，新增状态时需要同步 `control_states.py` 和 `Slave_TC264D/code/State.h`。
- 无有效分割误差或感知质量不可信但未超过 `3 s` 时，当前 `RECOVER_LINE` 会短暂保持/衰减上一帧有效目标线，默认速度由 `control_task_state_machine.py` 管理。
- 连续 `3 s` 没有有效 `track_error` 时，RK3588S 进入 `LINE_LOSS_SAFE_STOP`，并按状态契约下发 `STATE_LINE_LOSS_SAFE_STOP`；再次识别到语义分割中线后，下一帧恢复 `STATE_TRACK + NORMAL_TRACK`。
- 手柄遥控仅用于调试和采集数据：默认关闭，只有定位 EXE 勾选 `Gamepad Mode` 后才通过 UDP `9010` 接管；关闭、丢包超时或赛方定位模块没有遥控包时，一律回到视觉控车。
- `Gamepad Mode` 在定位 EXE 内部用独立定时器发送，频率与当前相机/视频帧率一致；它不依赖固定 Tag、车载 Tag、标定状态或 `robot_position` 是否成功发送。
- 遥控模式下定位 UDP `9005` 仍正常发送并被 RK3588S 转发到 `9006`，不会因为手柄接管而停止 AR 融合。
- 遥控映射：`RT` 前进、`LT` 倒车，合成为 `target_speed=(RT-LT)*1.0 m/s`；左摇杆横轴 `LX` 控制 `track_error`；`B` 键或手柄断连时发送 `STATE_SAFE_STOP`。Windows 定位 EXE 优先读取 XInput，读不到时会用 pygame/DirectInput 兜底，便于蓝牙手柄调试。
- 如果采集数据时只打开纯净 AR 融合流、没有启动 `ar_receiver.py`，但仍需要定位转发和手柄控车，可在 RK3588S 手动运行 `python3 Master_RK3588S/setupUI/standalone_control_bridge.py`；该备用脚本不要和 `ar_receiver.py` 同时运行。
- TC264D 保留现有串口协议，同时增加 `2.5 s` 本地输入超时；若上位机进程崩溃导致串口帧停止，下位机会自行进入 `STATE_SAFE_STOP`。
- `performance_monitor.py` 已接入 `ar_receiver.py` 主循环：HUD 显示关键性能摘要，完整样本默认写入 `Master_RK3588S/setupUI/performance_debug.csv`，用于判断瓶颈在 AR 源头、视觉推理、HUD 绘制、窗口显示还是硬件降频。

## 架构改进记录

近期已收敛：

1. `control_task_state_machine.py` 顶部已集中 `TASK_SPEED_DEFAULTS`、`TASK_TIMING_DEFAULTS`、`TASK_RULE_DEFAULTS` 和 `PERCEPTION_QUALITY_DEFAULTS`，不同状态速度、检测触发阈值、分割质量阈值和检测 age 限制都在文件开头统一修改。
2. `control_task_state_machine.py` 已建立基础感知质量契约：状态机不再只看 `line_valid`，还会检查 segmentation `age/source/road_ratio/road_state/midline_state` 和 detection `age`，质量不可信时先进入 `RECOVER_LINE`，连续超时后才进入 `LINE_LOSS_SAFE_STOP`。
3. `control_local_planner.py` 顶部已集中 `PLANNER_DEFAULTS`，避障偏置、金币吸引、最大误差、恢复衰减和 road mask 左右侧评分阈值都不再使用环境变量覆盖。

后续仍需改进：

1. `RECOVER_LINE` 还可以更聪明：当前丢线恢复主要依赖 `control_local_planner.py` 保持/衰减上一帧 `final_track_error`。如果入弯时转向偏慢、目标线跑出视野，单纯衰减可能会越恢复越接近直行。后续应加入最近误差趋势、弯道方向保持和低速搜索恢复。
2. 上下位机状态契约需要自动校验：当前状态码同时存在于 `control_states.py`、`control_serial_comm.py`、`control_car_link.py` 和 `Slave_TC264D/code/State.h`。虽然文档已对齐，但后续新增状态时仍有手工漏改风险。建议增加轻量检查脚本或测试，验证 Python 状态映射和 TC264D 枚举值一致。
3. `ar_receiver.py` 仍承担过多运行细节：它现在保持主入口名称不变，但同时负责帧循环、无新帧重复下发、状态写入、HUD 显示和模块生命周期。后续可以在不改入口名的前提下，把控制循环、运行状态组装和显示更新拆成更小模块。
4. WebUI/HUD 调车信息还可以更完整：当前能看到控制状态、误差、速度和串口反馈，但对状态机 `reason`、`perception_quality`、`planner_reason`、`line_loss_age`、检测目标摘要等信息展示不足。后续应把 `task_decision` 和 `plan_result` 写入 `/pose_status`。
5. 缺少离线回放和单元测试：状态切换、丢线恢复、避人/避车/金币偏置、串口状态码映射都适合用保存下来的 perception 数据做回放测试。后续应补充最小测试集，先验证不改协议、不误触发、不在短时丢线时过早停车。

## 定位与端口

官方定位包：

```json
{"type":"robot_position","pos":[x,0.16,z],"euler":[0.0,yaw,0.0]}
```

- Windows 定位程序发送到板卡局域网 IP 的 UDP `9005`。
- `ar_receiver.py` 转发到板卡本机 `127.0.0.1:9006`；转发给官方 AR 时只交换平移轴为 `pos=[z,0.16,x]`，`yaw=euler[1]` 保持不变。
- WebUI 定位数据地址填写 `127.0.0.1`，端口填写 `9006`。
- 手柄遥控输入为独立 UDP `9010`，只接收 `type=gamepad_control`，不复用定位包。
- UNITY 同步端口为 `9003`。
- 控车串口固定为 `/dev/ttyUSB0`，`460800` baud；若启动时串口不存在或读写异常，RK3588S 会保持约 `1 s` 间隔自动重连。

场地为 `4 m × 3 m`，Windows 定位预览原点在右下角，`+X` 向左、`+Z` 向上；Yaw 使用 `euler[1]`，`0° -> +X`、`+90° -> +Z`。固定标定 Tag 1～4 均为 `20 cm × 20 cm`，Windows 定位端会对输出坐标和 Yaw 做轻量滤波。由于官方 AR 场景资产的 X/Z 平移轴与定位预览互换，RK3588S 仅在转发到 `9006` 时交换 `pos[0]` 和 `pos[2]`，不改变 Yaw。这些坐标只用于 AR 定位与融合，不用于直接控车。

## 待解决与待确认

### 当前巡线

1. 使用真实融合视频重新训练或优化分割模型，使 `track_error` 足以稳定巡线。
2. 架空车轮验证误差方向、舵机方向、PID、速度、`flags=0x01` 和反馈帧。
3. 首次落地前使用状态机默认 `0.3 m/s` 验证巡线和 `RECOVER_LINE`；需要更慢或更快时直接修改 `control_task_state_machine.py` 开头的速度表，重启 `ar_receiver.py` 后生效。
4. 当前 `target_speed` 已按同款 CarDo 车模参数粗换算为 `m/s`，TC264D 的 `actual_speed` 由编码器计数换算得到；后续仍需确认编码器正负号，并用实测速度修正轮径、周期或比例误差。
5. 若用手柄采集数据，先确认 `Gamepad Mode` 勾选、WebUI `GAMEPAD` 状态、TC264D 反馈和现场急停手段，再低速落地。

### 定位与 AR

1. 工业相机到货后完成固定 Tag 中心实测、相机参数验证、内参与畸变标定。
2. 确认车载 Tag ID、朝向、安装偏角，以及 AR 坐标和 Yaw 方向。
3. 实测并校准 WebUI 的 `pos_offset` 与 `euler_offset`。

### 后续扩展

1. 优化或更换检测模型，并接入 OCR/API。
2. 在 `control_task_state_machine.py` 中扩展新任务规则，在 `control_local_planner.py` 中扩展对应局部规划策略。
3. 为每个新增状态分别定义进入条件、保持条件、退出条件和失效策略，保持现有串口协议兼容。

## 当前联调顺序

1. 确认定位能稳定驱动 AR 融合画面。
2. 先以 `NORMAL_TRACK` 低速验证 `融合视频 -> 分割模型 -> 状态机 -> final_track_error -> TC264D`。
3. 模型稳定后再逐项实测 `AVOID_HUMAN / AVOID_CAR / COLLECT_GOLD`，最后接入 OCR/API 和更复杂任务状态。

详细使用和协议见各子工程 README。
