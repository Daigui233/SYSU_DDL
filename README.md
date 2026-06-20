# SYSU_DDL

RK3588S 上位机 + TC264D 下位机智能车工程。当前阶段已搭建视觉巡线、结构化感知、比赛事件状态机、上位机任务状态机、图像坐标局部规划和 TC264D 串口控制链路；默认仍以低速 `NORMAL_TRACK` 巡线为主，`gold / car / human / stone / traffic_light / door / begin_sign / end_sign` 已接入第一阶段结构化逻辑。

## 正确系统架构

```text
Windows 顶置相机
  -> AprilTag 定位 robot_position
  -> UDP 板卡IP:9005 -> pose_ar_bridge.py -> 127.0.0.1:9006
  -> 官方 AR 引擎根据定位更新融合画面
  -> 融合视频流 shm_ar_video
  -> RK3588S 视觉模型与上位机决策
       - 分割 -> 基础中线 track_error / line_valid
       - 检测 -> gold / car / human / stone / traffic_light / door / begin_sign / end_sign 感知输入
       - control_race_state_machine.py -> 圈数、红绿灯、终点事件
       - control_task_state_machine.py -> task_state / desired_speed / planner_intent
       - control_local_planner.py -> final_track_error
       - control_arbitrator.py -> track_error 直通下发
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
| `Master_RK3588S/setupUI/vision_pipeline.py` | 分割/检测模型初始化、同帧并行推理、结果同步和后处理，输出结构化 `segmentation + detections` 并绘制基础视觉结果 |
| `Master_RK3588S/setupUI/vision_traffic_light.py` | 在 `TrafficLight` 检测框内用 OpenCV HSV 判断红/绿/未知 |
| `Master_RK3588S/setupUI/control_states.py` | 上位机任务状态、TC264D 状态码、局部规划模式和状态映射契约 |
| `Master_RK3588S/setupUI/control_race_state_machine.py` | 比赛事件状态机：只基于图像检测框跟踪 Door、BeginSign、EndSign、红绿灯和圈数 |
| `Master_RK3588S/setupUI/control_task_state_machine.py` | 上位机任务状态机：根据分割、检测和掉线计时输出 `task_state / desired_speed / planner_intent` |
| `Master_RK3588S/setupUI/control_local_planner.py` | 图像坐标局部规划：根据状态机意图输出最终 `final_track_error`，并绘制最终目标线 |
| `Master_RK3588S/setupUI/control_arbitrator.py` | 最终控制仲裁：合成状态机/局部规划结果和可选手柄覆盖，生成串口控制帧 |
| `Master_RK3588S/setupUI/control_car_link.py` | 封装 RK3588S 到 TC264D 的串口控制链路 |
| `Master_RK3588S/setupUI/status_runtime.py` | 写入 `/pose_status` 所需的定位、控制、AI 和 TC264D 状态 |
| `Master_RK3588S/setupUI/performance_monitor.py` | 轻量性能探针：统计帧率、分割/检测推理与后处理耗时、CPU/NPU/GPU负载和温度，并写入 CSV |
| `Master_RK3588S/setupUI/webui_status_server.py` | WebUI 状态 HTTP 服务、配置接口和轻量调试 API |
| `Master_RK3588S/setupUI/ui_hud_renderer.py` | OpenCV 预览窗口 HUD 绘制：左侧裁判事件提示，中间 AR 视频，右侧链路调试栏 |
| `Master_RK3588S/setupUI/ui_debug_render_worker.py` | latest-only 调试画面渲染线程：绘制分割/检测/规划/HUD，慢时丢 debug 帧，不阻塞控车主循环 |
| `Master_RK3588S/setupUI/ui_debug_stream_server.py` | 独立调试视频流服务，把完整 HUD 调试画面发布到浏览器可访问的 MJPEG 端口 |
| `Master_RK3588S/setupUI/vision_frame_source.py` | 从 `shm_ar_video` 读取一致帧快照，复制前后校验 `fid/尺寸` 并转换图像 |
| `Master_RK3588S/setupUI/debug_tools.py` | 调试日志轮转和定位链路分段打印 |
| `Master_RK3588S/setupUI/standalone_control_bridge.py` | 手动备用桥：不启动 `ar_receiver.py` 时接收定位和手柄，并可用手柄控车 |
| `Master_RK3588S/setupUI/control_serial_comm.py` | 向 TC264D 下发控制帧并接收反馈 |
| `Slave_TC264D/` | 执行上位机给出的状态、速度和转向误差 |
| `Tools_Windows/gamepad_mapper.py` | 手柄输入和映射测试工具，不直接控车 |

命名前缀用于表达大模块归属：`control_*` 属于控车、状态机、串口和手柄接管；`vision_*` 属于视频帧和模型感知；`pose_*` 属于定位与 AR 桥接；`ui_* / webui_* / status_*` 属于显示和状态服务；`performance_*` 属于性能监控与事后分析；`debug_*` 属于调试辅助。`ar_receiver.py` 保持主入口名称不变，只负责调度这些模块。

## 当前阶段

- 工程约定：后续新增上位机功能时，优先新建可复用模块文件，再由 `ar_receiver.py` 调度；不要把状态机、路径规划、OCR/API、数据记录等大块逻辑直接堆进 `ar_receiver.py`。核心调车参数、阈值和速度表由对应模块维护，`ar_receiver.py` 只负责实例化和连接模块。
- 分割模型从融合视频生成基础 `track_error`；目标检测作为感知输入进入 `control_task_state_machine.py`，由状态机输出 `task_state / desired_speed / planner_intent`。
- `control_local_planner.py` 根据任务状态输出最终 `final_track_error`，`control_arbitrator.py` 不再缩放、死区、限幅、限步或非线性增强视觉误差，视觉自动驾驶时直接下发 `command["track_error"] = final_track_error`。
- `final_track_error` 定义为当前帧目标线 `lookahead_y=300` 处相对摄像头视觉中心线的实际像素偏差；上位机不再对该误差做人为上限、跨帧平滑或逐帧追赶。
- 调试画面中红色曲线表示语义分割得到的赛道中线；紫色曲线表示局部规划后的最终目标路径。避障时紫色线从近处红线连续延伸出去，中远处逐渐偏向绕行侧，不再对整条红线做瞬时平移。
- 目标检测任务分为硬规则层、风险池和奖励池：红灯近处停车、连续丢线停车保持硬规则；`human / car / stone` 进入风险池，按 `distance_level + path_level + state_hold_bonus` 组成的简单 `risk_score` 排序；`gold` 属于奖励池，只有风险池为空且金币足够近、路径代价不大时才触发 `COLLECT_GOLD`。当前 `EndSign` 终点停车默认禁用，只保留观测。
- `control_race_state_machine.py` 独立处理比赛事件：`TrafficLight` 框内红灯远处只记录为 `red_far`，进入近处停车区才触发 `TRAFFIC_LIGHT_STOP`，绿灯通行；`Door + BeginSign` 启动第 1 圈，之后每次有效经过 `Door` 更新圈数；当前 `EndSign` 因容易与 `BeginSign` 混淆，只记录 `end_under_door / end_sign_allowed / end_sign_stop_enabled`，不会进入 `ENDSIGN_STOP`。
- 避人/避车时 `final_track_error` 直接从当前紫色规划线前瞻点计算。`AVOID_HUMAN` 会估计行人 bbox 横向速度，并把人的短时预测占用区纳入 road mask 左右走廊评分；运动方向参与动态占用区预测和接近打平时的倾向。若车身已偏向计划绕行侧、规划横移过大或选中走廊过窄，局部规划器会进入 `yield_wait`，临时把速度覆盖为 `0.0`，等待行人继续移动后再重新判定。
- 当前控制链路为：`segmentation track_error -> control_task_state_machine.py -> control_local_planner.py final_track_error -> control_arbitrator.py 直通下发 -> TC264D input_track_error -> 舵机线性 P / 电机 PID / PWM 硬限幅`。
- 上位机控制层不做 error 限幅、不做 CMD error 限步、不做 error 非线性增强；下位机是唯一控制执行和硬保护位置。
- 摄像头/图像中心线只作为 `track_error` 的计算参考，默认不在调试画面中显示；调车时主要看红色中线和紫色最终目标路径。
- 当前分割和检测模型效果较差，仍需优化；`gold / car / human / stone / TrafficLight / Door / BeginSign / EndSign` 已接入第一阶段逻辑，但需要低速实车验证。
- 当前检测模型还包含 `SpeedSign / TurnSign / Crosswalk`，这些类别暂未接入任务状态机，只作为后续 OCR/API 或赛题任务扩展入口。
- 当前状态机速度表中所有非停车状态默认 `0.05 m/s`，`LINE_LOSS_SAFE_STOP` 保持 `0.0 m/s`。
- OCR 和外部 API 尚未接入；当前 `gold / car / human / stone` 只接入第一阶段图像坐标局部策略，红绿灯和圈数/终点只作为比赛事件状态机输入，全局规划暂不接入。
- 控制协议中的 `state_cmd / target_speed / track_error / flags` 保持不变；当前状态契约见 `Master_RK3588S/setupUI/state_contract.md`，新增状态时需要同步 `control_states.py` 和 `Slave_TC264D/code/State.h`。
- 无有效分割误差或感知质量不可信但未超过约 `0.8 s` 时，当前 `RECOVER_LINE` 会短暂保持/衰减上一帧有效目标线，恢复衰减窗口约 `0.5 s`，默认速度由 `control_task_state_machine.py` 管理。
- 连续约 `0.8 s` 没有有效 `track_error` 时，RK3588S 进入 `LINE_LOSS_SAFE_STOP`，并按状态契约下发 `STATE_LINE_LOSS_SAFE_STOP`；再次识别到语义分割中线后，下一帧恢复 `STATE_TRACK + NORMAL_TRACK`。
- 手柄遥控仅用于调试和采集数据：默认关闭，只有定位 EXE 勾选 `Gamepad Mode` 后才通过 UDP `9010` 接管；关闭、丢包超时或赛方定位模块没有遥控包时，一律回到视觉控车。
- `Gamepad Mode` 在定位 EXE 内部用独立定时器发送，频率与当前相机/视频帧率一致；它不依赖固定 Tag、车载 Tag、标定状态或 `robot_position` 是否成功发送。
- 遥控模式下定位 UDP `9005` 仍正常发送并被 RK3588S 转发到 `9006`，不会因为手柄接管而停止 AR 融合。
- 遥控映射：`RT` 前进、`LT` 倒车，合成为 `target_speed=(RT-LT)*1.0 m/s`；左摇杆横轴 `LX` 控制 `track_error`；`B` 键或手柄断连时发送 `STATE_SAFE_STOP`。Windows 定位 EXE 优先读取 XInput，读不到时会用 pygame/DirectInput 兜底，便于蓝牙手柄调试。
- 如果采集数据时只打开纯净 AR 融合流、没有启动 `ar_receiver.py`，但仍需要定位转发和手柄控车，可在 RK3588S 手动运行 `python3 Master_RK3588S/setupUI/standalone_control_bridge.py`；该备用脚本不要和 `ar_receiver.py` 同时运行。
- TC264D 保留现有串口协议，本地输入超时为 `0.5 s`；若上位机进程崩溃导致串口帧停止，下位机会自行进入 `STATE_SAFE_STOP`。舵机转向已关闭大误差 boost，当前使用 `FULL_STEER_ERROR_PX=320` 的线性 P 映射，再经 `Servo.h` 硬限幅。
- TC264D 反馈帧已扩展到 v3：HUD 会显示 `ERR cmd/tc/d`、`SERVO pwm/raw/lim`、`MOTOR tgt/act/out`、`MOTOR ff/pid`、`SAFE to/st/ss/ms/db` 和 `FB seq/bad/drop/fmt`，用于录视频后判断问题在上位机命令、串口传输、下位机限幅/PID 还是机械响应。
- `performance_monitor.py` 已接入 `ar_receiver.py` 主循环：右侧 HUD 只显示用于定位低帧率的 `FPS / LATENCY / COMPUTE`，完整样本默认写入 `Master_RK3588S/setupUI/performance_debug.csv`。
- `vision_pipeline.py` 对每个源 `fid` 先同时提交分割（NPU core 1）和检测（NPU core 0），再汇合并严格校验结果 `frame_id`；fork 分类器继续使用 core 2。帧间只保留一个正在处理的源帧，处理结束后直接读取共享内存最新帧，不建立历史帧队列，因此不会为了同步积压旧结果。
- `ui_debug_render_worker.py` 和 `ui_debug_stream_server.py` 已接入 `ar_receiver.py`：发布的是完整调试画面，即分割/检测/局部规划目标线 + 左侧裁判事件 + 右侧调试 HUD；debug 画面渲染和 MJPEG 编码在独立线程中 latest-only 处理，慢时丢 debug 帧，不阻塞控车主循环。
- HUD 右侧性能行含义：`FPS` 区分共享内存输入、控车循环和 debug/JPEG；`LATENCY` 分开显示分割与检测的 `infer/post`、视觉总耗时和整帧耗时；`COMPUTE` 显示 CPU 平均/最忙核心、NPU 三核以及可选 GPU 负载。Python 视觉模型通过 RKNN 运行在 NPU，OpenCV/NumPy 后处理运行在 CPU；GPU 数值只用于观察官方 AR 合成或桌面显示负载，不代表模型使用 GPU。

## 当前仍保留的软件处理

下面列出当前控制链路中仍会影响误差、路径或执行输出的软件处理，便于实车归因：

- `seg_func.py` 分割中线层仍保留：road mask 闭运算/开运算、连通域筛选、中线行扫描与同帧跳变保护、多项式拟合。已关闭旧 mask 帧保持、4 帧 mask 投票、底部锚点跨帧混合、中线点跨帧混合和 `track_error` 跨帧 EMA；当前分割 `track_error` 直接取 `lookahead_y=300` 处当前中线点相对图像中心的像素偏差。若当前帧无有效 road mask，则直接输出 LOST，不再沿用上一帧分割结果。
- `control_task_state_machine.py` 仍保留检测触发阈值、风险排序、状态 TTL/滞回、短时丢线进入 `RECOVER_LINE` 和超时安全停车。这些改变状态和目标，不直接滤波 CMD error。
- `control_local_planner.py` 仍保留路径几何处理：目标线按 y 位置平滑渐进偏移、目标线点限制在图像和 `road_mask` 内、金币吸引偏移上限、避障方向短时滞回、行人横向速度 EMA、行人短时占用区预测、行人不可行绕行时的 `yield_wait` 停等、stone 分支连续性选择，以及 `RECOVER_LINE` 对最后有效误差的线性衰减。正常 TRACK/AVOID/COLLECT/STONE 的 `final_track_error` 不再跨帧平滑、限步、限幅或非线性增强。
- `control_arbitrator.py` 对合法视觉误差不做后处理，直接下发；只保留手柄覆盖、安全停车清零、非法输入保护、命令重复间隔、状态/速度/flags 组帧。
- `control_gamepad_receiver.py` 仍对手柄候选命令做软件限幅，默认 `track_error` 为 `±240`、速度为配置的手柄最大速度；该限幅只影响手柄接管，不影响视觉自动驾驶。
- TC264D 舵机层不再有大误差 boost；当前为线性 P，`SERVO_LINEAR_KP=(SERVO_DUTY_MID-SERVO_DUTY_MIN)/320=0.5`，`KD=0`，软件输出范围为 `730±160`，再由 `SERVO_DUTY_MIN/MAX` 最终硬限幅。TC264D 电机层仍保留目标速度死区、启动前馈、速度 PI/PID 修正限幅和电机 PWM 硬限幅。
- TC264D v3 反馈额外回传输入年龄、保护/限幅 flags、舵机限幅前后输出、电机前馈、电机 PID 修正和反馈序号。这些字段只用于观测和 HUD 显示，不参与上位机控制决策。

## Windows 访问地址

当前板卡 IP 为 `192.168.43.196` 时：

| 用途 | 地址 |
| --- | --- |
| 官方纯 AR 融合视频流，用于录制干净数据集 | `http://192.168.43.196:8080/video_feed` |
| RK 上位机完整调试视频流，包含分割、路径规划和 HUD | `http://192.168.43.196:8090/debug_feed` |
| RK 上位机调试视频流首页 | `http://192.168.43.196:8090/` |
| RK 状态 JSON / WebUI 调试 API | `http://192.168.43.196:9105/pose_status` |

如果板卡 IP 变化，把上面地址中的 `192.168.43.196` 替换为新的局域网 IP。

## 架构改进记录

近期已收敛：

1. `control_task_state_machine.py` 顶部已集中 `TASK_SPEED_DEFAULTS`、`TASK_TIMING_DEFAULTS`、`TASK_RULE_DEFAULTS` 和 `PERCEPTION_QUALITY_DEFAULTS`，不同状态速度、检测触发阈值、分割质量阈值和检测 age 限制都在文件开头统一修改。
2. `control_task_state_machine.py` 已建立基础感知质量契约：状态机不再只看 `line_valid`，还会检查 segmentation `age/source/road_ratio/road_state/midline_state` 和 detection `age`，质量不可信时先进入 `RECOVER_LINE`，连续超时后才进入 `LINE_LOSS_SAFE_STOP`。
3. `control_local_planner.py` 顶部已集中 `PLANNER_DEFAULTS`，避障偏置、连续规划线 ramp、前瞻点、金币吸引、恢复衰减、行人运动预测、行人 `yield_wait` 停等阈值和 road mask 左右侧评分阈值都不再使用环境变量覆盖；最终误差不再按 AR 配置宽度限幅。
4. `control_task_state_machine.py` 已将普通目标选择改为风险池/奖励池：`human / car / stone` 平级按近处风险评分排序，`gold` 不参与风险抢占，只有无风险目标时才进入奖励池；`control_race_state_machine.py` 已把红灯分为远处记录和近处硬停，避免远处红灯过早压制近处避障。

后续仍需改进：

1. `RECOVER_LINE` 还可以更聪明：当前丢线恢复主要依赖 `control_local_planner.py` 保持/衰减上一帧 `final_track_error`。如果入弯时转向偏慢、目标线跑出视野，单纯衰减可能会越恢复越接近直行。后续应加入最近误差趋势、弯道方向保持和低速搜索恢复。
2. 上下位机状态契约需要自动校验：当前状态码同时存在于 `control_states.py`、`control_serial_comm.py`、`control_car_link.py` 和 `Slave_TC264D/code/State.h`。虽然文档已对齐，但后续新增状态时仍有手工漏改风险。建议增加轻量检查脚本或测试，验证 Python 状态映射和 TC264D 枚举值一致。
3. `ar_receiver.py` 仍承担较多运行调度：它现在保持主入口名称不变，负责帧循环、无新帧重复下发、状态写入和模块生命周期；HUD/调试画面绘制已拆到 `ui_debug_render_worker.py`，后续还可以继续把控制循环和运行状态组装拆成更小模块。
4. WebUI/HUD 调车信息还可以更完整：当前能看到控制状态、误差、速度和串口反馈，但对状态机 `reason`、`perception_quality`、`planner_reason`、`line_loss_age`、检测目标摘要等信息展示不足。后续应把 `task_decision` 和 `plan_result` 写入 `/pose_status`。
5. 缺少离线回放和单元测试：状态切换、丢线恢复、避人/避车/金币偏置、串口状态码映射都适合用保存下来的 perception 数据做回放测试。后续应补充最小测试集，先验证不改协议、不误触发、不在短时丢线时过早停车。
6. 岔路识别语义：HUD 中的 `Branches` 仍只表示 `road_mask` 连通域数量，`Branches=1` 不代表没有岔路。当前已新增独立 `road_topology=SINGLE/FORK/MERGE`，由扫描行和拟合边界判断；fork 分类器不负责识别 `MERGE`，只用于确认可选道岔路是否需要进入后续 stone 内/外道决策。后续仍需用真实分割画面调扫描、分叉点和拟合残差阈值。
7. 控制链路已轻量化：局部规划输出的 `final_track_error` 直通到串口 `track_error`，TC264D 反馈中的 `input_track_error` 应与 HUD 的 CMD err 对齐；若不对齐，优先查串口帧、反馈解析或显示延迟。

## 当前实测风险与关注点

当前主链路已经闭环，但还不能简单认为只剩分割模型和目标检测模型问题。实车效果还会受到阈值、路径规划平滑、下位机舵机响应、车模机械状态和光照的共同影响。后续实测时优先关注下面几类问题：

1. 分割中线稳定性：`NORMAL_TRACK` 和所有局部规划都依赖红色分割中线作为基础路径。如果 `road_mask` 抖动、断裂、把岔路粘成一大片，紫色目标线会跟着不稳定。分割相关问题先看 `vision_pipeline.py` 的 `road_ratio / line_valid` 和 HUD 中红线是否合理；当前 `branch_count / Branches` 只表示连通域数量，不应作为岔路是否存在的唯一依据。
2. 岔路与 `stone`：当前 `seg_func.py` 先由粗扫描点独立发现双区域几何，再启用密集扫描；它使用已确认的左右分支内边界点拟合分割线并估计分叉点，根据双区域位于分叉点上方还是下方输出 `road_topology=FORK/MERGE`，只切对应一侧的连通 `road_mask`，共同区域保持不切。拟合点跨度或残差不合格时不会强行切分。fork 分类器只确认可选道岔路和后续 stone 变道需求，不参与 `MERGE` 识别；`MERGE` 第一次根据上一帧当前支路连续性选择一个分支 mask，随后锁定该 side，只沿对应 mask 中线汇入，不再被另一支路评分带偏。`AVOID_STONE` 仍只做图像坐标局部选择，不使用 AR 地图坐标。
3. 检测框远近判断：`human / car / stone` 目前进入风险池后按距离等级、路径遮挡等级和状态保持奖励组成 `risk_score` 排序，类别只保留很小的 tie-break；`gold` 属于奖励池，只有无风险目标且金币足够近时才处理。如果目标已经很近但状态没有切换，优先调 `control_task_state_machine.py` 顶部的 `RISK_*` 阈值；如果远处目标提前切换，说明底边触发带或路径横向范围过宽。
4. 避障转向激进程度：紫色线已经改成连续规划线，`AVOID_CAR / AVOID_HUMAN / AVOID_STONE` 的偏置、ramp、前瞻点 `lookahead_y=300` 和 road mask 走廊评分仍需实车调。`AVOID_HUMAN` 使用行人横向速度预测短时占用区，再在 road mask 内选择左右走廊；若计划绕行侧过激，会触发 `yield_wait` 停等并由 planner 临时覆盖速度为 `0.0`。若人运动方向误判或绕行侧不合理，优先调 `HUMAN_MOTION_*`、`HUMAN_MOTION_PREDICT_SECONDS`、`HUMAN_YIELD_*` 和 `ROAD_SIDE_*`。若 final/CMD err 已经足够大但转不过来，再看 TC264D 舵机线性 P 映射、硬限幅和机械舵机。
5. 红绿灯识别：`TrafficLight` 先由目标检测给框，再由 `vision_traffic_light.py` 在框内用 HSV 判断红/绿。曝光、灯光颜色、AR 画面压缩和检测框偏移都会影响判断。当前远处红灯只记录为 `red_far`，进入 `TRAFFIC_LIGHT_STOP_MIN_BOTTOM_RATIO` 定义的近处停车区后还要连续确认，确认完成才进入 `TRAFFIC_LIGHT_STOP`；有效绿灯会立即清除红灯保持并恢复通行。若红灯不停或绿灯误停，先看 HUD/日志里的 `traffic_light_state / traffic_light_stop_zone / traffic_light_red_confirm_age` 和置信度，再调 `vision_traffic_light.py` 顶部的 HSV、面积阈值以及 `control_race_state_machine.py` 顶部的停车区和确认阈值。
6. 计圈与终点：`Door + BeginSign` 用于起跑计圈，`Door` 用于过圈；当前 `ENDSIGN_STOP_ENABLED=False`，`EndSign` 只观测不停车，避免 `BeginSign` 被误识别为 `EndSign` 后卡死。后续模型稳定后再打开 `control_race_state_machine.py` 顶部的 `ENDSIGN_STOP_ENABLED`，并重新调 `ENDSIGN_MIN_SCORE / ENDSIGN_CONFIRM_SECONDS / ENDSIGN_LOST_STOP_DELAY`。
7. 丢线恢复：短时丢线会进入 `RECOVER_LINE`，连续约 `0.8 s` 无有效中线才进入 `LINE_LOSS_SAFE_STOP`。如果入弯时经常丢线后恢复失败，问题可能是分割视野、车速、舵机响应或恢复策略共同导致，不应只看模型。恢复计时在 `control_task_state_machine.py`，恢复误差衰减在 `control_local_planner.py`。
8. 下位机控制响应：上位机输出的是图像像素误差，TC264D 负责舵机线性 P、速度闭环、PWM 硬限幅和安全保护。若 HUD 中 `final_track_error / CMD err / TC264D input_track_error` 已经一致且数值足够大但车仍转不过来，优先检查 `Slave_TC264D/code/Control.c` 中 `SERVO_FULL_STEER_ERROR_PX`、`Servo.h` 硬限幅和机械舵机角度；若直道摆动，则优先减小小误差增益或重新讨论是否加入少量阻尼。
9. 调试显示与主循环负载：`8090/debug_feed` 会发布完整 HUD 画面，debug 渲染和 MJPEG 编码慢时只丢 debug 帧，不应阻塞控车主循环。若 FPS 明显下降，先看 HUD 中 `FPS ctrl/raw/loop` 判断实际控车帧率，再看 `DBG r/enc/pub/drop` 和 `MS ai/cmd/ctrl/dbg/jpg` 判断是视觉推理、控制主循环、debug 绘制还是 JPEG/网络链路瓶颈。
10. 状态显示可观测性：当前 HUD 已能显示主要状态和误差，但比赛事件的细节、状态机 `reason`、检测目标摘要还不够完整。若实测时出现“状态切换原因不清楚”，后续应优先把 `task_decision`、`race_state` 和 `plan_result` 更完整地写入 `/pose_status` 和 HUD。

主要调参入口：

- 巡线/避障速度、风险池 `RISK_*`、金币奖励池 `GOLD_*`、丢线计时：`Master_RK3588S/setupUI/control_task_state_machine.py`
- 避障路径、行人运动预测、stone 分支、目标线几何和前瞻点：`Master_RK3588S/setupUI/control_local_planner.py`
- 红绿灯 HSV 与颜色置信度：`Master_RK3588S/setupUI/vision_traffic_light.py`
- Door/BeginSign/EndSign/红灯停车的比赛事件时序：`Master_RK3588S/setupUI/control_race_state_machine.py`
- 舵机 PID、速度 PID、前馈、限幅和安全停车：`Slave_TC264D/code/Control.c`

## 定位与端口

官方定位包：

```json
{"type":"robot_position","pos":[x,0.16,z],"euler":[0.0,yaw,0.0]}
```

- Windows 定位程序发送到板卡局域网 IP 的 UDP `9005`。
- `ar_receiver.py` 转发到板卡本机 `127.0.0.1:9006`；转发给官方 AR 时只交换平移轴为 `pos=[z,0.16,x]`，`yaw=euler[1]` 保持不变。
- 定位 UDP 转发采用低延迟热路径：收到有效定位后先转发到 AR，再降频写 `xverse_control_live.json` 和 `ar_pose_status.json`。默认 `AR_POSE_RX_DRAIN_LATEST=1` 会丢弃 UDP 队列里的旧定位包，只保留最新包，避免 AR 继续消费过期位置。
- 默认关闭逐包定位打印和日志写入；需要临时排查时再打开 `AR_DEBUG_PRINT_POSE=1`、`AR_POSE_PATH_DEBUG=1` 或 `AR_POSE_LOG_TO_FILE=1`。HUD 右侧 `POSE_IO rx/drop/fwd/h` 分别表示定位接收计数、主动丢弃的旧定位包数、UDP 转发耗时和定位线程处理耗时。
- WebUI 定位数据地址填写 `127.0.0.1`，端口填写 `9006`。
- 手柄遥控输入为独立 UDP `9010`，只接收 `type=gamepad_control`，不复用定位包。
- UNITY 同步端口为 `9003`。
- 控车串口固定为 `/dev/ttyUSB0`，`460800` baud；若启动时串口不存在或读写异常，RK3588S 会保持约 `1 s` 间隔自动重连。

场地为 `4 m × 3 m`，Windows 定位预览右下角基准点为 `(x=0, z=0.30 m)`，因此底边 `z=0.30 m`、上边 `z=3.30 m`；`+X` 向左、`+Z` 向上。Yaw 使用 `euler[1]`，`0° -> +X`、`+90° -> +Z`。固定标定 Tag 1～4 均为 `20 cm × 20 cm`，Windows 定位端会对输出坐标和 Yaw 做轻量滤波。由于官方 AR 场景资产的 X/Z 平移轴与定位预览互换，RK3588S 仅在转发到 `9006` 时交换 `pos[0]` 和 `pos[2]`，不改变 Yaw。这些坐标只用于 AR 定位与融合，不用于直接控车。

## 待解决与待确认

### 当前巡线

1. 使用真实融合视频重新训练或优化分割模型，使 `track_error` 足以稳定巡线。
2. 架空车轮验证误差方向、舵机方向、PID、速度、`flags=0x01` 和反馈帧。
3. 首次落地前使用状态机默认 `0.05 m/s` 验证巡线、避障、金币、stone 分岔、红灯停车、终点停车和 `RECOVER_LINE`；需要更慢或更快时直接修改 `control_task_state_machine.py` 开头的速度表，重启 `ar_receiver.py` 后生效。
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
3. 模型稳定后再逐项实测 `AVOID_HUMAN / AVOID_CAR / COLLECT_GOLD / AVOID_STONE / TRAFFIC_LIGHT_STOP / ENDSIGN_STOP`，最后接入 OCR/API 和更复杂任务状态。

详细使用和协议见各子工程 README。
