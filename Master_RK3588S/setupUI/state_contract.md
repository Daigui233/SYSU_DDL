# RK 任务状态与 TC264D 状态契约

## 第一阶段约定

Windows AprilTag 定位只用于 AR 融合、HUD、WebUI 和日志记录，不能进入
`track_error`、局部规划或 TC264D 控制状态。

RK3588S 负责结构化感知、任务状态选择和图像坐标局部规划。TC264D 接收
对应的 `state_cmd` 作为状态执行和反馈字段，但最终 `track_error` 和
`target_speed` 仍由 RK3588S 给出。

| RK `task_state` | RK 行为 | TC264D `state_cmd` |
| --- | --- | --- |
| `NORMAL_TRACK` | 使用语义分割中线 `track_error`。 | `STATE_TRACK = 1` |
| `AVOID_CAR` | 在 road mask 内生成连续绕行目标线，并从规划线前瞻点计算 `final_track_error`。 | `STATE_AVOID_CAR = 2` |
| `AVOID_HUMAN` | 远离人，生成连续绕行目标线，并在人短暂丢失后保持 TTL。 | `STATE_AVOID_HUMAN = 3` |
| `COLLECT_GOLD` | 当金币较近且代价不大时，让目标线偏向金币。 | `STATE_COLLECT_GOLD = 4` |
| `RECOVER_LINE` | 短时保持/衰减上一帧有效局部规划误差。 | `STATE_RECOVER_LINE = 5` |
| `LINE_LOSS_SAFE_STOP` | 连续丢失中线超过超时时间后停车。 | `STATE_LINE_LOSS_SAFE_STOP = 6` |
| `AVOID_STONE` | 在分岔区域检测到 `stone` 且命中默认外圈候选路径时，选择内圈候选路径。 | `STATE_AVOID_STONE = 8` |
| `TRAFFIC_LIGHT_STOP` | `TrafficLight` 框内识别为红灯且进入有效触发区后停车；绿灯不进入该状态。 | `STATE_TRAFFIC_LIGHT_STOP = 9` |
| `ENDSIGN_STOP` | 看到 `EndSign` 后继续循迹，直到 `EndSign` 消失超过短 TTL 后停车。 | `STATE_ENDSIGN_STOP = 10` |

`STATE_IDLE = 0` 用于 TC264D 本地启动/空闲。`STATE_SAFE_STOP = 7` 是通用
硬停状态，用于手动急停、程序退出清零和 TC264D 本地串口输入超时。

## TC264D 职责

TC264D 继续负责低层电机和舵机控制：

- 固定 PID/PD 参数
- 输出限幅
- 电机启动前馈和本地速度环
- 舵机 PD 环
- 串口帧输入超时保护
- 最终安全停车执行

第一阶段中，`STATE_TRACK`、`STATE_AVOID_CAR`、`STATE_AVOID_HUMAN`、
`STATE_AVOID_STONE`、`STATE_COLLECT_GOLD` 和 `STATE_RECOVER_LINE` 都复用当前效果较稳定的
TC264D TRACK 电机/舵机控制参数。`STATE_TRAFFIC_LIGHT_STOP`、`STATE_ENDSIGN_STOP`、
`STATE_LINE_LOSS_SAFE_STOP` 和 `STATE_SAFE_STOP` 都按停车状态执行。早期制作的其他任务枚举不再属于当前有效状态契约。

## 接口形状

`vision_pipeline.py` 输出：

- `segmentation`：`line_valid`、`track_error`、`road_mask`、`mid_points` 和质量字段。
- `detections`：归一化检测对象，包含 `category`、`label`、`score`、`bbox`、`center`、`size`、`area_ratio`；`TrafficLight` 额外包含 `traffic_light_state` 和颜色置信度。

`control_race_state_machine.py` 输入结构化检测结果，输出：

- `race_started`、`current_lap`、`completed_laps`
- `traffic_light_state`、`traffic_light_stop_zone`、`traffic_light_stop`
- `finish_armed`、`finish_stop`

`control_task_state_machine.py` 输入结构化感知，输出：

- `task_state`
- `desired_speed`
- `planner_intent`
- 丢线计时等元数据

`control_local_planner.py` 输入结构化感知和状态机决策，基于红色分割中线生成连续紫色目标路径，输出：

- `final_track_error`
- 局部规划调试元数据

`control_arbitrator.py` 将 RK 最终命令转换成不变的串口协议字段：
`track_error`、`target_speed`、`state_cmd` 和 `flags`。

## 参数归属

`ar_receiver.py` 只作为运行调度入口：启动模块、读取帧、转发模块输出、发送最终命令并更新显示。

模块默认值、阈值、TTL、增益、速度表等调车参数归拥有对应行为的模块维护。
核心调车值保留为源码顶部可见表格/常量，不使用环境变量覆盖，避免代码默认值和实际运行值不一致。

- `vision_pipeline.py`：模型路径、分割/检测结果 TTL、检测类别标签。
- `vision_traffic_light.py`：红绿灯框内 HSV 阈值、有效颜色面积比例。
- `control_race_state_machine.py`：Door/BeginSign/EndSign/TrafficLight 触发区、TTL、圈数和停车判定。
- `control_task_state_machine.py`：任务状态速度表、丢线超时、状态保持 TTL。
- `control_local_planner.py`：图像坐标偏置、目标吸引增益、局部规划限幅。
- `control_arbitrator.py`：最终控制缩放、命令重复间隔、控制 flags。
- `control_car_link.py` / `control_serial_comm.py`：串口、波特率、TC264D 协议常量。

## 状态数量

RK Python 侧任务状态没有实际数量限制，但串口协议中的 `state_cmd` 是一个字节。
因此，只要 `State.h` 和 Python 映射保持一致，下位机最多支持 `0..255` 共 256
个数值状态码。

为了可读性和调试安全，第一阶段应远低于该上限。当前有效契约使用 11 个状态码：
`0` 空闲、`1..6` 常规 RK 任务状态、`7` 通用安全停车、`8` stone 分岔状态、`9` 红灯停车、`10` 终点停车。新增状态前需要先定义清楚进入、
保持、退出和失效策略。

## 文件分工

`control_states.py` 是状态契约文件：定义有哪些状态、状态码是多少、RK 任务状态如何映射到 TC264D 状态。

`control_task_state_machine.py` 是状态决策文件：每一帧读取结构化感知结果，按优先级、TTL 和丢线计时决定当前任务状态、目标速度和局部规划意图。

`control_race_state_machine.py` 是比赛事件状态机：每一帧读取结构化检测结果，按图像触发区和 TTL 维护红绿灯、Door、BeginSign、EndSign 和圈数事件。
