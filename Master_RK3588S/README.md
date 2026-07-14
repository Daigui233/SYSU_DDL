# Master_RK3588S

RK3588S 上位机当前基于官方 X-Verse AR Engine `1.0.7`。本阶段只迁移与旧视觉模型无关的基础 I/O，不恢复 1.06 的分割、检测、状态机和路径后处理。

## 运行链路

```text
Windows robot_position -> UDP :9005 -> pose_ar_bridge.py
                       -> UDP 127.0.0.1:9006 -> official AR engine

Windows gamepad_control -> UDP :9010 -> control_gamepad_receiver.py
                         -> control_serial_comm.py -> TC264D

official AR engine -> shm_ar_video -> vision_frame_source.py
                                      -> future single-RKNN pipeline
```

定位数据只驱动官方 AR 融合，不进入控车决策。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `setupUI/ar_receiver.py` | 官方 1.07 示例入口，当前保持原样 |
| `setupUI/pose_ar_bridge.py` | 验证并透明转发已经在 Windows 端完成轴映射的位姿 JSON |
| `setupUI/control_gamepad_receiver.py` | 接收手柄控制包，处理启用、停车、TTL 和手柄范围保护 |
| `setupUI/control_serial_comm.py` | TC264D 串口帧收发、自动重连和 v1/v2/v3 反馈解析 |
| `setupUI/control_car_link.py` | 串口控制轻量封装 |
| `setupUI/control_runtime.py` | 独立线程完成手柄优先、视觉回退和唯一串口下发 |
| `setupUI/standalone_control_bridge.py` | 不启动 `ar_receiver.py` 时使用的备用 I/O 入口 |
| `setupUI/vision_frame_source.py` | 校验共享内存头部，读取一致的 AR 视频帧 |
| `setupUI/dist/main_config.json` | 官方 1.07 配置；`network.control_port` 固定为 `9006` |

## 启动

```bash
cd ~/Desktop/setupUI
python3 ar_receiver.py
```

默认端口和设备：

- Windows 定位输入：`0.0.0.0:9005`
- 官方 AR 转发：`127.0.0.1:9006`
- 手柄输入：`0.0.0.0:9010`
- TC264D：`/dev/ttyUSB0`, `460800` baud

`ar_receiver.py` 会自动启动 `ControlRuntime`。手柄有效时使用 `GAMEPAD` 命令；手柄无效时自动选择最新的 `VISION` 命令。视觉控制默认下发到 TC264D；若只需要调试画面，可设置 `AR_VISION_CONTROL_SEND=0`。手柄从有效状态退出、视觉命令超时或视觉清空后会发送 `STATE_SAFE_STOP`。TC264D 自身的输入超时和硬保护继续有效。

`standalone_control_bridge.py` 只在不运行 `ar_receiver.py` 时使用，二者不能同时启动。后续多任务视觉控制通过 `ControlRuntime.update_vision_command()` 接入，不再创建第二条串口链路。

## 语义 mask 骨架路径试验

骨架路径源使用 `0.35/0.27` 空间双阈值生成 Mask：弱热值只有和当前帧强响应连通时才保留，不使用历史 Mask。相距不超过 6 个热力图像素的离散片段用 3 像素宽的最短桥接线连接，随后经过 9×9 边缘平滑、小连通域过滤，并且只填充不超过 32 像素的小孔洞。Zhang-Suen 骨架从最大 Y 端开始建立根系拓扑图，结合路径长度、热值、距边缘距离和历史路径选择最多两个有效叶分支；分叉前的根主干由两条路线共同保留，长度不足 10 个骨架像素的短毛刺会被剪除。蓝、绿线按弧长重采样后做二维时间滤波，结果仍须通过当前 Mask 热值支撑检查。AR Preview 同时以红色显示处理后的 Mask。

AR 运行时会将模型解码路径设为 `curve`，只为跳过随后会被骨架结果覆盖的旧 Python 热力图脊线追踪；原始热力图仍会完整交给骨架控制器。预览使用 `drive` 模式，不再先绘制一层会被红色 Mask 覆盖的原始热力图。处理后的红色 Mask 始终完整显示并保留浮点插值边缘，蓝绿线使用整条折线绘制。左右分支的时序身份只比较共享主干之后的独立尾部，短暂乱序时不会交换蓝、绿色身份。

岔路控制固定以左支作为直行和默认路线；无 OCR、OCR 未完成或 OCR 返回 `left` 都保持左支，只有当前确认的 OCR `right` 才切换右支。右转 OCR 锁到期后立即恢复默认左支，不会自动回退到另一条可见路线。

设置 `VISION_CONTROL_FAST_RENDER=0` 可恢复逐段概率渐变路径。显式设置 `MULTITASK_PATH_SOURCE=heatmap` 或 `MULTITASK_RENDER_MODE=heatmap` 仍可恢复旧调试行为。

首次配置或 OpenCV 依赖变更后执行：

```bash
cd ~/Desktop/setupUI
./configure_skeleton_opencv.sh
```

试跑时加载配置再启动：

```bash
cd ~/Desktop/setupUI
. ./vision_skeleton_env.sh
python3 ar_receiver.py
```

查看仅经过阈值和一次 3x3 闭运算的初始赛道分割 Mask：

```bash
cd ~/Desktop/setupUI
python3 ar_receiver.py
```

当前实验分支的 `ar_receiver.py` 默认直接进入 `road_mask` 模式，并关闭
视觉控制下发和车辆运行时；TurnSign OCR 仍在后台正常运行。需要恢复实验前入口时设置
`AR_ROAD_MASK_EXPERIMENT=0`；`vision_road_mask_env.sh` 仍可用于显式加载
全部 Mask 参数。

恢复原热力图追踪器：

```bash
export VISION_CONTROL_PATH_SOURCE=heatmap
```

骨架模式的主要参数是 `VISION_CONTROL_SKELETON_THRESHOLD`、`VISION_CONTROL_SKELETON_LOW_THRESHOLD`、`VISION_CONTROL_SKELETON_MIN_AREA`、`VISION_CONTROL_SKELETON_MIN_LENGTH`、`VISION_CONTROL_SKELETON_CLOSE_ITERATIONS`、`VISION_CONTROL_SKELETON_EDGE_KERNEL`、`VISION_CONTROL_SKELETON_MAX_HOLE_AREA`、`VISION_CONTROL_SKELETON_MAX_CONNECT_GAP`、`VISION_CONTROL_SKELETON_BRIDGE_THICKNESS`、`VISION_CONTROL_SKELETON_MIN_BRANCH_LENGTH` 和 `VISION_CONTROL_SKELETON_MAX_BRANCHES`。如果运行环境缺少 `cv2.ximgproc.thinning`，代码会回退到原逐行加权中心算法。

## 不在本阶段实现

- 除上述试验性骨架路径外，旧版完整分割中线和 fork 后处理。
- 旧目标检测后处理和多模型并行。
- 旧任务/比赛状态机和风险阈值。
- 旧局部规划、误差生成和控制仲裁。
- 旧 HUD、Web debug stream、性能 CSV、OCR/API。

这些模块将在新的单 RKNN 多任务模型输出契约稳定后重新轻量实现。
