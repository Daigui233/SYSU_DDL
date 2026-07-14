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

骨架路径源先以 `0.35` 阈值将中心线 heatmap 二值化为 Mask。相距不超过 8 个热力图像素的离散片段用 3 像素宽的最短桥接线连接，再经过 9×9 边缘平滑、小连通域过滤和 Zhang-Suen thinning。骨架像素从最大 Y 点开始按最近邻顺序追踪，遇到较大空间断点时停止。左路径显示为蓝线，右路径显示为绿线；AR Preview 同时以红色显示处理后的 Mask。

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

恢复原热力图追踪器：

```bash
export VISION_CONTROL_PATH_SOURCE=heatmap
```

骨架模式的主要参数是 `VISION_CONTROL_SKELETON_THRESHOLD`、`VISION_CONTROL_SKELETON_MIN_AREA`、`VISION_CONTROL_SKELETON_MIN_LENGTH`、`VISION_CONTROL_SKELETON_CLOSE_ITERATIONS`、`VISION_CONTROL_SKELETON_EDGE_KERNEL`、`VISION_CONTROL_SKELETON_MAX_CONNECT_GAP` 和 `VISION_CONTROL_SKELETON_BRIDGE_THICKNESS`。如果运行环境缺少 `cv2.ximgproc.thinning`，代码会回退到原逐行加权中心算法。

## 不在本阶段实现

- 除上述试验性骨架路径外，旧版完整分割中线和 fork 后处理。
- 旧目标检测后处理和多模型并行。
- 旧任务/比赛状态机和风险阈值。
- 旧局部规划、误差生成和控制仲裁。
- 旧 HUD、Web debug stream、性能 CSV、OCR/API。

这些模块将在新的单 RKNN 多任务模型输出契约稳定后重新轻量实现。
