# SYSU_DDL

中山大学智能车 AI 模型组工程。当前开发分支为 `v1.07-multitask`，以官方 X-Verse AR Engine `1.0.7` 为干净基线，正在迁移到单个共享骨干多任务 RKNN 模型。

## 当前边界

```text
Windows AprilTag 定位
  -> Windows 发送端映射为官方 AR 坐标
  -> UDP RK3588S:9005
  -> pose_ar_bridge.py 原样转发
  -> UDP 127.0.0.1:9006
  -> X-Verse 1.0.7 AR 融合
  -> shm_ar_video
  -> 官方 1.07 ar_receiver.py / 后续多任务模型

Windows Gamepad Mode
  -> UDP RK3588S:9010
  -> control_gamepad_receiver.py
  -> control_serial_comm.py
  -> /dev/ttyUSB0 @ 460800
  -> TC264D
```

AprilTag 定位只用于 AR 融合、显示和记录，不参与 `track_error`、任务决策或路径规划。

## 已迁移

以下模块与旧视觉模型无关，已从 1.06 收敛后迁移：

| 文件 | 作用 |
| --- | --- |
| `ArucoCalib-master/ArucoCalib-master/` | Windows AprilTag 定位、位姿滤波、官方 AR JSON 发送和可选手柄发送 |
| `Master_RK3588S/setupUI/pose_ar_bridge.py` | 校验 Windows 位姿包并原样转发到本机官方 AR 端口 |
| `Master_RK3588S/setupUI/control_gamepad_receiver.py` | 接收 `9010` 手柄包，维护手动控制 TTL |
| `Master_RK3588S/setupUI/control_serial_comm.py` | 保留既有 TC264D 串口协议、自动重连和反馈解析 |
| `Master_RK3588S/setupUI/control_car_link.py` | 串口控制的轻量线程安全封装 |
| `Master_RK3588S/setupUI/control_runtime.py` | `GAMEPAD > VISION > IDLE` 控制源选择、定位桥生命周期和唯一串口所有权 |
| `Master_RK3588S/setupUI/standalone_control_bridge.py` | 不启动 `ar_receiver.py` 时使用的备用 I/O 入口 |
| `Master_RK3588S/setupUI/vision_frame_source.py` | 从 `shm_ar_video` 读取同一 `frame_id` 的一致帧快照 |
| `Slave_TC264D/` | 保留电机、舵机、PID、硬限幅、安全保护和串口协议 |

官方 1.07 的 `dist/app`、`dist/setup_webui`、场景资源、配置结构、模板、模型包装和 `ar_receiver.py` 均保持为 1.07 基线，不使用 1.06 文件覆盖。

## 坐标契约

Windows 定位内部和预览继续使用场地坐标 `(x, z)`。仅在 Windows 网络发送边界执行一次映射：

```text
Windows internal: (x, z, yaw)
UDP robot_position.pos: [z, height, x]
UDP robot_position.euler: [0, yaw, 0]
RK pose bridge: no axis mapping
```

1.06 与 1.07 官方引擎的 `UDPControlThread` 已核对为相同实现：都读取 `pos[0:3]`、叠加 `pos_offset`，随后在引擎内部对第三轴取负；1.07 没有新增 X/Z 交换。因此 X/Z 映射仍需保留，但现在只存在于 Windows `aruco_core/udp_sender.py`。

端口约定：

| 用途 | 地址 |
| --- | --- |
| Windows 定位输入 | `RK3588S-IP:9005` |
| 官方 AR 本机输入 | `127.0.0.1:9006` |
| Windows 手柄输入 | `RK3588S-IP:9010` |
| 官方 AR 视频流 | `http://RK3588S-IP:8080/video_feed` |
| TC264D 串口 | `/dev/ttyUSB0`, `460800` baud |

## 当前运行

先启动官方 1.07 Setup WebUI/AR 引擎，再在 RK3588S 运行：

```bash
cd ~/Desktop/setupUI
python3 ar_receiver.py
```

`ar_receiver.py` 会自动启动定位、手柄和 TC264D 控制运行时。勾选 Windows `Gamepad Mode` 后手柄优先控车；未勾选时运行时选择视觉命令。当前 1.07 基线尚未产生视觉控制命令，因此未勾选手柄时不主动控车；手柄退出或超时会安全停车。

只需要定位和手柄、不运行视觉入口时，才单独运行 `python3 standalone_control_bridge.py`。它与 `ar_receiver.py` 不能同时运行，否则会争用 `9005/9010` 和 TC264D 串口。

Python 串口依赖：

```bash
python3 -m pip install pyserial
```

Windows 定位 EXE 已根据当前源码重新打包，包含发送端 X/Z 映射。

## 刻意未迁移

以下 1.06 内容依赖旧检测、分割或 fork 模型，本分支不保留：

- `vision_pipeline.py`、`seg_func.py`、`seg_infer.py` 和旧多 RKNN 调度。
- `control_task_state_machine.py`、`control_race_state_machine.py` 和旧检测阈值。
- `control_local_planner.py`、旧岔路 mask 切分和中线扫描后处理。
- `control_arbitrator.py`、旧视觉命令合成逻辑和旧状态映射文件。
- 旧 HUD、MJPEG 调试流、性能 CSV、裁判事件、OCR/API 和运行状态服务。
- 旧模型 `pp_seg.rknn`、`fork_mask_cls_fp16.rknn` 及其测试。
- 采集图片、比赛记录、日志和其他运行产物。

这些功能不会从 1.06 原样恢复。新 Paddle 多任务模型训练并转换为单个 RKNN 后，再围绕新输出契约实现轻量后处理、状态机和局部规划。

## 后续开发

当前训练模型以 [新的模型架构/README.md](../新的模型架构/README.md) 和 [AI Studio训练指南](../新的模型架构/AI_STUDIO_上传训练指南.md) 为准。[MULTITASK_RKNN_MIGRATION_PLAN.md](MULTITASK_RKNN_MIGRATION_PLAN.md) 仅保留为早期历史设计记录。下一阶段顺序：

1. 固定检测框、road mask、双候选热力图和有序曲线路径的训练数据契约。
2. 完成 Paddle 共享骨干多任务模型并验证 Paddle/ONNX/RKNN 数值一致性。
3. 接入 latest-only `采图 -> 单 RKNN 推理 -> 控制` 流水线。
4. 直接读取模型输出的 `path_points/path_count_scores`，重新实现风险决策和局部规划，不再扫描热力图连接中线。
5. 最后恢复必要的 HUD、Web 调试和 OCR/API。
