# Master_RK3588S

RK3588S 上位机负责转发定位给官方 AR、读取融合视频、运行视觉模型、生成控制命令，并与 TC264D 串口通信。

## 正确链路

```text
Windows AprilTag 定位 -> UDP 板卡IP:9005 -> ar_receiver.py -> 127.0.0.1:9006 -> 官方 AR
官方 AR 融合视频 -> shm_ar_video -> 分割/检测模型 -> 上位机决策 -> /dev/ttyUSB0 -> TC264D
```

定位只驱动 AR 融合和坐标显示，不直接参与转向控制。控车输入来自融合视频上的视觉处理结果。

## 重点文件

| 文件 | 作用 |
| --- | --- |
| `setupUI/ar_receiver.py` | 定位转发、融合视频读取、模型推理、控制命令和 HUD |
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
- 无有效分割误差时，当前进入 `TRACK_FALLBACK`，误差 `0`、默认速度 `120`，不会自动停车。
- 代码中仍残留定位追点 `PosePathPlanner/path_waypoints` 旁路，但它不属于正式架构，后续应清理。

## 后续扩展接口

串口控制帧已包含：

- `track_error`：路线或任务产生的转向误差。
- `target_speed`：当前状态目标速度。
- `state_cmd`：上位机任务状态。
- `flags`：控制选项。

后续模型和 OCR/API 可通过独立任务决策层生成这些字段。新增状态时应分别定义进入、保持、退出和失效规则，不修改定位链路，也尽量保持串口协议兼容。

## 定位与端口

```json
{"type":"robot_position","pos":[x,0.16,z],"euler":[0.0,yaw,0.0]}
```

- 外部定位入口：`0.0.0.0:9005`。
- 官方 AR 转发：`127.0.0.1:9006`。
- WebUI 定位数据地址：`127.0.0.1`。
- WebUI 端口号：`9006`。
- UNITY 同步端口：`9003`。

## 调试重点

WebUI/HUD 优先确认：

- `WIN-UDP ok` 和 `AR-FWD ok` 持续增长，AR 画面随定位变化。
- 分割结果和 `track_error` 是否稳定。
- HUD 控制来源应为 `VISION`；TC264D 反馈状态应为 `TRACK`。
- TC264D feedback 持续增长。
- `flags=0x01`，电机和舵机输出随误差变化。

状态接口：

- `http://<RK3588S-IP>:9105/pose_status`
- `http://<RK3588S-IP>:9105/pose_packet`