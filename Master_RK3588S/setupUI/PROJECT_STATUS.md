# RK3588S 上位机当前状态

## 正式链路

```text
Windows 顶置相机
  -> AprilTag 定位 robot_position
  -> UDP 板卡IP:9005
  -> ar_receiver.py
  -> 127.0.0.1:9006
  -> 官方 AR 引擎
  -> 融合视频 shm_ar_video
  -> 分割模型生成 track_error
  -> STATE_TRACK + target_speed + flags
  -> /dev/ttyUSB0
  -> TC264D
```

AprilTag 定位只服务官方 AR 融合、坐标显示和记录，不参与 `track_error`、`state_cmd` 或转向控制。

## 当前控制

- 正常视觉控制只启用视觉巡线和 `STATE_TRACK`。
- 有有效分割误差时，下发 `STATE_TRACK + TRACK_SPEED + flags=0x01`，默认速度 `80`。
- 无有效分割误差时，保持 `TRACK_FALLBACK`：误差 `0`、默认速度 `80`，不会因为模型效果差而停车。
- 控制主循环或共享内存连续 `2 s` 不再产生控制命令时，独立看门狗每 `0.2 s` 重复下发 `STATE_SAFE_STOP`。
- 看门狗阈值可通过 `AR_CONTROL_WATCHDOG_TIMEOUT` 调整；停车重复间隔可通过 `AR_SAFE_STOP_REPEAT_INTERVAL` 调整。
- TC264D 本地连续 `2.5 s` 未收到有效控制帧时，自行进入 `STATE_SAFE_STOP`。
- 检测模型继续运行，但只画 `gold / car / human` 检测框，不参与状态切换。
- OCR/API 和多状态任务决策尚未接入；`state_cmd / target_speed / track_error / flags` 接口为后续扩展保留。
- 当前分割和检测模型效果较差，仍需继续训练、调参与实车验证。

## 端口与设备

| 用途 | 当前配置 |
| --- | --- |
| Windows 定位入口 | `0.0.0.0:9005` |
| 官方 AR 转发 | `127.0.0.1:9006` |
| WebUI 定位数据 | `127.0.0.1:9006` |
| 定位状态 HTTP | `0.0.0.0:9105` |
| 融合视频共享内存 | `shm_ar_video` |
| TC264D 串口 | `/dev/ttyUSB0`，`460800` baud，写超时 `0.05 s` |

当前定位约定以 Windows AprilTag 程序为准：场地 `4 m × 3 m`，定位预览原点在右下角，`+X` 向左、`+Z` 向上；Yaw 使用 `euler[1]`，`0° -> +X`、`+90° -> +Z`。Windows 端已对输出位姿做轻量滤波。官方 AR 场景资产的 X/Z 平移轴与 Windows 定位预览互换，因此 RK3588S 仅在转发到 `127.0.0.1:9006` 时交换 `pos[0]` 和 `pos[2]`，Yaw 保持不变；定位仍不转换为控车误差。

## 重点文件

| 文件 | 作用 |
| --- | --- |
| `ar_receiver.py` | 定位接收/转发、融合视频读取、双模型推理、视觉巡线控制和 HUD |
| `serial_comm.py` | TC264D 控制帧和反馈帧串口通信 |
| `infer_wrap/base/seg_func.py` | 分割后处理、中线提取和 `track_error` 计算 |
| `infer_wrap/base/func.py` | 检测模型后处理与画框 |
| `dist/main_config.json` | 官方 AR/WebUI 网络配置 |

## 已知风险

- `TRACK_SPEED` 与 `TRACK_FALLBACK` 默认速度已临时降为 `80`；首次落地前仍应架空车轮确认方向和制动行为。
- TC264D 电机 PWM 硬限幅已降为 `±2500` duty，避免编码器异常或 PID 过冲时满占空比空转。
- 视觉模型失效时会继续按 `TRACK_FALLBACK` 直行，这是当前为了观察低质量模型完整跑圈而保留的行为。
- TC264D 本地串口输入超时已补充，但硬件断电、舵机/电机驱动异常和机械风险仍需现场急停手段。
- Windows 定位程序必须把目标 IP 配成 RK3588S 板卡局域网 IP；`127.0.0.1` 只适用于同机测试。
- 当前模型效果不足以直接假定稳定巡线，应先架空轮胎和低速验证误差方向、舵机方向、PID 与反馈。

## 运行

```bash
python3 ar_receiver.py
```
