# Master_RK3588S

RK3588S 上位机工程。当前核心任务是把 Windows 顶置相机定位、官方 AR 融合预览、路径/巡线控制和 TC264D 串口控车串起来，先完成低速跑一圈。

## 当前链路

```text
Windows AprilTag 定位 -> 板卡局域网 IP:9005 -> setupUI/ar_receiver.py
                    -> 最新定位/路径控制 -> setupUI/serial_comm.py -> /dev/ttyUSB0 -> TC264D
                    -> 127.0.0.1:9006 -> 官方 AR 引擎/WebUI 视频融合

TC264D -> 串口反馈 -> serial_comm.py -> ar_receiver.py HUD/状态接口
```

注意：

- Windows 到板卡 UDP `9005` 是定位输入链路。
- `ar_receiver.py` 收到定位后在内存中保留最新包，并转发到本机官方 AR 引擎 UDP `9006`。
- `/dev/ttyUSB0` 是 RK3588S 到 TC264D 的控车链路。
- TC264D 不关心定位来源，只执行上位机最终下发的控制帧。

## 重点文件

| 文件 | 作用 |
| --- | --- |
| `setupUI/ar_receiver.py` | AR 预览主入口，接收 Windows UDP 定位，转发 AR UDP，生成控车命令 |
| `setupUI/serial_comm.py` | RK3588S 与 TC264D 的串口收发 |
| `setupUI/dist/main_config.json` | 官方 WebUI/AR 配置 |
| `setupUI/path_waypoints.json` | 路径点文件，存在时优先走定位路径控制 |
| `setupUI/ar_pose_debug.log` | Windows UDP 输入、解析和 AR 转发分段调试日志 |
| `setupUI/ar_pose_status.json` | 当前定位、控制、串口反馈状态 |
| `setupUI/xverse_control_live.json` | 最近一次官方 `robot_position` 包 |

## 定位与 AR

Windows 定位程序向板卡局域网 IP 的 UDP `9005` 发送官方格式：

```json
{
  "type": "robot_position",
  "pos": [0.0, 0.16, 0.0],
  "euler": [0.0, 0.0, 0.0]
}
```

当前约定：

- `pos[0]`：横向 X。
- `pos[1]`：高度，默认 `0.16`。
- `pos[2]`：前后/赛道平面 Z。
- `euler[1]`：Yaw，单位为度。

默认链路：

- 外部定位入口：`0.0.0.0:9005`。
- 官方 AR 转发目标：`127.0.0.1:9006`。
- 官方 AR 端口读取 `dist/main_config.json -> network.control_port`。
- 如需临时恢复旧 BLE 输入，设置 `AR_POSE_INPUT_MODE=ble`。

修改端口配置后需要重启官方 AR/WebUI 预览进程。联调时先启动 `ar_receiver.py`，再启动 Windows 定位发送。

## WebUI 配置建议

| 项 | 值 |
| --- | --- |
| 定位数据地址 | `127.0.0.1` |
| 端口号 | `9006` |
| UNITY 同步目标 IP | 本机调试可填 `127.0.0.1`，外部 Unity 按实际 IP |
| UNITY 同步端口 | `9003` |
| 相机位移 X/Y/Z | `0 / 0.16 / 0.12` |
| 相机旋转 X/Y/Z | `0 / 0 / 0` |

WebUI 定位配置填写 `127.0.0.1` 和 `9006`。Windows 定位程序仍发送到板卡 IP 的 `9005`，由 `ar_receiver.py` 接收并转发到官方 AR 引擎；不要把端口拼到“定位数据地址”里。

## 控车状态

正常跑车应看到：

- 控制状态：`POSE` 或 `VISION`
- 下位机状态：`STATE_TRACK`
- `flags = 0x01`
- `target_speed > 0`

只有在定位路径和视觉误差都不可用时，上位机会兜底发：

- `STATE_SAFE_STOP`
- `target_speed = 0`
- `flags = 0`

当前路径点逻辑是循环路径，不会自动因为跑完一圈停车。

当前仓库暂未包含 `setupUI/path_waypoints.json`。不影响定位和 AR 转发，但定位路径控制会保持 `path disabled`；正式跑圈前需要按照 AprilTag 世界坐标系补齐路径点。

## 常用环境变量

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `AR_CONTROL_MODE` | `pose` | `pose/path/planner` 优先定位路径；无结果时退回视觉误差 |
| `AR_WAYPOINT_PATH` | `setupUI/path_waypoints.json` | 路径点文件 |
| `AR_POSE_CONTROL_SPEED` | `80.0` | 定位路径控制目标速度 |
| `AR_POSE_FRESH_TTL` | `0.8` | 定位数据新鲜度阈值，单位秒 |
| `AR_POSE_INPUT_MODE` | `udp` | 定位入口；临时恢复旧方案可设为 `ble` |
| `AR_POSE_INPUT_HOST` | `0.0.0.0` | Windows 定位 UDP 监听地址 |
| `AR_POSE_INPUT_PORT` | `9005` | Windows 定位 UDP 监听端口 |
| `AR_UDP_IP` | `127.0.0.1` | 官方 AR 转发目标 IP |
| `AR_UDP_PORT` | `9006` | 官方 AR 转发目标端口 |

## 调试检查

WebUI/HUD 或状态接口里优先看：

- `WIN-UDP` 是否为 receiving，`ok` 是否增长。
- POSE 坐标是否随 tag 移动变化。
- `AR-FWD ok` 是否增长。
- control state 是否为 `POSE` 或 `VISION`。
- TC264D feedback count 是否增长。
- `flags` 是否为 `0x01`。
- `motor_target / servo_output` 是否随控制变化。

状态接口：

- `http://<RK3588S-IP>:9105/pose_status`
- `http://<RK3588S-IP>:9105/pose_packet`

## 当前状态

- Windows 顶置相机定位已接入上位机 UDP 桥：板卡 `9005` 接收、路径控制、转发官方 AR `9006`。
- 上位机已接入串口控车和 TC264D 反馈显示。
- 当前目标是先低速跑完一圈，再继续优化模型、路径和任务识别。
