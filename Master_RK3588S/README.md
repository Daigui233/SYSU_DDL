# Master_RK3588S

RK3588S 上位机工程。当前核心任务是把 ESP32 BLE 定位、官方 AR 融合预览、路径/巡线控制和 TC264D 串口控车串起来，先完成低速跑一圈。

## 当前链路

```text
ESP32_BLE_Safe -> BLE notify -> setupUI/ar_receiver.py
               -> robot_position UDP -> 官方 AR 引擎/WebUI 视频融合
               -> 路径/巡线控制 -> setupUI/serial_comm.py -> /dev/ttyUSB0 -> TC264D

TC264D -> 串口反馈 -> serial_comm.py -> ar_receiver.py HUD/状态接口
```

注意：

- BLE 是定位链路。
- `/dev/ttyUSB0` 是 RK3588S 到 TC264D 的控车链路。
- ESP32 不负责 UDP 控车；UDP 是上位机把定位转给本机 AR 引擎。

## 重点文件

| 文件 | 作用 |
| --- | --- |
| `setupUI/ar_receiver.py` | AR 预览主入口，接 BLE 定位，发 AR UDP，生成控车命令 |
| `setupUI/serial_comm.py` | RK3588S 与 TC264D 的串口收发 |
| `setupUI/dist/main_config.json` | 官方 WebUI/AR 配置 |
| `setupUI/path_waypoints.json` | 路径点文件，存在时优先走定位路径控制 |
| `setupUI/ar_pose_debug.log` | BLE/解析/UDP 分段调试日志 |
| `setupUI/ar_pose_status.json` | 当前定位、控制、串口反馈状态 |
| `setupUI/xverse_control_live.json` | 最近一次官方 `robot_position` 包 |

## 定位与 AR

ESP32 蓝牙名：`ESP32_BLE_Safe`

BLE notify UUID：`6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

`ar_receiver.py` 支持无换行坐标 chunk，会解析文本或 JSON，并转成官方格式：

```json
{
  "type": "robot_position",
  "pos": [0.0, 0.16, 0.0],
  "euler": [0.0, 0.0, 0.0],
  "seq": 0,
  "timestamp": 0.0
}
```

当前约定：

- `pos[0]`：横向 X。
- `pos[1]`：高度，默认 `0.16`。
- `pos[2]`：前后/赛道平面 Z。
- `euler[2]`：Yaw。

默认 UDP 目标：`127.0.0.1:9005`，实际端口读取 `dist/main_config.json -> network.control_port`。

## WebUI 配置建议

| 项 | 值 |
| --- | --- |
| 定位数据地址 | `127.0.0.1` |
| 端口号 | `9005` |
| UNITY 同步目标 IP | 本机调试可填 `127.0.0.1`，外部 Unity 按实际 IP |
| UNITY 同步端口 | `9003` |
| 相机位移 X/Y/Z | `0 / 0.16 / 0.12` |
| 相机旋转 X/Y/Z | `0 / 0 / 0` |

不要把端口拼到“定位数据地址”里；地址和端口分开填。

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

## 常用环境变量

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `AR_CONTROL_MODE` | `pose` | `pose/path/planner` 优先定位路径；无结果时退回视觉误差 |
| `AR_WAYPOINT_PATH` | `setupUI/path_waypoints.json` | 路径点文件 |
| `AR_POSE_CONTROL_SPEED` | `80.0` | 定位路径控制目标速度 |
| `AR_POSE_FRESH_TTL` | `0.8` | 定位数据新鲜度阈值，单位秒 |
| `AR_UDP_IP` | `127.0.0.1` | AR UDP 目标 IP |
| `AR_UDP_PORT` | `9005` | AR UDP 目标端口 |
| `AR_BLE_DEVICE_NAME` | `ESP32_BLE_Safe` | ESP32 BLE 名称 |

## 调试检查

WebUI/HUD 或状态接口里优先看：

- BLE 是否 connected，`ok` 是否增长。
- POSE 坐标是否随 tag 移动变化。
- UDP ok 是否增长。
- control state 是否为 `POSE` 或 `VISION`。
- TC264D feedback count 是否增长。
- `flags` 是否为 `0x01`。
- `motor_target / servo_output` 是否随控制变化。

状态接口：

- `http://<RK3588S-IP>:9105/pose_status`
- `http://<RK3588S-IP>:9105/pose_packet`

## 当前状态

- AR 定位融合已跑通：移动 tag 后视频流会实时变化。
- 上位机已接入串口控车和 TC264D 反馈显示。
- 当前目标是先低速跑完一圈，再继续优化模型、路径和任务识别。