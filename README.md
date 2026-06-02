# SYSU_DDL

RK3588S 上位机 + TC264D 下位机智能车工程。当前阶段先不追求完整赛题效果，目标是把定位、AR 融合、路径/巡线控制、串口下发和下位机执行串成一条可跑通的最小链路，让车能先低速跑完一圈。

## 当前目标

- 已跑通：ESP32 BLE 定位数据进入 RK3588S，并能驱动 WebUI/AR 融合画面实时变化。
- 当前重点：用定位/路径规划或视觉误差进入 `TRACK` 状态，经 `/dev/ttyUSB0` 下发给 TC264D，先完成一圈实车运行。
- 暂不处理：OCR、红绿灯、金币、复杂避障、最终模型优化。
- 暂不启用：通信心跳/超时保护。当前为了排查更稳定，只有上位机明确无有效控制量时才发 `SAFE_STOP`。

## 目录

- `Master_RK3588S/setupUI/`：WebUI、AR 预览、BLE 定位桥接、路径/巡线控制、串口下发。
- `Slave_TC264D/`：TC264D 下位机，负责串口收包、电机/舵机/PID、状态执行和反馈回传。
- `Slave_TC264D/README.md`：下位机控制链路和通信协议细节。

## 总链路

```text
定位模块 -> ESP32 -> BLE -> RK3588S/ar_receiver.py
          -> 官方 robot_position UDP -> WebUI/AR 融合画面
          -> 路径/巡线控制 -> serial_comm.py -> /dev/ttyUSB0 -> TC264D -> 电机/舵机

TC264D -> 串口反馈 -> serial_comm.py -> ar_receiver.py HUD/WebUI
```

两条链路要分清：

- `ESP32 -> RK3588S` 是 BLE 定位链路。
- `RK3588S -> TC264D` 是控车串口链路，不负责定位。

## 定位与 AR 方案

ESP32 蓝牙名：`ESP32_BLE_Safe`

BLE notify UUID：`6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

`ar_receiver.py` 自动扫描 ESP32，接收无换行或有换行的坐标 chunk，支持两类输入：

- 文本：`x,z,yaw` 或近似三元数格式。
- JSON：包含 `pos/euler` 或 `x/y/z/yaw` 字段。

上位机会转成官方 AR 引擎需要的扁平 JSON：

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
- `euler[2]`：Yaw，按官方 WASD/本地 AR 预览链路使用。
- ESP32 原始坐标默认乘 `AR_POSE_SCALE=0.01`。

UDP 发送目标默认读取 `dist/main_config.json -> network.control_port`，当前为 `127.0.0.1:9005`。

## WebUI 关键配置

当前建议值：

| 项 | 值 |
| --- | --- |
| 定位数据地址 | `127.0.0.1` |
| 端口号 | `9005` |
| UNITY 同步目标 IP | 按实际 Unity/板卡目标填写，本机调试可用 `127.0.0.1` |
| UNITY 同步端口 | `9003` |
| 相机位移 X/Y/Z | `0 / 0.16 / 0.12` |
| 相机旋转 X/Y/Z | `0 / 0 / 0` |

实际影响 AR 接收 `robot_position` 的核心字段是 `network.control_port`。相机安装参数写入 `network.pos_offset` 和 `network.euler_offset`，用于 AR 相机/车体偏移。

## 控车逻辑

`ar_receiver.py` 每帧生成一个控制命令：

1. 有路径点文件且定位新鲜：走定位路径控制，HUD 显示 `POSE`。
2. 没有有效定位路径控制，但有视觉/巡线误差：走视觉误差控制，HUD 显示 `VISION`。
3. 两者都没有：发安全停车，HUD 显示 `SAFE_STOP`。

正常跑车时应看到：

- `state = TRACK`
- `flags = 0x01`
- `target_speed > 0`

`flags = 0` 只用于当前安全停车兜底，不作为正常跑车状态。

路径点默认文件：`Master_RK3588S/setupUI/path_waypoints.json`。当前路径点逻辑是循环路径，不会自动因为跑完一圈停车；终点/圈数逻辑后续再补。

## 运行与日志

常用状态文件在 `Master_RK3588S/setupUI/` 下：

| 文件/接口 | 作用 |
| --- | --- |
| `ar_pose_debug.log` | BLE、解析、JSON、UDP 分段调试日志 |
| `ar_pose_status.json` | 当前 BLE/定位/控制/TC264D 反馈状态 |
| `xverse_control_live.json` | 最近一次官方 `robot_position` JSON |
| `http://<RK3588S-IP>:9105/pose_status` | Web 状态接口 |
| `http://<RK3588S-IP>:9105/pose_packet` | 最近一次定位包 |

实车先看 HUD 里的几项：

- BLE 是否 connected/ok 增长。
- POSE 坐标是否实时变化。
- UDP ok 是否增长。
- 控制状态是否为 `POSE` 或 `VISION`。
- TC264D feedback count 是否增长。
- `flags` 是否为 `0x01`。
- `motor_target / servo_output` 是否随控制变化。

## 通信协议概览

串口：`/dev/ttyUSB0`，`460800` baud。

上位机到下位机：固定 14 字节控制帧。

下位机到上位机：固定 50 字节反馈帧。

字段细节见 `Slave_TC264D/README.md`。当前协议先保持稳定，后续接入 OCR、红绿灯、避障、金币后再统一扩展。

## 更新日志

### 2026-06-03

- AR 定位链路已跑通，移动 tag 后 WebUI/AR 视频流可实时变化。
- 上位机接入定位路径/视觉误差到 TC264D 串口控车链路。
- 下位机反馈帧扩展到 50 字节，WebUI/HUD 可显示速度、输出、PID、状态和 flags。
- 暂停心跳/超时保护，避免当前跑一圈调试阶段引入额外不稳定因素。
- TC264D 工程 ADS 编译已达到 `0 errors, 0 warnings`。

### 2026-04-29

- 完成第一版上下位机固定帧通信骨架和基础控制模块。
- 建立 `Communication / Control / State / PID / Motor / Servo / Init` 基础结构。