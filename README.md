# SYSU_DDL

RK3588S 上位机 + TC264D 下位机智能车工程。当前阶段先不追求完整赛题效果，目标是把定位、AR 融合、路径/巡线控制、串口下发和下位机执行串成一条可跑通的最小链路，让车能先低速跑完一圈。

## 当前目标

- 已跑通：Windows 顶置相机 AprilTag 定位程序输出官方 `robot_position`，并能驱动 WebUI/AR 融合画面实时变化。
- 当前重点：用定位/路径规划或视觉误差进入 `TRACK` 状态，经 `/dev/ttyUSB0` 下发给 TC264D，先完成一圈实车运行。
- 暂不处理：OCR、红绿灯、金币、复杂避障、最终模型优化。
- 暂不启用：通信心跳/超时保护。当前无有效定位和视觉控制量时会进入 `TRACK_FALLBACK` 继续直行，而不是自动 `SAFE_STOP`；仅适合低速、架空车轮或有人可立即断电的联调。

## 目录

- `Master_RK3588S/setupUI/`：WebUI、AR 预览、Windows UDP 定位桥接、路径/巡线控制、串口下发。
- `Slave_TC264D/`：TC264D 下位机，负责串口收包、电机/舵机/PID、状态执行和反馈回传。
- `Slave_TC264D/README.md`：下位机控制链路和通信协议细节。

## 总链路

```text
Windows 顶置相机 -> AprilTag 36h11 解算 -> UDP -> RK3588S:9005/ar_receiver.py
                  -> 最新定位/路径控制 -> serial_comm.py -> /dev/ttyUSB0 -> TC264D
                  -> 转发 127.0.0.1:9006 -> 官方 AR 引擎/WebUI 融合画面

TC264D -> 串口反馈 -> serial_comm.py -> ar_receiver.py HUD/WebUI
```

两条链路要分清：

- `Windows -> RK3588S:9005` 是外部定位输入链路。
- `RK3588S:9005 -> 127.0.0.1:9006` 是 `ar_receiver.py` 到官方 AR 引擎的板卡内部 UDP 转发。
- `RK3588S -> TC264D` 是控车串口链路，不负责定位。

## 定位与 AR 方案

Windows 运行 `ArucoCalib-master/ArucoCalib-master/dist/ArucoCalibLocator.exe`，完成 AprilTag 36h11 检测、平面定位和 yaw 解算，再向板卡局域网 IP 的 UDP `9005` 发送：

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

`ar_receiver.py` 默认监听 `0.0.0.0:9005`，收到定位后：

1. 保存为路径规划和控车使用的最新定位。
2. 转发到 `127.0.0.1:9006`。
3. 官方 AR 引擎监听 `dist/main_config.json -> network.control_port = 9006`。

如需临时恢复旧 BLE 定位，可在启动 `ar_receiver.py` 前设置 `AR_POSE_INPUT_MODE=ble`。

修改端口配置后需要重启官方 AR/WebUI 预览进程。联调时先启动板卡上的 `ar_receiver.py`，再启动 Windows 定位发送。

## WebUI 关键配置

当前建议值：

| 项 | 值 |
| --- | --- |
| 定位数据地址 | `127.0.0.1` |
| 端口号 | `9006` |
| UNITY 同步目标 IP | 按实际 Unity/板卡目标填写，本机调试可用 `127.0.0.1` |
| UNITY 同步端口 | `9003` |
| 相机位移 X/Y/Z | `0 / 0.16 / 0.12` |
| 相机旋转 X/Y/Z | `0 / 0 / 0` |

WebUI 中的定位数据地址和端口应填写 `127.0.0.1` 与 `9006`，对应板卡内部官方 AR 引擎。Windows 定位程序仍发送到“板卡局域网 IP:9005”，由 `ar_receiver.py` 接收并转发。相机安装参数写入 `network.pos_offset` 和 `network.euler_offset`，用于 AR 相机/车体偏移。

## 控车逻辑

`ar_receiver.py` 每帧生成一个控制命令：

1. 有路径点文件且 Windows UDP 定位新鲜：走定位路径控制，HUD 显示 `POSE`。
2. 没有有效定位路径控制，但有视觉/巡线误差：走视觉误差控制，HUD 显示 `VISION`。
3. 两者都没有：当前发 `TRACK_FALLBACK`，误差为 `0`、目标速度默认为 `120`，小车会继续直行。

正常跑车时应看到：

- `state = TRACK`
- `flags = 0x01`
- `target_speed > 0`

`flags = 0` 只用于当前安全停车兜底，不作为正常跑车状态。

路径点默认文件：`Master_RK3588S/setupUI/path_waypoints.json`。当前路径点逻辑是循环路径，不会自动因为跑完一圈停车；终点/圈数逻辑后续再补。

当前仓库暂未包含 `path_waypoints.json`。不影响定位进入 AR，但定位路径控制会显示 `path disabled`；正式跑圈前需要按照 AprilTag 世界坐标系生成路径点。

## 已知问题与跑圈前检查

以下内容按当前代码和配置审查得出。前四项会直接阻碍或明显影响定位跑圈，建议先处理。

### 跑圈前阻塞项

1. **定位世界坐标仍是示例值。**

   `ArucoCalib-master/ArucoCalib-master/config.yaml`、`aruco_core/config.yaml` 和 `dist/config.yaml` 中，固定 Tag 1~4 仍组成 `100 mm x 100 mm` 方形。必须按实际场地测量值改成真实毫米坐标，否则输出坐标范围、路径点和控制误差都会错误。

2. **定位路径文件尚不存在。**

   当前仓库没有 `Master_RK3588S/setupUI/path_waypoints.json`，因此定位路径规划实际处于 `path disabled`。此时即使 `WIN-UDP`、AR 视频和坐标正常，上位机也不会按定位路线跑圈，而会退回视觉控制或 `TRACK_FALLBACK`。

3. **坐标轴、Yaw 零点和方向尚未完成实车校验。**

   路径规划假定平面坐标为 X/Z、`yaw=0` 指向 `+Z`，并使用 `euler[1]`。正式落地前必须手动平移、旋转小车，确认 X/Z 增减方向、Yaw 正负方向以及 `yaw_offset_deg`，否则可能出现反向转向或沿错误方向追路径点。

4. **定位配置存在多份副本且已经有差异。**

   EXE 优先读取 `ArucoCalib-master/ArucoCalib-master/dist/config.yaml`；源码运行通常读取工程根目录 `config.yaml`。当前两份配置的 `camera.mirror` 和 `udp.enabled` 已不同。修改固定 Tag 坐标、板卡 IP、镜像或输出方向时，需要确认实际启动方式读取的是哪一份配置，并保持必要字段同步。

### 控车与失效风险

- 当前定位、视觉都失效时，`ar_receiver.py` 会发送 `TRACK_FALLBACK`，目标速度默认为 `120`，而不是停车。建议首次实车测试将 `AR_TRACK_FALLBACK_SPEED` 和其他速度降到很低，并保证可以立即断电。
- TC264D 当前没有通信心跳或输入超时停车。若 RK3588S 程序退出、串口断开或卡死，下位机会继续使用最后一次有效命令。
- 当前默认速度并不统一：定位路径控制默认为 `80`，视觉和 `TRACK_FALLBACK` 默认为 `120`，TC264D `STATE_TRACK` 内部默认值为 `100`；正常 `flags=0x01` 时以上位机下发速度为准。
- 路径规划目前是简单的逐点追踪，速度固定，没有曲率减速、前视点、加速度限制和圈数停车逻辑。弯道可能振荡或冲出路径，第一圈应使用低速且较平滑、较密的路径点。
- 上位机路径误差限制为 `+-120`，下位机舵机 PD 直接使用该误差。误差正负方向、比例和舵机极限仍需通过架空车轮测试校准。
- README 中的 `SAFE_STOP` 是可用状态，但当前正常主循环不会因定位/视觉丢失自动进入该状态。

### 定位性能与精度

- 定位程序在检测期间会同步保存每一帧 JPG，并且每帧重写不断增长的 `images.json`。当前 `dist/runs` 已约有 `11893` 个文件、约 `890 MB`；这会降低检测帧率、产生卡顿并持续占用磁盘。
- 当前定位在某一帧未识别到车载 Tag 时，会立即清空该帧定位并停止发送 UDP。RK3588S 只会将最近定位保留约 `0.8 s`，之后进入视觉或 `TRACK_FALLBACK`。
- 当前检测参数使用 `cornerRefinementMethod: NONE`。新工业相机到货后应根据实际高度、Tag 像素尺寸和运动模糊测试 `APRILTAG` 角点优化及阈值窗口，不能只依据笔记本摄像头结果定参数。
- 当前平面标定仅使用固定 Tag 中心计算单应矩阵，没有相机内参标定和镜头畸变校正。90 度广角镜头在场地边缘可能出现系统性坐标误差。
- 标定默认在首次同时识别到四个固定 Tag 后锁定。如果首次标定帧模糊或角点有误，会持续使用错误矩阵；发现轨迹比例或方向异常时应重置标定。
- 程序请求 `1920x1200 / YUY2 / 60 FPS`，但相机驱动可能拒绝并回退。当前曝光、增益、亮度等配置会被读取，但采集初始化并未真正应用这些参数；新相机到货后需要核对实际分辨率、实际 FPS、FOURCC 和曝光状态。

### 通信与工程维护

- UDP `sendto()` 成功只代表数据交给了本机网络栈，不代表 RK3588S 已收到。联调时必须同时确认 Windows 发送计数、板卡 HUD 的 `WIN-UDP ok`、`AR-FWD ok` 和 TC264D feedback 均持续增长。
- Windows 定位程序必须发送到板卡实际局域网 IP 的 `9005`；`127.0.0.1:9005` 只会发回 Windows 自己。板卡内部才使用 `127.0.0.1:9006` 转发给官方 AR 引擎。
- 端口 `9005` 或 `9006` 被其他进程占用时，定位输入或 AR 转发会失败。出现坐标不更新时应先检查端口监听和防火墙。
- 当前 Git 暂存区包含约 `11914` 个文件、接近 `1 GB`，其中包括约 `95 MB` 的 EXE 和大量 `dist/runs` 调试图片。推送会很慢，运行图片也会永久增大仓库历史；后续应决定哪些构建产物和运行日志需要保留。

### 建议的首次实车顺序

1. 写入真实固定 Tag 世界坐标，并确认 EXE 实际读取的 `dist/config.yaml`。
2. 架空车轮验证坐标轴、Yaw、舵机方向、`flags=0x01`、电机目标和反馈。
3. 生成低速、平滑的 `path_waypoints.json`，先将定位路径速度和 fallback 速度降到很低。
4. 在地面短距离测试定位丢失、程序退出和串口断开时的行为，再尝试完整一圈。
5. 工业相机到货后，再处理逐帧保存、检测参数、曝光和镜头畸变校正。

## 运行与日志

常用状态文件在 `Master_RK3588S/setupUI/` 下：

| 文件/接口 | 作用 |
| --- | --- |
| `ar_pose_debug.log` | Windows UDP 输入、解析、JSON、AR 转发分段调试日志 |
| `ar_pose_status.json` | 当前定位输入、AR 转发、控制和 TC264D 反馈状态 |
| `xverse_control_live.json` | 最近一次官方 `robot_position` JSON |
| `http://<RK3588S-IP>:9105/pose_status` | Web 状态接口 |
| `http://<RK3588S-IP>:9105/pose_packet` | 最近一次定位包 |

实车先看 HUD 里的几项：

- `WIN-UDP` 是否为 receiving，ok 是否增长。
- POSE 坐标是否实时变化。
- `AR-FWD ok` 是否增长。
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

### 2026-06-04

- 定位源切换为 Windows 顶置相机 AprilTag 36h11 定位。
- `ar_receiver.py` 默认监听板卡 UDP `9005`，供路径规划和控车使用，并转发到官方 AR 引擎 `127.0.0.1:9006`。
- 官方定位 yaw 统一使用 `euler[1]`；TC264D 串口协议保持不变。

### 2026-04-29

- 完成第一版上下位机固定帧通信骨架和基础控制模块。
- 建立 `Communication / Control / State / PID / Motor / Servo / Init` 基础结构。
