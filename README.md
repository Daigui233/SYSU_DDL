# SYSU_DDL

RK3588S 上位机 + TC264D 下位机智能车工程。当前阶段只先打通视觉巡线和上下位机控制，让小车低速跑起来；任务识别和多状态控制留待模型优化后接入。

## 正确系统架构

```text
Windows 顶置相机
  -> AprilTag 定位 robot_position
  -> UDP 板卡IP:9005 -> ar_receiver.py -> 127.0.0.1:9006
  -> 官方 AR 引擎根据定位更新融合画面
  -> 融合视频流 shm_ar_video
  -> RK3588S 视觉模型与上位机决策
       - 当前：分割结果 -> 路线 track_error -> STATE_TRACK
       - 后续：检测/OCR/API -> 任务决策 -> 其他 state_cmd
  -> /dev/ttyUSB0 -> TC264D
  -> 电机、舵机和状态执行

可选数据采集遥控链路：
Windows 定位 EXE 勾选 Gamepad Mode
  -> UDP 板卡IP:9010
  -> ar_receiver.py 临时覆盖视觉控制
  -> /dev/ttyUSB0 -> TC264D
```

AprilTag 定位只服务于 AR 融合、坐标显示和记录，不直接规划路线，也不直接生成舵机误差。默认情况下小车如何行驶，由融合视频上的视觉结果和上位机决策决定；只有 Windows 定位 EXE 中显式勾选 `Gamepad Mode` 且 RK3588S 收到新鲜遥控包时，才临时进入手柄遥控。

## 模块职责

| 模块 | 职责 |
| --- | --- |
| `ArucoCalib-master/ArucoCalib-master/` | Windows AprilTag 定位，发送官方 `robot_position`；可选发送手柄遥控包 |
| 官方 AR 引擎/WebUI | 根据定位更新 AR 世界并产生融合视频流 |
| `Master_RK3588S/setupUI/ar_receiver.py` | 读取融合视频、运行模型、生成控制命令、转发定位给 AR |
| `Master_RK3588S/setupUI/serial_comm.py` | 向 TC264D 下发控制帧并接收反馈 |
| `Slave_TC264D/` | 执行上位机给出的状态、速度和转向误差 |
| `Tools_Windows/gamepad_mapper.py` | 手柄输入和映射测试工具，不直接控车 |

## 当前阶段

- 正常视觉控制只使用 `STATE_TRACK` 巡线状态；仅当 RK3588S 连续 `3 s` 没有识别到语义分割中线时，才周期性下发 `STATE_SAFE_STOP`。
- 分割模型从融合视频生成 `track_error`，上位机以 `STATE_TRACK + flags=0x01` 下发给 TC264D。
- 当前分割和检测模型效果较差，仍需优化；检测模型的 `gold / car / human` 结果只用于画框。
- OCR 和外部 API 尚未接入，当前不进行金币、避车、红绿灯等任务状态切换。
- 控制协议中的 `state_cmd / target_speed / track_error / flags` 与下位机预留状态继续保留，供后续扩展，不在当前阶段写死任务规则。
- 无有效分割误差但未超过 `3 s` 时，当前 `TRACK_FALLBACK` 会以误差 `0`、默认速度 `0.5 m/s` 继续直行。
- 连续 `3 s` 没有有效 `track_error` 时，RK3588S 进入 `LINE_LOSS_SAFE_STOP` 并重复下发 `STATE_SAFE_STOP`；再次识别到语义分割中线后，下一帧恢复 `STATE_TRACK + VISION`。
- 手柄遥控仅用于调试和采集数据：默认关闭，只有定位 EXE 勾选 `Gamepad Mode` 后才通过 UDP `9010` 接管；关闭、丢包超时或赛方定位模块没有遥控包时，一律回到视觉控车。
- 遥控模式下定位 UDP `9005` 仍正常发送并被 RK3588S 转发到 `9006`，不会因为手柄接管而停止 AR 融合。
- 遥控映射：右扳机 `RT` 控制 `target_speed`，左摇杆横轴 `LX` 控制 `track_error`，左扳机 `LT >= 90%` 或手柄断连时发送 `STATE_SAFE_STOP`。
- TC264D 保留现有串口协议，同时增加 `2.5 s` 本地输入超时；若上位机进程崩溃导致串口帧停止，下位机会自行进入 `STATE_SAFE_STOP`。

## 定位与端口

官方定位包：

```json
{"type":"robot_position","pos":[x,0.16,z],"euler":[0.0,yaw,0.0]}
```

- Windows 定位程序发送到板卡局域网 IP 的 UDP `9005`。
- `ar_receiver.py` 转发到板卡本机 `127.0.0.1:9006`；转发给官方 AR 时只交换平移轴为 `pos=[z,0.16,x]`，`yaw=euler[1]` 保持不变。
- WebUI 定位数据地址填写 `127.0.0.1`，端口填写 `9006`。
- 手柄遥控输入为独立 UDP `9010`，只接收 `type=gamepad_control`，不复用定位包。
- UNITY 同步端口为 `9003`。
- 控车串口默认 `/dev/ttyUSB0`、`460800` baud，上位机串口写入带 `write_timeout=0.05 s`；启动时打不开会自动重试，现场可用 `AR_CAR_SERIAL_PORT` 临时指定实际设备。

场地为 `4 m × 3 m`，Windows 定位预览原点在右下角，`+X` 向左、`+Z` 向上；Yaw 使用 `euler[1]`，`0° -> +X`、`+90° -> +Z`。固定标定 Tag 1～4 均为 `20 cm × 20 cm`，Windows 定位端会对输出坐标和 Yaw 做轻量滤波。由于官方 AR 场景资产的 X/Z 平移轴与定位预览互换，RK3588S 仅在转发到 `9006` 时交换 `pos[0]` 和 `pos[2]`，不改变 Yaw。这些坐标只用于 AR 定位与融合，不用于直接控车。

## 待解决与待确认

### 当前巡线

1. 使用真实融合视频重新训练或优化分割模型，使 `track_error` 足以稳定巡线。
2. 架空车轮验证误差方向、舵机方向、PID、速度、`flags=0x01` 和反馈帧。
3. 首次落地前使用默认 `0.5 m/s` 验证巡线和 `TRACK_FALLBACK`；需要更慢或更快时通过环境变量 `AR_TRACK_SPEED` / `AR_TRACK_FALLBACK_SPEED` 调整。
4. 当前 `target_speed` 已按同款 CarDo 车模参数粗换算为 `m/s`，TC264D 的 `actual_speed` 由编码器计数换算得到；后续仍需确认编码器正负号，并用实测速度修正轮径、周期或比例误差。
5. 若用手柄采集数据，先确认 `Gamepad Mode` 勾选、WebUI `GAMEPAD` 状态、TC264D 反馈和现场急停手段，再低速落地。

### 定位与 AR

1. 工业相机到货后完成固定 Tag 中心实测、相机参数验证、内参与畸变标定。
2. 确认车载 Tag ID、朝向、安装偏角，以及 AR 坐标和 Yaw 方向。
3. 实测并校准 WebUI 的 `pos_offset` 与 `euler_offset`。

### 后续扩展

1. 优化或更换检测模型，并接入 OCR/API。
2. 新增独立的上位机任务决策层，将识别结果转换为 `state_cmd`、`target_speed` 和必要的误差修正。
3. 为每个新增状态分别定义进入条件、保持条件、退出条件和失效策略，保持现有串口协议兼容。

## 当前联调顺序

1. 确认定位能稳定驱动 AR 融合画面。
2. 只启用 `STATE_TRACK`，低速验证 `融合视频 -> 分割模型 -> track_error -> TC264D`。
3. 模型稳定后再逐项接入检测、OCR/API 和其他任务状态。

详细使用和协议见各子工程 README。
