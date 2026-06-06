# Slave_TC264D

TC264D 下位机负责接收 RK3588S 控制帧，执行电机/舵机控制，并回传实时状态。下位机不处理定位、模型识别或任务决策。

## 系统边界

- AprilTag 定位只服务官方 AR 融合、坐标显示和记录，不参与 `track_error` 或下位机状态控制。
- 当前只使用融合视频上的视觉巡线和 `STATE_TRACK`。
- 当前分割和检测模型效果较差；检测模型只画 `gold / car / human` 检测框。
- 检测任务决策、OCR/API 和多状态控制均属于后续扩展。
- 可选手柄遥控只在 RK3588S 上位机侧接管控制帧；TC264D 不区分视觉来源或手柄来源，只执行收到的 `state_cmd / target_speed / track_error / flags`。

## 串口

| 项 | 值 |
| --- | --- |
| 模块 | `UART_1` |
| 波特率 | `460800` |
| TX | `P20.10` |
| RX | `P33.13` |
| RK3588S 设备 | `/dev/ttyUSB0` |

使用 `3.3V` 电平，TX/RX 交叉并共地。主循环每 `10 ms` 更新控制并发送反馈。

## 当前状态

正常视觉控制只使用 `STATE_TRACK`：

- 上位机根据融合视频生成 `track_error`。
- 上位机发送 `STATE_TRACK + target_speed + flags=0x01`。
- 下位机使用误差控制舵机，并使用上位机目标速度控制电机。
- 电机 PWM 硬限幅当前为 `±2500` duty，防止编码器异常或 PID 调参不当时直接冲到满占空比。
- `target_speed` 当前按同款 CarDo 车模参数粗换算为 `m/s`；`actual_speed` 由编码器计数、`64 mm` 轮径、`512` 线编码器、`4` 倍频、`2.7` 减速比和 `10 ms` 控制周期换算得到。
- `STATE_TRACK` 电机速度环当前采用约 `1450` duty 启动前馈 + 小 PI 修正：目标速度非零时先越过电机启动死区，PID 只负责稳速微调，最终输出仍受 `±2500` duty 硬限幅保护。
- 前馈 duty 不是速度标定，只是静摩擦/启动死区补偿；若正转时 `actual_speed` 为负，需要把 `Control.c` 中的 `MOTOR_ENCODER_SIGN` 改为 `-1.0f` 后再调 PID。

`STATE_SAFE_STOP` 当前用于三类停车来源：RK3588S 连续 `3 s` 没有识别到语义分割中线时，上位机重复下发停车命令；手柄遥控模式下 `LT >= 90%` 或手柄断连时，上位机下发停车命令；TC264D 连续约 `2.5 s` 没收到有效控制帧时，本地自行进入停车状态。模型输出无效但未超过 `3 s` 时，上位机会继续发送 `STATE_TRACK + TRACK_FALLBACK`；再次识别到中线后恢复 `STATE_TRACK + VISION`。

`STATE_LIMIT_SPEED / WAIT_LIGHT / AVOID / NAV_LEFT / NAV_RIGHT` 等枚举和 `state_cmd` 接口作为后续扩展保留，当前没有完整任务切换规则。

TC264D 本地输入超时不改变串口协议，只使用已解码控制帧的接收时间。若整个 RK3588S 进程直接崩溃导致串口帧停止，下位机会在约 `2.5 s` 后重置输入并进入 `STATE_SAFE_STOP`。

## 控制帧

上位机到下位机，固定 `14` 字节，小端格式。

| 偏移 | 字段 | 类型 |
| --- | --- | --- |
| `0` | 帧头 `0x42` | `uint8` |
| `1` | 地址 `0x10` | `uint8` |
| `2` | 载荷长度 `10` | `uint8` |
| `3~6` | `track_error` | `float` |
| `7~10` | `target_speed` | `float` |
| `11` | `state_cmd` | `uint8` |
| `12` | `flags` | `uint8` |
| `13` | 前 13 字节累加和低 8 位 | `uint8` |

`flags=0x01` 表示使用上位机下发速度。

## 反馈帧

下位机到上位机，固定 `50` 字节，小端格式。

| 偏移 | 字段 | 类型 |
| --- | --- | --- |
| `0~2` | 帧头 `0x42`、地址 `0x90`、载荷长度 `46` | `uint8` |
| `3~18` | `actual_speed`、`motor_target`、输入速度、输入误差 | `float × 4` |
| `19~26` | `motor_output`、`servo_output` | `int32 × 2` |
| `27~46` | 电机 PID 与舵机 PD 参数 | `float × 5` |
| `47` | 当前状态 | `uint8` |
| `48` | 当前输入 flags | `uint8` |
| `49` | 前 49 字节累加和低 8 位 | `uint8` |

## 调试重点

车不动时先看上位机 HUD/WebUI：

1. feedback 是否持续增长。
2. `state` 是否为 `TRACK`，`flags` 是否为 `0x01`。
3. `input_target_speed` 和 `motor_target` 是否大于 `0`，单位是否为预期的 `m/s`。
4. `motor_output`、`actual_speed` 和 `servo_output` 是否变化；架空正转时若 `actual_speed` 为负，先修正 `MOTOR_ENCODER_SIGN`。
5. 若 `motor_output` 长时间接近 `±2500` 但 `actual_speed` 仍接近 `0`，优先检查编码器方向、机械空转和速度换算参数。

源码 `.c/.h` 保持 GBK 编码以匹配 AURIX Development Studio；Markdown 使用 UTF-8。当前工程在 ADS 中可达到 `0 errors, 0 warnings`。
