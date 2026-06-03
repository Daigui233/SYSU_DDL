# Slave_TC264D

TC264D 下位机工程，基于逐飞 TC264D 开源库。当前职责是接收 RK3588S 上位机控制帧，执行电机/舵机/PID 控制，并把关键实时状态回传给上位机 WebUI/HUD。

## 当前目标

先让车能根据上位机给出的路径/巡线误差低速跑完一圈。

当前不追求最终赛题完整效果，暂不接入 OCR、红绿灯、金币、复杂避障，也暂不启用通信心跳。后续功能扩展时保持当前协议和状态机继续演进。

## 控制链路

```text
UART1 RX -> communication_rx_byte()
         -> communication_decode_frame()
         -> control_set_input()
         -> control_update()
         -> Motor PID / Servo PD
         -> communication_send_feedback()
```

主循环当前按控制周期执行：

- `control_update()`：更新状态、电机目标、舵机输出。
- `communication_send_feedback()`：回传实时参数给上位机。

当前控制周期：`10ms`。

## 串口参数

| 项 | 值 |
| --- | --- |
| 串口模块 | `UART_1` |
| 波特率 | `460800` |
| TX | `P20.10` / `UART1_TX_P20_10` |
| RX | `P33.13` / `UART1_RX_P33_13` |
| 电平 | `3.3V` |
| 接线 | TX/RX 交叉，GND 共地 |

RK3588S 侧当前使用 `/dev/ttyUSB0`。

## 状态与 flags

状态枚举：

| 状态 | 值 | 说明 |
| --- | --- | --- |
| `STATE_IDLE` | `0` | 空闲 |
| `STATE_TRACK` | `1` | 正常跟踪/巡线/路径控制 |
| `STATE_LIMIT_SPEED` | `2` | 预留限速 |
| `STATE_WAIT_LIGHT` | `3` | 预留红灯等待 |
| `STATE_AVOID` | `4` | 预留避障 |
| `STATE_NAV_LEFT` | `5` | 预留左转导航 |
| `STATE_NAV_RIGHT` | `6` | 预留右转导航 |
| `STATE_SAFE_STOP` | `7` | 安全停车 |

当前只重点使用：`STATE_TRACK` 和 `STATE_SAFE_STOP`。

`flags` 当前只定义一位：

| 位 | 值 | 说明 |
| --- | --- | --- |
| `CONTROL_FLAG_USE_TARGET_SPEED` | `0x01` | 下位机使用上位机传来的 `target_speed` |

正常跑车时应为：`STATE_TRACK + flags=0x01`。

`flags=0` 当前只用于安全停车兜底。下位机没有心跳自动停车逻辑；若上位机没有有效定位/视觉控制量，会主动发送 `STATE_SAFE_STOP + speed=0 + flags=0`。

## 上位机 -> 下位机控制帧

固定 14 字节，小端格式。

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| `0` | `frame_head` | `uint8` | 固定 `0x42` |
| `1` | `addr` | `uint8` | 固定 `0x10` |
| `2` | `payload_len` | `uint8` | 固定 `10` |
| `3~6` | `track_error` | `float` | 路径/巡线误差 |
| `7~10` | `target_speed` | `float` | 上位机目标速度 |
| `11` | `state_cmd` | `uint8` | 状态命令 |
| `12` | `flags` | `uint8` | 控制标志位 |
| `13` | `checksum` | `uint8` | `0~12` 累加和低 8 位 |

## 下位机 -> 上位机反馈帧

固定 50 字节，小端格式。

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| `0` | `frame_head` | `uint8` | 固定 `0x42` |
| `1` | `addr` | `uint8` | 固定 `0x90` |
| `2` | `payload_len` | `uint8` | 固定 `46` |
| `3~6` | `actual_speed` | `float` | 编码器/速度反馈 |
| `7~10` | `motor_target` | `float` | 下位机实际采用的目标速度 |
| `11~14` | `input_target_speed` | `float` | 上位机输入目标速度 |
| `15~18` | `input_track_error` | `float` | 上位机输入误差 |
| `19~22` | `motor_output` | `int32` | 电机 PWM/控制输出 |
| `23~26` | `servo_output` | `int32` | 舵机输出 |
| `27~30` | `motor_kp` | `float` | 电机 PID Kp |
| `31~34` | `motor_ki` | `float` | 电机 PID Ki |
| `35~38` | `motor_kd` | `float` | 电机 PID Kd |
| `39~42` | `servo_kp` | `float` | 舵机 PD Kp |
| `43~46` | `servo_kd` | `float` | 舵机 PD Kd |
| `47` | `state` | `uint8` | 当前状态 |
| `48` | `flags` | `uint8` | 当前输入 flags |
| `49` | `checksum` | `uint8` | `0~48` 累加和低 8 位 |

## 当前控制行为

- `STATE_TRACK`：允许电机速度覆盖；若 `flags & 0x01`，使用上位机 `target_speed`。
- `STATE_SAFE_STOP`：目标速度为 0，舵机回中/保持安全输出。
- 其他状态暂时保留接口，后续接赛题任务再细化。

当前没有启用心跳超时函数。这样做是为了先把“定位 -> 上位机 -> 下位机 -> 实车跑一圈”排通，避免停车原因不好区分。

## 源码编码

TC264D 工程源码 .c/.h 保持 GBK 编码，以匹配 AURIX Development Studio 默认编码。README 等 Markdown 文档保持 UTF-8。

## 编译

使用 AURIX Development Studio 打开本工程编译。当前工程已在 ADS 中达到：

```text
0 errors, 0 warnings
```

生成物在 `Debug/` 目录下，包括 `Seekfree_TC264_Opensource_Library.elf` 和 `.hex`。

## 调试观察

优先看上位机 WebUI/HUD 中的下位机反馈：

- feedback count 是否持续增长。
- `state` 是否为 `TRACK`。
- `flags` 是否为 `0x01`。
- `input_target_speed` 是否等于上位机下发速度。
- `motor_target / actual_speed / motor_output` 是否变化。
- `servo_output` 是否随误差变化。

若车不动但串口解包正常，优先确认：

1. 上位机是否发 `flags=0x01`。
2. 下位机反馈里的 `motor_target` 是否大于 0。
3. 电机 PWM、编码器、电源、使能和舵机输出是否正常。

## 更新日志

### 2026-06-03

- 保持现有固定帧协议，补齐 50 字节反馈帧解析/回传。
- 清理 UART 调试字符串，避免文本混入二进制协议。
- 正常控车使用 `STATE_TRACK + flags=0x01`；`flags=0` 仅用于安全停车兜底。
- 暂不启用通信心跳/超时保护。
- 主循环保持 10ms 控制更新和反馈回传。
- ADS 编译达到 `0 errors, 0 warnings`。

### 2026-04-29

- 完成第一版 `Communication / Control / State / PID / Motor / Servo / Init` 基础框架。
- 建立上位机到下位机固定帧控制协议。