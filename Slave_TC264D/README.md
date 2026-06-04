# Slave_TC264D

TC264D 下位机负责接收 RK3588S 控制帧，执行电机/舵机控制，并回传实时状态。下位机不处理定位、模型识别或任务决策。

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

当前阶段只使用 `STATE_TRACK`：

- 上位机根据融合视频生成 `track_error`。
- 上位机发送 `STATE_TRACK + target_speed + flags=0x01`。
- 下位机使用误差控制舵机，并使用上位机目标速度控制电机。

`STATE_LIMIT_SPEED / WAIT_LIGHT / AVOID / NAV_LEFT / NAV_RIGHT / SAFE_STOP` 等枚举和 `state_cmd` 接口作为后续扩展保留，当前没有完整任务切换规则。新增状态时由上位机负责决策，下位机只实现对应执行参数和动作。

当前没有心跳或输入超时停车；收到控制帧后会持续使用最近一次命令。

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
3. `input_target_speed` 和 `motor_target` 是否大于 `0`。
4. `motor_output`、`actual_speed` 和 `servo_output` 是否变化。

源码 `.c/.h` 保持 GBK 编码以匹配 AURIX Development Studio；Markdown 使用 UTF-8。当前工程在 ADS 中可达到 `0 errors, 0 warnings`。