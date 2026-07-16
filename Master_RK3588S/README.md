# Master_RK3588S

RK3588S 上位机当前基于官方 X-Verse AR Engine `1.0.7`。本阶段只迁移与旧视觉模型无关的基础 I/O，不恢复 1.06 的分割、检测、状态机和路径后处理。

## 运行链路

```text
Windows robot_position -> UDP :9005 -> pose_ar_bridge.py
                       -> UDP 127.0.0.1:9006 -> official AR engine

Windows gamepad_control -> UDP :9010 -> control_gamepad_receiver.py
                         -> control_serial_comm.py -> TC264D

official AR engine -> shm_ar_video -> vision_frame_source.py
                                      -> future single-RKNN pipeline
```

定位数据只驱动官方 AR 融合，不进入控车决策。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `setupUI/ar_receiver.py` | 官方 1.07 示例入口，当前保持原样 |
| `setupUI/pose_ar_bridge.py` | 验证并透明转发已经在 Windows 端完成轴映射的位姿 JSON |
| `setupUI/control_gamepad_receiver.py` | 接收手柄控制包，处理启用、停车、TTL 和手柄范围保护 |
| `setupUI/control_serial_comm.py` | TC264D 串口帧收发、自动重连和 v1/v2/v3 反馈解析 |
| `setupUI/control_car_link.py` | 串口控制轻量封装 |
| `setupUI/control_runtime.py` | 独立线程完成手柄优先、视觉回退和唯一串口下发 |
| `setupUI/standalone_control_bridge.py` | 不启动 `ar_receiver.py` 时使用的备用 I/O 入口 |
| `setupUI/steering_calibration_tool.py` | 实时调节 `track_error`，记录阿克曼左右轮实测角并导出标定表 |
| `setupUI/vision_frame_source.py` | 校验共享内存头部，读取一致的 AR 视频帧 |
| `setupUI/dist/main_config.json` | 官方 1.07 配置；`network.control_port` 固定为 `9006` |

## 启动

```bash
cd ~/Desktop/setupUI
python3 ar_receiver.py
```

默认端口和设备：

- Windows 定位输入：`0.0.0.0:9005`
- 官方 AR 转发：`127.0.0.1:9006`
- 手柄输入：`0.0.0.0:9010`
- TC264D：`/dev/ttyUSB0`, `460800` baud

`ar_receiver.py` 会自动启动 `ControlRuntime`。手柄有效时使用 `GAMEPAD` 命令；手柄无效时自动选择最新的 `VISION` 命令。视觉控制默认下发到 TC264D；若只需要调试画面，可设置 `AR_VISION_CONTROL_SEND=0`。手柄从有效状态退出、视觉命令超时或视觉清空后会发送 `STATE_SAFE_STOP`。TC264D 自身的输入超时和硬保护继续有效。

`standalone_control_bridge.py` 只在不运行 `ar_receiver.py` 时使用，二者不能同时启动。后续多任务视觉控制通过 `ControlRuntime.update_vision_command()` 接入，不再创建第二条串口链路。

## 转向标定

标定时先停止 `ar_receiver.py`、`standalone_control_bridge.py` 等占用 TC264D 串口的程序，并架空驱动轮：

```bash
cd ~/Desktop/SYSU_DDL/Master_RK3588S/setupUI
python3 steering_calibration_tool.py
```

浏览器会打开 `http://127.0.0.1:8765`。连接串口后开启“舵机输出”，通过滑杆或数值框实时调整 `error`；测量左右前轮相对车身中轴线的实际锐角并保存。角度统一使用“顺时针右转为正、逆时针左转为负”，有效范围为 `(-90°, 90°)`。

程序只驱动舵机：电机目标速度固定为 `0 m/s`，并以 20 Hz 持续发送带速度覆盖标志的控制帧。工具采用单向下发模式，不要求 TC264D 回传；Linux 串口完整写入 14 字节控制帧后显示“已写入串口，持续发送”，发送失败时自动重试。关闭输出、断开串口或退出程序时会重复下发 `STATE_SAFE_STOP`。数据自动保存到 `setupUI/steering_calibration_data/`：

- `steering_calibration.json`：完整原始数据和车辆几何参数。
- `steering_calibration.csv`：便于拟合和绘图的原始测量表。
- `steering_calibration_fit.json`：正向/反向 1~3 次多项式、RMSE 和 R²。
- `steering_calibration_lut.h`：按 `error` 排序、重复点取均值的 C 查表，并提供 `steering_error_from_center_angle()` 分段线性反查函数。

轴距和前轮轮距只用于根据阿克曼几何计算等效中心转角、曲率和左右轮推算半径差，不会改变实际下发给 TC264D 的 `error`。

界面中的多项式用于观察整体关系和拟合质量。实际控车优先调用导出头文件中的分段线性反查函数：输入逆透视前瞻点得到的期望中心转角，输出应下发给当前 TC264D 控制链路的 `track_error`，并在标定范围两端自动限幅。

## 不在本阶段实现

- 旧分割中线和 fork 后处理。
- 旧目标检测后处理和多模型并行。
- 旧任务/比赛状态机和风险阈值。
- 旧局部规划、误差生成和控制仲裁。
- 旧 HUD、Web debug stream、性能 CSV、OCR/API。

这些模块将在新的单 RKNN 多任务模型输出契约稳定后重新轻量实现。
