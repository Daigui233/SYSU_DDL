# Windows AprilTag 定位程序

Windows 顶置相机定位程序负责检测 AprilTag 36h11，解算小车坐标和 Yaw，并向 RK3588S 发送官方 `robot_position`。

定位数据只用于官方 AR 融合、坐标显示和记录，不参与小车路线规划或转向控制。

## 系统当前状态

- 当前小车只使用融合视频上的视觉巡线和 `STATE_TRACK`，分割模型输出的 `track_error` 是唯一转向来源。
- 当前分割和检测模型效果较差；检测模型只画 `gold / car / human` 检测框。
- 检测任务决策、OCR/API 和多状态控制均属于后续扩展。

## 运行

直接启动：

```text
dist/ArucoCalibLocator.exe
```

前端必要操作：

- `Camera`：选择摄像头编号。
- `Reconnect & Save`：重连并保存摄像头编号。
- `Mirror`：修正镜像并保存。
- `RK3588S IP`：填写板卡局域网 IP。
- `UDP Enable`、`Apply & Save`：启用并保存 UDP 输出。
- `Start Detection`、`Stop`、`Reset Calibration`：控制检测和重新标定。
- 程序首次同时读到 4 个固定 Tag 后会自动锁定标定；锁定后固定 Tag 掉点或被碰到不会改变坐标系。需要重新标定时点击 `Reset Calibration` 并确认。

程序固定请求 `1920x1200 / YUY2 / 60 FPS`，UDP 目标端口固定为 `9005`。检测过程不会自动保存逐帧图片；仅在手动录像时保存 MP4。

## 定位配置

EXE 使用 `dist/config.yaml`。源码运行使用根目录 `config.yaml`。

- 固定 Tag：ID `1～4`，尺寸 `0.20 m`。
- 车载 Tag：当前 `vehicle_id: 0`。
- 场地：`4 m × 3 m`。
- 检测参数：当前使用稳定配置；若远端固定 Tag 仍不稳，优先增大 Tag 或改善安装/光照。
- 原点：右下角；`+X` 向左，`+Z` 向上。
- Yaw：`0° -> +X`，`+90° -> +Z`。
- 在场地图俯视方向：
  `0°` 向左、`+90°` 向上、`-90°` 向下、`±180°` 向右。
- 右上角 `UDP trajectory` 按同一方向显示，仅用于预览实际发送的定位和 Yaw。

当前固定 Tag 临时中心坐标：

| Tag | 中心坐标 `(X, Z)` |
| --- | --- |
| `1` | `(0, 0)` mm |
| `2` | `(0, 3000)` mm |
| `3` | `(4000, 0)` mm |
| `4` | `(4000, 3000)` mm |

正式安装后必须将 `world_coordinates` 改为现场实测的 Tag 中心坐标。相机和固定 Tag 移动后应重新标定。

## UDP 输出

```json
{"type":"robot_position","pos":[x,0.16,z],"euler":[0.0,yaw,0.0]}
```

坐标单位为米，Yaw 单位为度。目标地址应填写 RK3588S 的局域网 IP，不要填写 Windows 自己的 `127.0.0.1`。

## 待现场确认

- 固定 Tag 实际中心坐标。
- 车载 Tag ID、安装朝向和 `yaw_offset_deg`。程序使用 Tag 的 corner 1 -> corner 2
  作为原始朝向；如果该方向不是车头方向，必须现场调整 `yaw_offset_deg`。
- 工业相机实际分辨率、FPS、镜像和曝光。
- 相机内参与广角镜头畸变校正。

## 源码运行

```powershell
pip install -r requirements.txt
python main.py
```
