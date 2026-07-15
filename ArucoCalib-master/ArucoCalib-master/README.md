# Windows AprilTag 定位程序

Windows 顶置相机定位程序负责检测 AprilTag 36h11，解算小车坐标和 Yaw，并向 RK3588S 发送官方 `robot_position`。为了采集分割数据集，程序也内置了可选 `Gamepad Mode`，显式勾选后才会额外发送手柄遥控包。

定位数据只用于官方 AR 融合、坐标显示和记录，不参与小车路线规划或转向控制。

## 系统当前状态

- RK3588S 当前已切换到 X-Verse 1.0.7 干净基线，旧分割、检测后处理、状态机和路径规划没有迁移。
- 新的单 RKNN 多任务模型仍在训练与架构开发阶段；当前没有启用视觉自动控车。
- AprilTag 定位不参与 `track_error` 或路径规划，只用于官方 AR 融合、显示和记录。
- 手柄遥控默认关闭；只有前端勾选 `Gamepad Mode` 并持续发出 `gamepad_control` 包时，RK3588S 才向 TC264D 下发手动控制。
- `Gamepad Mode` 使用定位 EXE 内部独立定时器发送，频率与当前相机/视频帧率一致；它不依赖固定 Tag、车载 Tag、标定状态或 `robot_position` 是否成功发送。
- 遥控模式下定位 `robot_position` 仍正常发送到 `9005`，不会影响 AR 融合和轨迹显示。

## 运行

直接启动：

```text
dist/ArucoCalibLocator.exe
```

当前 EXE 已按本分支源码重新打包，包含 Windows 发送端 `[z,height,x]` 坐标映射。

前端必要操作：

- `Camera`：选择摄像头编号。
- `Reconnect & Save`：重连并保存摄像头编号。
- `Mirror`：修正镜像并保存。
- `RK3588S IP`：填写板卡局域网 IP。
- `UDP Enable`、`Apply & Save`：启用并保存 UDP 输出。
- `Gamepad Mode`：显式启用手柄遥控；默认每次启动都关闭，不写入配置，避免比赛定位模块或普通定位运行时误入遥控。
- `Start Detection`、`Stop`、`Reset Calibration`：控制检测和重新标定。
- 程序首次同时读到 4 个固定 Tag 后会自动锁定标定；锁定后固定 Tag 掉点或被碰到不会改变坐标系。需要重新标定时点击 `Reset Calibration` 并确认。

程序固定请求 `1920x1200 / YUY2 / 60 FPS`。定位 UDP 目标端口固定为 `9005`；手柄遥控使用独立端口 `9010`。检测过程不会自动保存逐帧图片；仅在手动录像时保存 MP4。

## 定位配置

EXE 使用 `dist/config.yaml`。源码运行使用根目录 `config.yaml`。

- 固定 Tag：ID `1～4`，尺寸 `0.20 m`。
- 车载 Tag：当前 `vehicle_id: 0`。
- 场地：`4 m × 3 m`。
- 检测参数：当前使用稳定配置；若远端固定 Tag 仍不稳，优先增大 Tag 或改善安装/光照。
- 右下角基准点：`(X=300, Z=400)` mm；底边 `Z=400` mm，上边 `Z=3400` mm，`+X` 向左，`+Z` 向上。
- Yaw：`0° -> +X`，`+90° -> +Z`。
- 在场地图俯视方向：
  `0°` 向左、`+90°` 向上、`-90°` 向下、`±180°` 向右。
- `pose_filter` 对输出坐标和 Yaw 做轻量滤波；左上角 HUD、右上角 `UDP trajectory` 和 UDP 发送使用同一份滤波后的定位。

当前固定 Tag 临时中心坐标：

| Tag | 中心坐标 `(X, Z)` |
| --- | --- |
| `1` | `(300, 400)` mm |
| `2` | `(300, 3400)` mm |
| `3` | `(4300, 400)` mm |
| `4` | `(4300, 3400)` mm |

正式安装后必须将 `world_coordinates` 改为现场实测的 Tag 中心坐标。相机和固定 Tag 移动后应重新标定。

## UDP 输出

定位包：

```json
{"type":"robot_position","pos":[z,0,x],"euler":[0.0,yaw,0.0]}
```

Windows 定位内部、预览、滤波和轨迹记录仍使用 `(x,z)`；仅在 UDP 组包时交换为官方 AR 轴顺序 `[z,height,x]`。RK3588S 不再二次交换坐标，只把该 JSON 原样转发到 `127.0.0.1:9006`。坐标单位为米，Yaw 单位为度。目标地址应填写 RK3588S 的局域网 IP，不要填写 Windows 自己的 `127.0.0.1`。

手柄遥控包仅在 `Gamepad Mode` 勾选时发送，端口为 `9010`，类型为 `gamepad_control`。它与定位包 `9005` 是独立链路；没有定位数据时仍可控车。映射关系：

- `RT` 前进、`LT` 倒车：`target_speed=(RT-LT)*1.0 m/s`。
- `LX`：`track_error`，默认比例 `210`。
- `B` 键或手柄断连：`STATE_SAFE_STOP`。
- 手柄读取优先使用 XInput；蓝牙手柄若不是 XInput，会尝试 pygame/DirectInput 兜底。
- 取消 `Gamepad Mode`、遥控包超时或关闭程序时，RK3588S 会结束手柄控制并发送安全停车；新视觉控制链路接入后再统一控制源仲裁。

这条遥控链路只覆盖 TC264D 控制帧，不改变定位输出、AR 转发或 WebUI 端口。

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
