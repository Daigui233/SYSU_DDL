# Windows Tools

## gamepad_mapper

`gamepad_mapper.py` / `dist/gamepad_mapper.exe` 是 Windows 手柄映射测试工具，用于在真正遥控小车前确认手柄输入是否正常。

当前用途：

- 读取 Windows 手柄输入，优先使用 XInput，必要时可切换 pygame 后端；定位 EXE 也已加入 pygame/DirectInput 兜底。
- 实时显示摇杆、扳机和按钮状态。
- 预览车辆控制映射：`target_speed`、`track_error`、安全停车触发和未来车辆状态。
- 保存本地测试映射到 `gamepad_mapping.json`。

默认映射：

- 右扳机 `RT`：控制 `target_speed`，满开对应设置的最大速度。
- 左摇杆横轴 `LX`：控制 `track_error`，用于手柄转向。
- 左扳机 `LT`：倒车输入；实际定位 EXE 中与 `RT` 合成为 `target_speed=(RT-LT)*1.0 m/s`。
- `B` 键：安全停车触发，进入 `STATE_SAFE_STOP`。
- 默认转向误差比例为 `210`，基本覆盖当前舵机限幅，同时比 `300` 更柔一些，适合采集数据时稳定操作。

这个工具只负责测试和保存映射，不直接发送 UDP，也不直接控车。真正的遥控链路集成在 Windows 定位 EXE 里：勾选 `Gamepad Mode` 后，定位程序会向 RK3588S 的 `9010` 端口发送 `gamepad_control` 包；取消勾选或遥控包超时后，RK3588S 自动回到视觉控车。

## 打包 EXE

在工程根目录或本目录运行：

```bat
Tools_Windows\build_gamepad_mapper_exe.bat
```

生成文件：

```text
Tools_Windows\dist\gamepad_mapper.exe
```

如果提示缺少 PyInstaller，先安装：

```bat
python -m pip install pyinstaller
```
