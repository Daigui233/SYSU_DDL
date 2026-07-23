# carTi 的 Keil / MSPM0G3507 使用说明

## 可以直接烧录

`carTi_MSPM0G3507.hex` 是从原 CCS 工程已有的 `Debug/carTi.out` 无损转换得到的 Intel HEX，目标器件是 MSPM0G3507，程序地址从 `0x00000000` 开始。只需要烧录、不需要修改源码时，优先使用这个文件。

- HEX SHA-256：`3C6DD64F361203E26F8563DB2D6B2C03AE7472BA26C24F595F6232C49046C969`
- BIN SHA-256：`2F08D491452545543D8A9C921CE65DB8E9EB92AF4A96825B1F96F640AC0E896D`

在 Keil 中选择 MSPM0G3507 和实际使用的调试器（CMSIS-DAP、J-Link、ULINK 或已安装支持的 XDS110），然后把该 HEX 下载到 Flash。烧录前请确认目标板确实是 MSPM0G3507，且不要启用整片擦除之外的配置区改写。

## 从源码编译

1. 安装 Keil MDK（Arm Compiler 6）及 `Texas Instruments MSPM0G1X0X_G3X0X_DFP` 设备包。
2. 安装 TI MSPM0 SDK。当前工程文件按本机已有的 `C:\ti\mspm0_sdk_2_01_00_03` 配置。
3. 打开 `carTi_MSPM0G3507.uvprojx`，执行 Rebuild。
4. 编译成功后，Keil 会生成 `Objects/carTi_MSPM0G3507.axf` 和 `Objects/carTi_MSPM0G3507.hex`。

如果 SDK 安装目录不同，请在 **Options for Target → C/C++ → Include Paths** 和 **Linker → Misc Controls** 中替换 SDK 路径。

原 `empty.syscfg` 由 SDK 2.04 生成，而本机现有 SDK 是 2.01。`ti_msp_dl_config_keil.c` 是仅供 Keil/SDK 2.01 编译的兼容副本：移除了旧版 `DL_TimerG_PWMConfig` 中不存在的三个 `isTimerWithFourCC` 初始化项，其余外设配置保持不变。若改用 SDK 2.04，请让项目直接编译 `../Debug/ti_msp_dl_config.c`，并把包含目录和 DriverLib 库改到 2.04。

## 硬件前提

- 芯片封装配置为 MSPM0G3507、LQFP-64。
- 系统时钟配置使用 40 MHz 外部高频晶振，并启用了 32.768 kHz 低频晶振；自制板没有这两个晶振时必须先修改 SysConfig。
- 软件有左、右两路直流电机闭环和两路正交编码器输入；它是双路差速驱动逻辑。

## 轮数判断

代码只提供 `leftMotor`、`rightMotor` 两个独立电机对象，两路速度 PID 和两组正交编码器；循迹转向也通过 `left = speed - correction`、`right = speed + correction` 完成。因此从软件可确定的是 **两路驱动（典型二驱差速小车）**。

这通常对应两个主动轮，外加一个不受控的万向轮。若机械底盘是四轮、同一侧两个电机并联到同一个驱动通道，这份程序也能带四个实体轮，但仍只能按左右两组控制，不能独立控制四轮。

电机和编码器信号如下：

| 通道 | H 桥 PWM 输出 | 编码器输入 |
|---|---|---|
| 左侧 | PA1 / TIMA0-CC1，PB13 / TIMG12-CC0 | PA2、PA8 |
| 右侧 | PA0 / TIMA0-CC0，PA22 / TIMG6-CC1 | PB14、PB15 |
