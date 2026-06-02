# 智能车视觉系统 - 项目状态

## 硬件平台

- **板卡**: Orange Pi 5 (RK3588)
- **NPU**: 3 核心，当前使用 Core 0（检测）+ Core 1（分割）
- **串口**: /dev/ttyS3, 波特率 460800
- **摄像头**: 通过共享内存 `shm_ar_video` 推流

## 模型

| 模型 | 文件 | 大小 | NPU核心 | 用途 |
|------|------|------|---------|------|
| 目标检测 | `rknn_lt.rknn` | 9.2MB | Core 0 | 检测 gold/car/human |
| 语义分割 | `pp_seg.rknn` | 9.3MB | Core 1 | 道路可行驶区域分割 |

## 文件结构

```
setupUI/
├── ar_receiver.py          # 主循环：共享内存读帧 → 双AI推理 → 串口控制
├── serial_comm.py           # 串口通信，向下位机发 track_error + target_speed
├── dataset_collector.py     # 数据集采集工具（SPACE截图 + WASD标注方向）
├── infer_wrap/
│   ├── __init__.py          # 导出 InferWrap, PPSegInfer
│   └── base/
│       ├── infer_wrap.py    # 检测模型包装器，加载 rknn_lt.rknn
│       ├── seg_infer.py     # 分割模型包装器，加载 pp_seg.rknn
│       ├── func.py          # 检测后处理（YOLO解码、NMS、绘图）
│       ├── seg_func.py      # 分割后处理（argmax、mask过滤、中线提取、track_error计算）
│       ├── rknnpool.py      # RKNN 推理池（多核心并行）
│       └── model/           # 模型文件目录
│           ├── rknn_lt.rknn
│           ├── pp_seg.rknn
│           └── coco.names   # 检测类别: gold, car, human
└── dataset_collector.py     # 数据集采集工具
```

## 数据流

```
摄像头 → 共享内存(shm_ar_video) → ar_receiver.py
                                        │
                    ┌───────────────────┼───────────────────┐
                    ▼                   ▼                   ▼
              frame.copy()       frame.copy()         frame.copy()
                    │                   │                   │
                    ▼                   ▼                   ▼
            infer_seg.infer()   infer_det.infer()     final_frame
            (Core 1, 分割)      (Core 0, 检测)            │
                    │                   │                   │
                    ▼                   ▼                   ▼
            extract_centerline()    draw()            cv2.imshow()
            (mask→中线→error)   (画检测框)             显示画面
                    │                   │
                    ▼                   ▼
              final_frame 叠加       final_frame 叠加
                    │
                    ▼
            track_error → 串口 → 下位机（舵机控制）
```

## 接口

### 串口协议（→下位机）

| 字段 | 类型 | 说明 |
|------|------|------|
| header | 0x42 | 消息头 |
| cmd | 0x10 | 控制命令 |
| len | 10 | 负载长度 |
| track_error | float | 道路偏差（像素），×0.5 后发给下位机 |
| target_speed | float | 目标速度，当前固定 1.0 |
| state_cmd | uint8 | 状态，当前固定 1 |
| flags | uint8 | 标志位，保留 |
| checksum | uint8 | 校验和 |

### 共享内存（←摄像头）

| 字段 | 偏移 | 类型 | 说明 |
|------|------|------|------|
| fid | 0 | uint64 | 帧ID |
| w | 8 | uint32 | 图像宽度 |
| h | 12 | uint32 | 图像高度 |
| data | 16 | uint8[w*h*3] | RGB 图像数据(Y轴翻转) |

## seg_func.py 改进说明

> 当前版本相比原始代码的改动（仅 seg_func.py，其他文件未修改）

| 层 | 位置 | 改动 | 目的 |
|----|------|------|------|
| 量化偏置 | `_to_class_map` | road_score − 0.10 → argmax | 抑制 int8 量化噪声导致的误检 |
| 多帧投票 | `extract_centerline` | 4 帧 mask 多数投票 | 过滤单帧随机翻转的像素 |
| 底部锚定 | `_build_midline` | 底部行中心 EMA(0.80/0.20) 追踪 | 底部路径固定不抖 |
| 扫描加权 | `_build_midline` | 底部 75%锚→顶部 25%锚 | 底部锁死，顶部跟踪弯道 |
| 帧间融合 | `_build_midline` | 底部 70%旧→顶部 25%旧 | 底部稳定，顶部灵敏 |
| 误差计算 | `extract_centerline` | 底部 1/3 点加权平均 | 单点噪声被多点稀释 |
| 误差平滑 | `extract_centerline` | EMA 0.70/0.30 | 平衡稳定与响应 |
| 视觉反馈 | `extract_centerline` | 绿点标记加权平均位置 | 调试可视化 |

## 已知问题

| 问题 | 原因 | 解决方向 |
|------|------|----------|
| 分割抖动 | int8 量化噪声，边界像素每帧翻转 | 当前后处理已接近极限，根治需 PTQ/QAT 校准 |
| 非道路误检 | 同上，量化后道路类偏置 | bias correction 已缓解 |
| 底部路径不够稳 | 锚定+多帧投票已改善 | 可进一步调整参数 |

## 下一步

1. **短期**: 微调 seg_func.py 参数，找最佳平衡点
2. **中期**: 对 pp_seg.rknn 做 PTQ 校准（喂数百张典型路面图，不需重训练）
3. **长期**: 换 YOLOPv2 或 PP-LiteSeg + QAT 量化感知训练

## 运行命令

```bash
# 主程序（AI 推理 + 控制）
python3 setupUI/ar_receiver.py

# 数据集采集
python3 setupUI/dataset_collector.py
```
