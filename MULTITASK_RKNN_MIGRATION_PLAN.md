# Paddle 三头多任务感知与 RKNN 异步流水线迁移计划

## 1. 文档目标

本计划用于指导当前 RK3588S 感知系统从“多个独立 RKNN 模型并行运行”迁移到“单个共享骨干、三个轻量任务 Head 的 RKNN 模型”，并将主循环重构为 latest-only 的异步采图、推理、控制和调试渲染流水线。

本计划只重构感知与运行调度，不改变以下系统边界：

- AprilTag/AR 定位只用于 AR 融合、HUD、记录和 WebUI 显示，不参与控车决策、`track_error` 或路径规划。
- 上位机状态机继续负责任务决策。
- 局部规划器继续负责生成最终路径与 `final_track_error`。
- TC264D 继续负责电机、舵机控制、固定 PID、硬限幅和安全保护。
- RK3588S 到 TC264D 的现有串口协议不因模型迁移而改变。

## 2. 当前问题与迁移动机

当前系统同时运行目标检测、语义分割和 fork 分类 RKNN 模型。实测表明：

- AR 原始输入 `RAW FPS` 通常可达到 40-50 FPS，视频源不是主要瓶颈。
- 普通场景下检测模型约消耗 30-40 ms，已经限制稳定 30 FPS。
- fork 场景会同时触发密集 mask 扫描、左右 mask 切分、重复中线计算和 fork 分类。
- fork 场景的分割路径后处理可超过 150 ms。
- 多个 RKNN 模型虽然绑定不同 NPU 核心，但仍共享 NPU 调度、DDR 和数据通路；NPU 满载时检测推理可能从约 30 ms 上升到约 100 ms。
- debug 渲染和 JPEG 编码运行在独立线程，但仍会争用 CPU 和内存带宽。

迁移目标不是继续优化多模型并行，而是从计算源头减少重复骨干、重复预处理、重复 NPU 调度和复杂 fork 后处理。

## 3. 官方约束

根据《智能汽车 AI 模型组-小白入门手册》：

- 深度学习框架必须使用 PaddlePaddle。
- 模型结构应参照官方 PPYOLOe 和 PPSeg Baseline。
- 训练平台推荐百度 AIStudio。
- 模型最终需要转换为 RKNN 并部署到 X-01/RK3588S 板卡。
- RKNN 图内不应包含不受支持的 NMS，检测 Head 输出原始张量，在 CPU 侧完成轻量解码。
- 正式比赛使用 setupUI 下的 AR 融合视频流。

正式投入开发前，需要向赛事方确认“基于 PPYOLOe/PPSeg 组件构建共享骨干多任务模型”是否满足“参照官方 Baseline”的规则要求。

## 4. 目标模型架构

目标模型为单个 PaddlePaddle 多任务网络：

```text
输入图像
   |
   v
共享轻量 Backbone + Neck
   |----------------------------|
   |                            |
   v                            v
轻量检测分支                    共享轻量特征
   |                            |-------------|
   v                            v             v
Detection Head            Road Head      Direct Path Head
bbox/class/score           road_mask      paths/topology
```

三个逻辑 Head：

1. Detection Head：输出赛道目标的类别、置信度和检测框。
2. Road Head：输出二值可行驶赛道 mask。
3. Direct Path Head：优先直接输出最多两条候选路径的固定行锚点分布、路径有效性以及 `NORMAL/FORK/MERGE` 拓扑分类。

推荐以官方 PPYOLOe Baseline 的检测训练链路为主体，复用其 Backbone/Neck 和检测 Loss，再增加 Road Head 与 Direct Path Head。Direct Path 方案作为第一尝试：它把路径分离和拓扑判断尽量放入模型，RK3588S CPU 侧只做少量 `argmax`、排序和插值。原高斯热力图 Centerline Head 保留为同标注、可回退的第二方案；Direct Path 验证失败时不需要重新标注数据。

## 5. 为什么选择三头而不是全语义分割双头

当前目标中存在 `Coin`、`TrafficLight`、`BeginSign`、`EndSign`、`SpeedSign` 和 `TurnSign` 等小目标。全部改为低分辨率语义分割会带来：

- 小目标像素过少，召回率容易下降。
- 多个同类目标可能粘连，实例难以分离。
- 需要为所有目标绘制精细 Polygon，标注成本较高。
- 运行时仍需连通域分析才能恢复状态机需要的 bbox。
- 多类别全图密集输出可能增加输出带宽和 CPU 处理量。

三头模型仍然只执行一次共享骨干推理。需要避免的是多个独立 RKNN 模型，不是一个 RKNN 模型包含多个轻量 Head。

## 6. 数据标注规范

每张原图包含三类人工标注。

### 6.1 Road Polygon

- 标注完整可行驶赛道区域。
- 普通道路、岔路和汇入区域都标注全部可行驶分支。
- Road Head 与 Detection Head 相互独立，因此道路允许在障碍物后方连续标注。
- 无法可靠判断的区域使用 `ignore`，不要人工猜测边界。

### 6.2 Object Bounding Box

初始类别与当前系统保持一致：

```text
Door
SpeedSign
TurnSign
Stone
BeginSign
EndSign
Crosswalk
TrafficLight
Coin
Human
Car
```

标注要求：

- bbox 紧贴目标可见范围，避免大量背景。
- 遮挡目标增加 `occluded` 属性。
- 无法可靠辨认的小目标设为 `ignore`。
- `Door` 与其下方 `BeginSign/EndSign` 允许 bbox 重叠。
- `TrafficLight` 增加 `color=red/green/unknown` 属性，为后续颜色分类保留数据。
- `SpeedSign` 和 `TurnSign` 保留文字或方向属性，为后续 OCR/API 使用。

### 6.3 Centerline Polyline

- 所有候选中线统一使用同一个 `centerline` 标签，不定义永久的 `line1/line2`、内环/外环语义。
- 普通道路标注一条完整候选中心线。
- 岔路标注两条完整候选路径，公共段允许重合。
- 汇入标注两条进入路径，汇入后的公共段允许重合。
- 点按照车辆前进方向排列，从画面近处指向远处。
- 中线表示无障碍情况下的道路基准路径，不因 Human、Car 或 Stone 临时绕行。
- 中线允许穿过障碍物，绕行由上位机状态机和局部规划器决定。

Polyline 只作为分辨率无关的原始标注保存。第一方案在训练前把每条折线重采样为固定纵向行上的横向格子、有效性 mask 和拓扑标签；第二方案才把相同折线栅格化为软高斯热力图。禁止要求外包分别制作两套中线标注，也禁止使用人工绘制的 1 px 硬线作为最终训练标签。

拓扑标签可由折线关系自动生成：一条路径为 `NORMAL`；两条路径近处重合、远处分开为 `FORK`；近处分开、远处重合为 `MERGE`。复杂样本允许在 LabelMe 图片级 `flags` 中人工复核 `normal/fork/merge`，但中线 shape 标签仍统一为 `centerline`。

### 6.4 数据组织

推荐所有人工标注统一保存到一个 JSON，其他训练标签自动生成：

```text
dataset/
|-- images/
|-- annotations/       # Polygon + bbox + Polyline 原始 JSON
|-- generated/
    |-- road_masks/    # 自动生成二值/ignore mask
    |-- path_targets/  # 第一方案：固定行锚点、有效性和拓扑标签
    |-- center_maps/   # 回退方案：自动生成 float 热力图
    |-- det_labels/    # 自动生成 Paddle 检测标签
```

数据集必须按照完整视频或采集场次划分 train/val/test，禁止把相邻帧随机拆分到不同集合造成数据泄漏。

## 7. 输入输出尺寸

优先测试保持 4:3 画幅的输入尺寸：

```text
候选 A：512 x 384
候选 B：640 x 480
```

两种尺寸均可被 32 整除，不需要把 4:3 图像拉伸为 640 x 640。

Road Head 建议输出步长为 4：

```text
512 x 384 输入 -> 128 x 96 输出
640 x 480 输入 -> 160 x 120 输出
```

Direct Path Head 使用固定数量的纵向行锚点与横向分类格子，第一版建议：

```text
path_slots = 2
row_anchors = 8 或 12
x_bins = 64 或 128
topology_classes = 3  # NORMAL/FORK/MERGE
```

每个路径槽位额外包含 `no-point/invalid` 状态。推理后再根据分岔后的横向位置把候选排序为 `LEFT/RIGHT`，训练标注不承担左右身份。

若 Direct Path 方案未达到复杂弯道和岔路验收指标，则切换到 stride=4 的 Centerline Heatmap，仍使用同一套 Polyline JSON。

最终尺寸必须通过以下指标共同决定：

- 小目标召回率。
- 中线像素误差和连续性。
- Paddle、ONNX 和 RKNN 单帧延迟。
- RK3588S DDR 和 CPU 后处理负载。

## 8. Loss 设计与梯度平衡

总 Loss：

```text
L_total = lambda_det * L_det
        + lambda_road * L_road
        + lambda_path * L_path
        + lambda_topology * L_topology
```

初始参数：

```text
lambda_det  = 1.0
lambda_road = 1.0
lambda_path = 1.0
lambda_topology = 1.0
```

各任务 Loss：

- Detection：复用官方 PPYOLOe 检测 Loss。
- Road：BCE/CE 与 Dice 组合，支持 ignore 区域。
- Direct Path：固定行横向格子的 CrossEntropy、路径有效性 Loss 与拓扑 CrossEntropy。
- 两个路径槽位采用排列无关匹配：计算 `预测0->标注A/预测1->标注B` 与交换后的两种 Loss，取较小值，避免槽位身份切换污染训练。

热力图回退方案继续使用 Focal-MSE 或带正样本增强的 MSE，并显式暴露：

- `lambda_heat`，默认 75，允许在 50-100 范围调节。
- `heat_positive_weight`，提高高斯中心区域权重。
- `heat_focal_gamma`，强调难预测像素。
- `heat_valid_mask`，排除 ignore 区域。

训练日志必须分别记录：

- Detection、Road、Path、Topology 的原始 Loss。
- 上述各项的加权 Loss 与总 Loss。
- 三个任务对共享 Backbone 的梯度范数。
- 可选的任务梯度余弦相似度。

不预设三任务必然发生梯度冲突。只有在验证集指标和梯度日志证明持续冲突时，才引入 GradNorm、PCGrad 或分阶段冻结，避免训练方案过度复杂。

推荐训练顺序：

1. 使用官方权重初始化 Backbone/Neck。
2. 先训练 Road + Direct Path Head，确认单路径、双路径和拓扑分类能够收敛。
3. 加入 Detection Head，短期冻结部分 Backbone。
4. 使用较小学习率全模型联合微调。

## 9. Direct Path 第一方案与热力图回退

### 9.1 Direct Path 第一方案

标注转换器将每条 `centerline` 折线与固定纵向行求交，把横坐标量化到 `x_bins`。普通道路生成一个有效路径槽位；岔路和汇入生成两个目标路径；缺失交点写入 `no-point` 并通过 valid mask 排除。

模型输出一个固定结构的 `path_output`，逻辑上包含：

```text
path_logits[2, K, x_bins + 1]
path_valid[2]
topology_logits[3]
```

为保持 RKNN 输出契约简单，导出时可把上述字段按固定顺序打包为一个张量。CPU 后处理只执行：

1. 对每个路径槽位和行锚点执行 `argmax`。
2. 删除 `no-point`，按有效点拟合或分段插值。
3. 在 `FORK` 时按分岔后横向位置排序为 `LEFT/RIGHT`。
4. 根据状态机意图锁定一条路径；另一条候选不进入转向误差计算。
5. 使用短时滞回和置信度门限稳定 `NORMAL/FORK/MERGE` 状态。

该方案不执行全图扫描、骨架化、连通域或热力图峰值连接。目标 CPU 耗时：平均小于 0.5 ms，P95 小于 1 ms。

适用前提：车辆前视图中的候选路径能够在绝大多数有效采样行上表示为单个横向位置。若连续急弯、遮挡或非单调投影导致这一表示频繁失效，则进入热力图回退评估。

### 9.2 Centerline Heatmap 回退方案

Centerline Heatmap 禁止执行全图二维峰值搜索、骨架化或连通域分析。

固定归一化采样行：

```text
scan_ratios = [0.82, 0.68, 0.54, 0.40]
```

处理过程：

1. 根据输出高度把比例转换为固定行号。
2. 只读取这些行的一维 Logits/概率数组。
3. 使用手写一维局部极大值检测，每行最多保留两个峰。
4. 根据相邻行的横向距离连接成一条或两条候选路径。
5. 水平行双峰只能作为候选信号，不能单独判定岔路；必须结合从车辆近处开始的方向连续性和共享前缀，避免单条弯线与同一水平行相交两次造成误判。
6. 根据连通方向和双峰从近到远的出现/消失顺序区分 `FORK` 和 `MERGE`。
7. 使用二次加权拟合或分段线性插值生成平滑路径。
8. 只对最终候选路径点做一次轻量 EMA，不对误差重复滤波。

目标耗时：平均小于 1 ms，P95 不超过 2 ms。

## 10. Road 与检测后处理

Road Head：

- 在低分辨率输出上完成阈值化和质量判断。
- 控制线程不把 mask 放大到原图尺寸。
- 仅 HUD/调试线程需要时执行一次最近邻放大。
- Road mask 只作为路径安全约束，不再用于复杂 fork mask 切分。

Detection Head：

- NMS 不放入 Paddle/RKNN 计算图。
- 优先复用官方 PPYOLOe 的已验证 CPU 解码方式。
- 后续如 CPU 解码仍重，再评估中心点 + 宽高 + Top-K 的轻量检测 Head。
- 输出适配当前 `class_name/confidence/bbox/center/bottom_ratio` 契约。

## 11. 单 RKNN 推理封装

RKNN 模型一次推理输出三个逻辑结果：

```text
detection_raw
road_logits
path_output          # 第一方案：路径槽位 + valid + topology
```

若 Direct Path 验证失败，第三个逻辑输出替换为 `centerline_logits`。两种模型都保持单 RKNN、三个逻辑输出；部署适配器根据模型契约选择解析器，不影响状态机上层的 `candidate_paths/road_topology` 接口。

实施要求：

- 只创建一个 RKNNLite 上下文。
- 同一帧只进行一次颜色转换、resize 和输入拷贝。
- 输出张量顺序和 shape 写入明确的模型契约文档。
- Paddle、ONNX 和 RKNN 三端使用同一组测试图片比较数值。
- INT8 校准集必须覆盖普通道路、岔路、汇入、遮挡、小目标和不同曝光。
- Direct Path 量化后需要专项验证横向格子、路径有效性和拓扑分类；热力图回退模型则验证峰值位置和置信度。必要时对最终输出层使用混合量化或保留更高精度。
- 分别测试单 NPU 核与多核 core mask，不假设多核一定更快。

## 12. 异步 latest-only 流水线

目标线程结构：

```text
AR 共享内存采图线程
        |
        v
capture_queue(maxsize=1)
        |
        v
单 RKNN 推理线程
        |----------------------|
        v                      v
control_queue(maxsize=1)  render_queue(maxsize=1)
        |                      |
        v                      v
状态机/规划/串口线程       HUD/MJPEG 调试线程
```

统一帧载荷：

```python
@dataclass
class FramePayload:
    frame_id: int
    capture_ts: float
    raw_image: np.ndarray
    detection_raw: object | None = None
    detections: list | None = None
    road_mask: np.ndarray | None = None
    centerline_peaks: list | None = None
    candidate_paths: list | None = None
    inference_ms: float | None = None
    result_ts: float | None = None
```

队列规则：

- 所有实时数据队列固定 `maxsize=1`。
- Producer 使用 `put_nowait()`。
- 捕获 `queue.Full` 后显式 `get_nowait()` 丢弃旧包，再放入最新包。
- 推理线程不维护历史帧队列。
- 控制和渲染使用推理完成后的同一个 `FramePayload`。
- 调试画面必须用 payload 内的 `raw_image` 绘制，禁止重新读取当前帧后叠加旧结果。

控制线程增加：

- `frame_id` 单调性检查。
- `capture_ts` 到控制时刻的结果年龄检查。
- 超龄结果拒绝执行，不用旧结果继续控车。
- 推理异常或长期无新结果时输出安全状态，下位机输入超时保护继续保留。

## 13. 与现有上位机接口的兼容

新感知模块继续输出当前结构化契约：

```text
perception
|-- timestamp
|-- frame_id
|-- frame_shape
|-- segmentation
|   |-- road_mask
|   |-- road_valid
|   |-- line_valid
|   |-- mid_points
|   |-- track_error
|   |-- road_topology
|   `-- candidate_paths
|-- detections
`-- timings_ms
```

保持以下模块职责不变：

- `control_task_state_machine.py` 继续从结构化感知生成任务状态和 planner intent。
- `control_local_planner.py` 继续根据候选路径和障碍物生成最终规划线。
- `control_arbitrator.py` 继续输出 `final_track_error` 和 `desired_speed`。
- `control_car_link.py` 和 TC264D 串口协议不变。
- AR 定位字段不得进入感知路径选择和规划计算。

模型输出的是道路基准候选线，不负责直接绕障。Human、Car、Stone 和 Coin 的路径决策仍由状态机和局部规划器完成。

## 14. 建议新增模块

训练工程建议独立于当前 RK 运行代码：

```text
Master_RK3588S/multitask_perception/
|-- configs/
|-- datasets/
|-- models/
|   |-- multitask_model.py
|   |-- road_head.py
|   |-- direct_path_head.py
|   `-- centerline_heatmap_head.py   # 回退方案
|-- losses/
|-- tools/
|   |-- convert_annotations.py
|   |-- generate_path_targets.py
|   |-- generate_center_heatmap.py   # 回退方案
|   |-- train.py
|   |-- evaluate.py
|   `-- export.py
`-- tests/
```

RK 部署侧建议新增：

```text
Master_RK3588S/setupUI/
|-- vision_multitask_engine.py
|-- vision_path_decoder.py
|-- vision_centerline_postprocess.py  # 回退方案
|-- vision_perception_adapter.py
`-- vision_async_pipeline.py
```

`ar_receiver.py` 仍只负责模块装配、生命周期和调度，不放置模型参数、Loss 参数或中线算法参数。

## 15. 分阶段实施计划

### Phase 0：规则确认与基线冻结

工作内容：

- 向赛事方确认 Paddle 三头融合模型合规性。
- 固化当前模型、数据、延迟和控制效果基线。
- 保存普通、岔路、汇入和小目标典型测试视频。
- 冻结当前串口协议和状态机接口。

通过条件：规则明确，基线视频和性能数据可重复。

### Phase 1：统一标注格式与转换工具

工作内容：

- 定义 JSON schema、类别 ID 和 ignore 规则。
- 实现 Polygon 到 road mask 转换。
- 实现 Polyline 到固定行路径目标、valid mask 和 `NORMAL/FORK/MERGE` 标签的转换。
- 同时保留 Polyline 到高斯热力图的转换工具，作为无需重标数据的回退路径。
- 实现 bbox 到 Paddle 检测标签转换。
- 实现标注可视化和一致性检查。

通过条件：随机抽取至少 100 张图，三种标签叠加检查无错位。

### Phase 2：Paddle 三头模型原型

工作内容：

- 从官方 PPYOLOe Baseline 建立可复现训练环境。
- 增加 Road Head 和 Direct Path Head，Direct Path 作为第一原型。
- 接入三任务 Dataset 和 Loss。
- 实现三任务独立指标和梯度日志。

通过条件：小数据集可以过拟合；检测与 Road 输出方向正确；Direct Path 能稳定区分单路径/双路径，并在人工样本上正确输出 `NORMAL/FORK/MERGE`。若该条件未满足，再启用 Centerline Heatmap 原型，不重新标注。

### Phase 3：正式训练与消融实验

至少比较：

- 512 x 384 与 640 x 480。
- Direct Path 的 8/12 个行锚点与 64/128 个横向格子。
- Direct Path 与 Centerline Heatmap 回退方案。
- 热力图回退时比较 `lambda_heat` 为 50、75、100，以及普通 MSE 与 Focal-MSE。
- 分阶段训练与直接联合训练。

通过条件：检测召回、Road 指标和中线指标同时达到最低门槛，没有某个任务明显崩溃。

### Phase 4：模型导出与 RKNN 转换

工作内容：

- Paddle 静态图导出。
- ONNX 输出节点和 shape 检查。
- RKNN INT8 转换。
- 三端逐图数值对齐。
- RK3588S 单模型延迟和资源测试。

通过条件：RKNN 输出结构稳定，精度下降在允许范围，单次推理达到性能预算。

### Phase 5：离线视频适配

工作内容：

- 实现 RKNN 单模型封装。
- 实现检测解码、Road 后处理和 Direct Path 解码；Direct Path 只做 `argmax`、valid 过滤、排序和插值。
- 保留热力图固定行/局部连通后处理作为可切换回退实现。
- 通过 adapter 生成现有 `perception` 数据。
- 使用历史比赛视频离线运行全部状态机和局部规划器。

通过条件：同一输入视频多次运行结果一致，无 frame_id 错位，状态机无需读取新模型私有字段。

### Phase 6：板卡 Shadow Mode

新模型在板卡运行并记录结果，但不下发控车命令。对比：

- 帧率和端到端结果年龄。
- 中线抖动和岔路候选路径。
- 目标召回和误检。
- Paddle/ONNX/RKNN 一致性。
- CPU/NPU/DDR 占用和温度。

通过条件：性能和感知质量达到验收门槛，连续运行无队列积压和内存增长。

### Phase 7：低速实车接管

工作内容：

- 手柄随时接管。
- 首先只启用 NORMAL_TRACK。
- 再逐个启用 Car、Human、Stone、Coin 和比赛事件状态。
- 每次只开放一个新行为，保留回退开关。

通过条件：低速连续多圈无过期结果控车、无分支跳变和无异常停车。

### Phase 8：旧链路退役

只有新模型通过完整实车验收后，才删除：

- 独立目标检测 RKNN 上下文。
- 独立 PPSeg RKNN 上下文。
- fork 分类 RKNN 上下文。
- 原有重型 mask 切分和重复中线扫描。

## 16. 性能预算

第一阶段目标：

| 项目 | 平均目标 | P95 目标 |
|---|---:|---:|
| 采图与预处理 | <= 3 ms | <= 5 ms |
| 单 RKNN 推理 | <= 30 ms | <= 40 ms |
| 检测 CPU 解码 | <= 4 ms | <= 7 ms |
| Road 后处理 | <= 1 ms | <= 2 ms |
| Direct Path 解码 | <= 0.5 ms | <= 1 ms |
| Heatmap 回退后处理 | <= 1 ms | <= 2 ms |
| 状态机 + 局部规划 | <= 2 ms | <= 4 ms |
| 控制结果年龄 | <= 60 ms | <= 80 ms |
| 控制循环帧率 | >= 25 FPS | 不持续低于 20 FPS |

debug 渲染和 JPEG 编码不计入控制热路径，并继续采用独立 latest-only 线程。

## 17. 感知验收指标

不能只看 mAP 或 mIoU，需要同时评估控车相关指标。

Detection：

- 各类别 Precision、Recall、mAP。
- 小目标 Recall。
- EndSign、TrafficLight 和 Human 的误检率。
- bbox 底部位置误差。

Road：

- Road IoU/Dice。
- 近车区域 Road Recall。
- 断裂率和异常全屏率。

Path/Centerline：

- Direct Path 固定行锚点上的横向像素误差和有效点准确率。
- 两个路径槽位的排列无关路径误差。
- 静止视频连续帧抖动。
- 中线连续率。
- 普通/岔路/汇入拓扑准确率。
- 分叉点纵向误差。
- 候选路径数量准确率。
- 热力图回退方案额外评估峰值位置、局部连通性和弯道误判率。

系统：

- `frame_id` 完全一致。
- 队列长度永远不超过 1。
- 无持续处理旧帧。
- 无模型异常导致串口继续发送陈旧命令。

## 18. 主要风险与缓解

### 18.1 多任务梯度失衡

缓解：Direct Path 分类 Loss、拓扑 Loss 分项记录，采用分阶段训练和梯度日志；热力图回退时使用高斯软标签、`lambda_heat=75` 和 Focal-MSE。只有确认冲突后再引入 PCGrad/GradNorm。

### 18.2 小目标精度下降

缓解：保留独立轻量检测 Head；优先保持高分辨率浅层特征；增加小目标采样和难例；比较 512 x 384 与 640 x 480。

### 18.3 RKNN 不支持算子或输出错误

缓解：优先使用官方 Baseline 已验证算子；NMS 保留在 CPU；尽早制作最小模型进行 Paddle -> ONNX -> RKNN 冒烟测试。

### 18.4 Direct Path 表达能力或 INT8 损失

缓解：校准集覆盖急弯、岔路、汇入、遮挡和不同曝光；比较量化前后的横向格子、valid 和 topology。若固定行锚点无法稳定表示复杂投影，直接使用同一 Polyline JSON 切换到 Centerline Heatmap；必要时对输出层使用混合量化或保留更高精度。

### 18.5 异步结果过期

缓解：`maxsize=1`、latest-only 丢帧、FramePayload、frame_id 校验、结果年龄阈值和 TC264D 输入超时保护。

### 18.6 一次改动范围过大

缓解：新旧感知通过 adapter 保持同一契约；先离线和 Shadow Mode；旧模型在最终验收前不删除；使用配置开关快速回退。

## 19. 回退策略

迁移期间保留两个感知后端：

```text
VISION_BACKEND=legacy
VISION_BACKEND=multitask
```

回退只切换感知实现，不修改状态机、规划器、串口或 TC264D 固件。每个 Phase 未达到通过条件时，不进入下一阶段。

## 20. 完成定义

满足以下条件才认为迁移完成：

- 使用 PaddlePaddle 训练并通过赛事规则确认。
- 单个 RKNN 模型稳定输出检测、Road 和 Path 三任务结果；若 Direct Path 未通过验收，则允许以同标注训练的 Centerline Heatmap 作为第三输出。
- RK3588S 上连续运行性能达到预算，无模型并行带宽争用。
- 路径解码不再执行全图重型 fork mask 切分；Direct Path 优先避免热力图扫描，回退方案也只允许轻量局部处理。
- 异步流水线不存在队列积压和图像/结果错位。
- 当前全部状态机可通过兼容感知契约继续工作。
- NORMAL_TRACK、Human、Car、Stone、Coin、TrafficLight、BeginSign/EndSign 等状态完成分阶段实车验证。
- 旧链路可以通过配置立即回退，直至新链路完成最终验收。
