# 多任务 RKNN 板端放置说明

此目录只使用新单 RKNN 多任务模型。旧的 `rknn_lt.rknn` 不会被自动选择。

## 模型文件名

- FP16：`multitask_ppyoloe_fp16.rknn`
- INT8：`multitask_ppyoloe_int8.rknn`

默认加载 FP16。选择 INT8：

```bash
export MULTITASK_RKNN_VARIANT=int8
```

也可直接指定绝对路径：

```bash
export MULTITASK_RKNN_MODEL=/path/to/model.rknn
```

## 固定模型契约

- 输入：RGB uint8，`[1,480,640,3]`；RKNN 内部执行训练时的均值方差归一化。
- 输出 0：`det_boxes [1,6300,4]`，模型输入坐标中的 xyxy。
- 输出 1：`det_scores [1,6300,8]`，NMS 前类别概率。
- 输出 2：`pixel_logits [1,2,120,160]`，道路和候选中线。
- 输出 3：`topology_logits [1,4]`，normal/fork/merge/unknown。

检测类别顺序由 `coco.names` 固定，不可自行交换。

## 运行参数

```bash
export MULTITASK_DET_THRESHOLD=0.25
export MULTITASK_NMS_THRESHOLD=0.60
export MULTITASK_ROAD_THRESHOLD=0.50
export MULTITASK_CENTERLINE_THRESHOLD=0.25
export MULTITASK_CENTERLINE_MIN_POINTS=8
export MULTITASK_CENTERLINE_MAX_PEAKS_PER_ROW=3
export MULTITASK_CENTERLINE_MAX_JUMP=8
export MULTITASK_CENTERLINE_ROAD_FLOOR=0.10
export MULTITASK_CENTERLINE_FIT_BLEND=0.45
export MULTITASK_TOPOLOGY_THRESHOLD=0.45
```

智能车默认创建 3 个 RKNN 运行时并分别绑定 RK3588S 的 3 个 NPU 核：
`MULTITASK_RKNN_TPES=3`、`MULTITASK_PIPELINE_DEPTH=3`。这样会保持 3 帧并行推理，
结果大约落后 2 帧；如需最低延迟可将两个参数同时设为 1。

## Python 结果接口

`InferWrap.infer(frame)` 返回 `(result, ready)`。`result` 包含：

- `frame`：绘制检测框、候选中线和拓扑摘要后的 BGR 图。
- `detections`：NMS 后检测列表，坐标已映射到原图。
- `ocr_frame`：检测到 `TurnSign` 时提供给现有 OCR 的未绘制原图。
- `road`：`probability`、二值 `mask`、阈值和 stride。
- `centerline`：热图、道路软约束得分，以及长度达标且平均置信度最高的两条 `paths`。
  路径解码会限制横向跳变和空白行续接，峰值使用邻域加权中心，最终路径使用置信度
  加权二次拟合平滑；这些处理均为单帧空间处理，不增加帧间延迟。
- `topology`：类别、置信度、是否达到可靠阈值及四类概率。

NMS、道路阈值、中线峰值/动态规划和拓扑解析都在 CPU，不进入 RKNN 图。
