## LabelMe 标注 SOP

所有图片使用 LabelMe 标注。原图不得缩放、裁剪、加框或覆盖保存。图片与 JSON 必须同名，例如 `000001.jpg` / `000001.json`。训练以 JSON 为准，预览图只用于验收。

## 一、标注标签

### 1. 赛道区域

- 标签：`track`
- 类型：Polygon
- 只使用这一个赛道类别。
- 普通路、分岔、汇入处都标出全部可行驶区域。
- 障碍物遮挡处不要在赛道 mask 中挖洞。

### 2. 中线

- 标签：`centerline`
- 类型：LineStrip
- 所有中线只用这一个标签。
- 普通路画一条；分岔/汇入时把可见候选路径都画出来。
- 不区分左/右、内/外圈、`line1/line2`。
- 公共路段允许多条中线重合。

### 3. 目标检测框

- 类型：Rectangle
- 类别只能使用以下 11 类，大小写必须完全一致：

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

要求：画面中符合标准的目标必须全部标注；禁止同义词、中文标签、临时新增类别。

## 二、整图状态和任务开关

每张 JSON 的 `flags` 必须包含两类信息。

### 1. 拓扑状态，四选一

```text
topology_normal   单一路径
topology_fork     前方出现分岔
topology_merge    多条路径正在汇入
topology_unknown  看不清或无法判断
```

四个字段必须且只能有一个为 `true`。如果某组数据不要求判断拓扑，也仍然保留这四个字段，并统一填 `topology_unknown=true`，同时把 `has_topology=false`。

### 2. 任务开关

```text
has_det       是否完成目标检测标注
has_road      是否完成赛道分割标注
has_path      是否完成中线标注
has_topology  是否完成拓扑状态标注
```

注意：`has_det=true` 且没有目标框，表示人工确认无目标；`has_det=false` 且没有目标框，表示未做目标检测标注，训练时不能当成无目标。

## 三、三类数据

### A 类：完整标注

包含：目标框 + `track` + `centerline` + 拓扑状态。

```json
"has_det": true,
"has_road": true,
"has_path": true,
"has_topology": true
```

### B 类：仅目标检测

只画目标 Rectangle，不画赛道和中线。

```json
"topology_normal": false,
"topology_fork": false,
"topology_merge": false,
"topology_unknown": true,

"has_det": true,
"has_road": false,
"has_path": false,
"has_topology": false
```

B 类不需要人工判断 `normal/fork/merge`。这里的 `topology_unknown=true` 只是为了保持 JSON 结构统一，训练时不会计算拓扑 loss。

### C 类：赛道 + 中线

包含：`track` + `centerline` + 拓扑状态，不需要目标框。

```json
"has_det": false,
"has_road": true,
"has_path": true,
"has_topology": true
```

旧数据复用时：原 `track` 保留，原 `fork` Polygon 统一改成 `track`，再补 `centerline` 和 `flags`。旧 Polygon 标成 `fork` 不等于整图一定是 fork，拓扑状态必须重新人工判断。

## 四、JSON 示例

```json
{
  "version": "5.0.1",
  "flags": {
    "topology_normal": false,
    "topology_fork": true,
    "topology_merge": false,
    "topology_unknown": false,

    "has_det": true,
    "has_road": true,
    "has_path": true,
    "has_topology": true
  },
  "shapes": [
    {
      "label": "track",
      "points": [[120, 430], [220, 260], [420, 240], [600, 430]],
      "group_id": null,
      "shape_type": "polygon",
      "flags": {}
    },
    {
      "label": "centerline",
      "points": [[310, 430], [330, 360], [360, 300], [410, 250]],
      "group_id": null,
      "shape_type": "linestrip",
      "flags": {}
    },
    {
      "label": "centerline",
      "points": [[310, 430], [280, 360], [230, 300], [180, 250]],
      "group_id": null,
      "shape_type": "linestrip",
      "flags": {}
    },
    {
      "label": "Car",
      "points": [[450, 210], [520, 280]],
      "group_id": null,
      "shape_type": "rectangle",
      "flags": {}
    }
  ],
  "imagePath": "000001.jpg",
  "imageHeight": 480,
  "imageWidth": 640
}
```

## 五、交付内容

- 未修改的原图
- 同名 LabelMe JSON
- 分割预览图
- 检测框预览图
- 每类标签数量统计
- `normal/fork/merge/unknown` 数量统计
- 各目标检测类别实例数量统计
