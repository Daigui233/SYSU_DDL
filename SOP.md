## LabelMe 标注 SOP

所有图片使用 LabelMe 标注。原图不得缩放、裁剪、加框或覆盖保存。图片与 JSON 必须同名，例如：

```text
000001.jpg
000001.json
```

训练以 JSON 为准，预览图只用于验收。

## 一、标注内容

### 1. 赛道区域

- 标签：`track`
- 类型：Polygon
- 只使用一个赛道类别。
- 普通路、岔路、汇入处都标出全部可行驶区域。
- 障碍物遮挡处不要在赛道 mask 中挖洞。

### 2. 中线

- 标签：`centerline`
- 类型：LineStrip
- 所有中线只用这一类标签。
- 普通路画一条中线。
- 分岔或汇入时，把画面中可见的候选路径都画出来。
- 不区分 `line1/line2`、左线/右线、内圈/外圈。
- 公共路段允许多条中线重合。
- 中线表示无障碍情况下的基准行驶路径，不因为临时障碍物绕行。

### 3. 目标检测框

- 类型：Rectangle
- 类别名按我提供的固定类别表标注。
- 画面中符合标准的目标必须全部标注。
- 类别名必须完全一致，禁止同义词、中文英文混用或临时新增类别。

### 4. 整图拓扑状态

每张图必须额外标一个整图状态，四选一：

```text
normal   单一路径
fork     前方出现分岔
merge    多条路径正在汇入
unknown  看不清或无法判断
```

## 二、三类数据

### A 类：完整多任务标注，约 400 张

每张图包含：

```text
目标框 + track + centerline + topology
```

任务开关：

```json
"has_det": true,
"has_road": true,
"has_path": true,
"has_topology": true
```

### B 类：仅目标检测，约 900 张

只画目标 Rectangle，不画赛道和中线。

任务开关：

```json
"has_det": true,
"has_road": false,
"has_path": false,
"has_topology": false
```

注意：

- `has_det=true` 且没有目标框，表示人工确认这张图没有目标，是有效负样本。
- `has_det=false` 且没有目标框，表示没有做目标检测标注，训练时不能当成无目标。

### C 类：赛道 + 中线标注，约 900 张

可以复用旧赛道数据。

每张图包含：

```text
track + centerline + topology
```

不需要标目标框。

任务开关：

```json
"has_det": false,
"has_road": true,
"has_path": true,
"has_topology": true
```

旧数据处理规则：

- 原来的 `track` 保留。
- 原来的 `fork` Polygon 统一改成 `track`。
- 不重新绘制已经合格的赛道边界。
- 旧 Polygon 标成 `fork` 不等于整图一定是 fork，拓扑状态必须根据画面重新判断。

## 三、JSON 必须包含的字段示例

下面是完整标注样例。实际点位只作示意。

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

拓扑状态要求：

```text
topology_normal
topology_fork
topology_merge
topology_unknown
```

四个字段必须且只能有一个为 `true`。

## 四、交付内容

每组分别交付：

- 未修改的原图
- 同名 LabelMe JSON
- 分割预览图
- 检测框预览图
- 每种 shape 和标签数量统计
- `normal/fork/merge/unknown` 数量统计
- 各检测类别实例数量统计

