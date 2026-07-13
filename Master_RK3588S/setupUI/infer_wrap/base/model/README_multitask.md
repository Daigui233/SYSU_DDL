# Multi-task RKNN model contract

Place the newly exported six-output RKNN model here as one of:

- `multitask_ppyoloe_int8.rknn` (default)
- `multitask_ppyoloe_fp16.rknn`

Select a model with `MULTITASK_RKNN_VARIANT=int8|fp16`, or override the
absolute path with `MULTITASK_RKNN_MODEL`.

## Fixed I/O contract

Input is RGB uint8 `[1, 480, 640, 3]`. Normalization is part of the RKNN
conversion configuration. The RKNN must return these outputs in this order:

1. `det_boxes`: `[1, 6300, 4]`, raw xyxy boxes at 640x480.
2. `det_scores`: `[1, 6300, 8]`, raw class scores before CPU NMS.
3. `pixel_logits`: `[1, 3, 120, 160]`, road, path-slot-0 heatmap,
   path-slot-1 heatmap.
4. `path_points`: `[1, 2, 32, 2]`, normalized B-spline points in `(x, y)`.
5. `path_scores`: `[1, 2]`, raw per-slot path-existence logits.
6. `path_count_scores`: `[1, 3]`, raw logits for zero, one, or two paths.

The board applies NMS only to detection. It thresholds road for the overlay,
then scales the model-provided B-spline points directly to the camera frame.
It does not rebuild paths from heatmap peaks and does not select a driving
branch. If the model predicts two paths, both are drawn and returned.

`rknn_lt.rknn` and old four-output models are incompatible with this code.
They fail fast rather than silently producing incorrect paths.

## Useful runtime variables

```bash
export MULTITASK_RKNN_VARIANT=int8
export MULTITASK_DET_THRESHOLD=0.25
export MULTITASK_NMS_THRESHOLD=0.60
export MULTITASK_ROAD_THRESHOLD=0.50
export MULTITASK_ROAD_OVERLAY_ALPHA=0.28
export MULTITASK_PATH_SCORE_FALLBACK_THRESHOLD=0.20
export MULTITASK_RKNN_TPES=3
export MULTITASK_PIPELINE_DEPTH=3
```

`MULTITASK_PATH_SCORE_FALLBACK_THRESHOLD` is only used when the count head
predicts zero paths. A one-path or two-path count result keeps its respective
candidate slots without additional filtering.
