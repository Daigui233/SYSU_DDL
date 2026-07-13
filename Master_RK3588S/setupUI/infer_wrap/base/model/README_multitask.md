# Multi-task RKNN model contract

Place the trained six-output RKNN model here as one of:

- `multitask_ppyoloe_int8.rknn` (default)
- `multitask_ppyoloe_fp16.rknn`

Select it with `MULTITASK_RKNN_VARIANT=int8|fp16`, or set an absolute path
with `MULTITASK_RKNN_MODEL`.

## Fixed I/O contract

Input is RGB uint8 `[1,480,640,3]`. The RKNN conversion configuration owns
the training normalization. Do not normalize again on the board.

The RKNN must return these tensors in this exact order:

1. `det_boxes [1,6300,4]`: decoded xyxy boxes in the 640x480 input space.
2. `det_scores [1,6300,8]`: class probabilities; do not apply sigmoid.
3. `pixel_logits [1,3,120,160]`: road, single/left, and right logits.
4. `path_points [1,2,32,2]`: normalized `(x,y)` B-spline points.
5. `path_scores [1,2]`: sigmoid probabilities; do not apply sigmoid again.
6. `path_count_scores [1,3]`: softmax probabilities for 0/1/2 paths; do
   not apply softmax again.

`pixel_logits` may be returned as `[1,120,160,3]`; the post-processor also
accepts that layout. INT8 outputs must be requested as dequantized float
tensors from RKNNLite.

Path slots have conditional roles:

- count 0: both slots are invalid.
- count 1: slot 0 is `single`.
- count 2: slot 0 is `left`, slot 1 is `right`.

The board maps `path_points` directly to the camera frame. It does not scan,
skeletonize, reconnect, or fit the path heatmaps.

## Runtime tuning

```bash
export MULTITASK_RKNN_VARIANT=int8
export MULTITASK_NPU_MODE=parallel
export MULTITASK_RKNN_TPES=3
export MULTITASK_PIPELINE_DEPTH=3
export MULTITASK_OPENCV_THREADS=2
export MULTITASK_RKNN_WARMUP=1
export MULTITASK_DET_THRESHOLD=0.25
export MULTITASK_NMS_THRESHOLD=0.60
export MULTITASK_PRE_NMS_TOP_K=1000
export MULTITASK_MAX_DETECTIONS=100
export MULTITASK_COIN_MIN_SHORT_SIDE=10
export MULTITASK_ROAD_THRESHOLD=0.50
export MULTITASK_RENDER_DET_THRESHOLD=0.45
export MULTITASK_RENDER_MAX_DETECTIONS=6
export MULTITASK_RENDER_MAX_PER_CLASS=2
export MULTITASK_RENDER_PATH_MIN_SCORE=0.35
export MULTITASK_RENDER_PATH_MIN_COUNT_CONFIDENCE=0.40
export MULTITASK_PATH_TEMPORAL_FILTER=1
export MULTITASK_PATH_EMA_ALPHA=0.45
export MULTITASK_PATH_COUNT_CONFIRM_FRAMES=2
export MULTITASK_PATH_LOST_HOLD_FRAMES=3
export MULTITASK_PATH_MAX_JUMP_RATIO=0.15
export MULTITASK_PATH_RESET_GAP_SECONDS=0.50
```

Temporal filtering runs after the FIFO returns results in frame order. It is
causal, processes at most 64 points, and does not wait for future frames. A
path-count change must persist for two frames; a zero-path result must persist
for three frames. Large one-frame jumps are held briefly, while a persistent
new path is accepted and reinitialized. `raw_paths` and `raw_path_count` remain
available for model comparison and threshold tuning. A camera or inference gap
longer than 0.5 seconds resets stale path history.

`MULTITASK_COIN_MIN_SHORT_SIDE=10` mirrors the training dataset rule: Coin
detections whose short side is below 10 pixels in the fixed 640x480 model input
are discarded before NMS. Set it to `0` only for a deliberate tiny-Coin test.

NPU workers only run preprocessing and `rknn_lite.inference()`. CPU NMS and
path decoding run after the FIFO returns the oldest result, overlapping with
the next NPU job. OpenCV defaults to two shared CPU threads to avoid nesting
eight OpenCV threads inside each of the three worker threads.

`MULTITASK_NPU_MODE=parallel` is the default throughput mode: three RKNN
runtimes are pinned to core 0/1/2. `all_cores` uses one runtime with the
0+1+2 core mask and forces pipeline depth 1; compare it with the final model
when minimum single-frame latency matters.

Startup performs one black-frame inference on every RKNN runtime by default.
This warms the runtime, validates all six outputs before the live loop starts,
and prints each runtime's first inference time. Disable it only for diagnosis
with `MULTITASK_RKNN_WARMUP=0`.

After startup, inspect live timing without enabling video drawing:

```bash
curl -s http://127.0.0.1:9105/api/preview | \
  python3 -m json.tool
```

The `perception.timings_ms` object separates NPU inference, FIFO wait, and CPU
postprocess. Temporal filtering is disabled.

Debug rendering is performed by the drop-frame preview thread, not by an NPU
worker. Select it with:

```bash
export MULTITASK_RENDER_MODE=heatmap  # off, heatmap, drive, debug, or full
export MULTITASK_ROAD_OVERLAY_ALPHA=0.28
export MULTITASK_PATH_HEATMAP_ALPHA=0.45
export MULTITASK_PATH_HEATMAP_THRESHOLD=0.25
```

- `off`: no perception drawing.
- `heatmap`: diagnostic default. Draw every post-NMS detection as a full box,
  overlay both raw sigmoid path heatmaps, and draw the centerline traced from
  the current frame only.
- `drive`: low-clutter mode. No road fill or labels; paths are thin, Coin uses
  a dot, and other detections use corner marks.
- `debug`: drive mode plus the binary road overlay.
- `full`: road, heatmaps, up to 20 detections, all 32 path points, roles, and
  confidence text.

Drive rendering is separate from perception output. OCR and control still
receive every post-NMS detection even though preview drawing defaults to at
most six detections, two per class. Legacy `path` and `road` mode names map to
`drive` and `debug`.

Set `AR_LOCAL_PREVIEW=0` to skip full-frame rendering. Heatmap decoding still
runs because `paths` now comes from the per-frame heatmaps. Three RKNN runtimes
remain in flight by default, one on each RK3588S NPU core.

## Python result

The result exposes both the unified top-level fields and the existing nested
`road`/`centerline` compatibility fields:

- `detections`: class id/name, score, and original-image `box_xyxy`.
- `road_probability`: `[120,160]` float32.
- `road_mask`: `[120,160]` uint8.
- `path_heatmaps`: raw sigmoid probabilities with shape `[2,120,160]`.
- `path_count` and `path_count_scores`.
- `paths`: current-frame row-peak traces decoded from the heatmaps.
- `curve_paths`: the model's direct `[32,2]` B-spline outputs for comparison.
- `timings_ms`: preprocess, RKNN inference, FIFO wait, postprocess, and total
  latency visible to the perception consumer.

The centerline uses the previous model's row-local-peak and adjacent-row
connection method independently on each active heatmap slot. There is no EMA,
hold, hysteresis, cross-frame jump rejection, or any other temporal state.

The old `rknn_lt.rknn` and old four-output models are incompatible and fail
fast instead of producing incorrect paths.

`ar_receiver.py` holds `/tmp/sysu_ddl_ar_receiver.lock`. A repeated launch asks
the existing process to open its preview and then exits. Failure to bind the
optional preview-control port only disables that HTTP endpoint; inference and
the local preview continue to run.
