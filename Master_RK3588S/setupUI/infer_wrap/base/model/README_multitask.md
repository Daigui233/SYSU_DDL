# Multi-task RKNN model contract

Place the trained six-output RKNN model here as one of:

- `multitask_ppyoloe_int8.rknn` (default)
- `multitask_ppyoloe_fp16.rknn`

Select it with `MULTITASK_RKNN_VARIANT=int8|fp16|curve_best|multitask_best`,
or set an absolute path with `MULTITASK_RKNN_MODEL`. The included
`curve_best` variant is the Stage3 row-classifier curve checkpoint.

## Fixed I/O contract

Input is RGB uint8 `[1,480,640,3]`. The RKNN conversion configuration owns
the training normalization. Do not normalize again on the board.

The RKNN must return these tensors in this exact order:

1. `det_boxes [1,6300,4]`: decoded xyxy boxes in the 640x480 input space.
2. `det_scores [1,6300,8]`: class probabilities; do not apply sigmoid.
3. `pixel_logits [1,3,120,160]`: road, single/left, and right logits.
4. `row_path_logits [1,2,32,161]`: two fixed path queries, 32 bottom-to-top
   row anchors, 160 horizontal bins, and one final `no_path` class.
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

`row_path_logits` is the only path source. Post-processing first exposes all
32 anchors for both slots as `raw_curve_paths`, without threshold filtering or
fitting. `vision_control.py` then removes unsupported points with the semantic
road mask and local association checks, compensates sparse lower points, fits
the smooth majority curve, and publishes the final control curves as `paths`.

## Runtime tuning

```bash
export MULTITASK_RKNN_VARIANT=curve_best
export MULTITASK_NPU_MODE=parallel
export MULTITASK_RKNN_TPES=3
export MULTITASK_PIPELINE_DEPTH=3
export MULTITASK_OPENCV_THREADS=2
export MULTITASK_RKNN_WARMUP=1
export MULTITASK_DET_THRESHOLD=0.50
export MULTITASK_TURNSIGN_THRESHOLD=0.40
export MULTITASK_NMS_THRESHOLD=0.45
export MULTITASK_PRE_NMS_TOP_K=1000
export MULTITASK_MAX_DETECTIONS=100
export MULTITASK_COIN_MIN_SHORT_SIDE=10
export MULTITASK_ROAD_THRESHOLD=0.50
export MULTITASK_ROAD_TOP_CROP_RATIO=0.34
export MULTITASK_ROAD_MIN_RATIO=0.001
export MULTITASK_ROAD_MAX_RATIO=0.60
export MULTITASK_ROAD_MAX_RAW_RATIO=0.86
export MULTITASK_ROAD_MIN_COMPONENT_AREA_RATIO=0.0015
export MULTITASK_ROAD_MAX_COMPONENTS=3
export MULTITASK_RENDER_PATH_MIN_SCORE=0.35
export MULTITASK_RENDER_PATH_MIN_COUNT_CONFIDENCE=0.40
export VISION_CONTROL_LOOKAHEAD_Y_RATIO=0.625
export VISION_CONTROL_POINT_LOWER_CONFIDENCE_BOOST=0.35
export VISION_CONTROL_POINT_UPPER_CONFIDENCE_DECAY=0.55
export VISION_CONTROL_MAX_STEP_640=24
export VISION_CONTROL_NORMAL_SPEED=0.10
export VISION_CONTROL_RECOVER_SPEED=0.04
export VISION_CONTROL_HUMAN_PASS_SPEED=0.35
export VISION_CONTROL_HUMAN_SPEED_HOLD_S=0.5
export VISION_CONTROL_CAR_HUMAN_PASS_SPEED=0.35
export VISION_CONTROL_CAR_HUMAN_PASS_HOLD_S=2.0
export VISION_CONTROL_HUMAN_STOP_PROGRESS_RATIO=0.84
export VISION_CONTROL_HUMAN_STOP_LINE_OFFSET_PX_480=35
export VISION_CONTROL_HUMAN_STOP_ABSENCE_RESTART_S=5.0
export VISION_CONTROL_HUMAN_STOP_REVERSE_SPEED=-0.10
export VISION_CONTROL_HUMAN_STOP_REVERSE_DURATION_S=0.3
export VISION_CONTROL_HUMAN_PRELINE_MISSING_PX_480=20
export VISION_CONTROL_CAR_AVOID_OFFSET_640=55
export VISION_CONTROL_CAR_AVOID_HOLD_S=2.0
export VISION_CONTROL_AVOID_BOX_WIDTH_GAIN=0.40
export VISION_CONTROL_HAZARD_BOTTOM_RATIO=0.58
export VISION_CONTROL_TURNSIGN_SLOW_SPEED=0.10
export VISION_CONTROL_TURNSIGN_REVERSE_SPEED=-0.08
export VISION_CONTROL_TURNSIGN_REVERSE_DURATION_S=0.5
export VISION_CONTROL_TURNSIGN_TRIM_FAR_Y_RATIO=0.35
export VISION_CONTROL_TURNSIGN_TRIM_SAMPLE_ROWS=6
export VISION_CONTROL_TURNSIGN_TRIM_LOW_SEPARATION_640=175
export VISION_CONTROL_TURNSIGN_TRIM_HIGH_SEPARATION_640=220
export VISION_CONTROL_TURNSIGN_TRIM_SETTLE_S=0.8
export VISION_CONTROL_TURNSIGN_TRIM_STABLE_FRAMES=3
export VISION_CONTROL_TURNSIGN_TRIM_CENTER_DEADBAND_640=24
export VISION_CONTROL_TURNSIGN_TRIM_SPLIT_640=36
export VISION_CONTROL_TURNSIGN_TRIM_SPLIT_FRAMES=3
export VISION_CONTROL_TURNSIGN_TRIM_COLLAPSE_FRAMES=2
export VISION_CONTROL_TURNSIGN_TRIM_DROP_640=45
export VISION_CONTROL_TURNSIGN_TRIM_STEER_DEADBAND_640=4
export VISION_CONTROL_TURNSIGN_TRIM_MIN_STEER_640=44
export VISION_CONTROL_TURNSIGN_TRIM_STEER_GAIN=0.75
export VISION_CONTROL_TURNSIGN_TRIM_MAX_STEER_640=100
export VISION_CONTROL_TURNSIGN_TRIM_MISSING_STEER_GAIN=1.05
export VISION_CONTROL_TURNSIGN_TRIM_MISSING_MAX_STEER_640=135
export VISION_CONTROL_TURNSIGN_TRIM_SEVERE_MISSING_FRAMES=3
export VISION_CONTROL_TURNSIGN_TRIM_SEVERE_STEER_GAIN=1.30
export VISION_CONTROL_TURNSIGN_TRIM_SEVERE_MAX_STEER_640=165
export VISION_CONTROL_SIGN_LATCH_FRAMES=3
export AR_TURNSIGN_CONFIRM_FRAMES=1
export AR_TURNSIGN_SNAPSHOT_MIN_AREA_RATIO=0.02
export AR_TURNSIGN_SNAPSHOT_EDGE_MARGIN_RATIO=0.10
export AR_TURNSIGN_DOOR_CONFLICT_SCORE=0.80
export AR_TURNSIGN_DOOR_CONFLICT_DISTANCE_PX_640=120
export AR_TURNSIGN_DETECTION_LINE_RATIO=0.3854167
export AR_TURNSIGN_PRECONFIRM_LINE_DISTANCE_480=45
export AR_TURNSIGN_SESSION_ABSENCE_TIMEOUT_S=3.0
export AR_TURNSIGN_RESPONSE_TIMEOUT_S=10.0
export MULTITASK_PATH_CONSTRAIN_TO_ROAD=1
export MULTITASK_PATH_ROAD_SNAP_RADIUS=10
export VISION_CONTROL_BLANK_PROBABILITY=0.05
export VISION_CONTROL_PEAK_SCAN_TOP_RATIO=0.45
export VISION_CONTROL_PEAK_SCAN_BOTTOM_RATIO=0.55
export VISION_CONTROL_MIN_PEAK_COMPONENT_AREA=20
export VISION_CONTROL_BOTTOM_REACH_RATIO=0.90
export VISION_CONTROL_SIDE_EXIT_MIN_Y_RATIO=0.667
export VISION_CONTROL_SIDE_EXIT_MARGIN_RATIO=0.05
export VISION_CONTROL_RECOVERY_MAX_GAP_ROWS=12
export VISION_CONTROL_RECOVERY_MAX_RADIUS=24
export VISION_CONTROL_RECOVERY_MIN_PROBABILITY=0.35
export VISION_CONTROL_RECOVERY_MIN_CONTINUATION_ROWS=8
export VISION_CONTROL_RECOVERY_AMBIGUITY_MARGIN=0.08
export VISION_CONTROL_CURVE_FIT_BLEND=0.25
export VISION_CONTROL_HISTORY_WEIGHT=0.035
export VISION_CONTROL_PATH_EMA_ALPHA=0.45
export VISION_CONTROL_PATH_SMOOTH_WINDOW=5
export VISION_CONTROL_PATH_MAX_STEP_640=40
export VISION_CONTROL_PATH_HOLD_FRAMES=8
export VISION_CONTROL_CURVE_GREEN_MAX_STEP_640=20
export VISION_CONTROL_CURVE_GREEN_EMA_ALPHA=0.30
export VISION_CONTROL_CURVE_GREEN_HOLD_FRAMES=3
export VISION_CONTROL_ROUTE_CONFIRM_FRAMES=6
export VISION_CONTROL_BRANCH_RELEASE_FRAMES=20
export VISION_CONTROL_CURVE_MERGE_FAST_SPAN_RATIO=0.55
export VISION_CONTROL_CURVE_MERGE_FAST_BLUE_JUMP_640=60
export VISION_CONTROL_CURVE_MERGE_FAST_GREEN_JUMP_640=40
export VISION_CONTROL_CURVE_MERGE_FAST_RECOVERY_FRAMES=2
export VISION_CONTROL_CURVE_BLUE_ANGLE_JUMP_DEG=12
export VISION_CONTROL_CURVE_BLUE_INSTABILITY_EVIDENCE=3
export VISION_CONTROL_CURVE_GREEN_MISSING_RELEASE_FRAMES=3
```

Fitted centerlines retain a lightweight temporal guard: a lateral jump over
24 px/640 must be seen again within 12 px before it replaces the previous
fitted curve. Candidate paths also keep the established slot EMA, per-frame
step limit, short missing-path hold, green-curve guard, and route confirmation.
Ordinary line following uses the historical left/right error slew limits and
trend feedback; trend slopes with an absolute value below 10 px/frame are
ignored. Pedestrian/car avoidance remains outside that restored feedback.
Safety/task action timing remains independent of path filtering.

`MULTITASK_COIN_MIN_SHORT_SIDE=10` mirrors the training dataset rule: Coin
detections whose short side is below 10 pixels in the fixed 640x480 model input
are discarded before NMS. Set it to `0` only for a deliberate tiny-Coin test.
Detection follows the `main` branch policy: each box is assigned only to its
highest-scoring class, candidates below 0.50 are removed, and same-class boxes
are suppressed at IoU 0.45. TurnSign alone uses the class-specific threshold
0.40; every other class, including Human and Car, remains at 0.50.

Road-mask cleanup also follows the reusable part of `main`: close 5x5, open
3x3, rank connected components by area, bottom contact, center proximity, and
height, then retain at most three road bodies. Implausibly full, tiny, or large
masks are rejected. `road_mask_raw` keeps the unmodified threshold result.

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
postprocess. Visual-control timing includes path search and causal filtering.

Debug rendering is performed by the drop-frame preview thread, not by an NPU
worker. Select it with:

```bash
export MULTITASK_RENDER_MODE=drive  # off, drive, debug, or full
export MULTITASK_ROAD_OVERLAY_ALPHA=0.28
```

- `off`: no perception drawing.
- `drive`: realtime default. Draw compact detections and the active control
  paths. No road fill or labels; paths are thin, Coin uses a dot, and other
  detections use corner marks. Frame-spanning boxes are hidden from rendering
  only and remain available to OCR/control.
- `debug`: drive mode plus the binary road overlay.
- `full`: road, up to 20 detections, final path points, roles, and
  confidence text.

Drive rendering is separate from perception output. OCR and control still
receive every post-NMS detection even though preview drawing defaults to at
most six detections, two per class. Legacy `path` and `road` mode names map to
`drive` and `debug`.

There is no runtime path-source switch. The preview consumes the same final
`paths` objects used by steering, so it does not repeat curve fitting or retain
a second control-line implementation.

Route classification changes only after six consecutive confirming frames.
The selected path slot is held across eight missing frames, and an OCR-locked
branch never falls back to the opposite slot. Branch context is released only
after 20 stable single-route frames.

Set `AR_LOCAL_PREVIEW=0` to skip full-frame rendering. Three RKNN runtimes
remain in flight by default, one on each RK3588S NPU core.

## Python result

The result exposes both the unified top-level fields and the existing nested
`road`/`centerline` compatibility fields:

- `detections`: class id/name, score, and original-image `box_xyxy`.
- `road_probability`: `[120,160]` float32.
- `road_mask`: `[120,160]` uint8.
- `model_path_count` and `path_count_scores`: raw count-head diagnostics.
- `raw_curve_paths`: both slots and all 32 row-classifier anchors.
- `paths`: final road-supported, compensated, fitted control curves, added by
  `VisionControlPlanner.update`.
- `timings_ms`: preprocess, RKNN inference, FIFO wait, postprocess, and total
  latency visible to the perception consumer.

The old `rknn_lt.rknn` and old four-output models are incompatible and fail
fast instead of producing incorrect paths.

`ar_receiver.py` holds `/tmp/sysu_ddl_ar_receiver.lock`. A repeated launch asks
the existing process to open its preview and then exits. Failure to bind the
optional preview-control port only disables that HTTP endpoint; inference and
the local preview continue to run.
