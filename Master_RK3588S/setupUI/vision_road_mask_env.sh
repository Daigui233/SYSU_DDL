#!/bin/sh

# Minimal road-segmentation preview for the inverse-perspective experiment.
# The model's road probability receives only a threshold and one 3x3 closing
# operation.  Disable the old path controller and vehicle output so the local
# preview contains only the red road mask and the runtime FPS text.
export AR_ROAD_MASK_EXPERIMENT="1"
export MULTITASK_PATH_SOURCE="curve"
export MULTITASK_RENDER_MODE="road_mask"
export MULTITASK_ROAD_THRESHOLD="0.50"
export MULTITASK_ROAD_SIMPLE_CLOSE_ITERATIONS="1"
export MULTITASK_ROAD_MASK_PREVIEW_ALPHA="0.55"
# Diagnostic perspective trapezoid, expressed as native-mask ratios.
export MULTITASK_ROAD_BEV_TOP_Y="0.32"
export MULTITASK_ROAD_BEV_TOP_LEFT_X="0.15"
export MULTITASK_ROAD_BEV_TOP_RIGHT_X="0.85"
export MULTITASK_ROAD_BEV_BOTTOM_LEFT_X="0.02"
export MULTITASK_ROAD_BEV_BOTTOM_RIGHT_X="0.98"
export MULTITASK_ROAD_BEV_DEST_MARGIN="0.08"
export MULTITASK_ROAD_BEV_EXIT_BAND="12"
export MULTITASK_ROAD_BEV_EXIT_SIDE_REACH="0.70"
export MULTITASK_ROAD_BEV_EXIT_MIN_AREA="6"
export MULTITASK_ROAD_BEV_CENTER_WEIGHT="6.0"
export MULTITASK_ROAD_BEV_START_BAND="6"
export MULTITASK_ROAD_BEV_START_SEARCH_RADIUS="8"
export MULTITASK_ROAD_BEV_PLANNING_DOWNSAMPLE="2"
export VISION_CONTROL_PATH_SOURCE="curve"
export AR_VISION_CONTROL_DEBUG="0"
export AR_VISION_CONTROL_SEND="0"
export AR_CONTROL_RUNTIME_ENABLED="0"
