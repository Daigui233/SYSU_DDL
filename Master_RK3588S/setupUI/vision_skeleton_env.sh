#!/bin/sh

# Source this file before starting ar_receiver.py to try semantic-mask
# skeleton path extraction. The original heatmap tracer remains available by
# setting VISION_CONTROL_PATH_SOURCE=heatmap.
export VISION_CONTROL_PATH_SOURCE="skeleton"
# The skeleton controller consumes the raw heatmaps directly.  Keep the
# decoder on its lightweight curve path so the legacy Python ridge tracer is
# not run and discarded, and avoid drawing a raw heatmap below the red mask.
export MULTITASK_PATH_SOURCE="curve"
export MULTITASK_RENDER_MODE="drive"
export VISION_CONTROL_FAST_RENDER="1"
export VISION_CONTROL_SKELETON_THRESHOLD="0.35"
export VISION_CONTROL_SKELETON_LOW_THRESHOLD="0.27"
export VISION_CONTROL_SKELETON_MIN_AREA="15"
export VISION_CONTROL_SKELETON_MIN_LENGTH="8"
export VISION_CONTROL_SKELETON_CLOSE_ITERATIONS="1"
export VISION_CONTROL_SKELETON_EDGE_KERNEL="9"
export VISION_CONTROL_SKELETON_MAX_HOLE_AREA="32"
export VISION_CONTROL_SKELETON_MAX_CONNECT_GAP="6"
export VISION_CONTROL_SKELETON_BRIDGE_THICKNESS="3"
export VISION_CONTROL_SKELETON_MIN_BRANCH_LENGTH="10"
export VISION_CONTROL_SKELETON_MAX_BRANCHES="2"
