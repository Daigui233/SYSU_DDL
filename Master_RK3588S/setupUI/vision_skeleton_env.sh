#!/bin/sh

# Source this file before starting ar_receiver.py to try semantic-mask
# skeleton path extraction. The original heatmap tracer remains available by
# setting VISION_CONTROL_PATH_SOURCE=heatmap.
export VISION_CONTROL_PATH_SOURCE="skeleton"
export VISION_CONTROL_SKELETON_THRESHOLD="0.35"
export VISION_CONTROL_SKELETON_MIN_AREA="15"
export VISION_CONTROL_SKELETON_MIN_LENGTH="8"
export VISION_CONTROL_SKELETON_CLOSE_ITERATIONS="1"
export VISION_CONTROL_SKELETON_EDGE_KERNEL="9"
export VISION_CONTROL_SKELETON_MAX_CONNECT_GAP="8"
export VISION_CONTROL_SKELETON_BRIDGE_THICKNESS="3"
