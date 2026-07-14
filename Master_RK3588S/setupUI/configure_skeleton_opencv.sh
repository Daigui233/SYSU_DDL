#!/bin/sh
set -eu

PYTHON_BIN="${PYTHON_BIN:-python3}"
OPENCV_VERSION="${OPENCV_VERSION:-4.10.0.84}"

# PaddleOCR/Albumentations may install multiple OpenCV wheels which all write
# to the same cv2 namespace. Reinstall contrib last so ximgproc.thinning is
# the implementation actually imported by the runtime. Keep the existing
# NumPy version because it is shared with RKNN and PaddleOCR.
"${PYTHON_BIN}" -m pip install \
    --user --force-reinstall --no-deps \
    "opencv-contrib-python==${OPENCV_VERSION}"

"${PYTHON_BIN}" - <<'PY'
import cv2

thinning = getattr(getattr(cv2, "ximgproc", None), "thinning", None)
if not callable(thinning):
    raise SystemExit(
        "OpenCV was installed, but cv2.ximgproc.thinning is unavailable")
print("OpenCV {}: ximgproc.thinning ready".format(cv2.__version__))
PY
