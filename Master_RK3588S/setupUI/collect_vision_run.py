"""Start AR receiver with per-frame vision telemetry enabled.

Usage:
    python3 collect_vision_run.py

The receiver keeps its normal control/preview environment. Set
AR_VISION_CONTROL_SEND=0 before starting if the run is debug-only.
"""

import os
from pathlib import Path
import runpy


os.environ.setdefault("AR_VISION_TELEMETRY", "1")
os.environ.setdefault(
    "AR_VISION_TELEMETRY_DIR",
    str(Path(__file__).resolve().parent / "dist" / "vision_runs"),
)
runpy.run_path(str(Path(__file__).with_name("ar_receiver.py")), run_name="__main__")

