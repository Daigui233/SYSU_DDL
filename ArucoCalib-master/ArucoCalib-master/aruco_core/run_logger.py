"""
RunLogger: persist raw frames and timestamps for each detection session.

Output layout (relative to repo root):
- runs/<run_ts>/img/imgXXXXX_<time_ms>.jpg
- runs/<run_ts>/images.json
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2


def make_run_ts(now: Optional[datetime] = None) -> str:
    """Create a stable run timestamp string for directory naming."""
    now = now or datetime.now()
    # Human-readable + millisecond precision to avoid collisions.
    return now.strftime("%Y%m%d_%H%M%S") + f"_{now.microsecond // 1000:03d}"


@dataclass
class ImageLogEntry:
    img_file_name: str
    time: int  # time_ms

    def to_dict(self) -> Dict[str, Any]:
        return {"img_file_name": self.img_file_name, "time": self.time}


class RunLogger:
    """Save raw frames and maintain images.json for the current run."""

    def __init__(self, run_ts: str, runs_root_dir: Optional[Path] = None):
        """
        Args:
            run_ts: Folder name under `runs/`.
            runs_root_dir: Override runs root (defaults to repo-root/runs).
        """
        if runs_root_dir is None:
            repo_root = Path(__file__).resolve().parents[1]
            runs_root_dir = repo_root / "runs"

        self.run_ts = run_ts
        self.runs_root_dir = runs_root_dir
        self.run_dir = self.runs_root_dir / self.run_ts
        self.img_dir = self.run_dir / "img"
        self.images_json_path = self.run_dir / "images.json"

        self.img_dir.mkdir(parents=True, exist_ok=True)

        self.images_info: List[ImageLogEntry] = []
        if self.images_json_path.exists():
            try:
                data = json.loads(self.images_json_path.read_text(encoding="utf-8"))
                if isinstance(data, list):
                    self.images_info = [
                        ImageLogEntry(
                            img_file_name=str(it.get("img_file_name")),
                            time=int(it.get("time")),
                        )
                        for it in data
                        if isinstance(it, dict)
                    ]
            except Exception:
                # Corrupted json should not crash the UI.
                self.images_info = []

        self.img_counter = len(self.images_info)

        # Ensure the json file exists from the start of a run, even if no frame
        # has been saved yet.
        if not self.images_json_path.exists():
            with self.images_json_path.open("w", encoding="utf-8") as f:
                json.dump([], f, ensure_ascii=False, indent=2)

    def _time_ms(self) -> int:
        return int(time.time() * 1000)

    def save_frame(self, frame_bgr, time_ms: Optional[int] = None) -> str:
        """
        Save one raw frame to disk and append a record into images.json.

        Returns:
            The saved image file name (not full path).
        """
        if time_ms is None:
            time_ms = self._time_ms()

        img_file_name = f"img{self.img_counter:05d}_{time_ms}.jpg"
        img_path = self.img_dir / img_file_name

        ok = cv2.imwrite(str(img_path), frame_bgr)
        if not ok:
            raise RuntimeError(f"Failed to write image: {img_path}")

        self.img_counter += 1
        entry = ImageLogEntry(img_file_name=img_file_name, time=int(time_ms))
        self.images_info.append(entry)

        # Keep it simple & robust: rewrite json each time.
        # The UI runs in a single thread, so no concurrent writes.
        with self.images_json_path.open("w", encoding="utf-8") as f:
            json.dump([e.to_dict() for e in self.images_info], f, ensure_ascii=False, indent=2)

        return img_file_name

