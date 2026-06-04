"""
VideoRecorder: write raw frames to an mp4 file.

This module does not run any inference; it only persists frames.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import cv2  # type: ignore  # Some linters can't resolve cv2 types in this environment.


class VideoRecorder:
    """Small wrapper around OpenCV VideoWriter with lazy init."""

    def __init__(
        self,
        mp4_path: Path,
        fps: float = 30.0,
        fourcc_str: str = "mp4v",
    ):
        self.mp4_path = mp4_path
        self.fps = float(fps)
        self.fourcc_str = fourcc_str

        self._writer: Optional[cv2.VideoWriter] = None
        self._frame_size: Optional[Tuple[int, int]] = None  # (w, h)

    def _ensure_writer(self, frame_bgr) -> None:
        if self._writer is not None:
            return

        h, w = frame_bgr.shape[:2]
        self._frame_size = (w, h)

        self.mp4_path.parent.mkdir(parents=True, exist_ok=True)

        fourcc = cv2.VideoWriter_fourcc(*self.fourcc_str)
        self._writer = cv2.VideoWriter(
            str(self.mp4_path),
            fourcc,
            self.fps,
            self._frame_size,
        )

        if not self._writer.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter: {self.mp4_path}")

    def write_frame(self, frame_bgr) -> None:
        """Write one raw frame to mp4."""
        self._ensure_writer(frame_bgr)

        assert self._frame_size is not None
        w, h = self._frame_size
        fh, fw = frame_bgr.shape[:2]
        if (fw, fh) != (w, h):
            frame_bgr = cv2.resize(frame_bgr, (w, h), interpolation=cv2.INTER_LINEAR)

        self._writer.write(frame_bgr)

    def stop(self) -> None:
        """Release the underlying writer (safe to call multiple times)."""
        if self._writer is not None:
            self._writer.release()
            self._writer = None

