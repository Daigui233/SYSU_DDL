import sys
import types
import unittest

import numpy as np

from vision_runtime_controls import VisionRuntimeControls


class _FakeInfer:
    def __init__(self):
        self.calls = 0

    def infer_current(self, *_args, **_kwargs):
        self.calls += 1
        return None, False, {"frame_id": _kwargs.get("frame_id"), "timestamp": _kwargs.get("timestamp")}


def _install_fake_infer_modules():
    infer_wrap = types.ModuleType("infer_wrap")
    infer_wrap.InferWrap = object
    infer_wrap.PPSegInfer = object

    base = types.ModuleType("infer_wrap.base")
    func = types.ModuleType("infer_wrap.base.func")
    func.CLASSES = ["Stone", "BeginSign", "EndSign", "TrafficLight"]

    seg_func = types.ModuleType("infer_wrap.base.seg_func")
    seg_func.draw_centerline_debug = lambda frame, _segmentation, draw_text=False: frame
    seg_func.extract_centerline_info = lambda _seg_res, frame, **_kwargs: (
        frame,
        {"source": "current", "road_mask": None, "line_valid": False},
    )

    seg_infer = types.ModuleType("infer_wrap.base.seg_infer")
    seg_infer.ForkMaskClsInfer = None

    sys.modules.setdefault("infer_wrap", infer_wrap)
    sys.modules.setdefault("infer_wrap.base", base)
    sys.modules.setdefault("infer_wrap.base.func", func)
    sys.modules.setdefault("infer_wrap.base.seg_func", seg_func)
    sys.modules.setdefault("infer_wrap.base.seg_infer", seg_infer)


class VisionRuntimeControlsTest(unittest.TestCase):
    def test_disabled_models_are_not_called(self):
        _install_fake_infer_modules()
        from vision_pipeline import VisionPipeline

        controls = VisionRuntimeControls(
            {
                "enable_detection": False,
                "enable_segmentation": False,
                "enable_model_overlay": True,
            }
        )
        pipeline = VisionPipeline.__new__(VisionPipeline)
        pipeline.log_func = lambda *_args, **_kwargs: None
        pipeline.runtime_controls = controls
        pipeline.infer_det = _FakeInfer()
        pipeline.infer_seg = _FakeInfer()
        pipeline.infer_fork = None

        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        _out, perception = pipeline.process(frame, 1.0, frame_id=7, draw_debug=False)

        self.assertEqual(pipeline.infer_det.calls, 0)
        self.assertEqual(pipeline.infer_seg.calls, 0)
        self.assertEqual(perception["detection_status"]["source"], "disabled")
        self.assertEqual(perception["segmentation"]["source"], "disabled")
        self.assertEqual(perception["vision_controls"]["enable_detection"], False)
        self.assertEqual(perception["vision_controls"]["enable_segmentation"], False)


if __name__ == "__main__":
    unittest.main()
