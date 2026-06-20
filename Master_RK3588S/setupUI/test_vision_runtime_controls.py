import sys
import threading
import time
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


class _ParallelFakeInfer:
    def __init__(self, barrier, result, returned_frame_id=None):
        self.barrier = barrier
        self.result = result
        self.returned_frame_id = returned_frame_id
        self.started_at = None
        self.received_frame_id = None

    def infer_current(self, _frame, **kwargs):
        self.started_at = time.perf_counter()
        self.received_frame_id = kwargs.get("frame_id")
        self.barrier.wait(timeout=1.0)
        time.sleep(0.03)
        result_frame_id = self.received_frame_id if self.returned_frame_id is None else self.returned_frame_id
        return self.result, True, {
            "frame_id": result_frame_id,
            "timestamp": kwargs.get("timestamp"),
        }

    def release(self):
        return None


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

    def test_enabled_models_run_concurrently_on_the_same_frame(self):
        _install_fake_infer_modules()
        from vision_pipeline import VisionPipeline

        barrier = threading.Barrier(2)
        seg = _ParallelFakeInfer(barrier, np.zeros((48, 64), dtype=np.uint8))
        det = _ParallelFakeInfer(barrier, (None, None, None))
        pipeline = VisionPipeline.__new__(VisionPipeline)
        pipeline.log_func = lambda *_args, **_kwargs: None
        pipeline.runtime_controls = VisionRuntimeControls()
        pipeline.infer_det = det
        pipeline.infer_seg = seg
        pipeline.infer_fork = None
        pipeline._inference_executor = None

        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        _out, perception = pipeline.process(frame, 1.0, frame_id=17, draw_debug=False)

        self.assertEqual(seg.received_frame_id, 17)
        self.assertEqual(det.received_frame_id, 17)
        self.assertLess(abs(seg.started_at - det.started_at), 0.05)
        self.assertEqual(perception["segmentation"]["seg_frame_id"], 17)
        self.assertEqual(perception["detection_status"]["det_frame_id"], 17)
        self.assertGreater(perception["timings_ms"]["seg_infer_ms"], 0.0)
        self.assertGreater(perception["timings_ms"]["det_infer_ms"], 0.0)
        self.assertGreater(perception["timings_ms"]["vision_total_ms"], 0.0)
        pipeline.release()

    def test_mismatched_detection_frame_is_rejected(self):
        _install_fake_infer_modules()
        from vision_pipeline import VisionPipeline

        barrier = threading.Barrier(2)
        seg = _ParallelFakeInfer(barrier, np.zeros((48, 64), dtype=np.uint8))
        det = _ParallelFakeInfer(barrier, (None, None, None), returned_frame_id=16)
        pipeline = VisionPipeline.__new__(VisionPipeline)
        pipeline.log_func = lambda *_args, **_kwargs: None
        pipeline.runtime_controls = VisionRuntimeControls()
        pipeline.infer_det = det
        pipeline.infer_seg = seg
        pipeline.infer_fork = None
        pipeline._inference_executor = None

        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        _out, perception = pipeline.process(frame, 1.0, frame_id=17, draw_debug=False)

        self.assertEqual(perception["detection_status"]["source"], "stale_rejected")
        self.assertEqual(perception["detection_status"]["reason"], "frame_id_mismatch")
        self.assertEqual(perception["detections"], [])
        pipeline.release()


if __name__ == "__main__":
    unittest.main()
