import unittest

import numpy as np

from .infer_wrap import InferWrap
from .func import inference_worker
from .path_temporal_filter import PathTemporalFilter


def make_path(role, x, score=0.9):
    normalized = np.zeros((32, 2), dtype=np.float32)
    normalized[:, 0] = x
    normalized[:, 1] = np.linspace(0.95, 0.10, 32)
    points_xy = normalized.copy()
    points_xy[:, 0] *= 639
    points_xy[:, 1] *= 479
    slot = 1 if role == "right" else 0
    return {
        "slot": slot,
        "role": role,
        "score": score,
        "count_confidence": 0.9,
        "points_normalized": normalized,
        "points_xy": points_xy,
    }


def make_result(path_count, xs=()):
    if path_count == 0:
        paths = []
        scores = [0.94, 0.04, 0.02]
    elif path_count == 1:
        paths = [make_path("single", xs[0])]
        scores = [0.03, 0.94, 0.03]
    else:
        paths = [make_path("left", xs[0]), make_path("right", xs[1])]
        scores = [0.02, 0.05, 0.93]
    return {
        "path_count": path_count,
        "path_count_scores": scores,
        "paths": paths,
        "centerline": {
            "path_count": path_count,
            "paths": paths,
            "count_confidence": scores[path_count],
        },
    }


def make_model_outputs(x=0.40):
    boxes = np.zeros((1, 6300, 4), dtype=np.float32)
    scores = np.zeros((1, 6300, 8), dtype=np.float32)
    pixel = np.full((1, 3, 120, 160), -8.0, dtype=np.float32)
    pixel[0, 0, 40:, 20:140] = 8.0
    row_logits = np.full((1, 2, 32, 161), -8.0, dtype=np.float32)
    row_logits[0, 0, :, int(round(x * 159))] = 8.0
    row_logits[0, 1, :, 160] = 8.0
    return (
        boxes, scores, pixel, row_logits,
        np.asarray([[0.90, 0.10]], dtype=np.float32),
        np.asarray([[0.03, 0.94, 0.03]], dtype=np.float32),
    )


class PathTemporalFilterTest(unittest.TestCase):
    def test_ema_smooths_same_role_without_frame_delay(self):
        path_filter = PathTemporalFilter(
            alpha=0.5, count_confirm_frames=2,
            lost_hold_frames=3, max_jump_ratio=1.0)
        first = path_filter.update(make_result(1, (0.40,)))
        second = path_filter.update(make_result(1, (0.60,)))

        self.assertAlmostEqual(
            float(first["paths"][0]["points_normalized"][0, 0]),
            0.40, places=6)
        self.assertAlmostEqual(
            float(second["paths"][0]["points_normalized"][0, 0]),
            0.50, places=6)
        self.assertAlmostEqual(
            float(second["raw_paths"][0]["points_normalized"][0, 0]),
            0.60, places=6)

    def test_zero_count_must_persist_before_path_disappears(self):
        path_filter = PathTemporalFilter(
            alpha=0.5, count_confirm_frames=2,
            lost_hold_frames=3, max_jump_ratio=1.0)
        path_filter.update(make_result(1, (0.45,)))

        first_loss = path_filter.update(make_result(0))
        second_loss = path_filter.update(make_result(0))
        third_loss = path_filter.update(make_result(0))

        self.assertEqual(first_loss["path_count"], 1)
        self.assertEqual(second_loss["path_count"], 1)
        self.assertTrue(first_loss["paths"][0]["held"])
        self.assertEqual(third_loss["path_count"], 0)
        self.assertEqual(third_loss["paths"], [])

    def test_count_switch_resets_roles_instead_of_cross_blending(self):
        path_filter = PathTemporalFilter(
            alpha=0.2, count_confirm_frames=2,
            lost_hold_frames=3, max_jump_ratio=1.0)
        path_filter.update(make_result(1, (0.50,)))

        pending = path_filter.update(make_result(2, (0.20, 0.80)))
        accepted = path_filter.update(make_result(2, (0.20, 0.80)))

        self.assertEqual(pending["path_count"], 1)
        self.assertEqual(pending["paths"][0]["role"], "single")
        self.assertEqual(accepted["path_count"], 2)
        self.assertEqual([path["role"] for path in accepted["paths"]],
                         ["left", "right"])
        self.assertAlmostEqual(
            float(accepted["paths"][0]["points_normalized"][0, 0]),
            0.20, places=6)

    def test_repeated_large_jump_is_held_then_reinitialized(self):
        path_filter = PathTemporalFilter(
            alpha=0.5, count_confirm_frames=1,
            lost_hold_frames=3, max_jump_ratio=0.05)
        path_filter.update(make_result(1, (0.30,)))

        first = path_filter.update(make_result(1, (0.60,)))
        second = path_filter.update(make_result(1, (0.60,)))
        third = path_filter.update(make_result(1, (0.60,)))

        self.assertTrue(first["paths"][0]["jump_rejected"])
        self.assertTrue(second["paths"][0]["jump_rejected"])
        self.assertAlmostEqual(
            float(first["paths"][0]["points_normalized"][0, 0]),
            0.30, places=6)
        self.assertTrue(third["paths"][0]["reinitialized"])
        self.assertAlmostEqual(
            float(third["paths"][0]["points_normalized"][0, 0]),
            0.60, places=6)

    def test_filter_can_be_disabled(self):
        path_filter = PathTemporalFilter(enabled=False)
        result = make_result(1, (0.40,))
        self.assertIs(path_filter.update(result), result)
        self.assertNotIn("raw_paths", result)

    def test_long_frame_gap_resets_stale_path_state(self):
        path_filter = PathTemporalFilter(
            alpha=0.1, max_jump_ratio=0.05, reset_gap_seconds=0.5)
        path_filter.update(make_result(1, (0.30,)), timestamp=1.0)

        result = path_filter.update(
            make_result(1, (0.70,)), timestamp=2.0)

        self.assertFalse(result["paths"][0]["held"])
        self.assertAlmostEqual(
            float(result["paths"][0]["points_normalized"][0, 0]),
            0.70, places=6)

    def test_infer_wrap_returns_fifo_result_without_temporal_filter(self):
        outputs = make_model_outputs()

        class FakeRKNN:
            def inference(self, inputs):
                return outputs

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        worker_result = inference_worker(FakeRKNN(), frame)

        class FakePool:
            def put(self, frame):
                self.frame = frame

            def get(self):
                return worker_result, True

        infer = object.__new__(InferWrap)
        infer.rknn_pool = FakePool()
        infer.pipeline_depth = 1
        infer.pending = 0

        result, ready = infer.infer(frame)

        self.assertTrue(ready)
        self.assertNotIn("temporal", result)
        self.assertEqual(result["path_count"], 1)
        self.assertIn("inference", result["timings_ms"])
        self.assertIn("fifo_wait", result["timings_ms"])
        self.assertEqual(result["timings_ms"]["temporal_filter"], 0.0)

    def test_startup_warmup_validates_every_runtime(self):
        outputs = make_model_outputs()

        class FakeRKNN:
            def inference(self, inputs):
                return outputs

        class FakePool:
            rknnPool = [FakeRKNN(), FakeRKNN(), FakeRKNN()]

        infer = object.__new__(InferWrap)
        infer.rknn_pool = FakePool()
        infer.warmup_inference_ms = []

        infer._warmup_runtimes()

        self.assertEqual(len(infer.warmup_inference_ms), 3)
        self.assertTrue(all(value >= 0.0
                            for value in infer.warmup_inference_ms))

    def test_startup_warmup_rejects_old_output_contract(self):
        class OldRKNN:
            def inference(self, inputs):
                return make_model_outputs()[:4]

        class FakePool:
            rknnPool = [OldRKNN()]

        infer = object.__new__(InferWrap)
        infer.rknn_pool = FakePool()
        infer.warmup_inference_ms = []

        with self.assertRaises(ValueError):
            infer._warmup_runtimes()


if __name__ == "__main__":
    unittest.main()
