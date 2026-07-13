import unittest

import numpy as np

from .func import (
    CLASSES,
    RENDER_MODE,
    _select_detections_for_render,
    decode_outputs,
    detection_nms,
    myFunc,
    parse_outputs,
    render_result,
)


class MultiTaskPostprocessTest(unittest.TestCase):
    def make_outputs(self):
        boxes = np.zeros((1, 6300, 4), dtype=np.float32)
        scores = np.zeros((1, 6300, 8), dtype=np.float32)
        pixel = np.full((1, 3, 120, 160), -8.0, dtype=np.float32)
        pixel[0, 0, :, :] = 8.0
        pixel[0, 1, 24:119, 66] = 8.0
        pixel[0, 2, 24:119, 108] = 8.0
        points = np.zeros((1, 2, 32, 2), dtype=np.float32)
        points[0, :, :, 1] = np.linspace(0.95, 0.10, 32)
        points[0, 0, :, 0] = 0.42
        points[0, 1, :, 0] = 0.68
        path_scores = np.asarray([[0.92, 0.81]], dtype=np.float32)
        count_scores = np.asarray([[0.02, 0.08, 0.90]], dtype=np.float32)
        return boxes, scores, pixel, points, path_scores, count_scores

    def test_output_contract(self):
        boxes, scores, pixel, points, path_scores, count_scores = (
            parse_outputs(self.make_outputs()))
        self.assertEqual(boxes.shape, (6300, 4))
        self.assertEqual(scores.shape, (6300, 8))
        self.assertEqual(pixel.shape, (3, 120, 160))
        self.assertEqual(points.shape, (2, 32, 2))
        self.assertEqual(path_scores.shape, (2,))
        self.assertEqual(count_scores.shape, (3,))
        self.assertEqual(CLASSES, (
            "Door", "TurnSign", "BeginSign", "EndSign",
            "Coin", "Human", "Car", "SpeedSign"))

    def test_accepts_nhwc_pixel_output(self):
        outputs = list(self.make_outputs())
        outputs[2] = outputs[2].transpose(0, 2, 3, 1)
        parsed = parse_outputs(outputs)
        self.assertEqual(parsed[2].shape, (3, 120, 160))

    def test_keeps_direct_curve_paths_for_model_comparison(self):
        outputs = list(self.make_outputs())
        outputs[0][0, 0] = [64, 48, 192, 144]
        outputs[1][0, 0, 7] = 0.90
        outputs[2][0, 0, 20:, 50:110] = 8.0
        result = decode_outputs(outputs, (240, 320, 3))

        self.assertEqual(len(result["detections"]), 1)
        self.assertEqual(result["detections"][0]["label"], "SpeedSign")
        self.assertEqual(result["path_count"], 2)
        self.assertEqual([item["role"] for item in result["curve_paths"]],
                         ["left", "right"])
        self.assertEqual(
            result["curve_paths"][0]["points_xy"].shape, (32, 2))
        self.assertAlmostEqual(
            float(result["curve_paths"][0]["points_xy"][0, 0]),
            0.42 * 319, places=4)
        self.assertAlmostEqual(
            float(result["curve_paths"][0]["points_xy"][0, 1]),
            0.95 * 239, places=4)

    def test_traces_each_active_heatmap_without_temporal_state(self):
        result = decode_outputs(self.make_outputs(), (480, 640, 3))

        self.assertEqual(result["centerline"]["source"], "heatmap")
        self.assertEqual([item["role"] for item in result["paths"]],
                         ["left", "right"])
        self.assertTrue(all(item["source"] == "heatmap"
                            for item in result["paths"]))
        self.assertGreater(len(result["paths"][0]["points_xy"]), 20)
        self.assertAlmostEqual(
            float(np.mean(result["paths"][0]["points_xy"][:, 0])),
            66 * 4, delta=2.0)
        self.assertNotIn("temporal", result)

    def test_probability_outputs_are_not_transformed_again(self):
        result = decode_outputs(self.make_outputs(), (480, 640, 3))
        self.assertAlmostEqual(result["paths"][0]["score"], 0.92, places=6)
        self.assertAlmostEqual(result["paths"][1]["score"], 0.81, places=6)
        self.assertAlmostEqual(result["path_count_scores"][2], 0.90, places=6)
        self.assertAlmostEqual(result["centerline"]["count_confidence"],
                               0.90, places=6)

    def test_one_path_uses_single_role_and_slot_zero(self):
        outputs = list(self.make_outputs())
        outputs[5] = np.asarray([[0.03, 0.94, 0.03]], dtype=np.float32)
        result = decode_outputs(outputs, (480, 640, 3))
        self.assertEqual(result["path_count"], 1)
        self.assertEqual(len(result["curve_paths"]), 1)
        self.assertEqual(result["curve_paths"][0]["slot"], 0)
        self.assertEqual(result["curve_paths"][0]["role"], "single")
        self.assertEqual(len(result["paths"]), 2)
        self.assertEqual(result["paths"][0]["slot"], 0)
        self.assertEqual(result["paths"][0]["role"], "single")

    def test_zero_path_count_disables_both_slots(self):
        outputs = list(self.make_outputs())
        outputs[4] = np.asarray([[0.99, 0.99]], dtype=np.float32)
        outputs[5] = np.asarray([[0.96, 0.03, 0.01]], dtype=np.float32)
        result = decode_outputs(outputs, (480, 640, 3))
        self.assertEqual(result["path_count"], 0)
        self.assertEqual([item["role"] for item in result["paths"]],
                         ["single", "right"])
        self.assertEqual(result["curve_paths"], [])

    def test_path_points_are_clipped_before_mapping(self):
        outputs = list(self.make_outputs())
        outputs[3][0, 0, 0] = [-0.20, 1.20]
        result = decode_outputs(outputs, (241, 321, 3))
        point = result["curve_paths"][0]["points_xy"][0]
        np.testing.assert_allclose(point, [0.0, 240.0])

    def test_pixel_logits_receive_exactly_one_sigmoid(self):
        outputs = list(self.make_outputs())
        outputs[2][:] = 0.0
        result = decode_outputs(outputs, (480, 640, 3))
        self.assertAlmostEqual(float(result["road_probability"][0, 0]),
                               0.5, places=6)
        self.assertAlmostEqual(float(result["path_heatmaps"][0, 0, 0]),
                               0.5, places=6)
        self.assertEqual(int(result["road_mask"][0, 0]), 1)

    def test_classwise_nms_applies_global_limit_after_all_classes(self):
        boxes = np.asarray([
            [0, 0, 10, 10],
            [20, 0, 30, 10],
            [40, 0, 50, 10],
            [60, 0, 70, 10],
        ], dtype=np.float32)
        scores = np.zeros((4, 8), dtype=np.float32)
        scores[:3, 0] = [0.90, 0.85, 0.80]
        scores[3, 7] = 0.99

        results = detection_nms(
            boxes, scores, score_threshold=0.25,
            pre_nms_top_k=1000, max_detections=2)

        self.assertEqual([item[0] for item in results], [7, 0])
        self.assertAlmostEqual(results[0][1], 0.99, places=6)

    def test_nms_suppresses_overlap_only_within_same_class(self):
        boxes = np.asarray([
            [10, 10, 50, 50],
            [12, 12, 48, 48],
            [10, 10, 50, 50],
        ], dtype=np.float32)
        scores = np.zeros((3, 8), dtype=np.float32)
        scores[0, 0] = 0.90
        scores[1, 0] = 0.80
        scores[2, 1] = 0.85

        results = detection_nms(boxes, scores, max_detections=10)

        self.assertEqual(len(results), 2)
        self.assertEqual([item[0] for item in results], [0, 1])

    def test_tiny_coin_filter_matches_training_policy(self):
        boxes = np.asarray([
            [10, 10, 18, 30],
            [30, 10, 42, 30],
            [50, 10, 58, 30],
        ], dtype=np.float32)
        scores = np.zeros((3, 8), dtype=np.float32)
        scores[0, 4] = 0.99  # 8 px Coin: ignored by the training dataset.
        scores[1, 4] = 0.90  # 12 px Coin: retained.
        scores[2, 6] = 0.80  # Small non-Coin detections are unaffected.

        results = detection_nms(
            boxes, scores, score_threshold=0.25,
            coin_min_short_side=10.0, max_detections=10)

        self.assertEqual([(item[0], round(item[1], 2)) for item in results],
                         [(4, 0.90), (6, 0.80)])

    def test_render_uses_heatmap_paths_when_heatmap_array_is_not_retained(self):
        result = decode_outputs(self.make_outputs(), (240, 320, 3),
                                include_path_heatmaps=False)
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        rendered = render_result(image, result, mode="path")
        self.assertGreater(int(rendered.sum()), 0)

    def test_drive_render_limits_detection_clutter_per_class(self):
        detections = []
        for index in range(8):
            detections.append({
                "label": "Coin",
                "score": 0.95 - index * 0.01,
                "bbox": [10 + index * 5, 10, 20 + index * 5, 20],
            })
        detections.append({
            "label": "TurnSign", "score": 0.99,
            "bbox": [100, 40, 180, 100],
        })

        selected = _select_detections_for_render(detections, "drive")

        labels = [item["label"] for item in selected]
        self.assertLessEqual(len(selected), 6)
        self.assertEqual(labels.count("Coin"), 2)
        self.assertEqual(labels.count("TurnSign"), 1)

    def test_heatmap_render_draws_all_post_nms_detection_boxes(self):
        detections = [{
            "label": "Coin", "score": 0.30,
            "bbox": [index * 10, 10, index * 10 + 8, 20],
        } for index in range(8)]

        selected = _select_detections_for_render(detections, "heatmap")

        self.assertEqual(len(selected), 8)

    def test_default_render_mode_is_raw_heatmap(self):
        self.assertEqual(RENDER_MODE, "heatmap")

    def test_worker_prepares_rgb_uint8_without_copying_source_frame(self):
        outputs = self.make_outputs()

        class FakeRKNN:
            model_input = None

            def inference(self, inputs):
                self.model_input = inputs[0]
                return outputs

        runtime = FakeRKNN()
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        image[:] = [10, 20, 30]
        result = myFunc(runtime, image)

        self.assertEqual(runtime.model_input.shape, (1, 480, 640, 3))
        self.assertEqual(runtime.model_input.dtype, np.uint8)
        np.testing.assert_array_equal(runtime.model_input[0, 0, 0],
                                      [30, 20, 10])
        self.assertIs(result["_source_frame"], image)
        self.assertIs(result["frame"], image)
        self.assertIsNotNone(result["path_heatmaps"])
        self.assertIsNotNone(result["road_probability"])
        self.assertNotIn("temporal", result)

    def test_rejects_old_four_output_model(self):
        with self.assertRaises(ValueError):
            parse_outputs(self.make_outputs()[:4])


if __name__ == "__main__":
    unittest.main()
