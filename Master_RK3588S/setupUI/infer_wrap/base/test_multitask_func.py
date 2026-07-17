import unittest

import numpy as np

from .func import (
    CLASSES,
    DET_CAR_SCORE_THRESHOLD,
    DET_HUMAN_SCORE_THRESHOLD,
    DET_SCORE_THRESHOLD,
    DET_TURNSIGN_SCORE_THRESHOLD,
    RENDER_MODE,
    _select_detections_for_render,
    clean_road_mask,
    decode_outputs,
    detection_nms,
    myFunc,
    parse_outputs,
    render_result,
)


class MultiTaskPostprocessTest(unittest.TestCase):
    @staticmethod
    def make_outputs():
        boxes = np.zeros((1, 6300, 4), dtype=np.float32)
        scores = np.zeros((1, 6300, 8), dtype=np.float32)
        pixel = np.full((1, 3, 120, 160), -8.0, dtype=np.float32)
        pixel[0, 0, 40:120, 20:140] = 8.0
        row_logits = np.full((1, 2, 32, 161), -8.0, dtype=np.float32)
        row_logits[0, 0, :, 66] = 8.0
        row_logits[0, 1, :, 108] = 8.0
        path_scores = np.asarray([[0.92, 0.81]], dtype=np.float32)
        count_scores = np.asarray([[0.02, 0.08, 0.90]], dtype=np.float32)
        return boxes, scores, pixel, row_logits, path_scores, count_scores

    def test_output_contract(self):
        boxes, scores, pixel, rows, path_scores, count_scores = (
            parse_outputs(self.make_outputs()))
        self.assertEqual(boxes.shape, (6300, 4))
        self.assertEqual(scores.shape, (6300, 8))
        self.assertEqual(pixel.shape, (3, 120, 160))
        self.assertEqual(rows.shape, (2, 32, 161))
        self.assertEqual(path_scores.shape, (2,))
        self.assertEqual(count_scores.shape, (3,))

    def test_accepts_nhwc_outputs(self):
        outputs = list(self.make_outputs())
        outputs[2] = outputs[2].transpose(0, 2, 3, 1)
        outputs[3] = outputs[3].transpose(0, 2, 3, 1)
        parsed = parse_outputs(outputs)
        self.assertEqual(parsed[2].shape, (3, 120, 160))
        self.assertEqual(parsed[3].shape, (2, 32, 161))

    def test_decodes_only_unfiltered_curve_anchors(self):
        result = decode_outputs(self.make_outputs(), (240, 320, 3))

        self.assertEqual(result["model_path_count"], 2)
        self.assertEqual(len(result["raw_curve_paths"]), 2)
        self.assertEqual(
            [item["role"] for item in result["raw_curve_paths"]],
            ["left", "right"])
        self.assertTrue(all(
            item["source"] == "row_classifier_raw"
            for item in result["raw_curve_paths"]))
        self.assertEqual(
            result["raw_curve_paths"][0]["points_xy"].shape, (32, 2))
        self.assertAlmostEqual(
            float(result["raw_curve_paths"][0]["points_xy"][0, 0]),
            66 / 159 * 319, places=3)
        self.assertAlmostEqual(
            float(result["raw_curve_paths"][0]["points_xy"][0, 1]),
            239.0, places=4)
        for removed_key in (
            "paths", "curve_paths", "heatmap_ridge_paths",
            "path_heatmaps", "path_source",
        ):
            self.assertNotIn(removed_key, result)

    def test_raw_curve_keeps_all_anchors_without_threshold_gating(self):
        outputs = list(self.make_outputs())
        outputs[3][:] = -8.0
        outputs[3][0, :, :, 160] = 8.0
        outputs[4] = np.asarray([[0.0, 0.0]], dtype=np.float32)

        result = decode_outputs(outputs, (480, 640, 3))

        self.assertEqual(
            [len(path["points_xy"])
             for path in result["raw_curve_paths"]],
            [32, 32])
        self.assertTrue(all(
            len(path["point_confidences"]) == 32
            for path in result["raw_curve_paths"]))

    def test_probability_outputs_receive_one_transform(self):
        outputs = list(self.make_outputs())
        outputs[2][:] = 0.0
        result = decode_outputs(outputs, (480, 640, 3))
        self.assertAlmostEqual(
            float(result["road_probability"][0, 0]), 0.5, places=6)
        self.assertEqual(int(result["road_mask_raw"][0, 0]), 1)
        self.assertEqual(int(result["road_mask"][0, 0]), 0)
        self.assertFalse(result["road"]["valid"])

    def test_road_cleanup_fills_cracks_and_removes_islands(self):
        raw = np.zeros((40, 40), dtype=np.uint8)
        raw[10:40, 8:32] = 1
        raw[24:26, 18:20] = 0
        raw[5:7, 5:7] = 1
        cleaned, info = clean_road_mask(
            raw, top_crop_ratio=0.0, return_info=True)
        self.assertTrue(info["valid"])
        self.assertEqual(int(cleaned[24, 18]), 1)
        self.assertEqual(int(cleaned[5, 5]), 0)

    def test_road_cleanup_rejects_full_frame_mask(self):
        cleaned, info = clean_road_mask(
            np.ones((120, 160), dtype=np.uint8),
            top_crop_ratio=0.0, return_info=True)
        self.assertFalse(info["valid"])
        self.assertEqual(info["reason"], "raw-full")
        self.assertEqual(int(cleaned.sum()), 0)

    def test_turnsign_human_and_car_use_class_specific_detection_thresholds(self):
        self.assertAlmostEqual(0.50, DET_SCORE_THRESHOLD)
        self.assertAlmostEqual(0.40, DET_TURNSIGN_SCORE_THRESHOLD)
        self.assertAlmostEqual(0.40, DET_HUMAN_SCORE_THRESHOLD)
        self.assertAlmostEqual(0.40, DET_CAR_SCORE_THRESHOLD)
        boxes = np.asarray([
            [0.0, 0.0, 20.0, 20.0],
            [30.0, 0.0, 50.0, 20.0],
            [60.0, 0.0, 80.0, 20.0],
            [90.0, 0.0, 110.0, 20.0],
            [120.0, 0.0, 140.0, 20.0],
            [150.0, 0.0, 170.0, 20.0],
        ], dtype=np.float32)
        scores = np.zeros((6, len(CLASSES)), dtype=np.float32)
        scores[0, CLASSES.index("TurnSign")] = 0.35
        scores[1, CLASSES.index("TurnSign")] = 0.45
        scores[2, CLASSES.index("Car")] = 0.39
        scores[3, CLASSES.index("Car")] = 0.40
        scores[4, CLASSES.index("Human")] = 0.39
        scores[5, CLASSES.index("Human")] = 0.40
        results = detection_nms(boxes, scores)
        self.assertEqual(3, len(results))
        self.assertEqual(
            {"TurnSign", "Car", "Human"},
            {CLASSES[item[0]] for item in results})

    def test_nms_is_classwise_then_globally_limited(self):
        boxes = np.asarray([
            [0, 0, 10, 10], [20, 0, 30, 10],
            [40, 0, 50, 10], [60, 0, 70, 10],
        ], dtype=np.float32)
        scores = np.zeros((4, 8), dtype=np.float32)
        scores[:3, 0] = [0.90, 0.85, 0.80]
        scores[3, 7] = 0.99
        results = detection_nms(
            boxes, scores, score_threshold=0.25,
            pre_nms_top_k=1000, max_detections=2)
        self.assertEqual([item[0] for item in results], [7, 0])

    def test_drive_render_keeps_every_post_nms_detection(self):
        detections = [{
            "label": "Coin", "score": 0.95 - index * 0.01,
            "bbox": [10 + index * 5, 10, 20 + index * 5, 20],
        } for index in range(8)]
        detections.append({
            "label": "TurnSign", "score": 0.99,
            "bbox": [100, 40, 180, 100],
        })
        selected = _select_detections_for_render(detections, "drive")
        labels = [item["label"] for item in selected]
        self.assertEqual(len(selected), len(detections))
        self.assertEqual(labels.count("Coin"), 8)
        self.assertEqual(labels.count("TurnSign"), 1)

    def test_drive_render_draws_complete_thick_box_and_label(self):
        image = np.zeros((120, 180, 3), dtype=np.uint8)
        result = {"detections": [{
            "label": "Car", "score": 0.876,
            "bbox": [30, 40, 130, 100],
        }]}
        rendered = render_result(image, result, mode="drive")
        color = np.asarray([0, 255, 0], dtype=np.uint8)
        self.assertTrue(np.array_equal(rendered[40, 80], color))
        self.assertTrue(np.array_equal(rendered[100, 80], color))
        self.assertTrue(np.array_equal(rendered[70, 30], color))
        self.assertTrue(np.array_equal(rendered[70, 130], color))
        self.assertGreater(int(rendered[20:40, 30:130].sum()), 0)

    def test_render_draws_only_finalized_paths(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        result = {
            "detections": [],
            "paths": [{
                "slot": 0,
                "role": "left",
                "score": 0.9,
                "count_confidence": 0.9,
                "points_xy": np.asarray(
                    [[160, 230], [150, 180], [145, 130]],
                    dtype=np.float32),
            }],
        }
        self.assertGreater(int(render_result(image, result).sum()), 0)

    def test_default_render_mode_is_drive(self):
        self.assertEqual(RENDER_MODE, "drive")

    def test_worker_returns_raw_curve_without_legacy_paths(self):
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
        np.testing.assert_array_equal(
            runtime.model_input[0, 0, 0], [30, 20, 10])
        self.assertIs(result["_source_frame"], image)
        self.assertIsNone(result["road_probability"])
        self.assertEqual(len(result["raw_curve_paths"]), 2)
        self.assertNotIn("paths", result)
        self.assertNotIn("path_heatmaps", result)

    def test_rejects_old_four_output_model(self):
        with self.assertRaises(ValueError):
            parse_outputs(self.make_outputs()[:4])


if __name__ == "__main__":
    unittest.main()
