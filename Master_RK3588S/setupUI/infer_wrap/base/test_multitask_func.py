import unittest

import numpy as np

from .func import (
    CLASSES,
    DET_SCORE_THRESHOLD,
    DET_TURNSIGN_SCORE_THRESHOLD,
    PATH_SOURCE,
    RENDER_MODE,
    _is_oversized_render_box,
    _select_detections_for_render,
    clean_road_mask,
    decode_outputs,
    detection_nms,
    myFunc,
    parse_outputs,
    render_result,
)


class MultiTaskPostprocessTest(unittest.TestCase):
    def test_turnsign_threshold_is_point_four_while_other_classes_use_point_five(self):
        self.assertAlmostEqual(0.50, DET_SCORE_THRESHOLD)
        self.assertAlmostEqual(0.40, DET_TURNSIGN_SCORE_THRESHOLD)

        boxes = np.asarray([
            [0.0, 0.0, 20.0, 20.0],
            [30.0, 0.0, 50.0, 20.0],
            [60.0, 0.0, 80.0, 20.0],
        ], dtype=np.float32)
        scores = np.zeros((3, len(CLASSES)), dtype=np.float32)
        scores[0, CLASSES.index("TurnSign")] = 0.35
        scores[1, CLASSES.index("TurnSign")] = 0.45
        scores[2, CLASSES.index("Human")] = 0.55

        results = detection_nms(boxes, scores)

        self.assertEqual(2, len(results))
        self.assertEqual(
            {CLASSES.index("TurnSign"), CLASSES.index("Human")},
            {item[0] for item in results},
        )

    def make_outputs(self):
        boxes = np.zeros((1, 6300, 4), dtype=np.float32)
        scores = np.zeros((1, 6300, 8), dtype=np.float32)
        pixel = np.full((1, 3, 120, 160), -8.0, dtype=np.float32)
        pixel[0, 0, 40:120, 20:140] = 8.0
        pixel[0, 1, 40:119, 66] = 8.0
        pixel[0, 2, 40:119, 108] = 8.0
        row_logits = np.full((1, 2, 32, 161), -8.0, dtype=np.float32)
        row_logits[0, 0, :, 66] = 8.0
        row_logits[0, 1, :, 108] = 8.0
        path_scores = np.asarray([[0.92, 0.81]], dtype=np.float32)
        count_scores = np.asarray([[0.02, 0.08, 0.90]], dtype=np.float32)
        return boxes, scores, pixel, row_logits, path_scores, count_scores

    def test_output_contract(self):
        boxes, scores, pixel, row_logits, path_scores, count_scores = (
            parse_outputs(self.make_outputs()))
        self.assertEqual(boxes.shape, (6300, 4))
        self.assertEqual(scores.shape, (6300, 8))
        self.assertEqual(pixel.shape, (3, 120, 160))
        self.assertEqual(row_logits.shape, (2, 32, 161))
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

    def test_accepts_nhwc_row_classifier_output(self):
        outputs = list(self.make_outputs())
        outputs[3] = outputs[3].transpose(0, 2, 3, 1)
        parsed = parse_outputs(outputs)
        self.assertEqual(parsed[3].shape, (2, 32, 161))

    def test_decodes_row_classifier_paths_for_model_comparison(self):
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
            66 / 159 * 319, places=3)
        self.assertAlmostEqual(
            float(result["curve_paths"][0]["points_xy"][0, 1]),
            239.0, places=4)

    def test_default_paths_use_row_classifier(self):
        result = decode_outputs(self.make_outputs(), (480, 640, 3))

        self.assertEqual(
            result["centerline"]["display_source"], "row_classifier")
        self.assertEqual(
            result["centerline"]["path_source"], "row_classifier")
        self.assertEqual([item["role"] for item in result["paths"]],
                          ["left", "right"])
        self.assertTrue(all(item["source"] == "row_classifier"
                              for item in result["paths"]))
        self.assertEqual(len(result["paths"][0]["points_xy"]), 32)
        self.assertAlmostEqual(
            float(np.mean(result["paths"][0]["points_xy"][:, 0])),
            66 / 159 * 639, delta=2.0)
        self.assertEqual(result["paths"], result["curve_paths"])
        self.assertNotIn("temporal", result)

    def test_heatmap_switch_restores_one_ridge_per_channel(self):
        result = decode_outputs(
            self.make_outputs(), (480, 640, 3), path_source="heatmap")
        self.assertEqual(result["centerline"]["path_source"], "heatmap_ridge")
        self.assertTrue(all(item["source"] == "heatmap_ridge"
                            for item in result["paths"]))
        self.assertEqual(result["paths"], result["heatmap_ridge_paths"])

    def test_keeps_only_longest_ridge_in_each_heatmap(self):
        outputs = list(self.make_outputs())
        outputs[2][0, 2, 24:72, 132] = 8.0

        result = decode_outputs(outputs, (480, 640, 3), path_source="heatmap")

        right_paths = [item for item in result["paths"]
                       if item["slot"] == 1]
        self.assertEqual(len(right_paths), 1)
        self.assertAlmostEqual(
            float(np.mean(right_paths[0]["points_xy"][:, 0])),
            108 * 4, delta=2.0)

    def test_ridge_bridges_single_missing_heatmap_row(self):
        outputs = list(self.make_outputs())
        outputs[2][0, 2, 60, 108] = -8.0

        result = decode_outputs(outputs, (480, 640, 3), path_source="heatmap")
        right_paths = [item for item in result["paths"]
                       if item["slot"] == 1]

        self.assertEqual(len(right_paths), 1)
        self.assertGreater(len(right_paths[0]["points_xy"]), 30)

    def test_probability_outputs_are_not_transformed_again(self):
        result = decode_outputs(self.make_outputs(), (480, 640, 3))
        self.assertAlmostEqual(result["paths"][0]["score"], 0.92, places=6)
        self.assertAlmostEqual(result["paths"][1]["score"], 0.81, places=6)
        self.assertAlmostEqual(result["path_count_scores"][2], 0.90, places=6)
        self.assertAlmostEqual(result["centerline"]["count_confidence"],
                               0.90, places=6)

    def test_row_classifier_one_path_uses_single_role_and_slot_zero(self):
        outputs = list(self.make_outputs())
        outputs[5] = np.asarray([[0.03, 0.94, 0.03]], dtype=np.float32)
        outputs[3][0, 1, :, :] = -8.0
        outputs[3][0, 1, :, 160] = 8.0
        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")
        self.assertEqual(result["path_count"], 1)
        self.assertEqual(len(result["curve_paths"]), 1)
        self.assertEqual(result["curve_paths"][0]["slot"], 0)
        self.assertEqual(result["curve_paths"][0]["role"], "single")
        self.assertEqual(len(result["paths"]), 1)
        self.assertEqual(result["paths"][0]["slot"], 0)
        self.assertEqual(result["paths"][0]["role"], "single")

    def test_row_support_is_not_erased_by_count_head(self):
        outputs = list(self.make_outputs())
        outputs[4] = np.asarray([[0.99, 0.99]], dtype=np.float32)
        outputs[5] = np.asarray([[0.96, 0.03, 0.01]], dtype=np.float32)
        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")
        self.assertEqual(result["model_path_count"], 0)
        self.assertEqual(result["path_count"], 2)
        self.assertEqual(len(result["curve_paths"]), 2)

    def test_green_curve_uses_its_lower_score_gate_only(self):
        outputs = list(self.make_outputs())
        outputs[4] = np.asarray([[0.02, 0.04]], dtype=np.float32)

        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")

        self.assertEqual(len(result["curve_paths"]), 1)
        self.assertEqual(result["curve_paths"][0]["slot"], 1)

    def test_green_curve_uses_relaxed_row_visibility_only(self):
        outputs = list(self.make_outputs())
        outputs[3][:] = -12.0
        outputs[3][0, :, :, 108] = 0.0
        # With one coordinate bin at 0.0 and the no-path bin at log(3),
        # no-path probability is 0.75: rejected by slot 0's 0.50 gate but
        # accepted by slot 1's deliberately relaxed 0.93 gate.
        outputs[3][0, :, :, 160] = np.log(3.0)

        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")

        self.assertEqual(len(result["curve_paths"]), 1)
        self.assertEqual(result["curve_paths"][0]["slot"], 1)

    def test_green_curve_bridges_six_missing_row_anchors(self):
        outputs = list(self.make_outputs())
        outputs[3][0, 1, 2:8, :] = -8.0
        outputs[3][0, 1, 2:8, 160] = 8.0

        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")
        green = next(
            path for path in result["curve_paths"] if path["slot"] == 1)

        self.assertIn(1, green["row_indices"])
        self.assertIn(8, green["row_indices"])
        self.assertEqual(len(green["display_segments_xy"]), 1)

    def test_raw_curve_preview_keeps_all_points_without_gating(self):
        outputs = list(self.make_outputs())
        outputs[3][:] = -8.0
        outputs[3][0, :, :, 160] = 8.0
        outputs[4] = np.asarray([[0.0, 0.0]], dtype=np.float32)

        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")

        self.assertEqual(len(result["curve_paths"]), 0)
        self.assertEqual(len(result["raw_curve_paths"]), 2)
        self.assertEqual(
            [len(path["points_xy"])
             for path in result["raw_curve_paths"]],
            [32, 32],
        )

    def test_row_path_is_projected_inside_road_mask(self):
        outputs = list(self.make_outputs())
        outputs[3][0, 0, :, :] = -8.0
        outputs[3][0, 0, :, 10] = 8.0
        result = decode_outputs(outputs, (241, 321, 3))
        path = result["curve_paths"][0]
        self.assertTrue(path["road_constrained"])
        self.assertTrue(np.all(path["points_xy"][:, 0] >= 20 / 159 * 320))
        expected_y = np.linspace(1.0, 0.40, 32) * 240
        np.testing.assert_allclose(
            path["points_xy"][:, 1], expected_y, atol=1e-4)

    def test_row_path_truncates_tail_after_large_lateral_jump(self):
        outputs = list(self.make_outputs())
        outputs[3][0, 1, :, :] = -8.0
        outputs[3][0, 1, :24, 108] = 8.0
        outputs[3][0, 1, 24:, 30] = 8.0

        result = decode_outputs(outputs, (480, 640, 3))
        right = result["curve_paths"][1]

        self.assertEqual(right["row_indices"].tolist(), list(range(24)))
        self.assertEqual(len(right["points_xy"]), 24)

    def test_pixel_logits_receive_exactly_one_sigmoid(self):
        outputs = list(self.make_outputs())
        outputs[2][:] = 0.0
        result = decode_outputs(outputs, (480, 640, 3))
        self.assertAlmostEqual(float(result["road_probability"][0, 0]),
                               0.5, places=6)
        self.assertAlmostEqual(float(result["path_heatmaps"][0, 0, 0]),
                               0.5, places=6)
        self.assertEqual(int(result["road_mask_raw"][0, 0]), 1)
        self.assertEqual(int(result["road_mask"][0, 0]), 0)
        self.assertFalse(result["road"]["valid"])
        self.assertEqual(result["road"]["reason"], "raw-full")

    def test_road_cleanup_fills_cracks_and_removes_small_islands(self):
        raw = np.zeros((40, 40), dtype=np.uint8)
        raw[10:40, 8:32] = 1
        raw[24:26, 18:20] = 0
        raw[5:7, 5:7] = 1

        cleaned, info = clean_road_mask(
            raw, top_crop_ratio=0.0, return_info=True)

        self.assertTrue(info["valid"])
        self.assertEqual(int(cleaned[24, 18]), 1)
        self.assertEqual(int(cleaned[5, 5]), 0)
        self.assertEqual(info["component_count"], 1)

    def test_road_cleanup_rejects_full_frame_false_positive(self):
        raw = np.ones((120, 160), dtype=np.uint8)

        cleaned, info = clean_road_mask(
            raw, top_crop_ratio=0.0, return_info=True)

        self.assertFalse(info["valid"])
        self.assertEqual(info["reason"], "raw-full")
        self.assertEqual(int(cleaned.sum()), 0)

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

    def test_each_candidate_is_assigned_only_to_its_highest_class(self):
        boxes = np.asarray([[0, 0, 100, 100]], dtype=np.float32)
        scores = np.zeros((1, 8), dtype=np.float32)
        scores[0, 0] = 0.90
        scores[0, 1] = 0.80

        results = detection_nms(
            boxes, scores, score_threshold=0.50,
            coin_min_short_side=0, max_detections=10)

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0][0], 0)
        self.assertAlmostEqual(results[0][1], 0.90, places=6)

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

    def test_render_uses_row_curve_when_heatmap_array_is_not_retained(self):
        result = decode_outputs(self.make_outputs(), (240, 320, 3),
                                include_path_heatmaps=False)
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        rendered = render_result(image, result, mode="path")
        self.assertGreater(int(rendered.sum()), 0)

    def test_render_clips_heatmap_pixels_to_road_mask(self):
        outputs = list(self.make_outputs())
        outputs[2][0, 1, 80, 10] = 8.0
        result = decode_outputs(outputs, (480, 640, 3))
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        rendered = render_result(image, result, mode="heatmap")

        self.assertEqual(int(rendered[320, 40].sum()), 0)
        self.assertGreater(int(rendered[320, 264].sum()), 0)

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
            "label": "Coin", "score": 0.60,
            "bbox": [index * 10, 10, index * 10 + 8, 20],
        } for index in range(8)]

        selected = _select_detections_for_render(detections, "heatmap")

        self.assertEqual(len(selected), 8)

    def test_frame_spanning_detection_box_is_hidden_from_preview(self):
        large = {"bbox": [5, 40, 635, 180]}
        normal = {"bbox": [100, 100, 220, 240]}

        self.assertTrue(_is_oversized_render_box(large, (480, 640, 3)))
        self.assertFalse(_is_oversized_render_box(normal, (480, 640, 3)))

    def test_default_path_source_is_curve(self):
        self.assertEqual(PATH_SOURCE, "curve")

    def test_default_render_mode_is_drive(self):
        self.assertEqual(RENDER_MODE, "drive")

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
        self.assertIsNone(result["path_heatmaps"])
        self.assertIsNone(result["road_probability"])
        self.assertNotIn("_pixel_logits", result)
        self.assertEqual(result["curve_paths"], [])
        self.assertEqual(result["paths"], [])
        self.assertEqual(len(result["raw_curve_paths"]), 2)
        self.assertNotIn("temporal", result)

    def test_rejects_old_four_output_model(self):
        with self.assertRaises(ValueError):
            parse_outputs(self.make_outputs()[:4])


if __name__ == "__main__":
    unittest.main()
