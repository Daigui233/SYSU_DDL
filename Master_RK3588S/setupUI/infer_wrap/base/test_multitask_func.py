import unittest

import numpy as np
import cv2

from .func import (
    CLASSES,
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
    simple_road_mask,
)
from .road_mask_diagnostics import (
    build_birds_eye_mask,
    detect_frontier_exits,
    plan_center_biased_paths,
    plan_medial_axis_paths,
    road_clearance_map,
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

    def test_default_paths_use_heatmap_ridges(self):
        result = decode_outputs(self.make_outputs(), (480, 640, 3))

        self.assertEqual(
            result["centerline"]["display_source"], "heatmap_ridge")
        self.assertEqual(
            result["centerline"]["path_source"], "heatmap_ridge")
        self.assertEqual([item["role"] for item in result["paths"]],
                          ["left", "right"])
        self.assertTrue(all(item["source"] == "heatmap_ridge"
                             for item in result["paths"]))
        self.assertGreater(len(result["paths"][0]["points_xy"]), 32)
        self.assertAlmostEqual(
            float(np.mean(result["paths"][0]["points_xy"][:, 0])),
            66 * 4, delta=2.0)
        self.assertEqual(result["paths"], result["heatmap_ridge_paths"])
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

    def test_direct_curve_one_path_uses_single_role_and_slot_zero(self):
        outputs = list(self.make_outputs())
        outputs[5] = np.asarray([[0.03, 0.94, 0.03]], dtype=np.float32)
        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")
        self.assertEqual(result["path_count"], 1)
        self.assertEqual(len(result["curve_paths"]), 1)
        self.assertEqual(result["curve_paths"][0]["slot"], 0)
        self.assertEqual(result["curve_paths"][0]["role"], "single")
        self.assertEqual(len(result["paths"]), 1)
        self.assertEqual(result["paths"][0]["slot"], 0)
        self.assertEqual(result["paths"][0]["role"], "single")

    def test_direct_curve_zero_path_count_disables_both_slots(self):
        outputs = list(self.make_outputs())
        outputs[4] = np.asarray([[0.99, 0.99]], dtype=np.float32)
        outputs[5] = np.asarray([[0.96, 0.03, 0.01]], dtype=np.float32)
        result = decode_outputs(
            outputs, (480, 640, 3), path_source="curve")
        self.assertEqual(result["path_count"], 0)
        self.assertEqual(result["paths"], [])
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

    def test_simple_road_mask_only_closes_small_gaps(self):
        raw = np.zeros((9, 9), dtype=np.uint8)
        raw[2:7, 2:7] = 1
        raw[4, 4] = 0
        raw[0, 0] = 1

        mask = simple_road_mask(raw, close_iterations=1)

        self.assertEqual(int(mask[4, 4]), 1)
        self.assertEqual(int(mask[0, 0]), 1)

    def test_decode_exposes_simple_road_mask_without_old_rejection(self):
        result = decode_outputs(self.make_outputs(), (480, 640, 3))

        self.assertEqual(int(result["road_mask_simple"].sum()), 120 * 160)
        self.assertIs(result["road"]["simple_mask"],
                      result["road_mask_simple"])
        self.assertEqual(int(result["road_mask"].sum()), 0)

    def test_road_mask_render_draws_only_simple_mask(self):
        image = np.zeros((8, 8, 3), dtype=np.uint8)
        result = {
            "road_mask_simple": np.pad(
                np.ones((2, 2), dtype=np.uint8), ((1, 1), (1, 1))),
            "detections": [{
                "label": "Coin", "score": 0.99,
                "bbox": [0, 0, 7, 7],
            }],
            "paths": [{
                "role": "single", "score": 1.0,
                "count_confidence": 1.0,
                "points_xy": np.asarray([[0, 0], [7, 7]]),
            }],
        }

        rendered = render_result(image, result, mode="road_mask")

        self.assertEqual(int(rendered[0, 0].sum()), 0)
        self.assertGreater(int(rendered[4, 4, 2]), 0)
        self.assertEqual(int(rendered[4, 4, 0]), 0)
        self.assertEqual(int(rendered[4, 4, 1]), 0)

    def test_frontier_detects_two_persistent_y_exits(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        cv2.line(mask, (40, 59), (40, 34), 1, 9)
        cv2.line(mask, (40, 36), (18, 0), 1, 9)
        cv2.line(mask, (40, 36), (62, 0), 1, 9)

        exits, frontier = detect_frontier_exits(
            mask, band_width=4, side_reach_ratio=0.7,
            minimum_area=6)

        self.assertEqual(len(exits), 2)
        self.assertTrue(np.all(frontier[:4]))
        self.assertLess(exits[0]["center_xy"][0],
                        exits[1]["center_xy"][0])

    def test_frontier_treats_straight_road_as_one_exit(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[:, 32:48] = 1

        exits, _frontier = detect_frontier_exits(mask)

        self.assertEqual(len(exits), 1)
        self.assertEqual(exits[0]["boundary"], "top")

    def test_birds_eye_mask_keeps_binary_shape_and_source_quad(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[25:, 10:70] = 1

        birds_eye, matrix, source, destination = build_birds_eye_mask(mask)

        self.assertEqual(birds_eye.shape, mask.shape)
        self.assertEqual(set(np.unique(birds_eye).tolist()), {0, 1})
        self.assertEqual(matrix.shape, (3, 3))
        self.assertEqual(source.shape, (4, 2))
        self.assertEqual(destination.shape, (4, 2))

    def test_clearance_is_highest_away_from_road_edges(self):
        mask = np.zeros((20, 30), dtype=np.uint8)
        mask[:, 8:22] = 1

        clearance = road_clearance_map(mask)

        self.assertGreater(float(clearance[10, 15]),
                           float(clearance[10, 8]))
        self.assertGreater(float(clearance[10, 15]), 5.0)

    def test_one_dijkstra_tree_returns_two_shared_trunk_paths(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        cv2.line(mask, (40, 59), (40, 34), 1, 11)
        cv2.line(mask, (40, 36), (18, 0), 1, 11)
        cv2.line(mask, (40, 36), (62, 0), 1, 11)
        exits, _frontier = detect_frontier_exits(mask)

        start, paths, clearance = plan_center_biased_paths(mask, exits)

        self.assertIsNotNone(start)
        self.assertEqual(len(paths), 2)
        self.assertEqual(clearance.shape, mask.shape)
        first = paths[0]["points_xy"].astype(np.int32)
        second = paths[1]["points_xy"].astype(np.int32)
        common = 0
        for point_a, point_b in zip(first, second):
            if not np.array_equal(point_a, point_b):
                break
            common += 1
        self.assertGreaterEqual(common, 8)
        for path_info in paths:
            points = path_info["points_xy"].astype(np.int32)
            self.assertTrue(np.all(mask[points[:, 1], points[:, 0]] != 0))

    def test_medial_axis_returns_two_true_shared_trunk_paths(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        cv2.line(mask, (40, 59), (40, 34), 1, 11)
        cv2.line(mask, (40, 36), (18, 0), 1, 11)
        cv2.line(mask, (40, 36), (62, 0), 1, 11)
        exits, _frontier = detect_frontier_exits(mask)

        start, paths, skeleton, _clearance = plan_medial_axis_paths(
            mask, exits, planning_downsample=1)

        self.assertIsNotNone(start)
        self.assertEqual(len(paths), 2)
        self.assertGreater(int(skeleton.sum()), 20)
        first = paths[0]["points_xy"].astype(np.int32)
        second = paths[1]["points_xy"].astype(np.int32)
        common = 0
        for point_a, point_b in zip(first, second):
            if not np.array_equal(point_a, point_b):
                break
            common += 1
        self.assertGreater(common, 10)
        self.assertTrue(all(
            path_info["source"] == "medial_axis"
            for path_info in paths))

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

    def test_low_confidence_threshold_is_lowered_for_turnsign_only(self):
        boxes = np.asarray([
            [0, 0, 20, 20],
            [30, 0, 50, 20],
            [60, 0, 80, 20],
        ], dtype=np.float32)
        scores = np.zeros((3, 8), dtype=np.float32)
        scores[0, 1] = 0.20
        scores[1, 0] = 0.20
        scores[2, 1] = 0.19

        results = detection_nms(
            boxes, scores, score_threshold=0.50,
            turnsign_score_threshold=0.20,
            coin_min_short_side=0, max_detections=10)

        self.assertEqual(1, len(results))
        self.assertEqual(1, results[0][0])
        self.assertAlmostEqual(0.20, results[0][1], places=6)

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

    def test_render_uses_direct_curve_when_heatmap_array_is_not_retained(self):
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

    def test_render_keeps_low_confidence_turnsign_but_not_other_class(self):
        detections = [{
            "label": "TurnSign", "score": 0.20,
            "bbox": [100, 40, 180, 100],
        }, {
            "label": "Coin", "score": 0.20,
            "bbox": [30, 40, 60, 70],
        }]

        selected = _select_detections_for_render(detections, "drive")

        self.assertEqual(["TurnSign"], [item["label"] for item in selected])

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

    def test_default_path_source_is_heatmap(self):
        self.assertEqual(PATH_SOURCE, "heatmap")

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
