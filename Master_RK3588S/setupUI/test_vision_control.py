import os
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np


sys.path.insert(0, os.path.dirname(__file__))

from vision_control import (  # noqa: E402
    HeatmapPeakPathDetector,
    HeatmapPathSearch,
    ROUTE_MULTI_FORK,
    STATE_AVOID_CAR,
    STATE_AVOID_HUMAN,
    STATE_RECOVER_LINE,
    STATE_SAFE_STOP,
    STATE_TRACK,
    VisionControlConfig,
    VisionControlPlanner,
    _connect_binary_mask_samples,
    _draw_semantic_heat_distribution,
    _extract_heatmap_preview_lines,
    _identity_probability_color,
    _fill_binary_mask,
    _smooth_binary_mask_edges,
    _spatial_hysteresis_binary_mask,
    render_vision_control_debug,
)


def _config(**overrides):
    values = {
        "heat_threshold": 0.10,
        "heat_peak_top_k": 4,
        "row_step": 3,
        "min_path_points": 8,
        "min_path_coverage": 0.10,
        "min_mean_heat": 0.20,
        "max_link_jump_px": 12.0,
        "max_error_step_640": 1000.0,
        "default_outer_after_s": 15.0,
        "path_source": "heatmap",
    }
    values.update(overrides)
    return VisionControlConfig(**values)


def _draw_heat_path(heatmap, x_at_y):
    _draw_heat_path_range(heatmap, x_at_y, 20, heatmap.shape[0])


def _draw_heat_path_range(heatmap, x_at_y, start_y, end_y):
    h, w = heatmap.shape
    xs = np.arange(w, dtype=np.float32)
    for y in range(max(0, start_y), min(h, end_y)):
        x = float(x_at_y(y))
        row = np.exp(-0.5 * ((xs - x) / 1.6) ** 2)
        heatmap[y] = np.maximum(heatmap[y], row)


def _fork_heatmaps(shared_shift=0.0):
    h, w = 120, 160
    heatmaps = np.zeros((2, h, w), dtype=np.float32)

    def split_t(y):
        return np.clip((118.0 - float(y)) / 88.0, 0.0, 1.0)

    def shared_offset(y):
        return float(shared_shift) * (1.0 - split_t(y))

    _draw_heat_path(
        heatmaps[0], lambda y: 80.0 - 28.0 * split_t(y) + shared_offset(y))
    _draw_heat_path(
        heatmaps[1], lambda y: 80.0 + 28.0 * split_t(y) + shared_offset(y))
    return heatmaps


def _straight_heatmap(x):
    heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
    _draw_heat_path(heatmaps[0], lambda _y: float(x))
    return heatmaps


def _result(heatmaps, detections=None):
    return {
        "centerline": {"heatmaps": heatmaps},
        "road": {"mask": np.ones(heatmaps.shape[1:], dtype=np.float32)},
        "image_shape": (480, 640, 3),
        "detections": list(detections or []),
    }


def _curve_result(detections=None):
    ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
    left = np.stack((np.full_like(ys, 240.0), ys), axis=1)
    right = np.stack((np.full_like(ys, 400.0), ys), axis=1)
    return {
        "centerline": {"curve_paths": [
            {"slot": 0, "role": "left", "score": 0.9, "points_xy": left},
            {"slot": 1, "role": "right", "score": 0.9, "points_xy": right},
        ]},
        "image_shape": (480, 640, 3),
        "detections": list(detections or []),
    }


def _ocr_response(direction="right", active=False):
    return {
        "active": active,
        "instruction_current": True,
        "instruction": {"direction": direction},
    }


def _path_x_at(points, y):
    points = np.asarray(points, dtype=np.float32)
    order = np.argsort(points[:, 1])
    return float(np.interp(float(y), points[order, 1], points[order, 0]))


def _confirm_ocr(planner, direction="right", start=2.0, active=False):
    debug = None
    command = None
    command, debug = planner.update(
        _result(_fork_heatmaps()),
        _ocr_response(direction, active=active),
        now=start,
    )
    return command, debug


class HeatmapPeakPathDetectorTest(unittest.TestCase):
    def test_skips_small_high_peak_and_uses_next_two_valid_peaks(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        left = _straight_heatmap(34)[0] * 0.70
        right = _straight_heatmap(126)[0] * 0.65
        heatmaps[0] = np.maximum(left, right)
        heatmaps[0, 54:66, 79:82] = 1.0
        detector = HeatmapPeakPathDetector(_config(
            min_peak_component_area=50,
            bottom_reach_ratio=0.90,
        ))

        paths, debug = detector.extract(heatmaps, (480, 640, 3))

        self.assertEqual(2, len(paths))
        self.assertEqual(2, debug["detected_path_count"])
        self.assertEqual("component_too_small", debug["peaks_examined"][0]["reason"])
        self.assertEqual([34, 126], sorted(
            int(path["peak_x"]) for path in paths))

    def test_peak_below_high_threshold_is_ignored(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0] = np.maximum(
            _straight_heatmap(46)[0] * 0.80,
            _straight_heatmap(112)[0] * 0.08)
        detector = HeatmapPeakPathDetector(_config(
            heat_threshold=0.10,
            bottom_reach_ratio=0.90,
        ))

        paths, debug = detector.extract(heatmaps, (480, 640, 3))

        self.assertEqual(1, len(paths))
        self.assertEqual(1, debug["detected_path_count"])
        self.assertTrue(any(
            item.get("reason") == "peak_probability_too_low"
            for item in debug["peaks_examined"]))

    def test_path_must_reach_configured_bottom_region(self):
        short = np.zeros((1, 120, 160), dtype=np.float32)
        _draw_heat_path_range(short[0], lambda _y: 80.0, 20, 106)
        long_enough = np.zeros_like(short)
        _draw_heat_path_range(
            long_enough[0], lambda _y: 80.0, 20, 109)
        detector = HeatmapPeakPathDetector(_config(
            bottom_reach_ratio=0.90,
        ))

        short_paths, short_debug = detector.extract(
            short, (480, 640, 3))
        long_paths, long_debug = detector.extract(
            long_enough, (480, 640, 3))

        self.assertEqual([], short_paths)
        self.assertEqual(0, short_debug["valid_path_count"])
        self.assertEqual(1, len(long_paths))
        self.assertEqual(1, long_debug["valid_path_count"])
        self.assertGreaterEqual(
            float(np.max(long_paths[0]["points_xy"][:, 1])),
            0.90 * 479.0)

    def test_probability_below_point_zero_five_is_blank(self):
        at_threshold = np.zeros((1, 120, 160), dtype=np.float32)
        at_threshold[0, 20:72, 79:82] = 0.80
        at_threshold[0, 72:109, 79:82] = 0.05
        below_threshold = at_threshold.copy()
        below_threshold[0, 72:109, 79:82] = 0.049
        detector = HeatmapPeakPathDetector(_config(
            blank_probability=0.05,
            bottom_reach_ratio=0.90,
            min_peak_component_area=10,
        ))

        valid_paths, _debug = detector.extract(
            at_threshold, (480, 640, 3))
        blank_paths, blank_debug = detector.extract(
            below_threshold, (480, 640, 3))

        self.assertEqual(1, len(valid_paths))
        self.assertEqual([], blank_paths)
        self.assertEqual(0.05, blank_debug["blank_probability"])

    def test_ar_preview_overlay_contains_detected_path_count(self):
        result = _result(_fork_heatmaps())
        VisionControlPlanner(config=_config()).update(result, now=1.0)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        with patch("vision_control.cv2.putText") as put_text:
            render_vision_control_debug(frame, result)

        texts = [str(call.args[1]) for call in put_text.call_args_list]
        self.assertTrue(any(
            text.startswith("PATH COUNT: 2") for text in texts))

    def test_short_occlusion_gap_is_reconnected_to_strong_lower_line(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        curve = lambda y: 70.0 + 0.10 * float(y)
        _draw_heat_path_range(heatmaps[0], curve, 20, 76)
        _draw_heat_path_range(heatmaps[0], curve, 84, 120)
        detector = HeatmapPeakPathDetector(_config(
            recovery_max_gap_rows=12,
            recovery_min_probability=0.35,
        ))

        paths, debug = detector.extract(heatmaps, (480, 640, 3))

        self.assertEqual(1, len(paths))
        self.assertEqual(1, debug["valid_path_count"])
        self.assertEqual(1, paths[0]["occlusion_bridge_count"])
        self.assertEqual(8, paths[0]["occlusion_bridge_rows"])
        self.assertEqual("bottom", paths[0]["exit_type"])

    def test_ambiguous_lower_lines_are_not_reconnected(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        _draw_heat_path_range(
            heatmaps[0], lambda _y: 80.0, 20, 76)
        _draw_heat_path_range(
            heatmaps[0], lambda _y: 73.0, 84, 120)
        _draw_heat_path_range(
            heatmaps[0], lambda _y: 87.0, 84, 120)
        detector = HeatmapPeakPathDetector(_config(
            recovery_ambiguity_margin=0.08,
        ))

        paths, debug = detector.extract(heatmaps, (480, 640, 3))

        self.assertEqual([], paths)
        self.assertEqual(0, debug["valid_path_count"])
        self.assertEqual(
            "does_not_reach_allowed_exit",
            debug["peaks_examined"][0]["reason"])

    def test_quadratic_fit_filters_row_to_row_jitter(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)

        def base_curve(y):
            return 72.0 + 0.002 * (float(y) - 70.0) ** 2

        def noisy_curve(y):
            return base_curve(y) + (2.0 if y % 2 else -2.0)

        _draw_heat_path_range(heatmaps[0], noisy_curve, 20, 120)
        detector = HeatmapPeakPathDetector(_config(curve_fit_blend=0.50))

        paths, _debug = detector.extract(heatmaps, (480, 640, 3))

        self.assertEqual(1, len(paths))
        path = paths[0]
        heat_xs = path["points_xy"][:, 0] * 159.0 / 639.0
        heat_ys = path["points_xy"][:, 1] * 119.0 / 479.0
        expected = np.asarray(
            [base_curve(y) for y in heat_ys], dtype=np.float32)
        filtered_rmse = float(np.sqrt(np.mean(
            (heat_xs - expected) ** 2)))
        self.assertEqual(2, path["curve_fit_degree"])
        self.assertLess(filtered_rmse, 1.2)

    def test_side_exit_is_valid_only_in_lower_third(self):
        def side_curve(y):
            if y <= 65:
                return 30.0
            return max(0.0, 30.0 - 1.5 * (float(y) - 65.0))

        lower_exit = np.zeros((1, 120, 160), dtype=np.float32)
        _draw_heat_path_range(lower_exit[0], side_curve, 20, 90)
        upper_exit = np.zeros_like(lower_exit)
        _draw_heat_path_range(upper_exit[0], side_curve, 20, 75)
        detector = HeatmapPeakPathDetector(_config(
            side_exit_min_y_ratio=2.0 / 3.0,
            side_exit_margin_ratio=0.05,
        ))

        lower_paths, _lower_debug = detector.extract(
            lower_exit, (480, 640, 3))
        upper_paths, _upper_debug = detector.extract(
            upper_exit, (480, 640, 3))

        self.assertEqual(1, len(lower_paths))
        self.assertEqual("left_side", lower_paths[0]["exit_type"])
        self.assertEqual([], upper_paths)


class SemanticSkeletonPathTest(unittest.TestCase):
    def test_spatial_hysteresis_keeps_connected_weak_heat_only(self):
        heatmap = np.zeros((50, 70), dtype=np.float32)
        heatmap[8:42, 33:37] = 0.30
        heatmap[8:16, 33:37] = 0.60
        heatmap[30:38, 6:10] = 0.30

        mask = _spatial_hysteresis_binary_mask(
            heatmap, high_threshold=0.35, low_threshold=0.27)

        self.assertTrue(np.all(mask[8:42, 34] != 0))
        self.assertEqual(0, int(np.count_nonzero(mask[30:38, 6:10])))

    def test_small_holes_are_filled_without_filling_large_interior(self):
        mask = np.ones((40, 50), dtype=np.uint8)
        mask[10:12, 10:12] = 0
        mask[12:28, 25:42] = 0

        filled = _fill_binary_mask(mask, max_hole_area=16)

        self.assertTrue(np.all(filled[10:12, 10:12] != 0))
        self.assertTrue(np.all(filled[12:28, 25:42] == 0))

    def test_default_environment_uses_semantic_skeleton_path_source(self):
        with patch.dict(os.environ, {}, clear=True):
            config = VisionControlConfig.from_env()

        self.assertEqual("skeleton", config.path_source)
        self.assertEqual(0.35, config.skeleton_threshold)
        self.assertEqual(0.27, config.skeleton_low_threshold)
        self.assertEqual(10, config.skeleton_min_branch_length)
        self.assertEqual(2, config.skeleton_max_branches)

    def test_discrete_mask_samples_are_connected_but_far_noise_is_removed(self):
        mask = np.zeros((60, 80), dtype=np.uint8)
        mask[8:20, 38:41] = 1
        mask[25:37, 38:41] = 1
        mask[42:55, 38:41] = 1
        mask[5, 5] = 1

        connected = _connect_binary_mask_samples(
            mask, max_gap=8, minimum_group_area=15,
            bridge_thickness=3)

        component_count, _labels = cv2.connectedComponents(
            connected, connectivity=8)
        self.assertEqual(2, component_count)
        self.assertTrue(np.all(connected[19:44, 39] == 1))
        self.assertEqual(0, int(connected[5, 5]))

    def test_binary_edge_filter_rounds_corner_noise(self):
        mask = np.zeros((40, 40), dtype=np.uint8)
        mask[10:30, 10:30] = 1
        mask[9, 20] = 1

        smoothed = _smooth_binary_mask_edges(mask, kernel_size=5)

        self.assertEqual(0, int(smoothed[9, 20]))
        self.assertEqual(1, int(smoothed[20, 20]))
        self.assertEqual(np.uint8, smoothed.dtype)
        self.assertTrue(set(np.unique(smoothed)).issubset({0, 1}))

    def test_preview_reads_raw_heatmaps_without_skeleton_path_source(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0, 40:50, 60:70] = 0.40
        result = _result(heatmaps)
        frame = np.full((480, 640, 3), 100, dtype=np.uint8)

        _draw_semantic_heat_distribution(frame, result)

        self.assertTrue(np.array_equal(
            np.asarray([0, 0, 255], dtype=np.uint8), frame[180, 260]))
        self.assertTrue(np.array_equal(
            np.asarray([100, 100, 100], dtype=np.uint8), frame[20, 20]))

    def test_preview_renders_connected_mask_across_short_sampling_gap(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0, 35:55, 78:83] = 0.80
        heatmaps[0, 60:82, 78:83] = 0.80
        result = _result(heatmaps)
        frame = np.full((480, 640, 3), 100, dtype=np.uint8)

        _draw_semantic_heat_distribution(frame, result)

        self.assertTrue(np.array_equal(
            np.asarray([0, 0, 255], dtype=np.uint8), frame[230, 320]))

    def test_binary_heat_distribution_preview_uses_opaque_red(self):
        frame = np.full((120, 160, 3), 100, dtype=np.uint8)
        masks = np.zeros((2, 30, 40), dtype=np.uint8)
        masks[0, 10:20, 12:18] = 1
        masks[1, 10:20, 22:28] = 1

        _draw_semantic_heat_distribution(frame, {
            "semantic_heat_distribution_masks": masks,
        })

        first_channel_pixel = frame[60, 60]
        second_channel_pixel = frame[60, 100]
        self.assertTrue(np.array_equal(
            np.asarray([0, 0, 255], dtype=np.uint8),
            first_channel_pixel))
        self.assertTrue(np.array_equal(
            np.asarray([0, 0, 255], dtype=np.uint8),
            second_channel_pixel))
        self.assertTrue(np.array_equal(
            np.asarray([100, 100, 100], dtype=np.uint8), frame[5, 5]))

    def test_component_centerline_preserves_horizontal_geometry_and_scaling(self):
        mask = np.zeros((120, 160), dtype=np.uint8)
        cv2.line(mask, (10, 60), (150, 60), 1, 9, cv2.LINE_8)
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8)

        points = VisionControlPlanner._component_centerline_points(
            labels, 1, mask.astype(np.float32), (480, 640, 3),
            component_stats=stats[1])

        self.assertEqual(2, count)
        self.assertEqual(np.float32, points.dtype)
        self.assertEqual(2, points.shape[1])
        self.assertGreater(float(np.ptp(points[:, 0])), 500.0)
        self.assertLess(float(np.ptp(points[:, 1])), 8.0)

    def test_short_skeleton_spur_does_not_redirect_primary_path(self):
        mask = np.zeros((120, 160), dtype=np.uint8)
        cv2.line(mask, (80, 115), (80, 5), 1, 9, cv2.LINE_8)
        cv2.line(mask, (80, 80), (110, 80), 1, 7, cv2.LINE_8)
        _count, labels, stats, _centroids = (
            cv2.connectedComponentsWithStats(mask, connectivity=8))

        points = VisionControlPlanner._component_centerline_points(
            labels, 1, mask.astype(np.float32), (120, 160, 3),
            component_stats=stats[1])

        self.assertGreater(float(np.ptp(points[:, 1])), 100.0)
        self.assertLess(float(np.max(points[:, 0])), 90.0)

    def test_history_score_keeps_same_branch_at_near_equal_fork(self):
        mask = np.zeros((120, 160), dtype=np.uint8)
        cv2.line(mask, (80, 115), (80, 65), 1, 9, cv2.LINE_8)
        cv2.line(mask, (80, 65), (35, 10), 1, 9, cv2.LINE_8)
        cv2.line(mask, (80, 65), (125, 10), 1, 9, cv2.LINE_8)
        _count, labels, stats, _centroids = (
            cv2.connectedComponentsWithStats(mask, connectivity=8))
        history = np.asarray([
            [80.0, 115.0], [80.0, 65.0], [125.0, 10.0],
        ], dtype=np.float32)

        points = VisionControlPlanner._component_centerline_points(
            labels, 1, mask.astype(np.float32), (120, 160, 3),
            component_stats=stats[1], history_points=history)

        self.assertGreater(float(points[-1, 0]), 110.0)

    def test_y_fork_returns_two_paths_with_shared_root_trunk(self):
        mask = np.zeros((120, 160), dtype=np.uint8)
        cv2.line(mask, (80, 118), (80, 65), 1, 9, cv2.LINE_8)
        cv2.line(mask, (80, 65), (28, 8), 1, 9, cv2.LINE_8)
        cv2.line(mask, (80, 65), (132, 8), 1, 9, cv2.LINE_8)
        _count, labels, stats, _centroids = (
            cv2.connectedComponentsWithStats(mask, connectivity=8))

        paths = VisionControlPlanner._component_centerline_paths(
            labels, 1, mask.astype(np.float32), (120, 160, 3),
            component_stats=stats[1], max_paths=2,
            min_branch_length=10)

        self.assertEqual(2, len(paths))
        common_limit = min(len(paths[0]), len(paths[1]))
        common = np.all(
            np.isclose(
                paths[0][:common_limit], paths[1][:common_limit],
                atol=1e-5), axis=1)
        first_difference = np.flatnonzero(~common)
        shared_count = (
            int(first_difference[0]) if len(first_difference)
            else common_limit)
        self.assertGreater(shared_count, 35)
        self.assertGreater(
            abs(float(paths[0][-1, 0] - paths[1][-1, 0])), 90.0)

    def test_short_root_spur_is_not_promoted_to_second_branch(self):
        mask = np.zeros((120, 160), dtype=np.uint8)
        cv2.line(mask, (80, 118), (80, 8), 1, 7, cv2.LINE_8)
        cv2.line(mask, (80, 70), (86, 70), 1, 3, cv2.LINE_8)
        _count, labels, stats, _centroids = (
            cv2.connectedComponentsWithStats(mask, connectivity=8))

        paths = VisionControlPlanner._component_centerline_paths(
            labels, 1, mask.astype(np.float32), (120, 160, 3),
            component_stats=stats[1], max_paths=2,
            min_branch_length=10)

        self.assertEqual(1, len(paths))

    def test_planner_keeps_both_root_connected_fork_branches(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        cv2.line(heatmaps[0], (80, 119), (80, 65), 0.80, 9, cv2.LINE_8)
        cv2.line(heatmaps[0], (80, 65), (30, 10), 0.80, 9, cv2.LINE_8)
        cv2.line(heatmaps[0], (80, 65), (130, 10), 0.80, 9, cv2.LINE_8)
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", route_confirm_frames=1,
            skeleton_min_branch_length=10,
            skeleton_max_branches=2))

        _command, debug = planner.update(result, now=1.0)

        self.assertEqual(2, debug["candidate_count"])
        self.assertEqual(2, len(result["paths"]))
        self.assertTrue(all(
            path["source"] == "semantic_skeleton_root_branch"
            for path in result["paths"]))
        self.assertTrue(all(
            path["branch_count"] == 2 for path in result["paths"]))

    def test_arc_length_temporal_filter_stabilizes_horizontal_path(self):
        planner = VisionControlPlanner(config=_config(
            path_ema_alpha=0.30, path_max_step_px_640=40.0))
        xs = np.linspace(10.0, 150.0, 60, dtype=np.float32)
        previous = np.column_stack((
            xs, np.full_like(xs, 60.0))).astype(np.float32)
        current = np.column_stack((
            xs, np.full_like(xs, 64.0))).astype(np.float32)

        filtered = planner._smooth_path_points(
            current, previous, image_width=160, apply_spatial=False)

        self.assertEqual((24, 2), filtered.shape)
        self.assertGreater(float(np.mean(filtered[1:-1, 1])), 60.0)
        self.assertLess(float(np.mean(filtered[1:-1, 1])), 64.0)

    def test_temporal_identity_match_precedes_current_left_right_order(self):
        planner = VisionControlPlanner(config=_config(path_source="skeleton"))
        ys = np.linspace(470.0, 40.0, 80, dtype=np.float32)
        planner.last_slot_points = {
            0: np.column_stack((
                np.full_like(ys, 410.0), ys)).astype(np.float32),
            1: np.column_stack((
                np.full_like(ys, 210.0), ys)).astype(np.float32),
        }
        current_slot_zero = np.column_stack((
            np.full_like(ys, 420.0), ys)).astype(np.float32)
        current_slot_one = np.column_stack((
            np.full_like(ys, 220.0), ys)).astype(np.float32)

        assigned = planner._assign_heatmap_slots([
            {"source_slot": 0, "points_xy": current_slot_zero},
            {"source_slot": 1, "points_xy": current_slot_one},
        ], (480, 640, 3))

        self.assertEqual([0, 1], [item["slot"] for item in assigned])
        self.assertEqual(
            [0, 1], [item["source_slot"] for item in assigned])
        self.assertTrue(all(
            item["identity_source"] == "temporal_match"
            for item in assigned))

    def test_shared_trunk_identity_lock_matches_only_branch_tails(self):
        planner = VisionControlPlanner(config=_config(path_source="skeleton"))
        trunk = np.asarray([
            [320.0, 470.0], [320.0, 360.0], [320.0, 260.0],
        ], dtype=np.float32)
        left_tail = np.asarray([
            [320.0, 260.0], [230.0, 150.0], [120.0, 40.0],
        ], dtype=np.float32)
        right_tail = np.asarray([
            [320.0, 260.0], [410.0, 150.0], [520.0, 40.0],
        ], dtype=np.float32)
        previous_left = np.concatenate((trunk, left_tail[1:]), axis=0)
        previous_right = np.concatenate((trunk, right_tail[1:]), axis=0)
        planner.last_slot_points = {
            0: previous_left.copy(), 1: previous_right.copy(),
        }
        planner.last_slot_branch_tails = {
            0: left_tail.copy(), 1: right_tail.copy(),
        }

        # Deliberately present the right branch first.  The long shared trunk
        # must not dominate identity and swap the blue/green branch roles.
        assigned = planner._assign_heatmap_slots([
            {
                "source_slot": 0,
                "points_xy": previous_right,
                "branch_tail_points_xy": right_tail,
            },
            {
                "source_slot": 1,
                "points_xy": previous_left,
                "branch_tail_points_xy": left_tail,
            },
        ], (480, 640, 3))

        self.assertEqual([0, 1], [item["slot"] for item in assigned])
        self.assertEqual([1, 0], [item["source_slot"] for item in assigned])
        self.assertTrue(all(
            item["identity_source"] == "temporal_match"
            for item in assigned))

    def test_selected_fork_tail_stays_locked_when_slots_swap(self):
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", outer_slot=0))
        trunk = np.asarray([
            [320.0, 470.0], [320.0, 360.0], [320.0, 260.0],
        ], dtype=np.float32)
        left_tail = np.asarray([
            [320.0, 260.0], [230.0, 150.0], [120.0, 40.0],
        ], dtype=np.float32)
        right_tail = np.asarray([
            [320.0, 260.0], [410.0, 150.0], [520.0, 40.0],
        ], dtype=np.float32)
        left = np.concatenate((trunk, left_tail[1:]), axis=0)
        right = np.concatenate((trunk, right_tail[1:]), axis=0)

        first = planner._select_candidate([
            {"slot": 0, "points_xy": left,
             "branch_tail_points_xy": left_tail},
            {"slot": 1, "points_xy": right,
             "branch_tail_points_xy": right_tail},
        ], ROUTE_MULTI_FORK, (480, 640, 3))
        second = planner._select_candidate([
            {"slot": 0, "points_xy": right,
             "branch_tail_points_xy": right_tail},
            {"slot": 1, "points_xy": left,
             "branch_tail_points_xy": left_tail},
        ], ROUTE_MULTI_FORK, (480, 640, 3))

        self.assertLess(float(first["points_xy"][-1, 0]), 200.0)
        self.assertLess(float(second["points_xy"][-1, 0]), 200.0)
        self.assertEqual(1, planner.selected_slot_lock)

    def test_fragmented_lane_becomes_one_continuous_skeleton(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0, 15:45, 78:83] = 0.80
        heatmaps[0, 51:80, 78:83] = 0.80
        heatmaps[0, 86:119, 78:83] = 0.80
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", skeleton_threshold=0.40,
            skeleton_connect_max_gap=8, route_confirm_frames=1))

        command, debug = planner.update(result, now=1.0)

        self.assertIsNotNone(command)
        self.assertEqual(1, debug["candidate_count"])
        skeleton = result["semantic_skeleton_masks"][0]
        component_count, _labels = cv2.connectedComponents(
            skeleton.astype(np.uint8), connectivity=8)
        self.assertEqual(2, component_count)
        fitted_points = result["paths"][0]["points_xy"]
        self.assertGreater(float(np.ptp(fitted_points[:, 1])), 380.0)
        fitted_mask = result["semantic_heat_distribution_masks"][0]
        fitted_x = np.clip(np.rint(
            fitted_points[:, 0] * 159.0 / 639.0).astype(np.int32), 0, 159)
        fitted_y = np.clip(np.rint(
            fitted_points[:, 1] * 119.0 / 479.0).astype(np.int32), 0, 119)
        self.assertTrue(np.all(fitted_mask[fitted_y, fitted_x] != 0))

    def test_point_four_mask_is_kept_while_small_noise_is_removed(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0, 20:119, 79:82] = 0.40
        heatmaps[0, 52:55, 18:21] = 0.90
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", skeleton_threshold=0.40,
            skeleton_min_area=15, route_confirm_frames=1))

        command, debug = planner.update(result, now=1.0)

        self.assertIsNotNone(command)
        self.assertEqual(1, debug["candidate_count"])
        self.assertEqual(
            0.40,
            debug["heatmap_peak_detection"]["binary_threshold"])
        self.assertGreater(debug["candidates"][0]["near_x"], 300.0)
        self.assertLess(debug["candidates"][0]["near_x"], 350.0)
        self.assertAlmostEqual(
            0.40, result["paths"][0]["heat_support_min"], places=6)

    def test_probability_below_mask_threshold_is_rejected(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0, 20:119, 79:82] = 0.399
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", skeleton_threshold=0.40))

        command, debug = planner.update(result, now=1.0)

        self.assertIsNotNone(command)
        self.assertTrue(command["safe_stop"])
        self.assertEqual(0, debug["candidate_count"])

    def test_compact_blob_without_skeleton_continuity_is_rejected(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0, 50:60, 70:80] = 0.90
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", skeleton_threshold=0.40,
            skeleton_min_area=15, skeleton_min_length=8))

        command, debug = planner.update(result, now=1.0)

        self.assertIsNotNone(command)
        self.assertTrue(command["safe_stop"])
        self.assertEqual(0, debug["candidate_count"])

    def test_preview_strongly_colors_every_thresholded_heat_pixel(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        heatmaps[0, 20:119, 76:85] = 0.40
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", skeleton_threshold=0.40,
            route_confirm_frames=1))
        planner.update(result, now=1.0)
        frame = np.full((480, 640, 3), 200, dtype=np.uint8)

        render_vision_control_debug(frame, result)

        masks = result["semantic_heat_distribution_masks"]
        self.assertEqual((1, 120, 160), masks.shape)
        raw_mask = heatmaps[0] >= 0.40
        self.assertGreater(
            float(np.mean(masks[0][raw_mask] != 0)), 0.99)
        # This point is inside the heat region but away from its centerline.
        blue, green, red = frame[242, 338]
        self.assertEqual(int(blue), int(green))
        self.assertLess(int(green), 50)
        self.assertGreater(int(red), 220)
        self.assertTrue(np.all(frame[0, 0] == 200))

    def test_full_skeleton_mask_keeps_components_beyond_control_limit(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        for center_x in (30, 80, 130):
            heatmaps[0, 20:119, center_x - 2:center_x + 3] = 0.80
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            path_source="skeleton", skeleton_threshold=0.40,
            skeleton_min_area=15, skeleton_min_length=8))

        planner.update(result, now=1.0)

        skeleton = result["semantic_skeleton_masks"][0]
        component_count, _labels = cv2.connectedComponents(
            skeleton.astype(np.uint8), connectivity=8)
        self.assertEqual(4, component_count)
        self.assertEqual(2, len(result["paths"]))


class VisionControlPlannerTest(unittest.TestCase):
    def test_overlapping_heatmaps_can_still_be_multi_fork(self):
        planner = VisionControlPlanner(config=_config())
        command, debug = planner.update(_result(_fork_heatmaps()), now=1.0)

        self.assertIsNotNone(command)
        self.assertEqual(ROUTE_MULTI_FORK, debug["route_state"])
        self.assertEqual(2, debug["candidate_count"])

    def test_track_error_sign_matches_old_chain(self):
        right = VisionControlPlanner(config=_config()).update(_result(_straight_heatmap(104)), now=1.0)[0]
        left = VisionControlPlanner(config=_config()).update(_result(_straight_heatmap(56)), now=1.0)[0]

        self.assertGreater(right["track_error"], 0.0)
        self.assertLess(left["track_error"], 0.0)
        self.assertEqual(STATE_TRACK, right["state_cmd"])

    def test_direct_curve_source_uses_curve_points_without_heatmap(self):
        planner = VisionControlPlanner(config=_config(path_source="curve"))
        command, debug = planner.update(_curve_result(), now=1.0)

        self.assertIsNotNone(command)
        self.assertEqual(2, debug["candidate_count"])
        self.assertEqual("direct_curve", debug["candidates"][0]["source"])

    def test_heatmap_search_ignores_points_outside_road_mask(self):
        config = _config()
        heatmaps = _straight_heatmap(56)
        road = np.zeros(heatmaps.shape[1:], dtype=np.float32)
        road[:, 90:] = 1.0

        candidate = HeatmapPathSearch(config).search(
            heatmaps[0],
            road_mask=road,
            image_shape=(480, 640, 3),
            slot=0,
        )

        self.assertIsNone(candidate)

    def test_heatmap_component_output_is_limited_to_two_display_lines(self):
        heatmaps = np.zeros((2, 120, 160), dtype=np.float32)
        _draw_heat_path(heatmaps[0], lambda _y: 56.0)
        _draw_heat_path(heatmaps[0], lambda _y: 100.0)
        _draw_heat_path(heatmaps[1], lambda _y: 104.0)
        planner = VisionControlPlanner(config=_config())
        result = _result(heatmaps)

        _command, debug = planner.update(result, now=1.0)

        self.assertLessEqual(debug["candidate_count"], 2)
        self.assertLessEqual(len(debug["candidate_paths"]), 2)
        self.assertLessEqual(len(result["heatmap_debug_lines"]), 2)
        self.assertLessEqual(len(result["display_paths"]), 2)
        self.assertLessEqual(len(_extract_heatmap_preview_lines(
            result, (480, 640, 3))), 2)
        self.assertEqual((255, 0, 0), _identity_probability_color(0, 1.0))
        self.assertEqual((0, 255, 0), _identity_probability_color(1, 1.0))
        self.assertTrue(all(
            path["heat_supported"] and
            path["heat_support_low_run"] == 0
            for path in result["paths"]
        ))

    def test_heatmap_paths_fall_back_to_full_map_without_valid_road(self):
        heatmaps = _straight_heatmap(80)
        result = _result(heatmaps)
        result["road"]["mask"].fill(0.0)
        planner = VisionControlPlanner(config=_config())

        _command, debug = planner.update(result, now=1.0)

        self.assertEqual(1, debug["candidate_count"])
        self.assertEqual(1, len(result["paths"]))

    def test_only_heatmap_components_inside_road_are_kept(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        _draw_heat_path(heatmaps[0], lambda _y: 40.0)
        _draw_heat_path(heatmaps[0], lambda _y: 110.0)
        result = _result(heatmaps)
        result["road"]["mask"][:, :80] = 0.0
        planner = VisionControlPlanner(config=_config())

        _command, debug = planner.update(result, now=1.0)

        self.assertEqual(1, debug["candidate_count"])
        self.assertGreater(debug["candidates"][0]["near_x"], 400.0)

    def test_peak_without_middle_band_support_is_not_a_candidate(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        _draw_heat_path_range(heatmaps[0], lambda _y: 72.0, 20, 54)
        _draw_heat_path_range(heatmaps[0], lambda _y: 74.0, 70, 119)
        result = _result(heatmaps)
        planner = VisionControlPlanner(config=_config(
            heat_threshold=0.30,
            min_path_support_probability=0.12,
        ))

        _command, debug = planner.update(result, now=1.0)

        self.assertEqual(0, debug["candidate_count"])
        self.assertEqual(1, debug["detected_path_count"])
        self.assertEqual(0, debug["valid_path_count"])

    def test_overlapping_fork_keeps_two_independent_supported_paths(self):
        planner = VisionControlPlanner(config=_config(overlap_px_640=28.0))
        result = _result(_fork_heatmaps())

        _command, debug = planner.update(result, now=1.0)

        self.assertEqual(2, debug["candidate_count"])
        self.assertEqual(2, len(result["display_paths"]))
        self.assertTrue(all(
            path["heat_supported"] for path in result["paths"]))
        branch_y = 180.0
        self.assertGreater(abs(
            _path_x_at(result["paths"][0]["points_xy"], branch_y) -
            _path_x_at(result["paths"][1]["points_xy"], branch_y)), 100.0)

    def test_fork_identity_is_confirmed_by_row_scan_not_channel_slot(self):
        result = _result(_fork_heatmaps())
        planner = VisionControlPlanner(config=_config())

        _command, debug = planner.update(result, now=1.0)

        self.assertEqual(2, debug["candidate_count"])
        self.assertEqual(ROUTE_MULTI_FORK, debug["route_state"])
        self.assertEqual(["left", "right"], [
            path["identity"] for path in result["paths"]])
        self.assertEqual(["row_scan", "row_scan"], [
            path["identity_source"] for path in result["paths"]])
        self.assertLess(
            debug["candidates"][0]["lookahead_x"],
            debug["candidates"][1]["lookahead_x"])

    def test_shared_road_jitter_keeps_supported_branches_separate(self):
        planner = VisionControlPlanner(config=_config(
            path_ema_alpha=0.32,
            path_max_step_px_640=40.0,
        ))
        first_result = _result(_fork_heatmaps(shared_shift=0.0))
        second_result = _result(_fork_heatmaps(shared_shift=6.0))

        planner.update(first_result, now=1.0)
        planner.update(second_result, now=1.02)

        first_paths = first_result["paths"]
        second_paths = second_result["paths"]
        branch_y = 180.0
        first_separation = abs(
            _path_x_at(first_paths[0]["points_xy"], branch_y) -
            _path_x_at(first_paths[1]["points_xy"], branch_y)
        )
        second_separation = abs(
            _path_x_at(second_paths[0]["points_xy"], branch_y) -
            _path_x_at(second_paths[1]["points_xy"], branch_y)
        )
        self.assertGreater(second_separation, 100.0)
        self.assertLess(abs(second_separation - first_separation), 8.0)
        self.assertTrue(all(
            path["heat_supported"] and
            path["heat_support_low_run"] == 0
            for path in second_paths))

    def test_continuous_weak_path_beats_disconnected_strong_regions(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        weak = np.zeros((120, 160), dtype=np.float32)
        _draw_heat_path(weak, lambda _y: 52.0)
        heatmaps[0] = np.maximum(heatmaps[0], weak * 0.42)
        _draw_heat_path_range(
            heatmaps[0], lambda _y: 118.0, 20, 48)
        _draw_heat_path_range(
            heatmaps[0], lambda _y: 118.0, 78, 104)
        result = _result(heatmaps)

        VisionControlPlanner(config=_config()).update(result, now=1.0)

        longest = max(
            result["paths"],
            key=lambda path: float(np.ptp(path["points_xy"][:, 1])))
        self.assertGreater(float(np.ptp(longest["points_xy"][:, 1])), 350.0)
        self.assertLess(float(np.mean(longest["points_xy"][:, 0])), 260.0)
        self.assertTrue(longest["heat_supported"])
        self.assertEqual(0, longest["heat_support_low_run"])

    def test_two_middle_band_peaks_in_one_channel_make_two_paths(self):
        heatmaps = np.zeros((1, 120, 160), dtype=np.float32)
        _draw_heat_path(heatmaps[0], lambda _y: 55.0)
        _draw_heat_path(heatmaps[0], lambda _y: 75.0)
        heatmaps[0, 108, 55:76] = 1.0
        result = _result(heatmaps)

        VisionControlPlanner(config=_config()).update(result, now=1.0)

        self.assertEqual(2, len(result["paths"]))
        self.assertEqual(2, result["detected_path_count"])
        self.assertTrue(all(
            path["source"] == "heatmap_hysteresis_greedy" and
            path["reaches_bottom_region"]
            for path in result["paths"]))

    def test_left_right_identity_survives_heatmap_channel_swap(self):
        planner = VisionControlPlanner(config=_config())
        first = np.concatenate((
            _straight_heatmap(50), _straight_heatmap(110)), axis=0)
        swapped = np.concatenate((
            _straight_heatmap(110), _straight_heatmap(50)), axis=0)

        planner.update(_result(first), now=1.0)
        result = _result(swapped)
        planner.update(result, now=1.02)

        self.assertEqual(["left", "right"], [
            path["identity"] for path in result["paths"]])
        self.assertLess(
            float(np.mean(result["paths"][0]["points_xy"][:, 0])),
            float(np.mean(result["paths"][1]["points_xy"][:, 0])))
        self.assertEqual(1, result["paths"][0]["source_slot"])
        self.assertEqual(0, result["paths"][1]["source_slot"])

    def test_current_ocr_locks_right_branch(self):
        planner = VisionControlPlanner(config=_config())
        _command, debug = _confirm_ocr(planner, "right", start=2.0)

        self.assertEqual("right", debug["branch_lock"])
        self.assertEqual(1, debug["selected_slot"])
        self.assertTrue(debug["ocr_current"])
        self.assertEqual(1, debug["ocr_pending_frames"])

    def test_current_left_ocr_keeps_default_left_branch(self):
        planner = VisionControlPlanner(config=_config())
        _command, debug = _confirm_ocr(planner, "left", start=2.0)

        self.assertEqual("left", debug["branch_lock"])
        self.assertEqual("ocr", debug["branch_lock_source"])
        self.assertEqual(0, debug["selected_slot"])

    def test_locked_branch_never_falls_back_to_opposite_slot(self):
        planner = VisionControlPlanner(config=_config())
        _confirm_ocr(planner, start=2.0)

        command, debug = planner.update(
            _result(_straight_heatmap(56)),
            {"instruction_current": False},
            now=2.02,
        )

        self.assertEqual("right", debug["branch_lock"])
        self.assertEqual(1, debug["selected_slot_lock"])
        self.assertIsNone(debug["selected_slot"])
        self.assertEqual(STATE_RECOVER_LINE, command["state_cmd"])

    def test_ocr_branch_lock_expires_after_ten_seconds(self):
        planner = VisionControlPlanner(config=_config(
            ocr_lock_lifetime_s=10.0,
            default_outer_after_s=15.0,
            outer_slot=0,
        ))
        _confirm_ocr(planner, start=2.0)

        _command, before_expiry = planner.update(
            _result(_fork_heatmaps()),
            {"instruction_current": False},
            now=11.99,
        )
        _command, after_expiry = planner.update(
            _result(_fork_heatmaps()),
            {"instruction_current": False},
            now=12.0,
        )

        self.assertEqual("right", before_expiry["branch_lock"])
        self.assertEqual("ocr", before_expiry["branch_lock_source"])
        self.assertEqual(1, before_expiry["selected_slot"])
        self.assertTrue(after_expiry["ocr_lock_expired"])
        self.assertEqual("left", after_expiry["branch_lock"])
        self.assertEqual("default", after_expiry["branch_lock_source"])
        self.assertEqual(0, after_expiry["selected_slot"])

    def test_repeated_current_ocr_refreshes_lock_lifetime(self):
        planner = VisionControlPlanner(config=_config(
            ocr_lock_lifetime_s=10.0))
        _confirm_ocr(planner, start=2.0)
        _confirm_ocr(planner, start=9.0)

        _command, debug = planner.update(
            _result(_fork_heatmaps()),
            {"instruction_current": False},
            now=12.1,
        )

        self.assertEqual("right", debug["branch_lock"])
        self.assertEqual("ocr", debug["branch_lock_source"])
        self.assertAlmostEqual(6.9, debug["ocr_lock_remaining_s"], places=5)

    def test_route_state_ignores_one_frame_single_path_flicker(self):
        planner = VisionControlPlanner(config=_config(route_confirm_frames=3))
        planner.update(_result(_fork_heatmaps()), now=1.0)

        _command, debug = planner.update(
            _result(_straight_heatmap(56)), now=1.02)

        self.assertEqual("SINGLE", debug["raw_route_state"])
        self.assertEqual(ROUTE_MULTI_FORK, debug["route_state"])
        self.assertEqual("SINGLE", debug["pending_route_state"])

    def test_selected_slot_survives_short_opposite_only_flicker(self):
        planner = VisionControlPlanner(
            config=_config(path_state_hold_frames=3))
        planner.update(_result(_fork_heatmaps()), now=1.0)
        right_only = np.zeros((2, 120, 160), dtype=np.float32)
        right_only[1] = _straight_heatmap(104)[0]

        _command, missing_debug = planner.update(
            _result(right_only), now=1.02)
        _command, recovered_debug = planner.update(
            _result(_fork_heatmaps()), now=1.04)

        self.assertEqual(0, missing_debug["selected_slot_lock"])
        self.assertIsNone(missing_debug["selected_slot"])
        self.assertEqual(0, recovered_debug["selected_slot"])

    def test_uninstructed_slot_lock_releases_after_long_path_loss(self):
        planner = VisionControlPlanner(
            config=_config(path_state_hold_frames=2))
        planner.update(_result(_fork_heatmaps()), now=1.0)
        empty = np.zeros((2, 120, 160), dtype=np.float32)

        planner.update(_result(empty), now=1.02)
        planner.update(_result(empty), now=1.04)
        _command, debug = planner.update(_result(empty), now=1.06)

        self.assertIsNone(debug["branch_lock"])
        self.assertIsNone(debug["selected_slot_lock"])

    def test_path_ema_falls_back_when_blend_leaves_heat_ridge(self):
        planner = VisionControlPlanner(config=_config(
            path_ema_alpha=0.40,
            path_smooth_window=1,
            path_max_step_px_640=1000.0,
        ))
        _command, first = planner.update(
            _result(_straight_heatmap(45)), now=1.0)
        second_result = _result(_straight_heatmap(70))
        _command, second = planner.update(second_result, now=1.02)

        filtered_x = second["candidates"][0]["lookahead_x"]
        raw_x = 70.0 * 639.0 / 159.0
        self.assertAlmostEqual(filtered_x, raw_x, delta=4.1)
        self.assertFalse(second_result["paths"][0]["temporal_smoothed"])
        self.assertTrue(
            second_result["paths"][0]["smoothing_rejected_low_heat"])

    def test_spatial_filter_reduces_direct_curve_zigzag(self):
        ys = np.linspace(460.0, 60.0, 9, dtype=np.float32)
        xs = np.asarray([240.0, 400.0] * 4 + [240.0], dtype=np.float32)
        raw_points = np.stack((xs, ys), axis=1)
        result = {
            "centerline": {"curve_paths": [{
                "slot": 0, "role": "single", "score": 0.9,
                "points_xy": raw_points,
            }]},
            "image_shape": (480, 640, 3),
            "detections": [],
        }
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=5))

        planner.update(result, now=1.0)
        smoothed_x = result["paths"][0]["points_xy"][:, 0]

        self.assertLess(float(np.std(smoothed_x)), float(np.std(xs)))

    def test_no_current_ocr_defaults_to_left_immediately(self):
        planner = VisionControlPlanner(config=_config(default_outer_after_s=15.0, outer_slot=0))
        _command, debug = planner.update(
            _result(_fork_heatmaps()),
            {"instruction_current": False}, now=10.0)

        self.assertEqual("left", debug["branch_lock"])
        self.assertEqual("default", debug["branch_lock_source"])
        self.assertEqual(0, debug["selected_slot"])

    def test_car_in_path_sends_avoid_car_state_with_biased_error(self):
        detections = [{
            "label": "Car",
            "score": 0.90,
            "bbox": [282.0, 260.0, 358.0, 420.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _result(_straight_heatmap(80), detections=detections),
            now=1.0,
        )

        self.assertEqual(STATE_AVOID_CAR, command["state_cmd"])
        self.assertGreater(command["target_speed"], 0.0)
        self.assertEqual("car_in_path_bias", debug["control_target"]["task_reason"])

    def test_turnsign_detection_slows_before_stop_line(self):
        detections = [{
            "label": "TurnSign",
            "score": 0.90,
            "bbox": [300.0, 80.0, 350.0, 150.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _result(_straight_heatmap(80), detections=detections),
            now=1.0,
        )

        self.assertEqual(STATE_TRACK, command["state_cmd"])
        self.assertAlmostEqual(0.08, command["target_speed"])
        self.assertEqual("turnsign_slow", debug["control_target"]["task_reason"])

    def test_low_confidence_turnsign_is_display_only_for_control(self):
        detections = [{
            "label": "TurnSign",
            "score": 0.21,
            "bbox": [300.0, 80.0, 350.0, 150.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _result(_straight_heatmap(80), detections=detections),
            now=1.0,
        )

        self.assertEqual(STATE_TRACK, command["state_cmd"])
        self.assertAlmostEqual(0.15, command["target_speed"])
        self.assertEqual("track", debug["control_target"]["task_reason"])

    def test_low_confidence_turnsign_does_not_latch_control(self):
        planner = VisionControlPlanner(config=_config())
        sign = [{
            "label": "TurnSign",
            "score": 0.21,
            "bbox": [300.0, 80.0, 350.0, 150.0],
        }]

        planner.update(_result(_straight_heatmap(80), detections=sign), now=1.0)
        planner.update(_result(_straight_heatmap(80), detections=sign), now=1.1)
        slow_command, slow_debug = planner.update(
            _result(_straight_heatmap(80), detections=sign), now=1.2)
        stop_command, stop_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=1.3)
        resume_command, resume_debug = planner.update(
            _result(_fork_heatmaps()),
            _ocr_response("right", active=True),
            now=1.4,
        )

        self.assertEqual(STATE_TRACK, slow_command["state_cmd"])
        self.assertEqual("track", slow_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_TRACK, stop_command["state_cmd"])
        self.assertEqual("track", stop_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_TRACK, resume_command["state_cmd"])
        self.assertEqual("track", resume_debug["control_target"]["task_reason"])

    def test_large_turnsign_at_lookahead_stops_immediately(self):
        detections = [{
            "label": "TurnSign",
            "score": 0.95,
            "bbox": [250.0, 250.0, 390.0, 380.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _result(_straight_heatmap(80), detections=detections),
            now=1.0,
        )

        self.assertEqual(STATE_SAFE_STOP, command["state_cmd"])
        self.assertEqual(0.0, command["target_speed"])
        self.assertEqual(0, command["flags"])
        self.assertEqual("turnsign_stop", debug["control_target"]["task_reason"])

    def test_active_ocr_turnsign_slows_until_near_lookahead_then_stops(self):
        far_sign = [{
            "label": "TurnSign",
            "score": 0.95,
            "bbox": [380.0, 80.0, 430.0, 150.0],
        }]
        near_sign = [{
            "label": "TurnSign",
            "score": 0.95,
            "bbox": [250.0, 250.0, 390.0, 380.0],
        }]
        planner = VisionControlPlanner(config=_config())

        slow_command, slow_debug = planner.update(
            _result(_straight_heatmap(80), detections=far_sign),
            {"active": True, "status": "ocr_pending"},
            now=1.0,
        )
        stop_command, stop_debug = planner.update(
            _result(_straight_heatmap(80), detections=near_sign),
            {"active": True, "status": "ocr_pending"},
            now=1.1,
        )

        self.assertEqual(STATE_TRACK, slow_command["state_cmd"])
        self.assertAlmostEqual(0.08, slow_command["target_speed"])
        self.assertGreater(slow_command["track_error"], 0.0)
        self.assertEqual("turnsign_slow", slow_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_SAFE_STOP, stop_command["state_cmd"])
        self.assertEqual("turnsign_stop", stop_debug["control_target"]["task_reason"])

    def test_active_ocr_timeout_stays_stopped_without_result(self):
        sign = [{
            "label": "TurnSign",
            "score": 0.95,
            "bbox": [250.0, 250.0, 390.0, 380.0],
        }]
        planner = VisionControlPlanner(config=_config())

        planner.update(
            _result(_straight_heatmap(80), detections=sign),
            {"active": True, "status": "ocr_pending"},
            now=1.0,
        )
        waiting_command, waiting_debug = planner.update(
            _result(_straight_heatmap(80), detections=sign),
            {"active": True, "status": "ocr_pending"},
            now=9.01,
        )
        stop_command, stop_debug = planner.update(
            _result(_straight_heatmap(80), detections=sign),
            {"active": True, "status": "ocr_pending"},
            now=9.40,
        )

        self.assertEqual(STATE_SAFE_STOP, waiting_command["state_cmd"])
        self.assertEqual(0.0, waiting_command["target_speed"])
        self.assertEqual("turnsign_stop", waiting_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_SAFE_STOP, stop_command["state_cmd"])
        self.assertEqual("turnsign_stop", stop_debug["control_target"]["task_reason"])

    def test_active_ocr_stops_if_turnsign_temporarily_disappears(self):
        command, debug = VisionControlPlanner(config=_config()).update(
            _result(_straight_heatmap(80), detections=[]),
            {"active": True, "status": "ocr_pending"},
            now=1.0,
        )

        self.assertEqual(STATE_SAFE_STOP, command["state_cmd"])
        self.assertEqual(0.0, command["target_speed"])
        self.assertEqual("turnsign_stop", debug["control_target"]["task_reason"])

    def test_current_ocr_result_resumes_default_speed_with_branch_lock(self):
        planner = VisionControlPlanner(config=_config())
        command, debug = _confirm_ocr(planner, start=1.0, active=True)

        self.assertEqual(STATE_TRACK, command["state_cmd"])
        self.assertAlmostEqual(0.15, command["target_speed"])
        self.assertEqual("right", debug["branch_lock"])
        self.assertEqual("track", debug["control_target"]["task_reason"])

    def test_human_on_path_near_lookahead_stops_then_recovers_when_clear(self):
        planner = VisionControlPlanner(config=_config())
        human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [285.0, 330.0, 355.0, 430.0],
        }]

        stop_command, stop_debug = planner.update(
            _result(_straight_heatmap(80), detections=human),
            now=1.0,
        )
        resume_command, resume_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=1.1,
        )

        self.assertEqual(STATE_SAFE_STOP, stop_command["state_cmd"])
        self.assertEqual(0.0, stop_command["target_speed"])
        self.assertEqual("human_half_lookahead_stop", stop_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_TRACK, resume_command["state_cmd"])
        self.assertGreater(resume_command["target_speed"], 0.0)
        self.assertEqual("track", resume_debug["control_target"]["task_reason"])

    def test_human_crosses_path_then_passes_opposite_side_at_boost_speed(self):
        planner = VisionControlPlanner(config=_config())
        left_human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 430.0],
        }]
        right_human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 430.0],
        }]

        planner.update(_result(_straight_heatmap(80), detections=left_human), now=1.0)
        pass_command, pass_debug = planner.update(
            _result(_straight_heatmap(80), detections=right_human),
            now=1.1,
        )
        hold_command, hold_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=1.2,
        )
        resume_command, resume_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=1.7,
        )

        self.assertEqual(STATE_AVOID_HUMAN, pass_command["state_cmd"])
        self.assertAlmostEqual(0.35, pass_command["target_speed"])
        self.assertLess(pass_command["track_error"], 0.0)
        self.assertEqual("human_cross_pass", pass_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_AVOID_HUMAN, hold_command["state_cmd"])
        self.assertAlmostEqual(0.35, hold_command["target_speed"])
        self.assertEqual("human_speed_hold", hold_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_TRACK, resume_command["state_cmd"])
        self.assertEqual("track", resume_debug["control_target"]["task_reason"])


if __name__ == "__main__":
    unittest.main()
