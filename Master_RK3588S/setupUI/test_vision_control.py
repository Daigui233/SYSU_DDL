import os
import sys
import unittest
from unittest.mock import patch

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
    _extract_heatmap_preview_lines,
    _identity_probability_color,
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


def _curve_result_at(blue_x, green_x, slots=(0, 1)):
    ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
    paths = []
    for slot in slots:
        x = blue_x if slot == 0 else green_x
        paths.append({
            "slot": slot,
            "role": "left" if slot == 0 else "right",
            "score": 0.9,
            "points_xy": np.stack((np.full_like(ys, x), ys), axis=1),
        })
    return {
        "centerline": {"curve_paths": paths},
        "image_shape": (480, 640, 3),
        "detections": [],
    }


def _curve_merge_result(blue_short=True):
    blue_ys = np.linspace(
        460.0, 360.0 if blue_short else 60.0,
        8 if blue_short else 32, dtype=np.float32)
    green_ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
    return {
        "centerline": {"curve_paths": [
            {
                "slot": 0, "role": "left", "score": 0.9,
                "points_xy": np.stack((
                    np.full_like(blue_ys, 300.0), blue_ys), axis=1),
            },
            {
                "slot": 1, "role": "right", "score": 0.9,
                "points_xy": np.stack((
                    np.full_like(green_ys, 310.0), green_ys), axis=1),
            },
        ]},
        "image_shape": (480, 640, 3),
        "detections": [],
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
        self.assertEqual("row_classifier", debug["candidates"][0]["source"])

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

    def test_heatmap_paths_are_suppressed_without_valid_road(self):
        heatmaps = _straight_heatmap(80)
        result = _result(heatmaps)
        result["road"]["mask"].fill(0.0)
        planner = VisionControlPlanner(config=_config())

        _command, debug = planner.update(result, now=1.0)

        self.assertEqual(0, debug["candidate_count"])
        self.assertEqual(0, len(result["paths"]))

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

    def test_curve_defaults_to_blue_even_when_green_is_closer(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1))

        command, debug = planner.update(
            _curve_result_at(150.0, 315.0),
            {"instruction_current": False},
            now=1.0,
        )

        self.assertEqual("left", debug["branch_lock"])
        self.assertEqual("default", debug["branch_lock_source"])
        self.assertEqual(0, debug["selected_slot"])
        self.assertAlmostEqual(
            150.0, debug["control_target"]["target_x"], delta=1.0)
        self.assertLess(command["track_error"], 0.0)

    def test_curve_merge_uses_stable_green_after_blue_support_collapses(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            curve_merge_enter_evidence=4))

        _command, first = planner.update(
            _curve_merge_result(blue_short=True), now=1.0)
        _command, second = planner.update(
            _curve_merge_result(blue_short=True), now=1.1)

        self.assertEqual(0, first["selected_slot"])
        self.assertFalse(first["curve_merge_override"])
        self.assertEqual(1, second["selected_slot"])
        self.assertTrue(second["curve_merge_override"])
        self.assertEqual(
            "merge_continuity_green", second["selection_reason"])
        self.assertAlmostEqual(
            310.0, second["control_target"]["path_target_x"], delta=1.0)

    def test_curve_merge_takeover_ignores_one_frame_blue_reappearance(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            curve_merge_enter_evidence=4,
            curve_merge_release_frames=3))
        planner.update(_curve_merge_result(blue_short=True), now=1.0)
        planner.update(_curve_merge_result(blue_short=True), now=1.1)

        _command, debug = planner.update(
            _curve_merge_result(blue_short=False), now=1.2)

        self.assertTrue(debug["curve_merge_override"])
        self.assertEqual(1, debug["selected_slot"])
        self.assertEqual(1, debug["curve_merge_blue_stable_frames"])

    def test_curve_merge_returns_to_blue_only_after_stable_recovery(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            curve_merge_enter_evidence=4,
            curve_merge_release_frames=3))
        planner.update(_curve_merge_result(blue_short=True), now=1.0)
        planner.update(_curve_merge_result(blue_short=True), now=1.1)

        debug = None
        for index in range(3):
            _command, debug = planner.update(
                _curve_merge_result(blue_short=False),
                now=1.2 + index * 0.1)

        self.assertFalse(debug["curve_merge_override"])
        self.assertEqual(0, debug["selected_slot"])
        self.assertEqual("blue_recovered", debug["curve_merge_reason"])

    def test_curve_short_blue_does_not_fall_back_if_paths_are_not_merged(self):
        result = _curve_merge_result(blue_short=True)
        result["centerline"]["curve_paths"][1]["points_xy"][:, 0] = 500.0
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            curve_merge_enter_evidence=4))

        for index in range(4):
            _command, debug = planner.update(result, now=1.0 + index * 0.1)

        self.assertFalse(debug["curve_merge_override"])
        self.assertEqual(0, debug["selected_slot"])
        self.assertFalse(debug["curve_merge_metrics"]["shared_near"])

    def test_curve_pending_right_ocr_stays_blue_until_confirmed(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            ocr_confirm_frames=2))
        result = _curve_result_at(180.0, 430.0)

        _command, pending = planner.update(
            result, _ocr_response("right"), now=1.0)
        _command, confirmed = planner.update(
            _curve_result_at(180.0, 430.0),
            _ocr_response("right"), now=1.1)

        self.assertEqual(0, pending["selected_slot"])
        self.assertEqual("default", pending["branch_lock_source"])
        self.assertEqual(1, confirmed["selected_slot"])
        self.assertEqual("right", confirmed["branch_lock"])
        self.assertEqual("ocr", confirmed["branch_lock_source"])

    def test_curve_left_ocr_selects_blue(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1))

        _command, debug = planner.update(
            _curve_result_at(180.0, 430.0),
            _ocr_response("left"), now=1.0)

        self.assertEqual(0, debug["selected_slot"])
        self.assertEqual("left", debug["branch_lock"])
        self.assertEqual("ocr", debug["branch_lock_source"])

    def test_curve_never_falls_back_to_green_when_blue_is_missing(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1))

        command, debug = planner.update(
            _curve_result_at(180.0, 315.0, slots=(1,)), now=1.0)

        self.assertEqual(0, debug["selected_slot_lock"])
        self.assertIsNone(debug["selected_slot"])
        self.assertNotEqual(STATE_TRACK, command["state_cmd"])
        self.assertEqual(0.0, command["target_speed"])

    def test_curve_right_lock_expiry_returns_to_blue_same_frame(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            ocr_lock_lifetime_s=10.0))
        planner.update(
            _curve_result_at(180.0, 430.0),
            _ocr_response("right"), now=2.0)

        _command, debug = planner.update(
            _curve_result_at(180.0, 430.0),
            {"instruction_current": False}, now=12.0)

        self.assertTrue(debug["ocr_lock_expired"])
        self.assertEqual("left", debug["branch_lock"])
        self.assertEqual("default", debug["branch_lock_source"])
        self.assertEqual(0, debug["selected_slot"])

    def test_curve_locked_blue_keeps_target_during_ambiguous_geometry(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            overlap_px_640=20.0, branch_separation_px_640=70.0))

        command, debug = planner.update(
            _curve_result_at(290.0, 320.0), now=1.0)

        self.assertEqual("AMBIGUOUS", debug["route_state"])
        self.assertEqual(0, debug["selected_slot"])
        self.assertIsNotNone(debug["control_target"]["path_target_x"])
        self.assertNotEqual(STATE_SAFE_STOP, command["state_cmd"])

    def test_curve_uses_nearest_path_endpoint_when_lookahead_is_outside(self):
        ys = np.linspace(460.0, 350.0, 12, dtype=np.float32)
        points = np.stack((np.full_like(ys, 210.0), ys), axis=1)
        result = {
            "centerline": {"curve_paths": [{
                "slot": 0, "role": "left", "score": 0.9,
                "points_xy": points,
            }]},
            "image_shape": (480, 640, 3),
            "detections": [],
        }
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            lookahead_y_ratio=0.625))

        _command, debug = planner.update(result, now=1.0)
        target = debug["control_target"]

        self.assertTrue(target["path_target_adaptive_y"])
        self.assertAlmostEqual(210.0, target["path_target_x"], delta=1.0)
        self.assertAlmostEqual(300.0, target["path_target_y"], delta=1.0)

    def test_curve_brief_blue_loss_holds_last_route_marker(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1,
            recover_hold_s=0.5))
        planner.update(_curve_result_at(200.0, 430.0), now=1.0)

        _command, debug = planner.update(
            _curve_result_at(200.0, 430.0, slots=(1,)), now=1.1)

        target = debug["control_target"]
        self.assertIsNone(debug["selected_slot"])
        self.assertTrue(target["path_target_held"])
        self.assertAlmostEqual(200.0, target["path_target_x"], delta=1.0)

    def test_turnsign_slow_does_not_pull_route_marker_off_blue(self):
        result = _curve_result_at(200.0, 430.0)
        result["detections"] = [{
            "label": "TurnSign", "score": 0.90,
            "bbox": [430.0, 80.0, 500.0, 150.0],
        }]
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1))

        _command, debug = planner.update(result, now=1.0)
        target = debug["control_target"]

        self.assertEqual("turnsign_slow", target["task_reason"])
        self.assertAlmostEqual(target["path_target_x"], target["target_x"])

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
        self.assertIsNone(after_expiry["branch_lock"])
        self.assertIsNone(after_expiry["branch_lock_source"])
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

    def test_no_current_ocr_defaults_to_outer_after_timeout(self):
        planner = VisionControlPlanner(config=_config(default_outer_after_s=15.0, outer_slot=0))
        planner.update(_result(_fork_heatmaps()), {"instruction_current": False}, now=10.0)
        _command, debug = planner.update(_result(_fork_heatmaps()), {"instruction_current": False}, now=25.1)

        self.assertEqual("left", debug["branch_lock"])
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

    def test_low_confidence_turnsign_still_triggers_slowdown(self):
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
        self.assertAlmostEqual(0.08, command["target_speed"])
        self.assertEqual("turnsign_slow", debug["control_target"]["task_reason"])

    def test_turnsign_latches_after_three_detections_until_ocr_result(self):
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
        self.assertEqual("turnsign_slow", slow_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_SAFE_STOP, stop_command["state_cmd"])
        self.assertEqual("turnsign_stop", stop_debug["control_target"]["task_reason"])
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
