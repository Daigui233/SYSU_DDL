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

    def test_ar_preview_draws_task_target_offset_from_path_baseline(self):
        result = {
            "vision_control": {
                "selected_slot": 0,
                "control_target": {
                    "path_target_x": 200.0,
                    "path_target_y": 300.0,
                    "target_x": 255.0,
                    "task_offset_x": 55.0,
                },
                "command": {"track_error": -65.0},
            },
            "heatmap_peak_detection": {},
        }
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        with patch("vision_control.cv2.circle") as circle:
            render_vision_control_debug(frame, result)

        centers = [call.args[1] for call in circle.call_args_list]
        self.assertIn((200, 300), centers)
        self.assertIn((255, 300), centers)

    def test_ar_preview_draws_human_stop_line_at_y_320(self):
        result = {
            "vision_control": {
                "selected_slot": 0,
                "control_target": {
                    "path_target_x": 320.0,
                    "path_target_y": 300.0,
                    "target_x": 320.0,
                    "human_stop_line_y": 320.0,
                },
                "command": {"track_error": 0.0},
            },
            "heatmap_peak_detection": {},
        }
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        with patch("vision_control.cv2.line") as line:
            render_vision_control_debug(frame, result)

        segments = [
            (call.args[1], call.args[2], call.args[3])
            for call in line.call_args_list
        ]
        self.assertIn(((0, 320), (639, 320), (0, 255, 255)), segments)

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
    def test_default_human_stop_line_is_twenty_pixels_below_lookahead(self):
        planner = VisionControlPlanner(config=VisionControlConfig())

        stop_y = planner._human_stop_line_y((480, 640, 3), 300.0)

        self.assertAlmostEqual(320.0, stop_y)
        self.assertAlmostEqual(20.0, stop_y - 300.0)

    def test_human_touching_line_stops_even_far_from_planned_path(self):
        planner = VisionControlPlanner(config=_config())
        off_path_human_at_line = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [20.0, 260.0, 80.0, 350.0],
        }]

        command, debug = planner.update(
            _result(
                _straight_heatmap(80),
                detections=off_path_human_at_line),
            now=1.0,
        )

        self.assertEqual(STATE_AVOID_HUMAN, command["state_cmd"])
        self.assertAlmostEqual(-0.05, command["target_speed"])
        self.assertEqual(0.0, command["track_error"])
        self.assertEqual(
            "human_brake_reverse",
            debug["control_target"]["task_reason"],
        )

    def test_human_reverse_brake_runs_once_then_holds_zero(self):
        planner = VisionControlPlanner(config=_config(
            human_brake_reverse_speed_mps=-0.05,
            human_brake_reverse_duration_s=0.5,
        ))
        human = [{
            "label": "Human", "score": 0.95,
            "bbox": [280.0, 230.0, 360.0, 330.0],
        }]

        first, first_debug = planner.update(
            _result(_straight_heatmap(80), detections=human), now=1.0)
        during, during_debug = planner.update(
            _result(_straight_heatmap(80), detections=human), now=1.49)
        stopped, stopped_debug = planner.update(
            _result(_straight_heatmap(80), detections=human), now=1.5)
        still_stopped, still_stopped_debug = planner.update(
            _result(_straight_heatmap(80), detections=human), now=2.0)

        self.assertAlmostEqual(-0.05, first["target_speed"])
        self.assertEqual("human_brake_reverse", first_debug["control_target"]["task_reason"])
        self.assertAlmostEqual(-0.05, during["target_speed"])
        self.assertEqual(0.0, during["track_error"])
        self.assertEqual("human_brake_reverse", during_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_SAFE_STOP, stopped["state_cmd"])
        self.assertEqual(0.0, stopped["target_speed"])
        self.assertEqual("human_half_lookahead_stop", stopped_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_SAFE_STOP, still_stopped["state_cmd"])
        self.assertEqual(0.0, still_stopped["target_speed"])
        self.assertEqual("human_half_lookahead_stop", still_stopped_debug["control_target"]["task_reason"])

    def test_human_line_brake_has_priority_over_turnsign_approach(self):
        planner = VisionControlPlanner(config=_config())
        human = [{
            "label": "Human", "score": 0.95,
            "bbox": [280.0, 230.0, 360.0, 330.0],
        }]
        ocr_response = {
            "active": True,
            "control_phase": "turnsign_approach",
        }

        command, debug = planner.update(
            _result(_straight_heatmap(80), detections=human),
            ocr_response,
            now=1.0,
        )

        self.assertEqual(STATE_AVOID_HUMAN, command["state_cmd"])
        self.assertAlmostEqual(-0.05, command["target_speed"])
        self.assertEqual(0.0, command["track_error"])
        self.assertEqual(
            "human_brake_reverse",
            debug["control_target"]["task_reason"],
        )

    def test_near_preline_disappearance_waits_and_reappearance_drives(self):
        planner = VisionControlPlanner(config=_config(
            human_absence_confirm_s=1.5,
            human_preline_missing_px_480=20.0,
        ))
        near_line_human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [20.0, 230.0, 80.0, 315.0],
        }]

        seen, _ = planner.update(
            _result(_straight_heatmap(80), detections=near_line_human),
            now=1.0,
        )
        missing, missing_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=1.1)
        reappeared, reappeared_debug = planner.update(
            _result(_straight_heatmap(80), detections=near_line_human),
            now=1.2,
        )
        missing_again, _ = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=1.3)
        timeout, timeout_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=2.81)

        self.assertEqual(STATE_TRACK, seen["state_cmd"])
        self.assertEqual(STATE_SAFE_STOP, missing["state_cmd"])
        self.assertEqual(
            "human_preline_absence_check",
            missing_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_TRACK, reappeared["state_cmd"])
        self.assertEqual(
            "track", reappeared_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_SAFE_STOP, missing_again["state_cmd"])
        self.assertEqual(STATE_TRACK, timeout["state_cmd"])
        self.assertEqual("track", timeout_debug["control_target"]["task_reason"])

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

    def test_error_trend_positive_feedback_uses_five_frame_growth(self):
        planner = VisionControlPlanner(config=_config(
            error_trend_window=5,
            error_trend_min_frames=3,
            error_trend_kd=0.5,
            error_trend_deadband_640=2.0,
            error_trend_max_adjust_640=20.0,
        ))

        result = None
        for value in (20.0, 30.0, 40.0, 50.0, 60.0):
            result = planner._apply_error_trend(value, "track")

        adjusted, adjustment, slope, frames = result
        self.assertAlmostEqual(80.0, adjusted)
        self.assertAlmostEqual(20.0, adjustment)
        self.assertAlmostEqual(10.0, slope)
        self.assertEqual(5, frames)

    def test_error_trend_reduces_decreasing_magnitude(self):
        planner = VisionControlPlanner(config=_config(
            error_trend_window=5,
            error_trend_min_frames=3,
            error_trend_kd=0.5,
            error_trend_deadband_640=2.0,
            error_trend_max_adjust_640=20.0,
        ))

        result = None
        for value in (60.0, 50.0, 40.0, 30.0, 20.0):
            result = planner._apply_error_trend(value, "track")

        adjusted, adjustment, slope, frames = result
        self.assertAlmostEqual(0.0, adjusted)
        self.assertAlmostEqual(-20.0, adjustment)
        self.assertAlmostEqual(-10.0, slope)
        self.assertEqual(5, frames)

    def test_error_trend_resets_when_direction_or_mode_changes(self):
        planner = VisionControlPlanner(config=_config())
        for value in (20.0, 30.0, 40.0):
            planner._apply_error_trend(value, "track")

        changed_direction = planner._apply_error_trend(-50.0, "track")
        changed_mode = planner._apply_error_trend(-60.0, "avoid_car")

        self.assertEqual(1, changed_direction[3])
        self.assertEqual(0.0, changed_direction[1])
        self.assertEqual(1, changed_mode[3])
        self.assertEqual(0.0, changed_mode[1])

    def test_default_track_applies_explicit_left_and_right_gains(self):
        left_planner = VisionControlPlanner(config=_config(
            default_track_left_error_gain=0.80,
            default_track_left_error_step_640=1000.0,
            default_track_right_error_gain=1.00,
            default_track_right_error_step_640=1000.0))
        right_planner = VisionControlPlanner(config=_config(
            default_track_left_error_gain=0.80,
            default_track_right_error_gain=1.00,
            default_track_right_error_step_640=1000.0))

        left_command, left_debug = left_planner.update(
            _result(_straight_heatmap(56)), now=1.0)
        right_command, right_debug = right_planner.update(
            _result(_straight_heatmap(104)), now=1.0)

        left_target = left_debug["control_target"]
        right_target = right_debug["control_target"]
        self.assertAlmostEqual(
            left_target["raw_track_error_640"] * 0.80,
            left_command["track_error"], delta=0.01)
        self.assertAlmostEqual(
            right_target["raw_track_error_640"],
            right_command["track_error"], delta=0.01)
        self.assertAlmostEqual(
            0.80, left_target["default_track_error_gain"])
        self.assertAlmostEqual(
            1.00, right_target["default_track_error_gain"])

    def test_default_track_right_gain_controls_aggressiveness(self):
        planner = VisionControlPlanner(config=_config(
            default_track_right_error_gain=0.50,
            default_track_right_error_step_640=1000.0,
        ))

        command, debug = planner.update(
            _result(_straight_heatmap(104)), now=1.0)

        target = debug["control_target"]
        self.assertAlmostEqual(
            target["raw_track_error_640"] * 0.50,
            command["track_error"],
            delta=0.01,
        )
        self.assertAlmostEqual(
            0.50, target["default_track_error_gain"])

    def test_default_track_uses_independent_left_210_32_and_right_210_36(self):
        config = _config(
            max_error_step_640=32.0,
            max_track_error_640=160.0,
            default_track_left_max_error_640=210.0,
            default_track_left_error_step_640=32.0,
            default_track_right_max_error_640=210.0,
            default_track_right_error_step_640=36.0,
            default_track_left_error_gain=0.80,
        )
        right_planner = VisionControlPlanner(config=config)
        left_planner = VisionControlPlanner(config=config)

        right_command, right_debug = right_planner.update(
            _result(_straight_heatmap(104)), now=1.0)
        left_command, left_debug = left_planner.update(
            _result(_straight_heatmap(56)), now=1.0)

        self.assertAlmostEqual(36.0, right_command["track_error"])
        self.assertAlmostEqual(
            36.0, right_debug["control_target"]["error_step_limit_640"])
        self.assertAlmostEqual(
            210.0, right_debug["control_target"]["error_limit_640"])
        self.assertAlmostEqual(-32.0, left_command["track_error"])
        self.assertAlmostEqual(
            32.0, left_debug["control_target"]["error_step_limit_640"])
        self.assertAlmostEqual(
            210.0, left_debug["control_target"]["error_limit_640"])

        right_planner.last_error = 200.0
        limited_command, _ = right_planner.update(
            _result(_straight_heatmap(150)), now=1.1)
        self.assertAlmostEqual(210.0, limited_command["track_error"])

        left_planner.last_error = -200.0
        left_limited_command, _ = left_planner.update(
            _result(_straight_heatmap(10)), now=1.1)
        self.assertAlmostEqual(-210.0, left_limited_command["track_error"])

    def test_turnsign_left_steering_does_not_use_default_left_gain(self):
        planner = VisionControlPlanner(config=_config(
            default_track_left_error_gain=0.20,
            max_track_error_640=1000.0,
        ))
        response = {
            "active": True,
            "control_phase": "turnsign_edge_left",
            "bbox_center_x": 200.0,
        }

        command, debug = planner.update(
            _result(_straight_heatmap(80)), response, now=1.0)
        target = debug["control_target"]

        self.assertEqual("turnsign_edge_steer", target["task_reason"])
        self.assertLess(command["track_error"], 0.0)
        self.assertAlmostEqual(
            target["raw_track_error_640"],
            command["track_error"], delta=0.01)

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
        self.assertEqual(STATE_SAFE_STOP, command["state_cmd"])
        self.assertEqual(0.0, command["target_speed"])
        self.assertEqual(
            "ocr_wait_route", debug["control_target"]["task_reason"])

    def test_explicit_turnsign_approach_uses_point_one_zero(self):
        planner = VisionControlPlanner(config=_config())
        response = {
            "active": True,
            "control_phase": "turnsign_approach",
            "bbox_center_x": 320.0,
        }

        command, debug = planner.update(
            _result(_straight_heatmap(80)), response, now=1.0)

        self.assertEqual(STATE_TRACK, command["state_cmd"])
        self.assertAlmostEqual(0.10, command["target_speed"])
        self.assertEqual(
            "turnsign_approach", debug["control_target"]["task_reason"])
        self.assertAlmostEqual(
            debug["control_target"]["path_target_x"],
            debug["control_target"]["target_x"],
        )

    def test_new_turnsign_session_brakes_half_second_before_approach(self):
        planner = VisionControlPlanner(config=_config(
            turnsign_initial_brake_s=0.5,
            turnsign_slow_speed_mps=0.10,
        ))

        def response():
            return {
                "active": True,
                "session_active": True,
                "session_id": 7,
                "control_phase": "turnsign_approach",
                "bbox_center_x": 320.0,
            }

        first, first_debug = planner.update(
            _result(_straight_heatmap(80)), response(), now=1.0)
        braking, braking_debug = planner.update(
            _result(_straight_heatmap(80)), response(), now=1.49)
        approach, approach_debug = planner.update(
            _result(_straight_heatmap(80)), response(), now=1.50)

        self.assertEqual(STATE_SAFE_STOP, first["state_cmd"])
        self.assertEqual(0.0, first["target_speed"])
        self.assertEqual(
            "turnsign_initial_brake",
            first_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_SAFE_STOP, braking["state_cmd"])
        self.assertEqual(
            "turnsign_initial_brake",
            braking_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_TRACK, approach["state_cmd"])
        self.assertAlmostEqual(0.10, approach["target_speed"])
        self.assertEqual(
            "turnsign_approach",
            approach_debug["control_target"]["task_reason"],
        )

    def test_fast_ocr_result_cannot_shorten_initial_turnsign_brake(self):
        planner = VisionControlPlanner(config=_config(
            turnsign_initial_brake_s=0.5))
        active = {
            "active": True,
            "session_active": True,
            "session_id": 8,
            "control_phase": "turnsign_approach",
        }
        resolved = {
            "active": False,
            "session_active": True,
            "session_id": 8,
            "turnsign_resolved": True,
            "control_phase": "turnsign_consumed",
        }

        planner.update(
            _result(_straight_heatmap(80)), active, now=1.0)
        still_braking, still_braking_debug = planner.update(
            _result(_straight_heatmap(80)), dict(resolved), now=1.1)
        released, released_debug = planner.update(
            _result(_straight_heatmap(80)), dict(resolved), now=1.5)

        self.assertEqual(STATE_SAFE_STOP, still_braking["state_cmd"])
        self.assertEqual(0.0, still_braking["target_speed"])
        self.assertEqual(
            "turnsign_initial_brake",
            still_braking_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_TRACK, released["state_cmd"])
        self.assertEqual(
            "track", released_debug["control_target"]["task_reason"])

    def test_explicit_turnsign_edge_uses_gain_one_point_five(self):
        planner = VisionControlPlanner(config=_config())
        response = {
            "active": True,
            "control_phase": "turnsign_edge_right",
            "bbox_center_x": 500.0,
        }

        command, debug = planner.update(
            _result(_straight_heatmap(80)), response, now=1.0)
        target = debug["control_target"]

        self.assertEqual(STATE_TRACK, command["state_cmd"])
        self.assertAlmostEqual(0.10, command["target_speed"])
        self.assertEqual("turnsign_edge_steer", target["task_reason"])
        expected_target = target["path_target_x"] + 1.5 * (500.0 - 320.0)
        self.assertAlmostEqual(expected_target, target["target_x"], delta=0.01)
        self.assertAlmostEqual(270.0, target["task_offset_x"], delta=1.0)

    def test_turnsign_over_line_at_edge_reverses_two_seconds_then_rechecks(self):
        planner = VisionControlPlanner(config=_config())

        first_response = {"control_phase": "turnsign_edge_over_line"}
        first, first_debug = planner.update(
            _result(_straight_heatmap(80)), first_response, now=1.0)
        during, _ = planner.update(
            _result(_straight_heatmap(80)),
            {"control_phase": "turnsign_edge_over_line"},
            now=2.99,
        )
        recheck, recheck_debug = planner.update(
            _result(_straight_heatmap(80)),
            {"control_phase": "turnsign_edge_over_line"},
            now=3.0,
        )
        repeated, _ = planner.update(
            _result(_straight_heatmap(80)),
            {"control_phase": "turnsign_edge_over_line"},
            now=3.01,
        )

        self.assertEqual(STATE_TRACK, first["state_cmd"])
        self.assertAlmostEqual(-0.08, first["target_speed"])
        self.assertAlmostEqual(0.0, first["track_error"])
        self.assertEqual(
            "turnsign_reverse", first_debug["control_target"]["task_reason"])
        self.assertAlmostEqual(-0.08, during["target_speed"])
        self.assertEqual(STATE_SAFE_STOP, recheck["state_cmd"])
        self.assertEqual(0.0, recheck["target_speed"])
        self.assertEqual(
            "turnsign_post_reverse_stop",
            recheck_debug["control_target"]["task_reason"],
        )
        self.assertAlmostEqual(-0.08, repeated["target_speed"])

    def test_preconfirm_rejected_turnsign_phase_does_not_slow_car(self):
        planner = VisionControlPlanner(config=_config())

        command, debug = planner.update(
            _result(_straight_heatmap(80)),
            {"control_phase": "turnsign_too_far"},
            now=1.0,
        )

        self.assertEqual(STATE_TRACK, command["state_cmd"])
        self.assertAlmostEqual(0.15, command["target_speed"])
        self.assertEqual("track", debug["control_target"]["task_reason"])

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
        command, debug = VisionControlPlanner(config=_config(
            normal_speed_mps=0.17,
            obstacle_speed_mps=0.04,
        )).update(
            _result(_straight_heatmap(80), detections=detections),
            now=1.0,
        )

        self.assertEqual(STATE_AVOID_CAR, command["state_cmd"])
        self.assertAlmostEqual(0.10, command["target_speed"])
        self.assertEqual("car_in_path_bias", debug["control_target"]["task_reason"])

    def test_locked_curve_is_baseline_for_car_avoidance_offset(self):
        result = _curve_result_at(200.0, 430.0)
        result["detections"] = [{
            "label": "Car",
            "score": 0.90,
            "bbox": [160.0, 280.0, 220.0, 420.0],
        }]
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1))

        first_command, first_debug = planner.update(result, now=1.0)
        _command, debug = planner.update(result, now=1.4)
        target = debug["control_target"]

        self.assertAlmostEqual(0.0, first_debug["control_target"]["task_offset_x"])
        self.assertEqual(0, debug["selected_slot"])
        self.assertEqual("car_in_path_bias", target["task_reason"])
        self.assertAlmostEqual(200.0, target["path_target_x"], delta=1.0)
        self.assertGreater(target["target_x"], target["path_target_x"])
        self.assertAlmostEqual(
            56.0 * 0.4 / 1.5,
            target["task_offset_x"],
            delta=0.01,
        )
        self.assertAlmostEqual(
            target["target_x"] - target["path_target_x"],
            target["task_offset_x"], delta=0.01)
        self.assertTrue(target["task_target_applied"])

    def test_car_avoidance_ramps_holds_parallel_then_ramps_back(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve",
            path_smooth_window=1,
            car_avoid_ramp_s=1.5,
            car_avoid_hold_s=1.0,
        ))
        with_car = _curve_result_at(200.0, 430.0)
        with_car["detections"] = [{
            "label": "Car",
            "score": 0.90,
            "bbox": [160.0, 280.0, 220.0, 420.0],
        }]

        _car_command, start_debug = planner.update(with_car, now=1.0)
        _full_command, full_debug = planner.update(with_car, now=2.5)
        hold_command, hold_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=3.49)
        return_start_command, return_start_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=3.51)
        return_mid_command, return_mid_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=4.26)
        resume_command, resume_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=5.01)

        start_target = start_debug["control_target"]
        full_target = full_debug["control_target"]
        hold_target = hold_debug["control_target"]
        return_start_target = return_start_debug["control_target"]
        return_mid_target = return_mid_debug["control_target"]
        self.assertAlmostEqual(0.0, start_target["task_offset_x"])
        self.assertAlmostEqual(56.0, full_target["task_offset_x"], delta=0.01)
        self.assertEqual(STATE_AVOID_CAR, hold_command["state_cmd"])
        self.assertEqual("car_avoid_hold", hold_target["task_reason"])
        self.assertAlmostEqual(
            full_target["task_offset_x"],
            hold_target["task_offset_x"],
            delta=0.01,
        )
        self.assertEqual(STATE_AVOID_CAR, return_start_command["state_cmd"])
        self.assertEqual("car_avoid_return", return_start_target["task_reason"])
        self.assertAlmostEqual(56.0, return_start_target["task_offset_x"], delta=0.01)
        self.assertEqual(STATE_AVOID_CAR, return_mid_command["state_cmd"])
        self.assertAlmostEqual(28.0, return_mid_target["task_offset_x"], delta=0.01)
        self.assertEqual(STATE_TRACK, resume_command["state_cmd"])
        self.assertEqual("track", resume_debug["control_target"]["task_reason"])

    def test_human_behind_car_cannot_flip_side_then_passes_at_point_four(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve",
            path_smooth_window=1,
            car_avoid_ramp_s=0.8,
            car_avoid_hold_s=2.0,
            car_human_pass_speed_mps=0.40,
            car_human_pass_hold_s=1.0,
        ))
        car_and_human = _curve_result_at(200.0, 430.0)
        car_and_human["detections"] = [
            {
                "label": "Car",
                "score": 0.90,
                "bbox": [160.0, 280.0, 220.0, 420.0],
            },
            {
                # The person is farther up-track and on the current right-side
                # avoidance route. Generic human avoidance would steer left.
                "label": "Human",
                "score": 0.95,
                "bbox": [270.0, 180.0, 330.0, 290.0],
            },
        ]
        crossed_human = _curve_result_at(200.0, 430.0)
        crossed_human["detections"] = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [190.0, 250.0, 250.0, 350.0],
        }]
        blocking_human = _curve_result_at(200.0, 430.0)
        blocking_human["detections"] = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [240.0, 250.0, 270.0, 350.0],
        }]

        approach_command, approach_debug = planner.update(
            car_and_human, now=1.0)
        wait_command, wait_debug = planner.update(
            blocking_human, now=1.05)
        missing_command, missing_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=1.56)
        pass_command, pass_debug = planner.update(
            crossed_human, now=1.6)
        hold_command, hold_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=2.5)
        resume_command, resume_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=2.61)
        consumed_command, consumed_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=3.61)
        parallel_command, parallel_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=4.2)
        return_command, return_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=4.4)
        returned_command, returned_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=5.2)

        approach_target = approach_debug["control_target"]
        wait_target = wait_debug["control_target"]
        pass_target = pass_debug["control_target"]
        hold_target = hold_debug["control_target"]
        self.assertEqual(STATE_AVOID_CAR, approach_command["state_cmd"])
        self.assertEqual(
            "car_human_preline_approach", approach_target["task_reason"])
        self.assertAlmostEqual(0.0, approach_target["task_offset_x"])
        self.assertEqual(STATE_AVOID_HUMAN, wait_command["state_cmd"])
        self.assertAlmostEqual(-0.05, wait_command["target_speed"])
        self.assertEqual("human_brake_reverse", wait_target["task_reason"])
        self.assertGreater(wait_target["task_offset_x"], 0.0)
        self.assertEqual(STATE_SAFE_STOP, missing_command["state_cmd"])
        self.assertEqual(0.0, missing_command["target_speed"])
        self.assertEqual(
            "car_human_absence_check",
            missing_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_AVOID_HUMAN, pass_command["state_cmd"])
        self.assertAlmostEqual(0.40, pass_command["target_speed"])
        self.assertEqual("car_human_same_side_pass", pass_target["task_reason"])
        self.assertGreater(pass_target["task_offset_x"], 0.0)
        self.assertGreater(pass_target["task_offset_x"], approach_target["task_offset_x"])
        self.assertEqual(STATE_AVOID_HUMAN, hold_command["state_cmd"])
        self.assertAlmostEqual(0.40, hold_command["target_speed"])
        self.assertEqual(
            "car_human_same_side_pass_hold", hold_target["task_reason"])
        self.assertGreater(hold_target["task_offset_x"], 0.0)
        self.assertEqual(STATE_AVOID_CAR, resume_command["state_cmd"])
        self.assertAlmostEqual(0.10, resume_command["target_speed"])
        self.assertEqual(
            "car_human_consumed_ignore",
            resume_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_AVOID_CAR, consumed_command["state_cmd"])
        self.assertEqual(
            "car_human_consumed_ignore",
            consumed_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_AVOID_CAR, parallel_command["state_cmd"])
        self.assertEqual(
            "car_avoid_hold", parallel_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_AVOID_CAR, return_command["state_cmd"])
        self.assertEqual(
            "car_avoid_return", return_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_TRACK, returned_command["state_cmd"])
        self.assertEqual(
            "track", returned_debug["control_target"]["task_reason"])

    def test_new_upper_person_overrides_car_human_pass_on_monitor_line(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve",
            path_smooth_window=1,
            car_avoid_hold_s=2.0,
            car_human_pass_speed_mps=0.40,
            car_human_pass_hold_s=1.0,
        ))
        car = {
            "label": "Car", "score": 0.90,
            "bbox": [160.0, 280.0, 220.0, 420.0],
        }
        preline = {
            "label": "Human", "score": 0.95,
            "bbox": [270.0, 180.0, 330.0, 290.0],
        }
        blocking = {
            "label": "Human", "score": 0.95,
            "bbox": [240.0, 250.0, 270.0, 350.0],
        }
        crossed = {
            "label": "Human", "score": 0.95,
            "bbox": [190.0, 250.0, 250.0, 350.0],
        }
        consumed_lower = {
            "label": "Human", "score": 0.95,
            "bbox": [190.0, 330.0, 250.0, 430.0],
        }
        new_upper = {
            "label": "Human", "score": 0.94,
            "bbox": [270.0, 180.0, 330.0, 290.0],
        }
        new_at_line = {
            "label": "Human", "score": 0.94,
            "bbox": [270.0, 220.0, 330.0, 330.0],
        }

        initial = _curve_result_at(200.0, 430.0)
        initial["detections"] = [car, preline]
        at_line = _curve_result_at(200.0, 430.0)
        at_line["detections"] = [car, blocking]
        crossed_result = _curve_result_at(200.0, 430.0)
        crossed_result["detections"] = [car, crossed]
        upper_result = _curve_result_at(200.0, 430.0)
        upper_result["detections"] = [car, consumed_lower, new_upper]
        new_line_result = _curve_result_at(200.0, 430.0)
        new_line_result["detections"] = [
            car, consumed_lower, new_at_line]

        planner.update(initial, now=1.0)
        planner.update(at_line, now=1.1)
        launched, launched_debug = planner.update(
            crossed_result, now=1.6)
        upper_seen, upper_seen_debug = planner.update(
            upper_result, now=1.7)
        stopped, stopped_debug = planner.update(
            new_line_result, now=1.8)

        self.assertEqual(STATE_AVOID_HUMAN, launched["state_cmd"])
        self.assertAlmostEqual(0.40, launched["target_speed"])
        self.assertGreater(
            launched_debug["control_target"]["task_offset_x"], 0.0)
        self.assertEqual(STATE_AVOID_HUMAN, upper_seen["state_cmd"])
        self.assertAlmostEqual(0.40, upper_seen["target_speed"])
        self.assertGreater(
            upper_seen_debug["control_target"]["task_offset_x"],
            launched_debug["control_target"]["task_offset_x"],
        )
        self.assertEqual(STATE_AVOID_HUMAN, stopped["state_cmd"])
        self.assertAlmostEqual(-0.05, stopped["target_speed"])
        self.assertEqual(
            "human_brake_reverse",
            stopped_debug["control_target"]["task_reason"],
        )
        self.assertEqual(0.0, planner.human_return_until)
        self.assertGreater(
            stopped_debug["control_target"]["task_offset_x"], 0.0)

    def test_car_human_far_preline_absence_is_ignored_during_car_hold(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve",
            path_smooth_window=1,
            car_avoid_ramp_s=0.8,
            car_avoid_hold_s=2.0,
            human_absence_confirm_s=1.5,
        ))
        car_and_human = _curve_result_at(200.0, 430.0)
        car_and_human["detections"] = [
            {
                "label": "Car",
                "score": 0.90,
                "bbox": [160.0, 280.0, 220.0, 420.0],
            },
            {
                "label": "Human",
                "score": 0.95,
                "bbox": [270.0, 180.0, 330.0, 290.0],
            },
        ]

        planner.update(car_and_human, now=1.0)
        checking_command, checking_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=2.49)
        car_hold_command, car_hold_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=2.51)
        hold_later_command, hold_later_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=3.01)
        return_command, return_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=3.81)
        clear_command, clear_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=4.61)

        self.assertEqual(STATE_AVOID_CAR, checking_command["state_cmd"])
        self.assertEqual(
            "car_avoid_hold",
            checking_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_AVOID_CAR, car_hold_command["state_cmd"])
        self.assertEqual(
            "car_avoid_hold", car_hold_debug["control_target"]["task_reason"])
        self.assertGreater(
            car_hold_debug["control_target"]["task_offset_x"], 0.0)
        self.assertEqual(STATE_AVOID_CAR, hold_later_command["state_cmd"])
        self.assertEqual(
            "car_avoid_hold",
            hold_later_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_AVOID_CAR, return_command["state_cmd"])
        self.assertEqual(
            "car_avoid_return", return_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_TRACK, clear_command["state_cmd"])
        self.assertEqual("track", clear_debug["control_target"]["task_reason"])

    def test_car_human_near_preline_dropout_waits_then_reappears_at_car_speed(self):
        planner = VisionControlPlanner(config=_config(
            path_source="curve",
            path_smooth_window=1,
            normal_speed_mps=0.17,
            obstacle_speed_mps=0.04,
            car_avoid_hold_s=2.0,
            human_absence_confirm_s=1.5,
            human_preline_missing_px_480=20.0,
        ))
        car_and_human = _curve_result_at(200.0, 430.0)
        car_and_human["detections"] = [
            {
                "label": "Car",
                "score": 0.90,
                "bbox": [160.0, 280.0, 220.0, 420.0],
            },
            {
                "label": "Human",
                "score": 0.95,
                "bbox": [270.0, 230.0, 330.0, 315.0],
            },
        ]
        human_reappeared = _curve_result_at(200.0, 430.0)
        human_reappeared["detections"] = [car_and_human["detections"][1]]

        approach, _ = planner.update(car_and_human, now=1.0)
        missing, missing_debug = planner.update(
            _curve_result_at(200.0, 430.0), now=1.1)
        reappeared, reappeared_debug = planner.update(
            human_reappeared, now=1.2)

        self.assertEqual(STATE_AVOID_CAR, approach["state_cmd"])
        self.assertAlmostEqual(0.10, approach["target_speed"])
        self.assertEqual(STATE_SAFE_STOP, missing["state_cmd"])
        self.assertEqual(
            "car_human_preline_absence_check",
            missing_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_AVOID_CAR, reappeared["state_cmd"])
        self.assertAlmostEqual(0.10, reappeared["target_speed"])
        self.assertEqual(
            "car_human_preline_approach",
            reappeared_debug["control_target"]["task_reason"],
        )

    def test_locked_curve_is_baseline_for_coin_offset(self):
        result = _curve_result_at(200.0, 430.0)
        result["detections"] = [{
            "label": "Coin",
            "score": 0.90,
            "bbox": [225.0, 300.0, 275.0, 420.0],
        }]
        planner = VisionControlPlanner(config=_config(
            path_source="curve", path_smooth_window=1))

        _command, debug = planner.update(result, now=1.0)
        target = debug["control_target"]

        self.assertEqual(0, debug["selected_slot"])
        self.assertEqual("coin_bias", target["task_reason"])
        self.assertAlmostEqual(200.0, target["path_target_x"], delta=1.0)
        self.assertGreater(target["target_x"], target["path_target_x"])
        self.assertAlmostEqual(22.5, target["task_offset_x"], delta=1.0)
        self.assertTrue(target["task_target_applied"])

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
        self.assertAlmostEqual(0.10, command["target_speed"])
        self.assertEqual("turnsign_slow", debug["control_target"]["task_reason"])

    def test_low_confidence_turnsign_does_not_trigger_slowdown(self):
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

    def test_legacy_turnsign_latches_after_three_detections(self):
        planner = VisionControlPlanner(config=_config())
        sign = [{
            "label": "TurnSign",
            "score": 0.90,
            "bbox": [300.0, 80.0, 350.0, 150.0],
        }]

        slow_command = None
        slow_debug = None
        for index in range(3):
            slow_command, slow_debug = planner.update(
                _result(_straight_heatmap(80), detections=sign),
                now=1.0 + index * 0.01,
            )
        stop_command, stop_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=1.2)
        resume_command, resume_debug = planner.update(
            _result(_fork_heatmaps()),
            _ocr_response("right", active=True),
            now=1.3,
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

    def test_preview_sized_turnsign_stops_before_lookahead(self):
        detections = [{
            "label": "TurnSign",
            "score": 0.59,
            "bbox": [285.0, 136.0, 425.0, 205.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _result(_straight_heatmap(80), detections=detections),
            {"active": True, "status": "ocr_pending"},
            now=1.0,
        )

        self.assertEqual(STATE_SAFE_STOP, command["state_cmd"])
        self.assertEqual(0.0, command["target_speed"])
        self.assertEqual(0, command["flags"])
        self.assertEqual("turnsign_stop", debug["control_target"]["task_reason"])

    def test_resolved_turnsign_does_not_stop_again_while_still_visible(self):
        detections = [{
            "label": "TurnSign",
            "score": 0.90,
            "bbox": [285.0, 136.0, 425.0, 205.0],
        }]
        ocr_response = {
            "active": False,
            "status": "route_ready_held",
            "turnsign_resolved": True,
            "instruction_current": False,
            "latest_instruction": {"direction": "right"},
        }

        command, debug = VisionControlPlanner(config=_config()).update(
            _result(_straight_heatmap(80), detections=detections),
            ocr_response,
            now=1.0,
        )

        self.assertEqual(STATE_TRACK, command["state_cmd"])
        self.assertGreater(command["target_speed"], 0.0)
        self.assertEqual("track", debug["control_target"]["task_reason"])

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
        self.assertAlmostEqual(0.10, slow_command["target_speed"])
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

    def test_human_detection_loss_stays_stopped_until_crossing_is_observed(self):
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
        missing_command, missing_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=1.5,
        )
        crossed = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 430.0],
        }]
        resume_command, resume_debug = planner.update(
            _result(_straight_heatmap(80), detections=crossed),
            now=1.6,
        )

        self.assertEqual(STATE_AVOID_HUMAN, stop_command["state_cmd"])
        self.assertAlmostEqual(-0.05, stop_command["target_speed"])
        self.assertEqual("human_brake_reverse", stop_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_SAFE_STOP, missing_command["state_cmd"])
        self.assertEqual(0.0, missing_command["target_speed"])
        self.assertEqual(
            "human_absence_check",
            missing_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_AVOID_HUMAN, resume_command["state_cmd"])
        self.assertGreater(resume_command["target_speed"], 0.0)
        self.assertEqual(
            "human_cross_pass", resume_debug["control_target"]["task_reason"])

    def test_human_far_preline_absence_is_ignored(self):
        planner = VisionControlPlanner(config=_config(
            human_absence_confirm_s=1.5))
        approaching_human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [285.0, 100.0, 355.0, 200.0],
        }]

        seen_command, _seen_debug = planner.update(
            _result(_straight_heatmap(80), detections=approaching_human),
            now=1.0,
        )
        checking_command, checking_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=2.49,
        )
        clear_command, clear_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=2.51,
        )

        self.assertEqual(STATE_TRACK, seen_command["state_cmd"])
        self.assertEqual(STATE_TRACK, checking_command["state_cmd"])
        self.assertEqual(
            "track",
            checking_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_TRACK, clear_command["state_cmd"])
        self.assertEqual("track", clear_debug["control_target"]["task_reason"])

    def test_human_reappearance_restarts_absence_confirmation(self):
        planner = VisionControlPlanner(config=_config(
            human_absence_confirm_s=1.5))
        human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [285.0, 330.0, 355.0, 430.0],
        }]

        planner.update(
            _result(_straight_heatmap(80), detections=human), now=1.0)
        first_gap, _first_gap_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=2.0)
        reappeared, _reappeared_debug = planner.update(
            _result(_straight_heatmap(80), detections=human), now=2.4)
        reset_gap, reset_gap_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=3.8)
        clear_command, clear_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]), now=3.91)

        self.assertEqual(STATE_SAFE_STOP, first_gap["state_cmd"])
        self.assertEqual(STATE_SAFE_STOP, reappeared["state_cmd"])
        self.assertEqual(STATE_SAFE_STOP, reset_gap["state_cmd"])
        self.assertEqual(
            "human_absence_check",
            reset_gap_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_TRACK, clear_command["state_cmd"])
        self.assertEqual("track", clear_debug["control_target"]["task_reason"])

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
            now=1.5,
        )
        hold_command, hold_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=1.6,
        )
        resume_command, resume_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=2.51,
        )
        mid_return_command, mid_return_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=3.0,
        )
        returned_command, returned_debug = planner.update(
            _result(_straight_heatmap(80), detections=[]),
            now=3.51,
        )

        self.assertEqual(STATE_AVOID_HUMAN, pass_command["state_cmd"])
        self.assertAlmostEqual(0.42, pass_command["target_speed"])
        self.assertLess(pass_command["track_error"], 0.0)
        self.assertEqual("human_cross_pass", pass_debug["control_target"]["task_reason"])
        self.assertEqual(STATE_AVOID_HUMAN, hold_command["state_cmd"])
        self.assertAlmostEqual(0.42, hold_command["target_speed"])
        self.assertEqual("human_cross_pass", hold_debug["control_target"]["task_reason"])
        self.assertAlmostEqual(
            pass_debug["control_target"]["task_offset_x"],
            hold_debug["control_target"]["task_offset_x"],
            delta=0.01,
        )
        self.assertTrue(hold_debug["control_target"]["task_target_applied"])
        self.assertEqual(STATE_TRACK, resume_command["state_cmd"])
        self.assertAlmostEqual(0.15, resume_command["target_speed"])
        self.assertEqual(
            "human_return", resume_debug["control_target"]["task_reason"])
        self.assertAlmostEqual(
            39.6,
            resume_debug["control_target"][
                "human_return_extra_error_640"],
            delta=0.01,
        )
        self.assertEqual(STATE_TRACK, mid_return_command["state_cmd"])
        self.assertAlmostEqual(0.15, mid_return_command["target_speed"])
        self.assertAlmostEqual(
            20.0,
            mid_return_debug["control_target"][
                "human_return_extra_error_640"],
            delta=0.01,
        )
        self.assertEqual(STATE_TRACK, returned_command["state_cmd"])
        self.assertEqual(
            "track", returned_debug["control_target"]["task_reason"])

    def test_consumed_person_is_ignored_but_new_upper_person_stops_pass(self):
        planner = VisionControlPlanner(config=_config(
            human_pass_speed_mps=0.40,
            human_speed_hold_s=1.0,
        ))
        old_left = {
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 430.0],
        }
        old_right = {
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 430.0],
        }
        new_upper = {
            "label": "Human", "score": 0.94,
            "bbox": [250.0, 180.0, 310.0, 290.0],
        }
        new_at_line = {
            "label": "Human", "score": 0.94,
            "bbox": [250.0, 220.0, 310.0, 330.0],
        }

        planner.update(
            _result(_straight_heatmap(80), detections=[old_left]),
            now=1.0,
        )
        launched, launched_debug = planner.update(
            _result(_straight_heatmap(80), detections=[old_right]),
            now=1.5,
        )
        old_only, old_only_debug = planner.update(
            _result(_straight_heatmap(80), detections=[old_right]),
            now=1.6,
        )
        upper_seen, upper_seen_debug = planner.update(
            _result(
                _straight_heatmap(80),
                detections=[old_right, new_upper],
            ),
            now=1.7,
        )
        stopped, stopped_debug = planner.update(
            _result(
                _straight_heatmap(80),
                detections=[old_right, new_at_line],
            ),
            now=1.8,
        )

        self.assertEqual(STATE_AVOID_HUMAN, launched["state_cmd"])
        self.assertAlmostEqual(0.40, launched["target_speed"])
        self.assertEqual(
            "human_cross_pass",
            launched_debug["control_target"]["task_reason"],
        )
        self.assertEqual(STATE_AVOID_HUMAN, old_only["state_cmd"])
        self.assertAlmostEqual(0.40, old_only["target_speed"])
        self.assertEqual(
            launched_debug["control_target"]["task_offset_x"],
            old_only_debug["control_target"]["task_offset_x"],
        )
        self.assertEqual(STATE_AVOID_HUMAN, upper_seen["state_cmd"])
        self.assertAlmostEqual(0.40, upper_seen["target_speed"])
        self.assertEqual(
            launched_debug["control_target"]["task_offset_x"],
            upper_seen_debug["control_target"]["task_offset_x"],
        )
        self.assertEqual(STATE_AVOID_HUMAN, stopped["state_cmd"])
        self.assertAlmostEqual(-0.05, stopped["target_speed"])
        self.assertEqual(
            "human_brake_reverse",
            stopped_debug["control_target"]["task_reason"],
        )

    def test_new_upper_person_missing_near_line_stops_without_using_old_person(self):
        planner = VisionControlPlanner(config=_config(
            human_pass_speed_mps=0.40,
            human_speed_hold_s=1.0,
            human_absence_confirm_s=1.5,
        ))
        old_left = {
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 430.0],
        }
        old_right = {
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 430.0],
        }
        new_upper = {
            "label": "Human", "score": 0.94,
            "bbox": [250.0, 180.0, 310.0, 290.0],
        }
        new_near_line = {
            "label": "Human", "score": 0.94,
            "bbox": [250.0, 200.0, 310.0, 310.0],
        }

        planner.update(
            _result(_straight_heatmap(80), detections=[old_left]),
            now=1.0,
        )
        planner.update(
            _result(_straight_heatmap(80), detections=[old_right]),
            now=1.5,
        )
        planner.update(
            _result(
                _straight_heatmap(80),
                detections=[old_right, new_upper],
            ),
            now=1.6,
        )
        planner.update(
            _result(
                _straight_heatmap(80),
                detections=[old_right, new_near_line],
            ),
            now=1.7,
        )
        missing, missing_debug = planner.update(
            _result(_straight_heatmap(80), detections=[old_right]),
            now=1.8,
        )

        self.assertEqual(STATE_SAFE_STOP, missing["state_cmd"])
        self.assertEqual(0.0, missing["target_speed"])
        self.assertEqual(
            "human_new_person_absence_check",
            missing_debug["control_target"]["task_reason"],
        )

    def test_new_upper_person_missing_far_from_line_does_not_latch(self):
        planner = VisionControlPlanner(config=_config(
            human_pass_speed_mps=0.40,
            human_speed_hold_s=1.0,
        ))
        old_left = {
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 430.0],
        }
        old_right = {
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 430.0],
        }
        new_far = {
            "label": "Human", "score": 0.94,
            "bbox": [250.0, 140.0, 310.0, 250.0],
        }
        old_at_line = {
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 250.0, 420.0, 350.0],
        }

        planner.update(
            _result(_straight_heatmap(80), detections=[old_left]),
            now=1.0,
        )
        planner.update(
            _result(_straight_heatmap(80), detections=[old_right]),
            now=1.5,
        )
        planner.update(
            _result(
                _straight_heatmap(80),
                detections=[old_right, new_far],
            ),
            now=1.6,
        )
        continued, continued_debug = planner.update(
            _result(_straight_heatmap(80), detections=[old_at_line]),
            now=1.7,
        )

        self.assertEqual(STATE_AVOID_HUMAN, continued["state_cmd"])
        self.assertAlmostEqual(0.40, continued["target_speed"])
        self.assertEqual(
            "human_cross_pass",
            continued_debug["control_target"]["task_reason"],
        )

    def test_human_can_also_cross_from_right_to_left_without_car(self):
        planner = VisionControlPlanner(config=_config())
        right_human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 430.0],
        }]
        left_human = [{
            "label": "Human",
            "score": 0.95,
            "bbox": [220.0, 330.0, 280.0, 430.0],
        }]

        stop_command, _stop_debug = planner.update(
            _result(_straight_heatmap(80), detections=right_human),
            now=1.0,
        )
        pass_command, pass_debug = planner.update(
            _result(_straight_heatmap(80), detections=left_human),
            now=1.5,
        )

        self.assertEqual(STATE_AVOID_HUMAN, stop_command["state_cmd"])
        self.assertAlmostEqual(-0.05, stop_command["target_speed"])
        self.assertEqual(STATE_AVOID_HUMAN, pass_command["state_cmd"])
        self.assertAlmostEqual(0.42, pass_command["target_speed"])
        self.assertGreater(pass_command["track_error"], 0.0)
        self.assertEqual(
            "human_cross_pass", pass_debug["control_target"]["task_reason"])


if __name__ == "__main__":
    unittest.main()
