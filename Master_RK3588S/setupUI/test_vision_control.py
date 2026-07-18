import os
import sys
import unittest
from unittest import mock

import numpy as np


sys.path.insert(0, os.path.dirname(__file__))

from vision_control import (  # noqa: E402
    STATE_AVOID_CAR,
    STATE_AVOID_HUMAN,
    STATE_COLLECT_GOLD,
    STATE_RECOVER_LINE,
    STATE_SAFE_STOP,
    STATE_TRACK,
    VisionControlBirdseyeRenderer,
    VisionControlConfig,
    VisionControlPlanner,
    _FittedControlPathTracker,
    _associated_point_mask,
    _birdseye_debug_source_quad_from_road,
    _densify_associated_lower_points,
    _extract_curve_preview_lines,
    _fit_smooth_majority_curve,
    _identity_probability_color,
    _semantic_road_point_mask,
    render_vision_control_birdseye,
    render_vision_control_debug,
)


def _config(**overrides):
    values = {
        "visual_center_x": 0.50,
        "max_error_step_640": 1000.0,
        "route_confirm_frames": 1,
        "ocr_confirm_frames": 1,
    }
    values.update(overrides)
    return VisionControlConfig(**values)


def _raw_path(slot, x_values, y_values=None):
    if y_values is None:
        y_values = np.linspace(460.0, 60.0, 32, dtype=np.float32)
    y_values = np.asarray(y_values, dtype=np.float32)
    if np.isscalar(x_values):
        x_values = np.full_like(y_values, float(x_values))
    return {
        "slot": int(slot),
        "role": "left" if int(slot) == 0 else "right",
        "source": "row_classifier_raw",
        "score": 0.9,
        "point_confidences": np.full(len(y_values), 0.8, dtype=np.float32),
        "points_xy": np.stack(
            (np.asarray(x_values, dtype=np.float32), y_values), axis=1),
    }


def _raw_result_at(blue_x=240.0, green_x=400.0, slots=(0, 1),
                   detections=None):
    paths = [
        _raw_path(slot, blue_x if slot == 0 else green_x)
        for slot in slots
    ]
    return {
        "raw_curve_paths": paths,
        "centerline": {"raw_curve_paths": paths},
        "road": {"mask": np.ones((120, 160), dtype=np.uint8)},
        "image_shape": (480, 640, 3),
        "detections": list(detections or []),
    }


def _trim_result(far_gap_640, lookahead_gap_640=None):
    ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
    lookahead_gap = (
        float(far_gap_640) if lookahead_gap_640 is None
        else float(lookahead_gap_640))
    blend = np.clip((ys - 168.0) / (300.0 - 168.0), 0.0, 1.0)
    gaps = float(far_gap_640) * (1.0 - blend) + lookahead_gap * blend
    paths = [_raw_path(0, 260.0, ys), _raw_path(1, 260.0 + gaps, ys)]
    return {
        "raw_curve_paths": paths,
        "centerline": {"raw_curve_paths": paths},
        "road": {"mask": np.ones((120, 160), dtype=np.uint8)},
        "image_shape": (480, 640, 3),
        "detections": [],
    }


def _road_mask_following_path(x_values, y_values, half_width_px=110.0):
    """Build a low-resolution road mask whose center follows a test path."""
    y_values = np.asarray(y_values, dtype=np.float32)
    x_values = np.asarray(x_values, dtype=np.float32)
    order = np.argsort(y_values)
    y_values = y_values[order]
    x_values = x_values[order]
    mask = np.zeros((120, 160), dtype=np.uint8)
    for mask_y in range(mask.shape[0]):
        image_y = float(mask_y) * 479.0 / float(mask.shape[0] - 1)
        center_x = float(np.interp(image_y, y_values, x_values))
        left = int(round((center_x - half_width_px) * 159.0 / 639.0))
        right = int(round((center_x + half_width_px) * 159.0 / 639.0))
        mask[mask_y, max(0, left):min(mask.shape[1], right + 1)] = 1
    return mask


class CurveConstructionTest(unittest.TestCase):
    def test_association_rejects_isolated_outliers(self):
        points = np.asarray([
            [200, 460], [202, 440], [204, 420],
            [500, 400], [208, 380], [210, 360], [212, 340],
        ], dtype=np.float32)
        keep = _associated_point_mask(points, 640)
        self.assertFalse(bool(keep[3]))
        self.assertGreaterEqual(int(np.count_nonzero(keep)), 6)

    def test_densify_compensates_missing_lower_anchors(self):
        points = np.asarray([
            [200, 460], [204, 440], [208, 420], [212, 400],
        ], dtype=np.float32)
        probabilities = np.full(4, 0.8, dtype=np.float32)
        associated = np.asarray([True, False, True, True])
        dense, dense_probabilities = _densify_associated_lower_points(
            points, probabilities, associated, 640, 480,
            lower_boundary_y=300.0)
        self.assertEqual(len(dense), 4)
        self.assertEqual(len(dense_probabilities), 4)
        self.assertFalse(np.any(np.isclose(dense[:, 1], 440.0)))
        self.assertTrue(np.any(np.isclose(dense[:, 1], 410.0)))

    def test_fit_uses_majority_and_rejects_outlier(self):
        ys = np.linspace(460.0, 120.0, 16, dtype=np.float32)
        xs = 240.0 + 0.0015 * (ys - 300.0) ** 2
        points = np.stack((xs, ys), axis=1)
        points[7, 0] += 180.0
        fitted, probabilities, inliers = _fit_smooth_majority_curve(
            points, np.full(16, 0.9, dtype=np.float32), 640,
            extend_to_y=300.0)
        self.assertGreaterEqual(int(np.count_nonzero(inliers)), 14)
        self.assertEqual(len(fitted), len(probabilities))
        self.assertLess(float(np.max(fitted[:, 0])), 320.0)

    def test_tracker_confirms_large_jump_on_second_frame(self):
        tracker = _FittedControlPathTracker()
        ys = np.asarray([200, 300, 400], dtype=np.float32)
        first = np.stack((np.full_like(ys, 220), ys), axis=1)
        jumped = np.stack((np.full_like(ys, 500), ys), axis=1)
        probabilities = np.ones(3, dtype=np.float32)
        tracker.update(0, first, probabilities, (480, 640, 3), now=1.0)
        pending, _ = tracker.update(
            0, jumped, probabilities, (480, 640, 3), now=1.04)
        confirmed, _ = tracker.update(
            0, jumped, probabilities, (480, 640, 3), now=1.08)
        self.assertAlmostEqual(220.0, float(np.mean(pending[:, 0])))
        self.assertAlmostEqual(500.0, float(np.mean(confirmed[:, 0])))

    def test_tracker_damps_small_jitter(self):
        tracker = _FittedControlPathTracker()
        ys = np.asarray([200, 300, 400], dtype=np.float32)
        probabilities = np.ones(3, dtype=np.float32)
        base = np.stack((np.full_like(ys, 320), ys), axis=1)
        jitter = np.stack((np.full_like(ys, 328), ys), axis=1)
        tracker.update(0, base, probabilities, (480, 640, 3), now=1.0)
        filtered, _ = tracker.update(
            0, jitter, probabilities, (480, 640, 3), now=1.04)
        self.assertGreater(float(np.mean(filtered[:, 0])), 320.0)
        self.assertLess(float(np.mean(filtered[:, 0])), 328.0)

    def test_tracker_accelerates_coherent_curve_motion(self):
        tracker = _FittedControlPathTracker()
        ys = np.asarray([200, 300, 400], dtype=np.float32)
        probabilities = np.ones(3, dtype=np.float32)
        base = np.stack((np.full_like(ys, 320), ys), axis=1)
        first_curve = np.stack((np.full_like(ys, 338), ys), axis=1)
        second_curve = np.stack((np.full_like(ys, 350), ys), axis=1)
        tracker.update(0, base, probabilities, (480, 640, 3), now=1.0)
        first, _ = tracker.update(
            0, first_curve, probabilities, (480, 640, 3), now=1.04)
        second, _ = tracker.update(
            0, second_curve, probabilities, (480, 640, 3), now=1.08)
        self.assertLess(float(np.mean(first[:, 0])), 338.0)
        self.assertGreater(float(np.mean(second[:, 0])), 345.0)

    def test_semantic_road_mask_removes_outside_points(self):
        road = np.zeros((120, 160), dtype=np.uint8)
        road[:, 40:100] = 1
        points = np.asarray([[200, 400], [500, 400]], dtype=np.float32)
        mask = _semantic_road_point_mask(points, road, (480, 640, 3))
        self.assertEqual(mask.tolist(), [True, False])


class VisionControlPlannerTest(unittest.TestCase):
    def test_conflicting_control_defaults_match_e7(self):
        config = VisionControlConfig()
        self.assertEqual(config.visual_center_x, 0.5625)
        self.assertEqual(config.lookahead_y_ratio, 0.625)
        self.assertEqual(config.straight_position_gain, 0.60)
        self.assertEqual(config.straight_heading_feedforward_gain, 0.60)
        self.assertEqual(config.straight_curve_feedforward_gain, 0.0)
        self.assertEqual(config.max_error_step_640, 18.0)
        self.assertEqual(config.hazard_bottom_ratio, 0.58)
        self.assertEqual(config.car_avoid_offset_px_640, 55.0)
        self.assertEqual(config.car_avoid_hold_s, 2.0)
        self.assertEqual(config.avoid_box_width_gain, 0.40)
        self.assertEqual(config.normal_speed_mps, 0.08)
        self.assertEqual(config.recover_speed_mps, 0.04)
        self.assertEqual(config.collect_speed_mps, 0.08)
        self.assertEqual(config.track_deadband_px_640, 0.0)
        self.assertEqual(config.track_small_error_gain, 0.15)
        self.assertEqual(config.curve_track_step_scale, 1.00)
        self.assertEqual(config.curve_state_enter_frames, 2)
        self.assertEqual(config.curve_state_exit_frames, 5)
        self.assertEqual(config.right_circle_capture_range_640, 24.0)
        self.assertEqual(config.right_circle_compensation_gain, 0.30)
        self.assertEqual(config.straight_error_limit_ratio, 0.50)
        self.assertEqual(config.curve_task_adjust_limit_ratio, 0.50)
        self.assertEqual(config.curve_task_adjust_soft_margin_ratio, 0.25)
        self.assertEqual(config.straight_min_span_ratio, 0.50)
        self.assertEqual(config.path_left_bias_px_640, 0.0)
        self.assertEqual(config.human_pass_speed_mps, 0.20)
        self.assertEqual(config.human_path_return_guard_s, 0.40)
        self.assertEqual(config.human_stop_absence_restart_s, 5.0)
        self.assertEqual(config.human_stop_reverse_speed_mps, -0.10)
        self.assertEqual(config.human_stop_reverse_duration_s, 0.30)
        self.assertEqual(config.human_speed_hold_s, 0.4)
        self.assertEqual(config.human_retrigger_lock_s, 1.5)
        self.assertAlmostEqual(
            config.human_restart_ignore_bottom_ratio, 1.0 / 3.0)
        self.assertEqual(config.human_same_person_distance_px_640, 38.0)
        self.assertEqual(config.human_stop_progress_ratio, 0.84)
        self.assertEqual(config.human_stop_line_offset_px_480, 35.0)
        self.assertEqual(config.car_human_pass_speed_mps, 0.20)
        self.assertEqual(config.ocr_right_center_latch_s, 5.0)

    def test_from_env_uses_eighteen_pixel_path_bias_by_default(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("VISION_CONTROL_PATH_LEFT_BIAS_PX_640", None)
            os.environ.pop("VISION_CONTROL_CENTER_X", None)
            config = VisionControlConfig.from_env()
        self.assertEqual(config.visual_center_x, 0.5625)
        self.assertEqual(config.path_left_bias_px_640, 18.0)

    def test_default_human_stop_line_is_shifted_up_thirty_five_pixels(self):
        planner = VisionControlPlanner(config=VisionControlConfig())
        image_shape = (480, 640, 3)
        lookahead_y = 300.0
        stop_y = planner._human_stop_line_y(image_shape, lookahead_y)

        self.assertAlmostEqual(stop_y, 293.8)
        self.assertTrue(planner._human_on_stop_line(
            {"bottom": 295.0}, image_shape, lookahead_y))
        self.assertFalse(planner._human_on_stop_line(
            {"bottom": 292.0}, image_shape, lookahead_y))

    def test_raw_curves_are_replaced_by_one_fitted_control_contract(self):
        planner = VisionControlPlanner(config=_config())
        result = _raw_result_at(220.0, 420.0)
        _command, debug = planner.update(result, now=1.0)
        self.assertEqual(len(result["paths"]), 2)
        self.assertTrue(all(
            item["source"] == "fitted_control_curve"
            for item in result["paths"]))
        self.assertNotIn("curve_paths", result)
        self.assertNotIn("fitted_control_paths", result)
        self.assertAlmostEqual(
            debug["control_target"]["path_target_x"], 220.0, delta=1.0)
        preview = _extract_curve_preview_lines(result)
        np.testing.assert_allclose(
            preview[0]["points_xy"], result["paths"][0]["points_xy"])

    def test_invalid_raw_curve_never_falls_back_to_legacy_paths(self):
        planner = VisionControlPlanner(config=_config())
        result = _raw_result_at()
        invalid = [
            {"slot": slot, "points_xy": np.asarray(
                [[200, 400], [210, 390]], dtype=np.float32)}
            for slot in (0, 1)
        ]
        result["raw_curve_paths"] = invalid
        result["centerline"]["raw_curve_paths"] = invalid
        result["curve_paths"] = [
            _raw_path(0, 100.0), _raw_path(1, 540.0)]
        command, _debug = planner.update(result, now=1.0)
        self.assertEqual(result["paths"], [])
        self.assertEqual(command["state_cmd"], STATE_RECOVER_LINE)
        self.assertGreater(command["target_speed"], 0.0)

    def test_line_loss_holds_angle_but_reduces_speed_until_reacquired(self):
        planner = VisionControlPlanner(config=_config())
        first_command, _debug = planner.update(
            _raw_result_at(260.0, 400.0), now=1.0)
        missing = _raw_result_at(slots=())
        held_command, held = planner.update(missing, now=1.6)
        long_held_command, long_held = planner.update(
            _raw_result_at(slots=()), now=10.0)
        self.assertEqual(held_command["state_cmd"], STATE_RECOVER_LINE)
        self.assertAlmostEqual(
            held_command["track_error"], first_command["track_error"])
        self.assertAlmostEqual(held_command["target_speed"], 0.04)
        self.assertTrue(held["control_target"]["line_loss_hold"])
        self.assertTrue(
            held["control_target"]["line_loss_speed_reduced"])
        self.assertAlmostEqual(
            held["control_target"]["line_loss_previous_speed_mps"],
            first_command["target_speed"])
        self.assertEqual(long_held_command["state_cmd"], STATE_RECOVER_LINE)
        self.assertAlmostEqual(
            long_held_command["track_error"], first_command["track_error"])
        self.assertAlmostEqual(long_held_command["target_speed"], 0.04)
        self.assertTrue(
            long_held["control_target"]["line_loss_indefinite"])

        reacquired_command, reacquired = planner.update(
            _raw_result_at(260.0, 400.0), now=10.1)
        self.assertEqual(reacquired["control_target"]["task_reason"], "track")
        self.assertEqual(reacquired_command["state_cmd"], STATE_TRACK)
        self.assertAlmostEqual(reacquired_command["target_speed"], 0.08)

    def test_car_detection_masks_line_loss_but_keeps_current_car_speed(self):
        planner = VisionControlPlanner(config=_config())
        planner.update(_raw_result_at(320.0, 430.0), now=1.0)
        car = [{
            "label": "Car", "score": 0.90,
            "bbox": [220.0, 260.0, 300.0, 420.0],
        }]

        command, active = planner.update(
            _raw_result_at(slots=(), detections=car), now=1.1)
        held_command, held = planner.update(
            _raw_result_at(slots=(), detections=[]), now=2.1)
        released_command, released = planner.update(
            _raw_result_at(slots=(), detections=[]), now=3.2)

        self.assertEqual(command["state_cmd"], STATE_RECOVER_LINE)
        self.assertEqual(held_command["state_cmd"], STATE_RECOVER_LINE)
        self.assertAlmostEqual(command["target_speed"], 0.04)
        self.assertTrue(active["line_loss_active"])
        self.assertFalse(active["line_loss_response_masked"])
        self.assertFalse(held["line_loss_response_masked"])
        self.assertEqual(released_command["state_cmd"], STATE_RECOVER_LINE)
        self.assertFalse(released["line_loss_response_masked"])

    def test_human_detection_masks_line_loss_response(self):
        planner = VisionControlPlanner(config=_config())
        planner.update(_raw_result_at(320.0, 430.0), now=1.0)
        human = [{
            "label": "Human", "score": 0.90,
            "bbox": [250.0, 180.0, 310.0, 260.0],
        }]

        command, debug = planner.update(
            _raw_result_at(slots=(), detections=human), now=1.1)

        self.assertEqual(command["state_cmd"], STATE_TRACK)
        self.assertAlmostEqual(command["target_speed"], 0.08)
        self.assertTrue(debug["line_loss_active"])
        self.assertTrue(debug["line_loss_response_masked"])
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "obstacle_line_loss_masked")

    def test_line_loss_does_not_release_latched_human_stop(self):
        planner = VisionControlPlanner(config=_config())
        human = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        planner.update(
            _raw_result_at(320.0, 430.0, detections=human), now=1.0)
        command, debug = planner.update(
            _raw_result_at(slots=(), detections=[]), now=1.4)

        self.assertNotEqual(command["state_cmd"], STATE_RECOVER_LINE)
        self.assertEqual(command["target_speed"], 0.0)
        self.assertTrue(debug["line_loss_response_masked"])
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "human_absence_check")

    def test_detached_near_path_is_treated_as_line_loss(self):
        planner = VisionControlPlanner(config=_config())
        planner.update(_raw_result_at(260.0, 400.0), now=1.0)
        previous_command, _pending = planner.update(
            _raw_result_at(20.0, 400.0), now=1.04)
        command, detached = planner.update(
            _raw_result_at(20.0, 400.0), now=1.08)
        self.assertTrue(detached["line_loss_active"])
        self.assertEqual(
            detached["line_connection_reason"],
            "detached_near_anchor")
        self.assertIsNone(detached["selected_slot"])
        self.assertEqual(detached["raw_selected_slot"], 0)
        self.assertEqual(command["state_cmd"], STATE_RECOVER_LINE)
        self.assertAlmostEqual(command["target_speed"], 0.04)
        self.assertAlmostEqual(
            command["track_error"], previous_command["track_error"])

    def test_fitted_control_large_jump_requires_confirmation(self):
        planner = VisionControlPlanner(config=_config())
        planner.update(_raw_result_at(220.0, 420.0), now=1.0)
        _command, pending = planner.update(
            _raw_result_at(500.0, 420.0), now=1.04)
        confirmed_result = _raw_result_at(500.0, 420.0)
        _command, confirmed = planner.update(confirmed_result, now=1.08)
        self.assertAlmostEqual(
            pending["control_target"]["path_target_x"], 220.0, delta=1.0)
        self.assertAlmostEqual(
            confirmed["control_target"]["path_target_x"], 500.0, delta=1.0)
        self.assertEqual(
            confirmed_result["temporal"]["mode"],
            "adaptive_curve_tracking")

    def test_track_error_damps_small_noise_and_fast_tracks_curve(self):
        planner = VisionControlPlanner(config=VisionControlConfig())
        planner.last_error = 0.0
        first = planner._limit_error(8.0, adaptive=True)
        planner.last_error = first
        second = planner._limit_error(-8.0, adaptive=True)
        self.assertLess(abs(first), 4.0)
        self.assertLess(abs(second), 4.0)

        planner.last_error = 0.0
        errors = []
        for _ in range(3):
            value = planner._limit_error(100.0, adaptive=True)
            planner.last_error = value
            errors.append(value)
        self.assertEqual(planner.track_error_response, "fast_curve")
        self.assertGreater(errors[1], errors[0])
        self.assertGreater(errors[2], errors[1])

    def test_road_shape_uses_curvature_not_straight_slope(self):
        planner = VisionControlPlanner(config=_config())
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        diagonal = _raw_path(0, 320.0 + 0.25 * (ys - 300.0), ys)
        diagonal_geometry = planner._path_geometry_metrics(
            diagonal, 300.0, (480, 640, 3))
        self.assertTrue(diagonal_geometry["valid"])
        self.assertAlmostEqual(
            diagonal_geometry["curvature_640"], 0.0, delta=0.1)

        curved = _raw_path(
            0, 320.0 + 0.0015 * (ys - 300.0) ** 2, ys)
        curve_geometry = planner._path_geometry_metrics(
            curved, 300.0, (480, 640, 3))
        self.assertTrue(curve_geometry["valid"])
        self.assertGreater(
            abs(curve_geometry["curvature_640"]), 8.0)
        self.assertEqual(
            tuple(curve_geometry["sample_rows_y"]),
            (300.0, 336.0, 372.0))

    def test_geometry_uses_control_curve_support_below_lookahead(self):
        planner = VisionControlPlanner(config=_config())
        ys = np.linspace(300.0, 380.0, 21, dtype=np.float32)
        path = _raw_path(
            0, 320.0 + 0.0015 * (ys - 300.0) ** 2, ys)
        geometry = planner._path_geometry_metrics(
            path, 300.0, (480, 640, 3))
        self.assertTrue(geometry["valid"])
        self.assertEqual(geometry["reason"], "valid")
        self.assertGreater(abs(geometry["curvature_640"]), 8.0)

    def test_green_merge_takeover_releases_when_control_rows_are_uncovered(self):
        planner = VisionControlPlanner(config=_config())
        planner.branch_lock = "left"
        planner.curve_merge_override = True
        planner.selected_slot_lock = 1
        blue = _raw_path(0, 320.0, np.linspace(300.0, 460.0, 20))
        # This is the failure shape seen during merge: green exists near the
        # lower part of the frame but no longer covers the control rows.
        green = _raw_path(1, 360.0, np.linspace(349.0, 395.0, 12))

        planner._update_curve_merge_continuity(
            [blue, green], (480, 640, 3))

        self.assertFalse(planner.curve_merge_override)
        self.assertEqual(planner.selected_slot_lock, 0)
        self.assertEqual(planner.curve_merge_reason, "green_support_lost")
        self.assertFalse(
            planner.curve_merge_metrics["green_covers_control_geometry"])
        self.assertIs(
            planner._select_candidate([blue, green]), blue)

    def test_adaptive_endpoint_is_not_a_fresh_control_target(self):
        planner = VisionControlPlanner(config=_config())
        selected = _raw_path(0, 360.0, np.linspace(320.0, 460.0, 12))
        command, target = planner._build_command(
            selected, "test", {"detections": []},
            (480, 640, 3), now=1.0)

        self.assertEqual(command["state_cmd"], STATE_RECOVER_LINE)
        self.assertAlmostEqual(command["target_speed"], 0.04)
        self.assertEqual(
            target["reason"], "adaptive_path_no_previous_target")
        self.assertTrue(target["path_target_adaptive_y"])
        self.assertIsNone(target["path_target_x"])

    def test_adaptive_target_uses_recent_held_target(self):
        planner = VisionControlPlanner(config=_config())
        planner.selected_slot_lock = 0
        planner.last_path_target_slot = 0
        planner.last_path_target_x = 250.0
        planner.last_path_target_y = 300.0
        planner.last_path_target_ts = 1.0
        selected = _raw_path(0, 360.0, np.linspace(320.0, 460.0, 12))
        command, target = planner._build_command(
            selected, "test", {"detections": []},
            (480, 640, 3), now=1.1)

        self.assertEqual(command["state_cmd"], STATE_TRACK)
        self.assertTrue(target["path_target_adaptive_y"])
        self.assertTrue(target["path_target_held"])
        self.assertAlmostEqual(target["path_target_x"], 250.0)

    def test_road_shape_state_stays_unified_without_latching(self):
        planner = VisionControlPlanner(config=_config())
        curve = {
            "valid": True,
            "heading_delta_640": 30.0,
            "curvature_640": 12.0,
        }
        invalid = {
            "valid": False,
            "heading_delta_640": 0.0,
            "curvature_640": 0.0,
        }
        planner._update_road_shape_state(curve)
        self.assertEqual(planner.road_shape_state, "unified")
        self.assertTrue(planner.road_shape_valid)
        self.assertAlmostEqual(planner.road_curvature_640, 12.0)
        planner._update_road_shape_state(invalid)
        self.assertEqual(planner.road_shape_state, "unified")
        self.assertFalse(planner.road_shape_valid)
        self.assertEqual(planner.road_curvature_640, 0.0)

    def test_invalid_geometry_does_not_latch_curve_response(self):
        planner = VisionControlPlanner(config=VisionControlConfig())
        planner._path_geometry_metrics = lambda *_args: {
            "valid": False,
            "reason": "insufficient_vertical_support",
            "heading_delta_640": 0.0,
            "curvature_640": 0.0,
            "sample_rows_y": (300.0, 336.0, 372.0),
            "support_y": (300.0, 360.0),
        }
        selected = _raw_path(0, 400.0)
        command, target = planner._build_command(
            selected, "test", {"detections": []},
            (480, 640, 3), now=1.0)
        self.assertAlmostEqual(command["track_error"], 6.3)
        self.assertEqual(target["track_error_response"], "straight_damping")
        self.assertFalse(target["road_shape_valid"])

    def test_repeated_fitted_bend_stays_in_unified_control_mode(self):
        planner = VisionControlPlanner(config=_config())
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        xs = 320.0 + 0.0015 * (ys - 300.0) ** 2

        def curved_result():
            result = _raw_result_at(320.0, 430.0)
            result["raw_curve_paths"][0] = _raw_path(0, xs, ys)
            result["centerline"]["raw_curve_paths"][0] = result[
                "raw_curve_paths"][0]
            result["road"]["mask"] = _road_mask_following_path(xs, ys)
            return result

        _command, first = planner.update(curved_result(), now=1.0)
        _command, second = planner.update(curved_result(), now=1.04)
        _command, third = planner.update(curved_result(), now=1.08)
        _command, fourth = planner.update(curved_result(), now=1.12)
        self.assertEqual(first["road_shape_state"], "unified")
        self.assertEqual(second["road_shape_state"], "unified")
        self.assertEqual(third["road_shape_state"], "unified")
        self.assertNotEqual(
            fourth["control_target"]["track_error_response"],
            "curve_track")
        self.assertGreater(
            abs(third["control_target"]["road_curvature_640"]), 8.0)

    def test_right_circle_feedforward_is_disabled(self):
        planner = VisionControlPlanner(config=_config(
            right_circle_feedforward_640=66.5,
            right_circle_trim_max_640=10.0,
            right_circle_capture_range_640=24.0,
            right_circle_compensation_gain=0.30,
        ))
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        xs = 320.0 + 0.0015 * (ys - 300.0) ** 2
        result = _raw_result_at(320.0, 430.0)
        result["raw_curve_paths"][0] = _raw_path(0, xs, ys)
        result["centerline"]["raw_curve_paths"][0] = result[
            "raw_curve_paths"][0]
        result["road"]["mask"] = _road_mask_following_path(xs, ys)
        _command, debug = planner.update(result, now=1.0)
        _command, debug = planner.update(result, now=1.04)
        target = debug["control_target"]
        self.assertEqual(target["right_circle_feedforward_640"], 0.0)
        self.assertFalse(target["right_circle_feedforward_active"])
        self.assertEqual(target["right_circle_compensation_640"], 0.0)
        self.assertAlmostEqual(
            target["raw_track_error_640"],
            target["line_based_track_error_640"])

    def test_right_circle_near_feedforward_is_still_disabled(self):
        planner = VisionControlPlanner(config=_config(
            right_circle_feedforward_640=66.5,
            right_circle_trim_max_640=10.0,
            right_circle_capture_range_640=24.0,
            right_circle_compensation_gain=0.30,
        ))
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        xs = 400.0 + 0.0015 * (ys - 300.0) ** 2
        result = _raw_result_at(400.0, 430.0)
        result["raw_curve_paths"][0] = _raw_path(0, xs, ys)
        result["centerline"]["raw_curve_paths"][0] = result[
            "raw_curve_paths"][0]
        result["road"]["mask"] = _road_mask_following_path(xs, ys)
        planner.update(result, now=1.0)
        command, debug = planner.update(result, now=1.04)
        target = debug["control_target"]
        self.assertFalse(target["right_circle_feedforward_active"])
        self.assertEqual(target["right_circle_trim_640"], 0.0)
        self.assertEqual(target["right_circle_compensation_640"], 0.0)
        self.assertAlmostEqual(
            command["track_error"], target["track_error_640"])

    def test_confirmed_curve_keeps_more_medium_error_than_straight(self):
        planner = VisionControlPlanner(config=VisionControlConfig())
        self.assertGreater(
            planner._shape_track_error(1.0, curve_mode=False), 0.0)
        straight = planner._shape_track_error(30.0, curve_mode=False)
        curve = planner._shape_track_error(30.0, curve_mode=True)
        self.assertAlmostEqual(straight, 16.45, delta=0.5)
        self.assertEqual(curve, 30.0)
        self.assertEqual(
            planner._shape_track_error(1.0, curve_mode=True), 1.0)
        self.assertAlmostEqual(
            planner._shape_track_error(100.0, curve_mode=False),
            planner._shape_track_error(100.0, curve_mode=True),
            delta=5.0)

    def test_confirmed_curve_step_is_limited_to_36_pixels(self):
        planner = VisionControlPlanner(config=VisionControlConfig())
        planner.last_error = 0.0
        error = planner._limit_error(
            100.0, adaptive=True, curve_mode=True)
        self.assertEqual(error, 18.0)
        self.assertEqual(planner.track_error_response, "curve_track")

    def test_unified_track_uses_curve_feedforward_immediately(self):
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        xs = 320.0 + 0.0015 * (ys - 300.0) ** 2
        result = _raw_result_at(320.0, 430.0)
        result["raw_curve_paths"][0] = _raw_path(0, xs, ys)
        result["centerline"]["raw_curve_paths"][0] = result[
            "raw_curve_paths"][0]
        result["road"]["mask"] = _road_mask_following_path(xs, ys)
        planner = VisionControlPlanner(config=_config())
        _command, debug = planner.update(result, now=1.0)
        _command, debug = planner.update(result, now=1.04)
        target = debug["control_target"]
        self.assertEqual(debug["road_shape_state"], "unified")
        self.assertTrue(target["straight_line_weights_active"])
        self.assertEqual(target["curve_feedforward_gain"], 1.15)
        self.assertGreater(target["curve_feedforward_640"], 0.0)

    def test_straight_nominal_position_and_heading_weights_are_applied(self):
        planner = VisionControlPlanner(config=_config())
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        xs = 340.0 + 0.10 * (ys - 300.0)
        selected = _raw_path(0, xs, ys)
        result = {
            "paths": [selected],
            "road": {"mask": _road_mask_following_path(xs, ys)},
            "detections": [],
        }
        path_geometry = planner._path_geometry_metrics(
            selected, 300.0, (480, 640, 3))
        _command, target = planner._build_command(
            selected, "test", result, (480, 640, 3), now=1.0)
        self.assertTrue(target["straight_line_weights_active"])
        self.assertEqual(target["path_position_gain"], 0.60)
        self.assertEqual(target["path_heading_feedforward_gain"], 0.30)
        self.assertEqual(target["curve_feedforward_gain"], 1.15)
        self.assertAlmostEqual(
            target["path_position_weighted_error_640"],
            target["path_raw_track_error_640"] * 0.60)
        self.assertAlmostEqual(
            target["path_heading_feedforward_640"],
            path_geometry["heading_delta_640"] * 0.30)
        self.assertEqual(target["curve_feedforward_640"], 0.0)
        self.assertAlmostEqual(
            target["line_based_track_error_640"],
            target["path_position_weighted_error_640"]
            + target["path_heading_feedforward_640"])

    def test_path_heading_prevents_left_target_from_commanding_late_left(self):
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        centered_y = ys - 300.0
        xs = 260.0 + 0.00520833 * centered_y ** 2 - (
            0.45833333 * centered_y)
        result = _raw_result_at(260.0, 430.0)
        result["raw_curve_paths"][0] = _raw_path(0, xs, ys)
        result["centerline"]["raw_curve_paths"][0] = result[
            "raw_curve_paths"][0]
        _command, debug = VisionControlPlanner(config=_config()).update(
            result, now=1.0)
        target = debug["control_target"]
        self.assertLess(target["path_raw_track_error_640"], -40.0)
        self.assertGreater(target["path_heading_feedforward_640"], 0.0)
        self.assertGreater(target["total_feedforward_640"], 0.0)
        self.assertGreater(
            target["raw_track_error_640"],
            target["path_raw_track_error_640"])

    def test_curve_defaults_to_blue_slot(self):
        command, debug = VisionControlPlanner(config=_config()).update(
            _raw_result_at(250.0, 330.0), now=1.0)
        self.assertEqual(debug["selected_slot"], 0)
        self.assertEqual(debug["branch_lock"], "left")
        self.assertLess(command["track_error"], 0.0)

    def test_generic_road_sign_does_not_enter_turnsign_fallback(self):
        planner = VisionControlPlanner(config=_config())
        self.assertTrue(planner._is_turnsign_detection({"label": "TurnSign"}))
        self.assertFalse(planner._is_turnsign_detection({"label": "RoadSign"}))
        self.assertFalse(planner._is_turnsign_detection({"label": "Sign"}))

    def test_stale_ocr_trim_keeps_selected_path_target(self):
        planner = VisionControlPlanner(config=_config())
        planner.turnsign_trim_current_fresh = False
        planner.turnsign_trim_position_delta_640 = -160.0
        target = planner._turnsign_trim_target_x(
            (480, 640, 3), -1, 320.0)
        self.assertAlmostEqual(target, 320.0)

    def test_ocr_parking_trim_is_disabled_but_route_tracking_remains(self):
        planner = VisionControlPlanner(config=_config())
        response = {
            "active": True,
            "session_active": True,
            "session_id": 41,
            "control_phase": "turnsign_approach",
            "bbox_center_x": 200.0,
            "current_detection_fresh": True,
            "instruction_current": True,
            "instruction": {"direction": "right"},
        }
        command, debug = planner.update(
            _trim_result(20.0), response, now=1.0)
        self.assertEqual(debug["branch_lock"], "right")
        self.assertEqual(debug["selected_slot"], 1)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "track")
        self.assertEqual(command["state_cmd"], STATE_TRACK)
        self.assertGreater(command["target_speed"], 0.0)
        self.assertAlmostEqual(
            debug["control_target"]["target_x"],
            debug["control_target"]["path_target_x"])

    def test_current_ocr_right_selects_green_slot(self):
        response = {
            "instruction_current": True,
            "instruction": {"direction": "right"},
        }
        _command, debug = VisionControlPlanner(config=_config()).update(
            _raw_result_at(), response, now=1.0)
        self.assertEqual(debug["branch_lock"], "right")
        self.assertEqual(debug["selected_slot"], 1)

    def test_current_ocr_right_recenters_reference_line_for_five_seconds(self):
        planner = VisionControlPlanner(config=_config(visual_center_x=0.5625))
        response = {
            "instruction_current": True,
            "instruction": {"direction": "right"},
        }

        _command, latched = planner.update(
            _raw_result_at(), response, now=1.0)
        _command, still_centered = planner.update(_raw_result_at(), now=5.9)
        _command, expired = planner.update(_raw_result_at(), now=6.1)

        self.assertEqual(latched["branch_lock"], "right")
        self.assertAlmostEqual(
            latched["control_target"]["visual_center_x"], 0.5)
        self.assertAlmostEqual(
            latched["control_target"]["reference_line_x"], 320.0)
        self.assertGreater(
            latched["control_target"][
                "ocr_right_center_latch_remaining_s"], 4.9)
        self.assertAlmostEqual(
            still_centered["control_target"]["visual_center_x"], 0.5)
        self.assertAlmostEqual(
            expired["control_target"]["visual_center_x"], 0.5625)
        self.assertAlmostEqual(
            expired["control_target"]["reference_line_x"], 360.0)

    def test_ocr_branch_lock_expires_to_blue_after_ten_seconds(self):
        planner = VisionControlPlanner(config=_config())
        response = {
            "instruction_current": True,
            "instruction": {"direction": "right"},
        }
        planner.update(_raw_result_at(), response, now=1.0)
        _command, debug = planner.update(_raw_result_at(), now=11.1)
        self.assertTrue(debug["ocr_lock_expired"])
        self.assertEqual(debug["branch_lock"], "left")
        self.assertEqual(debug["selected_slot"], 0)

    def test_e7_single_lookahead_and_error_slew_are_used(self):
        planner = VisionControlPlanner(config=VisionControlConfig())
        command, debug = planner.update(_raw_result_at(240.0, 400.0), now=1.0)
        self.assertAlmostEqual(debug["control_target"]["lookahead_y"], 300.0)
        self.assertEqual(command["track_error"], -18.0)
        self.assertNotIn("rear_path_target_x", debug["control_target"])
        self.assertNotIn("error_trend_mode", debug["control_target"])

    def test_only_long_end_to_end_straight_uses_hard_error_limit(self):
        planner = VisionControlPlanner(config=_config(
            right_circle_feedforward_640=66.5,
            straight_error_limit_ratio=0.50,
        ))
        result = _raw_result_at(500.0, 430.0)
        planner.update(result, now=1.0)
        command, debug = planner.update(result, now=1.04)
        target = debug["control_target"]
        self.assertFalse(target["high_confidence_straight"])
        self.assertFalse(target["straight_error_limited"])
        self.assertEqual(target["straight_error_limit_640"], 140.0)
        self.assertGreater(abs(target["raw_track_error_640"]), 33.25)
        self.assertEqual(
            target["road_curve_evidence"]["long_straight_reason"],
            "whole_segment_straight")

    def test_short_local_straight_does_not_use_hard_error_limit(self):
        planner = VisionControlPlanner(config=_config())
        ys = np.linspace(300.0, 380.0, 20, dtype=np.float32)
        selected = _raw_path(0, 500.0, ys)
        result = {
            "detections": [],
            "paths": [selected],
            "road": {"mask": np.ones((120, 160), dtype=np.uint8)},
        }
        planner._build_command(
            selected, "test", result, (480, 640, 3), now=1.0)
        _command, target = planner._build_command(
            selected, "test", result, (480, 640, 3), now=1.04)
        self.assertFalse(target["high_confidence_straight"])
        self.assertFalse(target["straight_error_limited"])
        self.assertGreater(abs(target["raw_track_error_640"]), 33.25)
        self.assertEqual(
            target["road_curve_evidence"]["long_straight_reason"],
            "path_too_short")

    def test_partly_curved_path_is_not_high_confidence_straight(self):
        planner = VisionControlPlanner(config=_config())
        ys = np.linspace(460.0, 60.0, 64, dtype=np.float32)
        far_bend = np.maximum(250.0 - ys, 0.0)
        xs = 500.0 + 0.003 * far_bend ** 2
        selected = _raw_path(0, xs, ys)
        result = {
            "detections": [],
            "paths": [selected],
            "road": {
                "mask": _road_mask_following_path(xs, ys),
            },
        }
        planner._build_command(
            selected, "test", result, (480, 640, 3), now=1.0)
        _command, target = planner._build_command(
            selected, "test", result, (480, 640, 3), now=1.04)
        self.assertFalse(target["high_confidence_straight"])
        self.assertFalse(target["straight_error_limited"])
        self.assertEqual(
            target["road_curve_evidence"]["long_straight_reason"],
            "path_not_straight_end_to_end")

    def test_curve_object_offsets_use_soft_limit_for_car_human_and_coin(self):
        for state, reason in (
            (STATE_AVOID_CAR, "car_in_path_bias"),
            (STATE_AVOID_HUMAN, "human_cross_pass"),
            (STATE_COLLECT_GOLD, "coin_bias"),
        ):
            with self.subTest(reason=reason):
                planner = VisionControlPlanner(config=_config())
                planner._road_shape_consensus_geometry = lambda *_args: {
                    "valid": True,
                    "reason": "curve_consensus",
                    "heading_delta_640": 0.0,
                    "curvature_640": 10.0,
                    "sample_rows_y": (300.0, 336.0, 372.0),
                    "support_y": (60.0, 460.0),
                }
                planner._task_from_detections = (
                    lambda *_args, _state=state, _reason=reason, **_kwargs:
                    (_state, 0.06, _reason, 420.0))
                selected = _raw_path(0, 320.0)
                command, target = planner._build_command(
                    selected, "test", {
                        "detections": [],
                        "paths": [selected],
                    }, (480, 640, 3), now=1.0)
                self.assertEqual(command["state_cmd"], state)
                self.assertAlmostEqual(
                    target["requested_task_adjustment_640"], 100.0)
                self.assertEqual(
                    target["applied_task_adjustment_640"], 100.0)
                self.assertFalse(target["curve_task_adjustment_limited"])
                self.assertFalse(target["straight_error_limited"])

    def test_straight_avoidance_task_bypasses_hard_limit(self):
        planner = VisionControlPlanner(config=_config())
        result = _raw_result_at(320.0, 430.0)
        planner.update(result, now=1.0)
        planner.update(result, now=1.04)
        planner._task_from_detections = (
            lambda *_args, **_kwargs:
            (STATE_AVOID_CAR, 0.06, "car_in_path_bias", 420.0))
        command, debug = planner.update(result, now=1.08)
        target = debug["control_target"]
        self.assertFalse(target["high_confidence_straight"])
        self.assertFalse(target["straight_limit_suppressed_by_task"])
        self.assertFalse(target["straight_hard_limit_enabled"])
        self.assertFalse(target["straight_error_limited"])
        self.assertGreater(target["raw_track_error_640"], 33.25)
        self.assertEqual(command["state_cmd"], STATE_AVOID_CAR)

    def test_car_in_path_uses_bounded_offset_and_normal_speed(self):
        car = [{
            "label": "Car", "score": 0.90,
            "bbox": [282.0, 260.0, 358.0, 420.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _raw_result_at(320.0, 430.0, detections=car), now=1.0)
        self.assertEqual(command["state_cmd"], STATE_TRACK)
        self.assertAlmostEqual(command["target_speed"], 0.08)
        self.assertEqual(
            debug["control_target"]["task_reason"], "track")
        self.assertAlmostEqual(
            abs(debug["control_target"]["task_offset_x"]), 0.0,
            delta=1.0)

    def test_wide_car_uses_restored_forty_percent_box_width(self):
        planner = VisionControlPlanner(config=_config())
        target_x = planner._avoid_target_x(
            "car", {"cx": 300.0, "box_w": 150.0},
            320.0, (480, 640, 3))
        self.assertAlmostEqual(target_x - 320.0, 42.0)

    def test_coin_collection_uses_collect_speed(self):
        coin = [{
            "label": "Coin", "score": 0.90,
            "bbox": [300.0, 300.0, 340.0, 440.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _raw_result_at(320.0, 430.0, detections=coin), now=1.0)
        self.assertEqual(command["state_cmd"], STATE_COLLECT_GOLD)
        self.assertAlmostEqual(command["target_speed"], 0.08)
        self.assertEqual(
            debug["control_target"]["task_reason"], "coin_bias")

    def test_coin_bias_starts_at_human_stop_height(self):
        planner = VisionControlPlanner(config=_config())
        coin_above_stop_line = [{
            "label": "Coin", "score": 0.90,
            "bbox": [380.0, 220.0, 420.0, 300.0],
        }]
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0,
                           detections=coin_above_stop_line), now=1.0)
        self.assertEqual(command["state_cmd"], STATE_TRACK)
        self.assertEqual(debug["control_target"]["task_reason"], "track")

        coin_at_stop_line = [{
            "label": "Coin", "score": 0.90,
            "bbox": [380.0, 260.0, 420.0, 330.0],
        }]
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0,
                           detections=coin_at_stop_line), now=1.1)
        self.assertEqual(command["state_cmd"], STATE_COLLECT_GOLD)
        self.assertEqual(debug["control_target"]["task_reason"], "coin_bias")

    def test_coin_path_bias_is_limited_to_forty_pixels(self):
        planner = VisionControlPlanner(config=_config())
        coin = [{
            "label": "Coin", "score": 0.90,
            "bbox": [450.0, 330.0, 490.0, 440.0],
        }]
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0, detections=coin), now=1.0)
        target = debug["control_target"]
        self.assertEqual(command["state_cmd"], STATE_COLLECT_GOLD)
        self.assertAlmostEqual(target["requested_task_adjustment_640"], 40.0)
        self.assertAlmostEqual(target["task_offset_x"], 40.0)

    def test_car_avoidance_offset_decays_during_detection_hold(self):
        planner = VisionControlPlanner(config=_config())
        car = [{
            "label": "Car", "score": 0.90,
            "bbox": [282.0, 260.0, 358.0, 420.0],
        }]
        _command, active = planner.update(
            _raw_result_at(320.0, 430.0, detections=car), now=1.0)
        _command, early_return = planner.update(
            _raw_result_at(320.0, 430.0), now=1.5)
        _command, held = planner.update(
            _raw_result_at(320.0, 430.0), now=2.0)
        _command, released = planner.update(
            _raw_result_at(320.0, 430.0), now=3.1)
        active_offset = abs(active["control_target"]["task_offset_x"])
        early_offset = abs(
            early_return["control_target"]["task_offset_x"])
        held_offset = abs(held["control_target"]["task_offset_x"])
        self.assertEqual(active["control_target"]["task_reason"], "track")
        self.assertEqual(
            early_return["control_target"]["task_reason"], "track")
        self.assertEqual(held["control_target"]["task_reason"], "track")
        self.assertEqual(released["control_target"]["task_reason"], "track")
        self.assertAlmostEqual(active_offset, 0.0)
        self.assertAlmostEqual(early_offset, 0.0)
        self.assertAlmostEqual(held_offset, 0.0)
        self.assertEqual(released["control_target"]["task_reason"], "track")

    def test_human_reaching_e7_stop_line_stops(self):
        human = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        planner = VisionControlPlanner(config=_config())
        command, debug = planner.update(
            _raw_result_at(detections=human), now=1.0)
        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertEqual(command["target_speed"], -0.10)
        self.assertEqual(command["flags"], 1)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "human_half_lookahead_stop")
        self.assertTrue(
            debug["control_target"]["human_stop_reverse_active"])

        command, debug = planner.update(
            _raw_result_at(detections=human), now=1.2)
        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertEqual(command["target_speed"], -0.10)

        command, debug = planner.update(
            _raw_result_at(detections=human), now=1.31)
        self.assertEqual(command["state_cmd"], STATE_SAFE_STOP)
        self.assertEqual(command["target_speed"], 0.0)
        self.assertEqual(command["flags"], 0)
        self.assertFalse(
            debug["control_target"]["human_stop_reverse_active"])

    def test_human_crossing_uses_e7_pass_speed(self):
        planner = VisionControlPlanner(config=_config())
        left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        right = [{
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 450.0],
        }]
        same_right = [{
            "label": "Human", "score": 0.95,
            "bbox": [364.0, 330.0, 424.0, 450.0],
        }]
        planner.update(
            _raw_result_at(320.0, 430.0, detections=left), now=1.0)
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0, detections=right), now=1.2)
        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertAlmostEqual(command["target_speed"], 0.08)
        self.assertEqual(
            debug["control_target"]["task_reason"], "human_cross_pass")
        self.assertLess(
            debug["control_target"]["base_target_x"],
            debug["control_target"]["path_target_x"])

    def test_human_crossing_from_right_to_left_keeps_waiting(self):
        planner = VisionControlPlanner(config=_config())
        right = [{
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 450.0],
        }]
        left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        planner.update(
            _raw_result_at(320.0, 430.0, detections=right), now=1.0)
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0, detections=left), now=1.2)
        target = debug["control_target"]
        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertEqual(command["target_speed"], -0.10)
        self.assertEqual(target["task_reason"], "human_half_lookahead_stop")

    def test_human_stop_latch_restarts_left_pass_after_five_second_dropout(self):
        planner = VisionControlPlanner(config=_config())
        human = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        planner.update(
            _raw_result_at(320.0, 430.0, detections=human), now=1.0)
        before_timeout, before_debug = planner.update(
            _raw_result_at(320.0, 430.0), now=5.99)
        restarted, restarted_debug = planner.update(
            _raw_result_at(320.0, 430.0), now=6.0)

        self.assertEqual(before_timeout["state_cmd"], STATE_SAFE_STOP)
        self.assertEqual(
            before_debug["control_target"]["task_reason"],
            "human_absence_check")
        self.assertEqual(restarted["state_cmd"], STATE_AVOID_HUMAN)
        self.assertAlmostEqual(restarted["target_speed"], 0.08)
        self.assertEqual(
            restarted_debug["control_target"]["task_reason"],
            "human_absence_restart")

    def test_committed_human_pass_has_priority_during_speed_hold(self):
        planner = VisionControlPlanner(config=_config())
        left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        right = [{
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 450.0],
        }]
        planner.update(
            _raw_result_at(320.0, 430.0, detections=left), now=1.0)
        planner.update(
            _raw_result_at(320.0, 430.0, detections=right), now=1.2)
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0, detections=left), now=1.3)

        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertAlmostEqual(command["target_speed"], 0.08)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "human_speed_hold")

    def test_human_retrigger_lock_ignores_same_person_after_speed_hold(self):
        planner = VisionControlPlanner(config=_config())
        selected = _raw_path(0, 320.0)
        image_shape = (480, 640, 3)
        lookahead_y = 300.0
        left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        right = [{
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 450.0],
        }]
        far_left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 170.0, 310.0, 260.0],
        }]
        planner._task_from_detections(
            {"detections": left}, selected, image_shape, lookahead_y,
            now=1.0, path_target_x=320.0)
        planner._task_from_detections(
            {"detections": right}, selected, image_shape, lookahead_y,
            now=1.2, path_target_x=320.0)
        action = planner._task_from_detections(
            {"detections": left}, selected, image_shape, lookahead_y,
            now=1.7, path_target_x=320.0)

        self.assertEqual(action[0], STATE_TRACK)
        self.assertAlmostEqual(action[1], 0.08)
        self.assertEqual(action[2], "human_path_return_guard")

        action = planner._task_from_detections(
            {"detections": left}, selected, image_shape, lookahead_y,
            now=2.2, path_target_x=320.0)
        self.assertEqual(action[0], STATE_TRACK)
        self.assertEqual(action[2], "track")

        action = planner._task_from_detections(
            {"detections": far_left}, selected, image_shape, lookahead_y,
            now=2.8, path_target_x=320.0)
        self.assertEqual(action[0], STATE_AVOID_HUMAN)
        self.assertAlmostEqual(action[1], 0.08)
        self.assertEqual(action[2], "human_cross_pass")

    def test_post_restart_ignores_human_in_lower_third(self):
        planner = VisionControlPlanner(config=_config())
        selected = _raw_path(0, 320.0)
        image_shape = (480, 640, 3)
        lookahead_y = 300.0
        left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        right = [{
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 450.0],
        }]
        same_right = [{
            "label": "Human", "score": 0.95,
            "bbox": [364.0, 330.0, 424.0, 450.0],
        }]
        planner._task_from_detections(
            {"detections": left}, selected, image_shape, lookahead_y,
            now=1.0, path_target_x=320.0)
        planner._task_from_detections(
            {"detections": right}, selected, image_shape, lookahead_y,
            now=1.2, path_target_x=320.0)

        action = planner._task_from_detections(
            {"detections": same_right}, selected, image_shape, lookahead_y,
            now=2.8, path_target_x=320.0)

        self.assertEqual(action[0], STATE_TRACK)
        self.assertAlmostEqual(action[1], 0.08)
        self.assertEqual(action[2], "human_path_return_guard")

    def test_post_restart_still_handles_lower_third_human_at_new_position(self):
        planner = VisionControlPlanner(config=_config())
        selected = _raw_path(0, 320.0)
        image_shape = (480, 640, 3)
        lookahead_y = 300.0
        left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        right = [{
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 450.0],
        }]
        new_position = [{
            "label": "Human", "score": 0.95,
            "bbox": [292.0, 330.0, 348.0, 450.0],
        }]
        planner._task_from_detections(
            {"detections": left}, selected, image_shape, lookahead_y,
            now=1.0, path_target_x=320.0)
        planner._task_from_detections(
            {"detections": right}, selected, image_shape, lookahead_y,
            now=1.2, path_target_x=320.0)

        action = planner._task_from_detections(
            {"detections": new_position}, selected, image_shape, lookahead_y,
            now=2.8, path_target_x=320.0)

        self.assertEqual(action[0], STATE_SAFE_STOP)
        self.assertAlmostEqual(action[1], 0.0)
        self.assertEqual(action[2], "human_half_lookahead_stop")

    def test_path_left_bias_helper_shifts_lookahead_point(self):
        planner = VisionControlPlanner(config=_config(
            path_left_bias_px_640=18.0))
        self.assertAlmostEqual(
            planner._apply_path_left_bias(320.0, (480, 640, 3)),
            302.0)
        self.assertAlmostEqual(
            planner._apply_path_left_bias(320.0, (480, 1280, 3)),
            284.0)

    def test_human_avoidance_and_final_steering_stop_at_road_mask_edge(self):
        planner = VisionControlPlanner(config=_config())
        planner._road_shape_consensus_geometry = lambda *_args: {
            "valid": True,
            "reason": "curve_consensus",
            "heading_delta_640": 0.0,
            "curvature_640": -10.0,
            "sample_rows_y": (300.0, 336.0, 372.0),
            "support_y": (60.0, 460.0),
        }
        planner._task_from_detections = lambda *_args, **_kwargs: (
            STATE_AVOID_HUMAN, 0.08, "human_cross_pass", 282.0)
        selected = _raw_path(0, 320.0)
        mask = np.zeros((120, 160), dtype=np.uint8)
        mask[:, 75:91] = 1

        command, target = planner._build_command(
            selected, "test", {
                "detections": [],
                "road": {"mask": mask},
            }, (480, 640, 3), now=1.0)

        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertTrue(target["human_mask_constraint_available"])
        self.assertTrue(target["human_mask_constraint_applied"])

    def test_car_context_does_not_pass_human_on_right(self):
        planner = VisionControlPlanner(config=_config())
        car_on_left = {
            "label": "Car", "score": 0.95,
            "bbox": [240.0, 260.0, 300.0, 420.0],
        }
        human_on_right = {
            "label": "Human", "score": 0.95,
            "bbox": [370.0, 330.0, 430.0, 450.0],
        }
        human_on_left = {
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }
        planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_left, human_on_right]), now=1.0)
        command, debug = planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_left, human_on_left]), now=1.4)
        target = debug["control_target"]
        self.assertEqual(planner.car_avoid_side, 0)
        self.assertEqual(command["state_cmd"], STATE_SAFE_STOP)
        self.assertEqual(
            target["task_reason"], "human_half_lookahead_stop")
        self.assertLessEqual(target["task_offset_x"], 0.0)

    def test_right_pass_block_unlocks_after_five_second_human_absence(self):
        planner = VisionControlPlanner(config=_config())
        car_on_left = {
            "label": "Car", "score": 0.95,
            "bbox": [240.0, 260.0, 300.0, 420.0],
        }
        human_on_right = {
            "label": "Human", "score": 0.95,
            "bbox": [370.0, 330.0, 430.0, 450.0],
        }
        human_on_left = {
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }
        planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_left, human_on_right]), now=1.0)
        planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_left, human_on_left]), now=1.4)

        before_unlock, before_debug = planner.update(
            _raw_result_at(320.0, 430.0), now=6.39)
        unlocked, unlocked_debug = planner.update(
            _raw_result_at(320.0, 430.0), now=6.4)

        self.assertEqual(before_unlock["state_cmd"], STATE_SAFE_STOP)
        self.assertEqual(
            before_debug["control_target"]["task_reason"],
            "human_absence_check")
        self.assertEqual(unlocked["state_cmd"], STATE_AVOID_HUMAN)
        self.assertEqual(
            unlocked_debug["control_target"]["task_reason"],
            "human_absence_restart")
        self.assertFalse(planner.car_human_active)
        self.assertEqual(planner.car_avoid_side, 0)

    def test_right_pass_absence_unlock_resumes_car_or_line_recovery(self):
        def blocked_planner():
            planner = VisionControlPlanner(config=_config())
            car = {
                "label": "Car", "score": 0.95,
                "bbox": [240.0, 260.0, 300.0, 420.0],
            }
            right = {
                "label": "Human", "score": 0.95,
                "bbox": [370.0, 330.0, 430.0, 450.0],
            }
            left = {
                "label": "Human", "score": 0.95,
                "bbox": [250.0, 330.0, 310.0, 450.0],
            }
            planner.update(_raw_result_at(
                320.0, 430.0, detections=[car, right]), now=1.0)
            planner.update(_raw_result_at(
                320.0, 430.0, detections=[car, left]), now=1.4)
            return planner, car

        with_car, car = blocked_planner()
        car_command, car_debug = with_car.update(
            _raw_result_at(320.0, 430.0, detections=[car]), now=6.4)
        self.assertEqual(car_command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertAlmostEqual(car_command["target_speed"], 0.08)
        self.assertEqual(
            car_debug["control_target"]["task_reason"],
            "human_absence_restart")

        without_path, _car = blocked_planner()
        line_command, line_debug = without_path.update(
            _raw_result_at(slots=(), detections=[]), now=6.4)
        self.assertEqual(line_command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertTrue(line_debug["line_loss_response_masked"])
        self.assertEqual(
            line_debug["control_target"]["task_reason"],
            "human_absence_restart")

    def test_left_car_human_wait_restarts_after_five_second_absence(self):
        planner = VisionControlPlanner(config=_config())
        car_on_right = {
            "label": "Car", "score": 0.95,
            "bbox": [340.0, 260.0, 420.0, 420.0],
        }
        human_on_left_route = {
            "label": "Human", "score": 0.95,
            "bbox": [190.0, 330.0, 250.0, 450.0],
        }
        planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_right, human_on_left_route]), now=1.0)
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0), now=6.0)

        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertAlmostEqual(command["target_speed"], 0.08)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "human_absence_restart")

    def test_left_car_human_pass_stays_latched_while_same_person_visible(self):
        planner = VisionControlPlanner(config=_config())
        car_on_right = {
            "label": "Car", "score": 0.95,
            "bbox": [340.0, 260.0, 420.0, 420.0],
        }
        human_on_left_route = {
            "label": "Human", "score": 0.95,
            "bbox": [190.0, 330.0, 250.0, 450.0],
        }
        same_human_after_crossing = {
            "label": "Human", "score": 0.95,
            "bbox": [350.0, 330.0, 410.0, 450.0],
        }

        planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_right, human_on_left_route]), now=1.0)
        pass_command, _ = planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_right, same_human_after_crossing]), now=1.1)
        held_command, held = planner.update(_raw_result_at(
            320.0, 430.0,
            detections=[car_on_right, same_human_after_crossing]), now=3.2)

        self.assertEqual(pass_command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertEqual(held_command["state_cmd"], STATE_TRACK)
        self.assertAlmostEqual(held_command["target_speed"], 0.08)
        self.assertEqual(
            held["control_target"]["task_reason"],
            "human_path_return_guard")

    def test_human_pass_returns_to_path_without_target_jump(self):
        planner = VisionControlPlanner(config=_config())
        left = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        right = [{
            "label": "Human", "score": 0.95,
            "bbox": [360.0, 330.0, 420.0, 450.0],
        }]
        planner.update(
            _raw_result_at(320.0, 430.0, detections=left), now=1.0)
        planner.update(
            _raw_result_at(320.0, 430.0, detections=right), now=1.2)

        command, debug = planner.update(
            _raw_result_at(500.0, 430.0), now=1.8)
        target = debug["control_target"]
        self.assertEqual(target["task_reason"], "human_path_return_guard")
        self.assertAlmostEqual(target["base_target_x"], 282.0, delta=1.0)
        self.assertLess(abs(command["track_error"]), 50.0)

        command, debug = planner.update(
            _raw_result_at(500.0, 430.0), now=2.0)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "human_path_return_guard")
        self.assertGreater(debug["control_target"]["base_target_x"], 282.0)
        self.assertLess(debug["control_target"]["base_target_x"], 450.0)

        command, debug = planner.update(
            _raw_result_at(500.0, 430.0), now=2.3)
        self.assertEqual(debug["control_target"]["task_reason"], "track")
        self.assertAlmostEqual(
            debug["control_target"]["base_target_x"], 500.0, delta=1.0)

    def test_new_turnsign_session_starts_adaptive_forward_trim(self):
        planner = VisionControlPlanner(config=_config(
            turnsign_trim_enabled=True))
        response = {
            "active": True,
            "session_active": True,
            "session_id": 7,
            "control_phase": "turnsign_approach",
            "bbox_center_x": 200.0,
            "current_detection_fresh": True,
        }
        command, debug = planner.update(
            _trim_result(20.0), response, now=1.0)
        self.assertEqual(command["state_cmd"], STATE_TRACK)
        self.assertAlmostEqual(command["target_speed"], 0.08)
        self.assertLess(command["track_error"], 0.0)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "turnsign_trim_forward")

    def test_turnsign_split_then_collapse_requests_reverse_trim(self):
        planner = VisionControlPlanner(config=_config(
            turnsign_trim_enabled=True,
            turnsign_trim_split_frames=1,
            turnsign_trim_collapse_frames=1,
            turnsign_reverse_duration_s=0.1,
            turnsign_trim_settle_s=0.0,
        ))
        response = {
            "active": True,
            "session_active": True,
            "session_id": 8,
            "control_phase": "turnsign_approach",
            "bbox_center_x": 420.0,
            "current_detection_fresh": True,
        }
        planner.update(_trim_result(120.0), response, now=1.0)
        planner.update(_trim_result(120.0), response, now=1.2)
        planner.update(_trim_result(10.0), response, now=1.4)
        command, debug = planner.update(
            _trim_result(10.0), response, now=1.6)
        self.assertLess(command["target_speed"], 0.0)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "turnsign_trim_reverse")

    def test_ocr_route_is_deferred_until_two_curves_are_clear(self):
        planner = VisionControlPlanner(config=_config(
            turnsign_trim_enabled=True,
            turnsign_trim_split_px_640=36.0,
            turnsign_trim_split_frames=2,
            turnsign_reverse_duration_s=0.05,
            turnsign_trim_settle_s=0.0,
        ))
        active = {
            "active": True,
            "session_active": True,
            "session_id": 34,
            "control_phase": "turnsign_ocr_wait",
            "bbox_center_x": 400.0,
            "current_detection_fresh": True,
        }
        api = dict(
            active,
            instruction_current=True,
            instruction={"direction": "right"},
            turnsign_resolved=True,
        )
        held = dict(
            active, instruction_current=False, turnsign_resolved=True)
        planner.update(_raw_result_at(slots=(0,)), active, now=1.0)
        _command, pending = planner.update(
            _raw_result_at(slots=(0,)), api, now=1.1)
        planner.update(_trim_result(100.0), held, now=1.2)
        _command, ready = planner.update(
            _trim_result(100.0), held, now=1.3)
        self.assertEqual(pending["branch_lock"], "left")
        self.assertEqual(
            pending["turnsign_trim_pending_ocr_direction"], "right")
        self.assertEqual(ready["branch_lock"], "right")
        self.assertIsNone(ready["turnsign_trim_pending_ocr_direction"])

    def test_preview_draws_final_curve_and_target(self):
        planner = VisionControlPlanner(config=_config())
        result = _raw_result_at()
        planner.update(result, now=1.0)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        rendered = render_vision_control_debug(frame, result)
        self.assertGreater(int(rendered.sum()), 0)
        self.assertEqual(_identity_probability_color(0, 1.0), (255, 0, 0))
        self.assertEqual(_identity_probability_color(1, 1.0), (0, 255, 0))

    def test_preview_draws_default_reference_line_forty_px_right(self):
        planner = VisionControlPlanner(config=VisionControlConfig())
        result = _raw_result_at()
        planner.update(result, now=1.0)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        rendered = render_vision_control_debug(frame, result)

        self.assertGreater(int(rendered[470, 360].sum()), 0)

    def test_birdseye_preview_projects_actual_path_and_target(self):
        planner = VisionControlPlanner(config=_config())
        result = _raw_result_at()
        planner.update(result, now=1.0)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        rendered = render_vision_control_birdseye(frame, result)

        self.assertEqual(rendered.shape, frame.shape)
        self.assertGreater(int(rendered.sum()), 0)
        magenta = (
            (rendered[:, :, 0] > 200)
            & (rendered[:, :, 1] < 80)
            & (rendered[:, :, 2] > 200))
        self.assertGreater(int(np.count_nonzero(magenta)), 0)

    def test_birdseye_preview_overlays_existing_semantic_road_mask(self):
        result = _raw_result_at()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        with mock.patch.dict(os.environ, {
                "AR_BIRDSEYE_ROAD_OVERLAY": "1",
        }):
            rendered = render_vision_control_birdseye(frame, result)

        road_tint = (
            (rendered[:, :, 2] > 120)
            & (rendered[:, :, 2] > rendered[:, :, 1] * 1.5)
            & (rendered[:, :, 2] > rendered[:, :, 0] * 1.5))
        self.assertGreater(int(np.count_nonzero(road_tint)), 200)
        self.assertEqual(int(frame.sum()), 0)

    def test_birdseye_roi_follows_selected_semantic_road_component(self):
        road = np.zeros((120, 160), dtype=np.uint8)
        for row in range(60, 116):
            blend = float(row - 60) / 55.0
            left = int(round(70.0 + blend * (35.0 - 70.0)))
            right = int(round(90.0 + blend * (125.0 - 90.0)))
            road[row, left:right + 1] = 1
        selected_path = [[320.0, 455.0], [320.0, 249.0]]
        result = {
            "road": {"mask": road},
            "road_mask": road,
            "vision_control": {
                "selected_path": selected_path,
                "control_target": {
                    "path_target_x": 320.0,
                    "path_target_y": 300.0,
                },
            },
        }

        with mock.patch.dict(os.environ, {
                "AR_BIRDSEYE_FOLLOW_ROAD_MASK": "1",
        }):
            source_quad = _birdseye_debug_source_quad_from_road(
                result, (480, 640, 3))

        self.assertIsNotNone(source_quad)
        self.assertAlmostEqual(float(source_quad[0, 0]), 0.428, delta=0.03)
        self.assertAlmostEqual(float(source_quad[1, 0]), 0.578, delta=0.03)
        self.assertAlmostEqual(float(source_quad[3, 0]), 0.214, delta=0.04)
        self.assertAlmostEqual(float(source_quad[2, 0]), 0.792, delta=0.04)

        with mock.patch.dict(os.environ, {
                "AR_BIRDSEYE_FOLLOW_ROAD_MASK": "1",
        }):
            renderer = VisionControlBirdseyeRenderer()
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            rendered = renderer(frame, result)
            renderer.draw_source_roi(frame)
        self.assertEqual(renderer.source_status, "ROAD MASK")
        self.assertGreater(int(rendered.sum()), 0)
        self.assertGreater(int(frame.sum()), 0)


if __name__ == "__main__":
    unittest.main()
