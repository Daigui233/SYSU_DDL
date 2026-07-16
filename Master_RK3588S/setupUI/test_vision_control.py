import os
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.dirname(__file__))

from vision_control import (  # noqa: E402
    STATE_AVOID_CAR,
    STATE_AVOID_HUMAN,
    STATE_LINE_LOSS_SAFE_STOP,
    STATE_RECOVER_LINE,
    STATE_SAFE_STOP,
    STATE_TRACK,
    VisionControlConfig,
    VisionControlPlanner,
    _FittedControlPathTracker,
    _associated_point_mask,
    _densify_associated_lower_points,
    _extract_curve_preview_lines,
    _fit_smooth_majority_curve,
    _identity_probability_color,
    _semantic_road_point_mask,
    render_vision_control_debug,
)


def _config(**overrides):
    values = {
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
        self.assertEqual(config.lookahead_y_ratio, 0.625)
        self.assertEqual(config.max_error_step_640, 24.0)
        self.assertEqual(config.hazard_bottom_ratio, 0.58)
        self.assertEqual(config.car_avoid_offset_px_640, 55.0)
        self.assertEqual(config.car_avoid_hold_s, 2.0)
        self.assertEqual(config.normal_speed_mps, 0.10)
        self.assertEqual(config.recover_speed_mps, 0.08)
        self.assertEqual(config.collect_speed_mps, 0.08)
        self.assertEqual(config.human_pass_speed_mps, 0.30)
        self.assertEqual(config.human_speed_hold_s, 0.5)

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
        self.assertEqual(command["state_cmd"], STATE_LINE_LOSS_SAFE_STOP)

    def test_short_line_loss_holds_last_steering_until_safe_stop(self):
        planner = VisionControlPlanner(config=_config())
        first_command, _debug = planner.update(
            _raw_result_at(260.0, 400.0), now=1.0)
        missing = _raw_result_at(slots=())
        held_command, held = planner.update(missing, now=1.6)
        stopped_command, _stopped = planner.update(
            _raw_result_at(slots=()), now=1.81)
        self.assertEqual(held_command["state_cmd"], STATE_RECOVER_LINE)
        self.assertAlmostEqual(
            held_command["track_error"], first_command["track_error"])
        self.assertTrue(held["control_target"]["line_loss_hold"])
        self.assertEqual(
            stopped_command["state_cmd"], STATE_LINE_LOSS_SAFE_STOP)
        self.assertEqual(stopped_command["target_speed"], 0.0)

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
        self.assertGreater(errors[1], 48.0)
        self.assertGreater(errors[2], errors[1])

    def test_light_curve_feedforward_anticipates_bend(self):
        ys = np.linspace(460.0, 60.0, 32, dtype=np.float32)
        xs = 320.0 + 0.0015 * (ys - 300.0) ** 2
        result = _raw_result_at(320.0, 430.0)
        result["raw_curve_paths"][0] = _raw_path(0, xs, ys)
        result["centerline"]["raw_curve_paths"][0] = result[
            "raw_curve_paths"][0]
        _command, debug = VisionControlPlanner(config=_config()).update(
            result, now=1.0)
        target = debug["control_target"]
        self.assertGreater(target["curve_feedforward_640"], 0.0)
        self.assertLessEqual(abs(target["curve_feedforward_640"]), 24.0)

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
        self.assertGreater(target["total_feedforward_640"], 30.0)
        self.assertGreater(
            target["raw_track_error_640"],
            target["path_raw_track_error_640"] + 30.0)

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
        self.assertAlmostEqual(command["track_error"], -24.0)
        self.assertNotIn("rear_path_target_x", debug["control_target"])
        self.assertNotIn("error_trend_mode", debug["control_target"])

    def test_car_in_path_uses_bounded_offset_and_normal_speed(self):
        car = [{
            "label": "Car", "score": 0.90,
            "bbox": [282.0, 260.0, 358.0, 420.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _raw_result_at(320.0, 430.0, detections=car), now=1.0)
        self.assertEqual(command["state_cmd"], STATE_AVOID_CAR)
        self.assertAlmostEqual(command["target_speed"], 0.10)
        self.assertEqual(
            debug["control_target"]["task_reason"], "car_in_path_bias")
        self.assertAlmostEqual(
            abs(debug["control_target"]["task_offset_x"]), 38.5,
            delta=1.0)

    def test_car_avoidance_offset_decays_during_detection_hold(self):
        planner = VisionControlPlanner(config=_config())
        car = [{
            "label": "Car", "score": 0.90,
            "bbox": [282.0, 260.0, 358.0, 420.0],
        }]
        _command, active = planner.update(
            _raw_result_at(320.0, 430.0, detections=car), now=1.0)
        _command, held = planner.update(
            _raw_result_at(320.0, 430.0), now=2.0)
        _command, released = planner.update(
            _raw_result_at(320.0, 430.0), now=3.1)
        active_offset = abs(active["control_target"]["task_offset_x"])
        held_offset = abs(held["control_target"]["task_offset_x"])
        self.assertEqual(
            held["control_target"]["task_reason"], "car_avoid_hold")
        self.assertGreater(active_offset, held_offset)
        self.assertAlmostEqual(held_offset, active_offset * 0.5, delta=1.0)
        self.assertEqual(released["control_target"]["task_reason"], "track")

    def test_human_reaching_e7_stop_line_stops(self):
        human = [{
            "label": "Human", "score": 0.95,
            "bbox": [250.0, 330.0, 310.0, 450.0],
        }]
        command, debug = VisionControlPlanner(config=_config()).update(
            _raw_result_at(detections=human), now=1.0)
        self.assertEqual(command["state_cmd"], STATE_SAFE_STOP)
        self.assertEqual(command["target_speed"], 0.0)
        self.assertEqual(
            debug["control_target"]["task_reason"],
            "human_half_lookahead_stop")

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
        planner.update(
            _raw_result_at(320.0, 430.0, detections=left), now=1.0)
        command, debug = planner.update(
            _raw_result_at(320.0, 430.0, detections=right), now=1.2)
        self.assertEqual(command["state_cmd"], STATE_AVOID_HUMAN)
        self.assertAlmostEqual(command["target_speed"], 0.30)
        self.assertEqual(
            debug["control_target"]["task_reason"], "human_cross_pass")

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


if __name__ == "__main__":
    unittest.main()
