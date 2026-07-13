import os
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.dirname(__file__))

from vision_control import (  # noqa: E402
    ROUTE_MULTI_FORK,
    STATE_AVOID_CAR,
    STATE_TRACK,
    VisionControlConfig,
    VisionControlPlanner,
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
    }
    values.update(overrides)
    return VisionControlConfig(**values)


def _draw_heat_path(heatmap, x_at_y):
    h, w = heatmap.shape
    xs = np.arange(w, dtype=np.float32)
    for y in range(20, h - 1):
        x = float(x_at_y(y))
        row = np.exp(-0.5 * ((xs - x) / 1.6) ** 2)
        heatmap[y] = np.maximum(heatmap[y], row)


def _fork_heatmaps():
    h, w = 120, 160
    heatmaps = np.zeros((2, h, w), dtype=np.float32)

    def split_t(y):
        return np.clip((118.0 - float(y)) / 88.0, 0.0, 1.0)

    _draw_heat_path(heatmaps[0], lambda y: 80.0 - 28.0 * split_t(y))
    _draw_heat_path(heatmaps[1], lambda y: 80.0 + 28.0 * split_t(y))
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

    def test_current_ocr_locks_right_branch(self):
        planner = VisionControlPlanner(config=_config())
        _command, debug = planner.update(
            _result(_fork_heatmaps()),
            {"instruction_current": True, "instruction": {"direction": "right"}},
            now=2.0,
        )

        self.assertEqual("right", debug["branch_lock"])
        self.assertEqual(1, debug["selected_slot"])

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


if __name__ == "__main__":
    unittest.main()
