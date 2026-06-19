import unittest

import numpy as np

from control_local_planner import LocalPlanner
from control_states import TaskState


def _human_decision(target_x=280):
    return {
        "task_state": TaskState.AVOID_HUMAN.value,
        "planner_intent": {
            "mode": "AVOID_OBSTACLE",
            "category": "human",
            "target": {
                "center": [target_x, 350],
                "bbox": [target_x - 30, 260, target_x + 30, 390],
                "size": [60, 130],
                "score": 0.9,
            },
            "dynamic": True,
        },
    }


def _perception(track_error):
    return {
        "timestamp": 1.0,
        "frame_shape": [480, 640],
        "segmentation": {
            "line_valid": True,
            "track_error": float(track_error),
            "center_x": 320,
            "road_mask": np.ones((480, 640), dtype=np.uint8),
            "mid_points": [(320, 450), (320, 380), (320, 300), (320, 230)],
        },
    }


class HumanYieldPlannerTest(unittest.TestCase):
    def test_yields_when_body_is_already_offset_toward_passing_side(self):
        planner = LocalPlanner()
        result = planner.plan(_perception(track_error=120), _human_decision(), now=1.0)

        self.assertTrue(result["yield_wait"])
        self.assertEqual(result["speed_override"], 0.0)
        self.assertEqual(result["final_track_error"], 0.0)
        self.assertIn("body_offset", result["planner_reason"])

    def test_passes_when_body_offset_is_small_and_corridor_is_clear(self):
        planner = LocalPlanner()
        result = planner.plan(_perception(track_error=0), _human_decision(), now=1.0)

        self.assertFalse(result["yield_wait"])
        self.assertIsNone(result["speed_override"])
        self.assertEqual(result["avoid_side"], "right")
        self.assertGreater(result["final_track_error"], 0.0)


if __name__ == "__main__":
    unittest.main()
