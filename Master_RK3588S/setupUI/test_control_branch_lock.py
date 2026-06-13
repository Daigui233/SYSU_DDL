import unittest

from control_local_planner import LocalPlanner
from control_states import TaskState


def _candidate(side, branch, x):
    return {
        "side": side,
        "branch": branch,
        "points": [(x, 450), (x, 380), (x, 300), (x, 230)],
    }


def _perception(with_candidates=True):
    segmentation = {
        "line_valid": True,
        "track_error": 0.0,
        "center_x": 320,
        "mid_points": [(320, 450), (320, 380), (320, 300), (320, 230)],
        "fork_state": "FORK_CONFIRMED" if with_candidates else "NORMAL",
        "fork_split_rows": 6,
        "fork_candidates": (
            [_candidate("left", "outer", 245), _candidate("right", "inner", 395)]
            if with_candidates
            else []
        ),
    }
    return {"timestamp": 1.0, "frame_shape": [480, 640], "segmentation": segmentation}


class BranchLockTest(unittest.TestCase):
    def test_early_fork_candidates_cannot_control_steering(self):
        planner = LocalPlanner()
        perception = _perception()
        perception["segmentation"]["fork_state"] = "FORK_EARLY"
        result = planner.plan(
            perception,
            {
                "task_state": TaskState.AVOID_STONE.value,
                "planner_intent": {"target": {"center": [245, 300], "size": [60, 60]}},
            },
            now=1.0,
        )
        self.assertEqual(result["branch_candidates"], [])
        self.assertIsNone(result["selected_branch"])

    def test_selected_branch_stays_locked_after_stone_state_clears(self):
        planner = LocalPlanner()
        stone_decision = {
            "task_state": TaskState.AVOID_STONE.value,
            "planner_intent": {
                "target": {"center": [245, 300], "size": [60, 60]},
            },
        }
        selected = planner.plan(_perception(), stone_decision, now=1.0)
        self.assertEqual(selected["selected_branch"], "inner")

        normal_decision = {"task_state": TaskState.NORMAL_TRACK.value, "planner_intent": {}}
        locked = planner.plan(_perception(), normal_decision, now=1.1)
        self.assertEqual(locked["selected_branch"], "inner")
        self.assertEqual(locked["planner_reason"], "branch_locked_inner")
        self.assertGreater(locked["final_track_error"], 0)

    def test_merge_mask_cannot_manufacture_branch_candidates(self):
        planner = LocalPlanner()
        perception = _perception(with_candidates=False)
        perception["segmentation"]["road_mask"] = [[1] * 640 for _ in range(480)]
        result = planner.plan(
            perception,
            {"task_state": TaskState.AVOID_STONE.value, "planner_intent": {"target": {}}},
            now=1.0,
        )
        self.assertEqual(result["branch_candidates"], [])


if __name__ == "__main__":
    unittest.main()
