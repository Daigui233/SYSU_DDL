import importlib.util
from pathlib import Path
import unittest

import numpy as np

from control_arbitrator import ControlArbitrator
from control_local_planner import LocalPlanner
from control_states import TaskState

SEG_FUNC_PATH = Path(__file__).resolve().parent / "infer_wrap" / "base" / "seg_func.py"
SEG_FUNC_SPEC = importlib.util.spec_from_file_location("seg_func_under_test", SEG_FUNC_PATH)
seg_func = importlib.util.module_from_spec(SEG_FUNC_SPEC)
SEG_FUNC_SPEC.loader.exec_module(seg_func)


class LightweightControlChainTest(unittest.TestCase):
    def setUp(self):
        self.mask = np.ones((480, 640), dtype=np.uint8)

    def _normal_track_plan(self, x, now):
        perception = {
            "timestamp": now,
            "frame_shape": [480, 640],
            "segmentation": {
                "line_valid": True,
                "track_error": 0.0,
                "center_x": 320,
                "road_mask": self.mask,
                "mid_points": [(320, 450), (320, 360), (x, 300), (320, 278), (320, 200)],
            },
        }
        decision = {
            "task_state": TaskState.NORMAL_TRACK.value,
            "planner_intent": {},
        }
        return perception, decision

    def test_planner_final_error_uses_current_lookahead(self):
        planner = LocalPlanner()
        perception, decision = self._normal_track_plan(480, 1.0)
        first = planner.plan(perception, decision, now=1.0)
        perception, decision = self._normal_track_plan(160, 1.1)
        second = planner.plan(perception, decision, now=1.1)

        self.assertEqual(160.0, first["final_track_error"])
        self.assertEqual(-160.0, second["final_track_error"])
        self.assertEqual(300.0, first["control_lookahead_y"])

    def test_arbitrator_passes_error_without_step_chasing(self):
        arbitrator = ControlArbitrator()
        outputs = [
            arbitrator.decide(i, value, desired_speed=0.05, state_text="NORMAL_TRACK")["track_error"]
            for i, value in enumerate([160.0, -160.0, 160.0])
        ]
        self.assertEqual([160.0, -160.0, 160.0], outputs)

    def test_invalid_input_safe_stops(self):
        command = ControlArbitrator().decide(0.0, float("nan"), desired_speed=0.05)
        self.assertEqual(0.0, command["track_error"])
        self.assertEqual(0.0, command["target_speed"])

    def test_planner_does_not_synthesize_error_without_target_path(self):
        planner = LocalPlanner()
        perception = {
            "timestamp": 1.0,
            "frame_shape": [480, 640],
            "segmentation": {
                "line_valid": True,
                "track_error": 80.0,
                "center_x": 320,
                "road_mask": self.mask,
                "mid_points": [],
            },
        }
        decision = {"task_state": TaskState.NORMAL_TRACK.value, "planner_intent": {}}
        result = planner.plan(perception, decision, now=1.0)
        self.assertIsNone(result["final_track_error"])

    def test_segmentation_does_not_hold_previous_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        valid_seg = np.zeros((480, 640), dtype=np.uint8)
        valid_seg[220:480, 260:380] = seg_func.ROAD_CLASS_ID

        _, first = seg_func.extract_centerline_info(valid_seg, frame)
        self.assertTrue(first["line_valid"])

        empty_seg = np.zeros((480, 640), dtype=np.uint8)
        _, second = seg_func.extract_centerline_info(empty_seg, frame)
        self.assertFalse(second["line_valid"])
        self.assertIsNone(second["track_error"])
        self.assertFalse(second["road_held"])
        self.assertEqual([], second["mid_points"])


if __name__ == "__main__":
    unittest.main()
