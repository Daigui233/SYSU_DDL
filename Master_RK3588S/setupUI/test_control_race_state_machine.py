import unittest

from control_race_state_machine import RaceStateMachine


def detection(category, score=0.95):
    bbox = [220.0, 150.0, 420.0, 300.0]
    return {
        "category": category,
        "score": score,
        "bbox": bbox,
        "center": [320.0, 225.0],
        "area_ratio": ((bbox[2] - bbox[0]) * (bbox[3] - bbox[1])) / (640.0 * 480.0),
        "age": 0.0,
    }


def perception(*categories):
    return {
        "frame_shape": [480, 640],
        "detections": [detection(category) for category in categories],
    }


class RaceSignOrderTest(unittest.TestCase):
    def setUp(self):
        self.machine = RaceStateMachine(
            {
                "DOOR_CROSS_COOLDOWN": 0.1,
                "ENDSIGN_CONFIRM_SECONDS": 0.2,
                "ENDSIGN_CONFIRM_GAP": 0.3,
                "ENDSIGN_LOST_STOP_DELAY": 0.2,
            }
        )

    def update(self, now, *categories):
        return self.machine.update(perception(*categories), now)

    def rearm_door(self, now):
        self.update(now)

    def start_race(self):
        state = self.update(0.0, "door", "begin_sign", "end_sign")
        self.assertTrue(state["race_started"])
        self.assertEqual("race_start", state["lap_event"])
        return state

    def enter_final_lap(self):
        self.start_race()
        self.rearm_door(0.2)
        self.update(0.3, "door")
        self.rearm_door(0.5)
        return self.update(0.6, "door")

    def test_end_sign_is_ignored_before_begin_sign(self):
        state = self.update(0.0, "door", "end_sign")

        self.assertFalse(state["race_started"])
        self.assertEqual("WAIT_BEGIN", state["sign_phase"])
        self.assertFalse(state["end_sign_allowed"])
        self.assertFalse(state["end_under_door"])
        self.assertTrue(state["end_sign_ignored"])
        self.assertFalse(state["finish_armed"])

    def test_begin_requirement_cannot_be_disabled_by_old_config(self):
        machine = RaceStateMachine(
            {
                "TOTAL_LAPS": 1,
                "ENDSIGN_REQUIRE_RACE_STARTED": False,
            }
        )
        state = machine.update(perception("door", "end_sign"), 0.0)

        self.assertFalse(state["end_sign_allowed"])
        self.assertTrue(state["end_sign_ignored"])
        self.assertEqual("WAIT_BEGIN", state["sign_phase"])

    def test_first_valid_begin_starts_lap_one_and_ignores_end_jump(self):
        state = self.start_race()

        self.assertEqual(1, state["current_lap"])
        self.assertFalse(state["begin_sign_allowed"])
        self.assertFalse(state["end_sign_allowed"])
        self.assertFalse(state["end_under_door"])
        self.assertTrue(state["end_sign_ignored"])

    def test_begin_sign_is_permanently_ignored_after_start(self):
        self.start_race()
        self.rearm_door(0.2)
        state = self.update(0.3, "door", "begin_sign")

        self.assertEqual("RACING", state["sign_phase"])
        self.assertFalse(state["begin_sign_allowed"])
        self.assertFalse(state["begin_under_door"])
        self.assertTrue(state["begin_sign_ignored"])
        self.assertEqual("lap_1", state["lap_event"])

    def test_final_lap_ignores_begin_jump_and_accepts_only_end(self):
        state = self.enter_final_lap()
        self.assertEqual(3, state["current_lap"])
        self.assertEqual("WAIT_END", state["sign_phase"])

        begin_jump = self.update(0.7, "door", "begin_sign")
        self.assertTrue(begin_jump["begin_sign_ignored"])
        self.assertFalse(begin_jump["begin_under_door"])
        self.assertFalse(begin_jump["finish_armed"])

        first_end = self.update(0.8, "door", "end_sign")
        confirmed_end = self.update(1.05, "door", "end_sign")
        stopped = self.update(1.30)

        self.assertFalse(first_end["finish_armed"])
        self.assertTrue(confirmed_end["finish_armed"])
        self.assertTrue(stopped["finish_stop"])
        self.assertEqual("FINISHED", stopped["sign_phase"])


if __name__ == "__main__":
    unittest.main()
