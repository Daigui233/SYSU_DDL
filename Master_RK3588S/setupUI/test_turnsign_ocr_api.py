import os
import queue
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.dirname(__file__))

from turnsign_ocr_api import (  # noqa: E402
    AsyncTurnSignOcrApiProcessor,
    TurnSignOcrApiProcessor,
)


class FakeOcrReader:
    def __init__(self, text, confidence):
        self.text = text
        self.confidence = confidence

    def read(self, _crop):
        return {"text": self.text, "confidence": self.confidence, "lines": []}


class FakeInterpreter:
    model = "fake"

    def interpret(self, source_text, confidence, _context):
        return {
            "source_text": source_text,
            "ocr_confidence": confidence,
            "preferred_branch": "right",
            "direction": "right",
            "reason": "test",
        }


class FakeAliveProcess:
    pid = 1234
    exitcode = None

    def is_alive(self):
        return True


class TurnSignOcrApiProcessorTest(unittest.TestCase):
    def _processor(self, confidence=0.40):
        return TurnSignOcrApiProcessor(
            min_ocr_confidence=0.40,
            stable_frames=1,
            stable_duration_s=0.50,
            ocr_interval=0.0,
            api_cooldown=0.0,
            cache_ttl=0.0,
            async_api=False,
            ocr_reader=FakeOcrReader("RIGHT", confidence),
            interpreter=FakeInterpreter(),
        )

    def test_requires_half_second_stable_text_before_current_direction(self):
        processor = self._processor()
        crop = np.ones((32, 64, 3), dtype=np.uint8)
        det = {"label": "TurnSign", "score": 0.9, "bbox": [0, 0, 64, 32]}

        first = processor.process_crop(crop, det, timestamp=1.0)
        almost = processor.process_crop(crop, det, timestamp=1.49)
        ready = processor.process_crop(crop, det, timestamp=1.50)

        self.assertEqual("waiting_stable_text", first["status"])
        self.assertEqual("waiting_stable_text", almost["status"])
        self.assertEqual("api_done", ready["status"])
        self.assertTrue(ready["instruction_current"])
        self.assertEqual("right", ready["instruction"]["direction"])

    def test_default_confidence_gate_rejects_below_point_four(self):
        processor = self._processor(confidence=0.39)
        crop = np.ones((32, 64, 3), dtype=np.uint8)
        det = {"label": "TurnSign", "score": 0.9, "bbox": [0, 0, 64, 32]}

        response = processor.process_crop(crop, det, timestamp=1.50)

        self.assertEqual("low_ocr_confidence", response["status"])
        self.assertFalse(response["instruction_current"])

    def test_async_defaults_to_immediate_complete_box_snapshot(self):
        processor = AsyncTurnSignOcrApiProcessor()
        sync_processor = TurnSignOcrApiProcessor()

        self.assertEqual(1, processor.confirm_frames)
        self.assertAlmostEqual(0.40, processor.min_det_score)
        self.assertAlmostEqual(0.02, processor.snapshot_min_area_ratio)
        self.assertAlmostEqual(0.10, processor.snapshot_edge_margin_ratio)
        self.assertAlmostEqual(0.80, processor.door_conflict_score)
        self.assertAlmostEqual(
            120.0, processor.door_conflict_distance_px_640)
        self.assertNotIn("snapshot_min_area_ratio", processor.processor_kwargs)
        self.assertNotIn("snapshot_edge_margin_ratio", processor.processor_kwargs)
        self.assertAlmostEqual(185.0 / 480.0, processor.detection_line_ratio)
        self.assertAlmostEqual(3.0, processor.session_absence_timeout_s)
        self.assertAlmostEqual(10.0, processor.ocr_response_timeout_s)
        self.assertAlmostEqual(0.40, sync_processor.min_det_score)
        self.assertAlmostEqual(0.40, sync_processor.min_ocr_confidence)

    def test_near_high_confidence_door_suppresses_turnsign(self):
        processor = AsyncTurnSignOcrApiProcessor(
            door_conflict_score=0.80,
            door_conflict_distance_px_640=120.0,
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = [
            {"label": "TurnSign", "score": 0.91,
             "area_ratio": 0.04, "bbox": [200, 100, 300, 200]},
            {"label": "Door", "score": 0.94,
             "area_ratio": 0.04, "bbox": [320, 100, 420, 200]},
        ]

        selected, status = processor._select_turnsign_with_status(
            detections, frame)

        self.assertIsNone(selected)
        self.assertEqual("turnsign_door_conflict", status)

    def test_far_or_low_confidence_door_does_not_suppress_turnsign(self):
        processor = AsyncTurnSignOcrApiProcessor(
            door_conflict_score=0.80,
            door_conflict_distance_px_640=120.0,
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        sign = {"label": "TurnSign", "score": 0.91,
                "area_ratio": 0.04, "bbox": [100, 100, 200, 200]}
        far_door = {"label": "Door", "score": 0.94,
                    "area_ratio": 0.04, "bbox": [500, 100, 600, 200]}
        weak_near_door = {"label": "Door", "score": 0.79,
                          "area_ratio": 0.04,
                          "bbox": [220, 100, 320, 200]}

        far_selected, far_status = processor._select_turnsign_with_status(
            [sign, far_door], frame)
        weak_selected, weak_status = processor._select_turnsign_with_status(
            [sign, weak_near_door], frame)

        self.assertIs(sign, far_selected)
        self.assertEqual("turnsign_candidate", far_status)
        self.assertIs(sign, weak_selected)
        self.assertEqual("turnsign_candidate", weak_status)

    def test_process_rejects_near_door_conflict_before_ocr_session(self):
        processor = AsyncTurnSignOcrApiProcessor(
            door_conflict_score=0.80,
            door_conflict_distance_px_640=120.0,
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = [
            {"label": "TurnSign", "score": 0.91,
             "area_ratio": 0.04, "bbox": [200, 100, 300, 200]},
            {"label": "Door", "score": 0.94,
             "area_ratio": 0.04, "bbox": [320, 100, 420, 200]},
        ]

        response = processor.process(frame, detections, timestamp=1.0)

        self.assertEqual("turnsign_door_conflict", response["status"])
        self.assertFalse(response["session_active"])
        self.assertFalse(response["instruction_current"])
        self.assertTrue(response["clear_result"])


    def test_ocr_waits_until_turnsign_reaches_stop_size(self):
        processor = TurnSignOcrApiProcessor(
            min_area_ratio=0.031,
            ocr_reader=FakeOcrReader("RIGHT", 0.90),
            interpreter=FakeInterpreter(),
        )
        too_far = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.030,
            "bbox": [285, 136, 425, 202],
        }
        stop_size = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.03145,
            "bbox": [285, 136, 425, 205],
        }

        self.assertIsNone(processor.select_turnsign([too_far]))
        self.assertIs(stop_size, processor.select_turnsign([stop_size]))

    def test_async_ocr_reuses_first_stop_size_snapshot(self):
        processor = AsyncTurnSignOcrApiProcessor.__new__(
            AsyncTurnSignOcrApiProcessor)
        processor.turnsign_snapshot_crop = None
        processor.turnsign_snapshot_detection = None
        processor.turnsign_snapshot_ts = 0.0
        det = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.03145,
            "bbox": [20, 10, 60, 40],
        }
        first_frame = np.zeros((80, 100, 3), dtype=np.uint8)
        first_frame[5:45, 15:65] = (10, 120, 240)
        first = processor._capture_turnsign_snapshot(
            first_frame, det, timestamp=1.0)
        frozen = first.copy()

        changed_frame = np.full_like(first_frame, 255)
        second = processor._capture_turnsign_snapshot(
            changed_frame, det, timestamp=1.2)

        self.assertTrue(np.array_equal(frozen, second))
        self.assertEqual(1.0, processor.turnsign_snapshot_ts)
        self.assertEqual(det, processor.turnsign_snapshot_detection)

    def test_async_starts_worker_on_third_detection_then_waits_for_size(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=3,
            confirm_iou=0.30,
            min_det_score=0.25,
            min_area_ratio=0.010,
            ocr_interval=0.0,
        )
        starts = []

        def start_worker():
            starts.append(processor.confirm_count)
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            return True

        processor._start_worker = start_worker
        self.assertIsNone(processor.process_handle)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        far = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.010,
            "bbox": [300, 100, 360, 150],
        }

        first = processor.process(frame, [far], timestamp=1.0)
        second = processor.process(frame, [far], timestamp=1.1)
        third = processor.process(frame, [far], timestamp=1.2)

        self.assertEqual("turnsign_confirming", first["status"])
        self.assertEqual(1, first["confirm_count"])
        self.assertEqual("turnsign_confirming", second["status"])
        self.assertEqual(2, second["confirm_count"])
        self.assertEqual("ocr_prewarming", third["status"])
        self.assertEqual([3], starts)
        self.assertFalse(third["snapshot_captured"])
        self.assertTrue(processor.request_queue.empty())

    def test_async_freezes_at_stop_size_and_submits_after_worker_ready(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=3,
            min_det_score=0.25,
            min_area_ratio=0.010,
            ocr_interval=0.0,
        )

        def start_worker():
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            return True

        processor._start_worker = start_worker
        far_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        far = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.010,
            "bbox": [300, 100, 360, 150],
        }
        for index in range(3):
            processor.process(far_frame, [far], timestamp=1.0 + index * 0.1)

        stop_frame = np.zeros_like(far_frame)
        stop_frame[190:280, 260:440] = (10, 120, 240)
        stop_size = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.03145,
            "bbox": [285, 200, 425, 269],
        }
        waiting = processor.process(stop_frame, [stop_size], timestamp=1.3)
        frozen = processor.turnsign_snapshot_crop.copy()

        self.assertEqual("snapshot_waiting_worker", waiting["status"])
        self.assertTrue(waiting["snapshot_captured"])
        self.assertTrue(processor.request_queue.empty())

        processor.worker_ready = True
        changed_frame = np.full_like(stop_frame, 255)
        submitted = processor.process(
            changed_frame, [stop_size], timestamp=1.4)
        payload = processor.request_queue.get_nowait()

        self.assertEqual("ocr_submitted", submitted["status"])
        self.assertEqual(processor.session_id, payload["session_id"])
        self.assertTrue(np.array_equal(frozen, payload["crop"]))
        self.assertFalse(np.array_equal(
            changed_frame[190:280, 260:440], frozen))

    def test_async_rejects_incomplete_box_but_accepts_complete_box(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=3,
            min_det_score=0.30,
            min_area_ratio=0.010,
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        incomplete = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.020,
            "bbox": [-1, 231, 139, 305],
        }
        complete = dict(incomplete, bbox=[250, 231, 390, 305])

        rejected = processor.process(frame, [incomplete], timestamp=1.0)
        accepted = processor.process(frame, [complete], timestamp=1.1)

        self.assertEqual(
            "turnsign_incomplete_bbox", rejected["candidate_status"])
        self.assertEqual(0, rejected["confirm_count"])
        self.assertEqual("turnsign_confirming", accepted["status"])
        self.assertEqual(1, accepted["confirm_count"])

    def test_snapshot_gate_uses_complete_two_percent_box_in_center_eighty(self):
        processor = AsyncTurnSignOcrApiProcessor()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        standard = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.0227, "bbox": [330, 149, 446, 209],
        }
        too_small = dict(standard, area_ratio=0.0199)
        edge = dict(standard, bbox=[0, 149, 100, 209])
        incomplete = dict(standard, bbox=[580, 149, 660, 209])

        self.assertTrue(processor._snapshot_ready(standard, frame))
        self.assertFalse(processor._snapshot_ready(too_small, frame))
        self.assertFalse(processor._snapshot_ready(edge, frame))
        self.assertFalse(processor._snapshot_ready(incomplete, frame))

    def test_first_eligible_box_immediately_freezes_ocr_snapshot(self):
        processor = AsyncTurnSignOcrApiProcessor(
            min_det_score=0.30,
            min_area_ratio=0.010,
            snapshot_min_area_ratio=0.020,
            snapshot_edge_margin_ratio=0.10,
        )

        def start_worker():
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            return True

        processor._start_worker = start_worker
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        sign = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.0227, "bbox": [330, 149, 446, 209],
        }

        response = processor.process(frame, [sign], timestamp=1.0)

        self.assertTrue(response["session_active"])
        self.assertEqual(1, response["confirm_count"])
        self.assertTrue(response["snapshot_ready"])
        self.assertTrue(response["snapshot_captured"])
        self.assertEqual("snapshot_waiting_worker", response["status"])

    def test_async_three_frames_need_not_have_same_bbox(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=3,
            confirm_iou=0.90,
            min_det_score=0.30,
            min_area_ratio=0.010,
            ocr_interval=0.0,
        )
        starts = []

        def start_worker():
            starts.append(processor.confirm_count)
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            return True

        processor._start_worker = start_worker
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        signs = [
            {"label": "TurnSign", "score": 0.90, "area_ratio": 0.02,
             "bbox": [40, 80, 140, 150]},
            {"label": "TurnSign", "score": 0.90, "area_ratio": 0.02,
             "bbox": [270, 90, 370, 160]},
            {"label": "TurnSign", "score": 0.90, "area_ratio": 0.02,
             "bbox": [490, 100, 590, 170]},
        ]

        responses = [
            processor.process(frame, [sign], timestamp=1.0 + index * 0.1)
            for index, sign in enumerate(signs)
        ]

        self.assertEqual([1, 2, 3], [item["confirm_count"] for item in responses])
        self.assertEqual([3], starts)
        self.assertTrue(processor.session_active)

    def test_async_confirmation_miss_resets_consecutive_count(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=3,
            min_det_score=0.30,
            min_area_ratio=0.010,
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        sign = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.020, "bbox": [270, 90, 370, 160],
        }

        first = processor.process(frame, [sign], timestamp=1.0)
        missed = processor.process(frame, [], timestamp=1.1)
        restarted = processor.process(frame, [sign], timestamp=1.2)

        self.assertEqual(1, first["confirm_count"])
        self.assertEqual(0, missed["confirm_count"])
        self.assertEqual(1, restarted["confirm_count"])

    def test_unresolved_session_exits_after_three_seconds_without_turnsign(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=1,
            min_det_score=0.30,
            min_area_ratio=0.010,
            session_absence_timeout_s=3.0,
            ocr_response_timeout_s=10.0,
        )

        def start_worker():
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            return True

        processor._start_worker = start_worker
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        sign = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.020, "bbox": [270, 90, 370, 160],
        }
        continuation = dict(sign, label="Door")

        locked = processor.process(frame, [sign], timestamp=1.0)
        held = processor.process(frame, [continuation], timestamp=3.99)
        exited = processor.process(frame, [continuation], timestamp=4.0)

        self.assertTrue(locked["session_active"])
        self.assertTrue(held["session_active"])
        self.assertEqual("turnsign_exit_no_sign_3s", exited["status"])
        self.assertEqual("turnsign_exit_no_sign", exited["control_phase"])
        self.assertFalse(exited["active"])
        self.assertFalse(processor.session_active)
        self.assertEqual(0, processor.confirm_count)

    def test_unanswered_ocr_request_exits_after_ten_seconds(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=1,
            min_det_score=0.30,
            min_area_ratio=0.010,
            ocr_interval=0.0,
            session_absence_timeout_s=3.0,
            ocr_response_timeout_s=10.0,
        )

        def start_worker():
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            processor.worker_ready = True
            return True

        processor._start_worker = start_worker
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        sign = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.03145, "bbox": [285, 200, 425, 269],
        }

        submitted = processor.process(frame, [sign], timestamp=1.0)
        old_request_id = processor.pending_request_id
        old_session_id = processor.session_id
        pending = processor.process(frame, [sign], timestamp=10.99)
        exited = processor.process(frame, [sign], timestamp=11.0)

        self.assertEqual("ocr_submitted", submitted["status"])
        self.assertEqual("ocr_pending", pending["status"])
        self.assertEqual("turnsign_exit_ocr_timeout_10s", exited["status"])
        self.assertEqual(
            "turnsign_exit_ocr_timeout", exited["control_phase"])
        self.assertFalse(processor.session_active)
        self.assertEqual(0, processor.pending_request_id)

        processor.result_queue.put_nowait({
            "type": "result",
            "request_id": old_request_id,
            "session_id": old_session_id,
            "response": {
                "active": True,
                "status": "api_done",
                "instruction_current": True,
                "instruction": {"direction": "right"},
            },
        })
        self.assertFalse(processor._drain_results())
        self.assertFalse(processor.session_resolved)

    def test_async_rejects_result_from_an_old_session(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=1,
            min_area_ratio=0.031,
            ocr_interval=0.0,
        )

        def start_worker():
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            processor.worker_ready = True
            return True

        processor._start_worker = start_worker
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        sign = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.03145,
            "bbox": [285, 200, 425, 269],
        }
        submitted = processor.process(frame, [sign], timestamp=1.0)
        request_id = processor.pending_request_id
        valid_response = {
            "active": True,
            "status": "api_done",
            "instruction_current": True,
            "instruction": {"direction": "right"},
        }
        processor.result_queue.put_nowait({
            "type": "result",
            "request_id": request_id,
            "session_id": processor.session_id - 1,
            "response": valid_response,
        })

        stale = processor.process(frame, [sign], timestamp=1.1)

        self.assertEqual("ocr_submitted", submitted["status"])
        self.assertEqual("ocr_pending", stale["status"])
        self.assertFalse(stale["instruction_current"])
        self.assertFalse(processor.session_resolved)

        processor.result_queue.put_nowait({
            "type": "result",
            "request_id": request_id,
            "session_id": processor.session_id,
            "response": valid_response,
        })
        current = processor.process(frame, [sign], timestamp=1.2)
        held = processor.process(frame, [sign], timestamp=1.3)

        self.assertEqual("api_done", current["status"])
        self.assertTrue(current["instruction_current"])
        self.assertTrue(current["turnsign_resolved"])
        self.assertEqual("api_done_held", held["status"])
        self.assertFalse(held["instruction_current"])
        self.assertTrue(held["turnsign_resolved"])

    def test_async_failed_read_refreshes_correctly_positioned_snapshot(self):
        processor = AsyncTurnSignOcrApiProcessor(
            confirm_frames=1,
            min_det_score=0.30,
            min_area_ratio=0.010,
            ocr_interval=0.0,
        )

        def start_worker():
            processor.process_handle = FakeAliveProcess()
            processor.request_queue = queue.Queue(maxsize=1)
            processor.result_queue = queue.Queue(maxsize=3)
            processor.worker_ready = True
            return True

        processor._start_worker = start_worker
        sign = {
            "label": "TurnSign", "score": 0.90,
            "area_ratio": 0.020, "bbox": [260, 200, 380, 274],
        }
        first_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        submitted = processor.process(first_frame, [sign], timestamp=1.0)
        request_id = processor.pending_request_id
        frozen = processor.turnsign_snapshot_crop.copy()
        processor.result_queue.put_nowait({
            "type": "result",
            "request_id": request_id,
            "session_id": processor.session_id,
            "response": {
                "active": True,
                "status": "empty_ocr_text",
                "instruction_current": False,
                "instruction": {"direction": "unknown"},
            },
        })

        changed_frame = np.full_like(first_frame, 255)
        failed = processor.process(changed_frame, [sign], timestamp=1.1)

        self.assertEqual("ocr_submitted", submitted["status"])
        self.assertTrue(failed["snapshot_refreshed"])
        self.assertEqual(1.1, processor.turnsign_snapshot_ts)
        self.assertFalse(np.array_equal(
            frozen, processor.turnsign_snapshot_crop))
        self.assertTrue(np.all(processor.turnsign_snapshot_crop == 255))


if __name__ == "__main__":
    unittest.main()
