import os
import sys
import unittest

import numpy as np


sys.path.insert(0, os.path.dirname(__file__))

from turnsign_ocr_api import TurnSignOcrApiProcessor  # noqa: E402


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


class TurnSignOcrApiProcessorTest(unittest.TestCase):
    def _processor(self, confidence=0.30):
        return TurnSignOcrApiProcessor(
            min_ocr_confidence=0.30,
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

    def test_default_confidence_gate_accepts_point_three(self):
        processor = self._processor(confidence=0.29)
        crop = np.ones((32, 64, 3), dtype=np.uint8)
        det = {"label": "TurnSign", "score": 0.9, "bbox": [0, 0, 64, 32]}

        response = processor.process_crop(crop, det, timestamp=1.50)

        self.assertEqual("low_ocr_confidence", response["status"])
        self.assertFalse(response["instruction_current"])


if __name__ == "__main__":
    unittest.main()
