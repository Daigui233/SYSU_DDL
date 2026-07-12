import unittest

import numpy as np

from .func import CLASSES, candidate_centerlines, decode_outputs, parse_outputs


class MultiTaskPostprocessTest(unittest.TestCase):
    def make_outputs(self):
        boxes = np.zeros((1, 6300, 4), dtype=np.float32)
        scores = np.zeros((1, 6300, 8), dtype=np.float32)
        pixel = np.full((1, 2, 120, 160), -8.0, dtype=np.float32)
        topology = np.asarray([[0.0, 4.0, -1.0, -2.0]], dtype=np.float32)
        return boxes, scores, pixel, topology

    def test_output_contract(self):
        outputs = self.make_outputs()
        boxes, scores, pixel, topology = parse_outputs(outputs)
        self.assertEqual(boxes.shape, (6300, 4))
        self.assertEqual(scores.shape, (6300, 8))
        self.assertEqual(pixel.shape, (2, 120, 160))
        self.assertEqual(topology.shape, (4,))
        self.assertEqual(CLASSES[7], "SpeedSign")

    def test_decode_all_tasks(self):
        boxes, scores, pixel, topology = self.make_outputs()
        boxes[0, 0] = [64, 48, 192, 144]
        boxes[0, 1] = [66, 50, 190, 142]
        scores[0, 0, 7] = 0.90
        scores[0, 1, 7] = 0.80

        pixel[0, 0, 20:, 50:110] = 8.0
        for row in range(25, 118):
            pixel[0, 1, row, 80] = 8.0

        result = decode_outputs(
            (boxes, scores, pixel, topology), (240, 320, 3))
        self.assertEqual(len(result["detections"]), 1)
        detection = result["detections"][0]
        self.assertEqual(detection["label"], "SpeedSign")
        self.assertAlmostEqual(detection["bbox"][0], 32.0)
        self.assertGreater(float(result["road"]["mask"].mean()), 0.2)
        self.assertTrue(result["centerline"]["paths"])
        self.assertNotIn("primary", result["centerline"])
        self.assertEqual(result["topology"]["label"], "fork")
        self.assertTrue(result["topology"]["reliable"])

    def test_rejects_old_model_outputs(self):
        with self.assertRaises(ValueError):
            parse_outputs([np.zeros((1, 4, 80, 80), dtype=np.float32)])

    def test_y_fork_returns_both_visible_centerlines(self):
        road = np.ones((120, 160), dtype=np.float32)
        center = np.zeros((120, 160), dtype=np.float32)
        for row in range(30, 118):
            center[row, 80] = 1.0
            if row < 75:
                center[row, 40 + row // 2] = 0.95

        paths, _ = candidate_centerlines(road, center)
        self.assertGreaterEqual(len(paths), 2)
        self.assertGreaterEqual(len(paths[0]), 10)
        self.assertGreaterEqual(len(paths[1]), 5)


if __name__ == "__main__":
    unittest.main()
