import unittest

import numpy as np

from .func import (
    CLASSES,
    _row_peaks,
    candidate_centerlines,
    decode_outputs,
    parse_outputs,
)


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

    def test_centerlines_keep_two_best_long_paths(self):
        road = np.ones((120, 160), dtype=np.float32)
        center = np.zeros((120, 160), dtype=np.float32)
        for row in range(30, 118):
            center[row, 40] = 0.70
            center[row, 80] = 0.95
            center[row, 120] = 0.85
        # This produces seven sampled points: strong, but still too short.
        center[98:119, 20] = 1.0

        paths, _ = candidate_centerlines(road, center)

        self.assertEqual(len(paths), 2)
        average_confidences = [
            sum(point[2] for point in path) / len(path) for path in paths
        ]
        self.assertGreater(average_confidences[0], average_confidences[1])
        self.assertAlmostEqual(average_confidences[0], 0.95, places=5)
        self.assertAlmostEqual(average_confidences[1], 0.85, places=5)

    def test_single_centerline_is_not_duplicated(self):
        road = np.ones((120, 160), dtype=np.float32)
        center = np.zeros((120, 160), dtype=np.float32)
        center[30:118, 80] = 0.90

        paths, _ = candidate_centerlines(road, center)

        self.assertEqual(len(paths), 1)

    def test_peak_position_uses_weighted_neighborhood(self):
        values = np.zeros(20, dtype=np.float32)
        values[9:13] = [0.50, 1.00, 0.80, 0.40]

        peaks = _row_peaks(values, threshold=0.25, max_peaks=1)

        self.assertEqual(len(peaks), 1)
        self.assertGreater(peaks[0][0], 10.0)
        self.assertLess(peaks[0][0], 11.0)

    def test_blank_rows_do_not_join_two_short_segments(self):
        road = np.ones((120, 160), dtype=np.float32)
        center = np.zeros((120, 160), dtype=np.float32)
        for row in [118, 115, 112, 109, 106, 88, 85, 82, 79, 76]:
            center[row, 80] = 0.95

        paths, _ = candidate_centerlines(road, center)

        self.assertEqual(paths, [])

    def test_curve_fit_reduces_single_frame_zigzag(self):
        road = np.ones((120, 160), dtype=np.float32)
        center = np.zeros((120, 160), dtype=np.float32)
        for index, row in enumerate(range(118, 29, -3)):
            center[row, 79 if index % 2 == 0 else 81] = 0.90

        paths, _ = candidate_centerlines(road, center)

        self.assertEqual(len(paths), 1)
        x_values = [point[0] for point in paths[0]]
        self.assertLess(max(x_values) - min(x_values), 8)


if __name__ == "__main__":
    unittest.main()
