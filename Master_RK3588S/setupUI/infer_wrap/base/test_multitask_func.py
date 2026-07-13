import unittest

import numpy as np

from .func import CLASSES, decode_outputs, parse_outputs


class MultiTaskPostprocessTest(unittest.TestCase):
    def make_outputs(self):
        boxes = np.zeros((1, 6300, 4), dtype=np.float32)
        scores = np.zeros((1, 6300, 8), dtype=np.float32)
        pixel = np.full((1, 3, 120, 160), -8.0, dtype=np.float32)
        points = np.zeros((1, 2, 32, 2), dtype=np.float32)
        points[0, :, :, 1] = np.linspace(0.95, 0.10, 32)
        points[0, 0, :, 0] = 0.42
        points[0, 1, :, 0] = 0.68
        path_scores = np.asarray([[4.0, 3.0]], dtype=np.float32)
        count_scores = np.asarray([[-3.0, -1.0, 5.0]], dtype=np.float32)
        return boxes, scores, pixel, points, path_scores, count_scores

    def test_output_contract(self):
        boxes, scores, pixel, points, path_scores, count_scores = parse_outputs(self.make_outputs())
        self.assertEqual(boxes.shape, (6300, 4))
        self.assertEqual(scores.shape, (6300, 8))
        self.assertEqual(pixel.shape, (3, 120, 160))
        self.assertEqual(points.shape, (2, 32, 2))
        self.assertEqual(path_scores.shape, (2,))
        self.assertEqual(count_scores.shape, (3,))
        self.assertEqual(CLASSES[7], "SpeedSign")

    def test_decode_two_direct_paths_without_heatmap_linking(self):
        outputs = list(self.make_outputs())
        outputs[0][0, 0] = [64, 48, 192, 144]
        outputs[1][0, 0, 7] = 0.90
        outputs[2][0, 0, 20:, 50:110] = 8.0
        result = decode_outputs(outputs, (240, 320, 3))
        self.assertEqual(len(result["detections"]), 1)
        self.assertEqual(result["detections"][0]["label"], "SpeedSign")
        self.assertEqual(result["centerline"]["path_count"], 2)
        self.assertEqual(len(result["centerline"]["paths"]), 2)
        self.assertEqual(len(result["centerline"]["paths"][0]["points"]), 32)
        self.assertLess(result["centerline"]["paths"][0]["points"][0][1], 480)

    def test_one_path_keeps_slot_zero(self):
        outputs = list(self.make_outputs())
        outputs[5] = np.asarray([[-3.0, 5.0, -1.0]], dtype=np.float32)
        result = decode_outputs(outputs, (480, 640, 3))
        self.assertEqual(result["centerline"]["path_count"], 1)
        self.assertEqual([item["slot"] for item in result["centerline"]["paths"]], [0])

    def test_rejects_old_four_output_model(self):
        with self.assertRaises(ValueError):
            parse_outputs(self.make_outputs()[:4])


if __name__ == "__main__":
    unittest.main()
