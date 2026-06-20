import unittest
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "infer_wrap", "base"))
import seg_func


class _ForkClassifier:
    def __init__(self, label, confidence):
        self.label = int(label)
        self.confidence = float(confidence)

    def infer_mask(self, _mask):
        return {"label": self.label, "confidence": self.confidence}


def _normal_mask(h=480, w=640):
    mask = np.zeros((h, w), dtype=np.uint8)
    polygon = np.array([[255, h - 1], [385, h - 1], [365, 165], [285, 165]], np.int32)
    cv2.fillPoly(mask, [polygon], 1)
    return mask


def _positive_y_mask(h=480, w=640):
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.rectangle(mask, (270, 300), (370, h - 1), 1, -1)
    left = np.array([[270, 320], [330, 320], [235, 160], [155, 160]], np.int32)
    right = np.array([[310, 320], [370, 320], [485, 160], [405, 160]], np.int32)
    cv2.fillPoly(mask, [left, right], 1)
    return mask


def _negative_y_mask(h=480, w=640):
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.rectangle(mask, (270, 160), (370, 320), 1, -1)
    left = np.array([[270, 300], [330, 300], [235, h - 1], [155, h - 1]], np.int32)
    right = np.array([[310, 300], [370, 300], [485, h - 1], [405, h - 1]], np.int32)
    cv2.fillPoly(mask, [left, right], 1)
    return mask


def _candidate_x(candidate, target_y):
    points = candidate.get("points") or candidate.get("path") or []
    point = min(points, key=lambda item: abs(int(item[1]) - int(target_y)))
    return float(point[0])


def _curved_channel_mask(with_merge=False, h=480, w=640):
    mask = np.zeros((h, w), dtype=np.uint8)
    ys = np.arange(h - 1, 155, -8)
    xs = 320 + 0.0016 * (h - 1 - ys) ** 2
    path = np.array([(int(x), int(y)) for x, y in zip(xs, ys)], np.int32)
    cv2.polylines(mask, [path], False, 1, 96)
    if with_merge:
        side_xs = 95 + 0.70 * (h - 1 - ys)
        side = np.array([(int(x), int(y)) for x, y in zip(side_xs, ys)], np.int32)
        cv2.polylines(mask, [side], False, 1, 96)
    return mask


class SegForkChannelTrackingTest(unittest.TestCase):
    def setUp(self):
        seg_func.reset_centerline_state()
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)

    def test_low_confidence_positive_y_only_opens_early_gate(self):
        classifier = _ForkClassifier(1, 0.36)
        _out, info = seg_func.extract_centerline_info(
            _positive_y_mask(), self.frame, fork_classifier=classifier, draw_debug=False
        )

        self.assertEqual(info["fork_state"], "FORK_EARLY")
        self.assertGreaterEqual(len(info["fork_candidates"]), 2)
        self.assertIsNone(info["fork_selected_side"])

    def test_strict_positive_y_requires_temporal_confirmation(self):
        classifier = _ForkClassifier(1, 0.90)
        _out, first = seg_func.extract_centerline_info(
            _positive_y_mask(), self.frame, fork_classifier=classifier, draw_debug=False
        )
        _out, second = seg_func.extract_centerline_info(
            _positive_y_mask(), self.frame, fork_classifier=classifier, draw_debug=False
        )

        self.assertNotEqual(first["fork_state"], "FORK_CONFIRMED")
        self.assertEqual(second["fork_state"], "FORK_CONFIRMED")
        self.assertGreaterEqual(len(second["fork_candidates"]), 2)

    def test_connected_upper_fork_is_partitioned_above_fork_point(self):
        mask = _positive_y_mask()
        self.assertEqual(cv2.connectedComponents(mask)[0] - 1, 1)
        classifier = _ForkClassifier(1, 0.90)
        seg_func.extract_centerline_info(mask, self.frame, fork_classifier=classifier, draw_debug=False)
        _out, info = seg_func.extract_centerline_info(
            mask, self.frame, fork_classifier=classifier, draw_debug=False
        )

        self.assertTrue(info["fork_partition_valid"])
        self.assertEqual(info["fork_partition_side"], "upper")
        self.assertIsNotNone(info["fork_point"])
        candidates = {item["side"]: item for item in info["fork_candidates"]}
        fork_y = int(info["fork_point"][1])
        common_y = min(mask.shape[0] - 10, fork_y + 70)
        split_y = max(int(mask.shape[0] * seg_func.TOP_CROP_RATIO) + 10, fork_y - 90)
        self.assertLess(abs(_candidate_x(candidates["left"], common_y) - _candidate_x(candidates["right"], common_y)), 24)
        self.assertGreater(abs(_candidate_x(candidates["left"], split_y) - _candidate_x(candidates["right"], split_y)), 60)

    def test_connected_lower_fork_is_partitioned_below_fork_point(self):
        mask = _negative_y_mask()
        self.assertEqual(cv2.connectedComponents(mask)[0] - 1, 1)
        classifier = _ForkClassifier(1, 0.90)
        seg_func.extract_centerline_info(mask, self.frame, fork_classifier=classifier, draw_debug=False)
        _out, info = seg_func.extract_centerline_info(
            mask, self.frame, fork_classifier=classifier, draw_debug=False
        )

        self.assertTrue(info["fork_partition_valid"])
        self.assertEqual(info["fork_partition_side"], "lower")
        self.assertIsNotNone(info["fork_point"])
        candidates = {item["side"]: item for item in info["fork_candidates"]}
        fork_y = int(info["fork_point"][1])
        common_y = max(int(mask.shape[0] * seg_func.TOP_CROP_RATIO) + 10, fork_y - 70)
        split_y = min(mask.shape[0] - 10, fork_y + 90)
        self.assertLess(abs(_candidate_x(candidates["left"], common_y) - _candidate_x(candidates["right"], common_y)), 24)
        self.assertGreater(abs(_candidate_x(candidates["left"], split_y) - _candidate_x(candidates["right"], split_y)), 60)

    def test_unconfirmed_merge_expansion_holds_previous_channel(self):
        classifier = _ForkClassifier(0, 0.95)
        for _ in range(3):
            _out, normal = seg_func.extract_centerline_info(
                _normal_mask(), self.frame, fork_classifier=classifier, draw_debug=False
            )

        merged = _normal_mask()
        side_road = np.array([[80, 330], [320, 330], [315, 205], [190, 205]], np.int32)
        cv2.fillPoly(merged, [side_road], 1)
        _out, held = seg_func.extract_centerline_info(
            merged, self.frame, fork_classifier=classifier, draw_debug=False
        )

        self.assertEqual(held["fork_state"], "NORMAL")
        self.assertEqual(held["channel_state"], "HOLD")
        self.assertLess(abs(held["target_x"] - normal["target_x"]), 35)

    def test_curved_merge_keeps_original_arc(self):
        classifier = _ForkClassifier(0, 0.95)
        for _ in range(3):
            _out, normal = seg_func.extract_centerline_info(
                _curved_channel_mask(), self.frame, fork_classifier=classifier, draw_debug=False
            )
        _out, held = seg_func.extract_centerline_info(
            _curved_channel_mask(with_merge=True),
            self.frame,
            fork_classifier=classifier,
            draw_debug=False,
        )

        self.assertEqual(held["channel_state"], "HOLD")
        self.assertLess(abs(held["target_x"] - normal["target_x"]), 35)


if __name__ == "__main__":
    unittest.main()
