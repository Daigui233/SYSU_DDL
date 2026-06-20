import unittest

import numpy as np

from infer_wrap.base import func


def _reference_post_process(input_data, img_shape):
    boxes = []
    class_confidences = []
    scores = []
    pair_per_branch = len(input_data) // 3
    for branch in range(3):
        boxes.append(func.box_process(input_data[pair_per_branch * branch], img_shape))
        class_output = input_data[pair_per_branch * branch + 1]
        class_confidences.append(class_output)
        scores.append(np.ones_like(class_output[:, :1, :, :], dtype=np.float32))

    def flatten(value):
        channels = value.shape[1]
        return value.transpose(0, 2, 3, 1).reshape(-1, channels)

    boxes = np.concatenate([flatten(value) for value in boxes])
    class_confidences = np.concatenate([flatten(value) for value in class_confidences])
    scores = np.concatenate([flatten(value) for value in scores])
    boxes, classes, scores = func.filter_boxes(boxes, scores, class_confidences)
    if boxes.size == 0:
        return None, None, None

    kept_boxes = []
    kept_classes = []
    kept_scores = []
    for class_id in set(classes):
        indices = np.where(classes == class_id)
        class_boxes = boxes[indices]
        class_ids = classes[indices]
        class_scores = scores[indices]
        keep = func.nms_boxes(class_boxes, class_scores)
        if len(keep):
            kept_boxes.append(class_boxes[keep])
            kept_classes.append(class_ids[keep])
            kept_scores.append(class_scores[keep])
    return np.concatenate(kept_boxes), np.concatenate(kept_classes), np.concatenate(kept_scores)


class DetectionPostProcessTest(unittest.TestCase):
    @staticmethod
    def _outputs(with_candidates):
        rng = np.random.default_rng(7)
        outputs = []
        for branch, grid_size in enumerate((20, 40, 80)):
            positions = rng.normal(size=(1, 68, grid_size, grid_size)).astype(np.float32)
            classes = np.zeros((1, func.NUM_CLASSES, grid_size, grid_size), dtype=np.float32)
            if with_candidates:
                classes[0, (branch + 2) % func.NUM_CLASSES, branch + 2, branch + 3] = 0.75 + branch * 0.05
            score_sum = np.zeros((1, 1, grid_size, grid_size), dtype=np.float32)
            outputs.extend((positions, classes, score_sum))
        return outputs

    def test_candidate_first_decode_matches_full_decode(self):
        outputs = self._outputs(with_candidates=True)
        expected = _reference_post_process(outputs, (480, 640))
        actual = func.post_process(outputs, (480, 640))

        np.testing.assert_allclose(actual[0], expected[0], rtol=1e-5, atol=1e-4)
        np.testing.assert_array_equal(actual[1], expected[1])
        np.testing.assert_allclose(actual[2], expected[2], rtol=0.0, atol=0.0)

    def test_no_candidates_returns_empty_result(self):
        result = func.post_process(self._outputs(with_candidates=False), (480, 640))
        self.assertEqual(result, (None, None, None))


if __name__ == "__main__":
    unittest.main()
