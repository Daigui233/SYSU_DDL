import unittest

import cv2
import numpy as np

from aruco_app.ui_main import MainWindow
from aruco_core.aruco_detector import ArUcoDetector
from aruco_core.config_loader import get_config
from aruco_core.coordinate_transformer import CoordinateTransformer


def marker(marker_id, center, size=20.0):
    cx, cy = center
    half = size / 2.0
    corners = np.asarray(
        [
            [cx - half, cy - half],
            [cx + half, cy - half],
            [cx + half, cy + half],
            [cx - half, cy + half],
        ],
        dtype=np.float32,
    )
    return {
        "id": int(marker_id),
        "center": np.asarray(center, dtype=np.float32),
        "corners": corners,
    }


class InstanceCalibrationTests(unittest.TestCase):
    def test_runtime_dictionary_allows_bounded_bit_correction(self):
        detector = ArUcoDetector()

        self.assertEqual(detector.aruco_dict.maxCorrectionBits, 5)
        self.assertAlmostEqual(detector.aruco_params.errorCorrectionRate, 0.6)

    def test_duplicate_ids_are_preserved_as_separate_instances(self):
        detector = ArUcoDetector.__new__(ArUcoDetector)
        items = [
            marker(0, (100, 100)),
            marker(7, (900, 100)),
            marker(7, (100, 700)),
            marker(9, (900, 700)),
            marker(0, (500, 400)),
        ]
        corners = [item["corners"].reshape(1, 4, 2) for item in items]
        ids = np.asarray([[item["id"]] for item in items], dtype=np.int32)

        instances = detector.get_marker_instances(corners, ids)

        self.assertEqual(len(instances), 5)
        self.assertEqual([item["id"] for item in instances].count(0), 2)
        self.assertEqual([item["id"] for item in instances].count(7), 2)

    def test_extra_center_vehicle_does_not_replace_four_field_corners(self):
        detector = ArUcoDetector.__new__(ArUcoDetector)
        fixed = [
            marker(0, (100, 100)),
            marker(8, (900, 120)),
            marker(6, (120, 700)),
            marker(8, (880, 720)),
        ]
        vehicle = marker(0, (500, 400))

        selected = detector.select_calibration_markers(fixed + [vehicle], required_count=4)

        self.assertEqual(len(selected), 4)
        self.assertFalse(any(item is vehicle for item in selected))

    def test_spatial_calibration_keeps_existing_axis_orientation(self):
        transformer = CoordinateTransformer()
        fixed = [
            marker(42, (100, 100)),   # screen top-left
            marker(0, (900, 120)),    # screen top-right
            marker(42, (120, 700)),   # screen bottom-left
            marker(17, (880, 720)),   # screen bottom-right
        ]

        self.assertTrue(transformer.calibrate(fixed))
        expected = {
            (100, 100): (4300.0, 3400.0),
            (900, 120): (300.0, 3400.0),
            (120, 700): (4300.0, 400.0),
            (880, 720): (300.0, 400.0),
        }
        for pixel, world in expected.items():
            actual = transformer.pixel_to_world(*pixel)
            self.assertTrue(np.allclose(actual, world, atol=0.1), (pixel, actual, world))

    def test_rectified_detection_enlarges_far_tag_and_round_trips(self):
        window = MainWindow.__new__(MainWindow)
        window.cfg = get_config()
        window.transformer = CoordinateTransformer()
        window.rectified_detection_geometry = None
        trapezoid = [
            marker(4, (600, 120)),    # top-left
            marker(2, (1200, 120)),   # top-right
            marker(3, (250, 1050)),   # bottom-left
            marker(0, (1550, 1050)),  # bottom-right
        ]
        self.assertTrue(window.transformer.calibrate(trapezoid))
        self.assertTrue(window._prepare_rectified_detection())

        image_to_rectified, rectified_to_image, output_size = window.rectified_detection_geometry
        self.assertEqual(output_size, (1290, 990))
        far_tag = np.asarray(
            [[890, 200], [910, 200], [910, 220], [890, 220]],
            dtype=np.float32,
        ).reshape(-1, 1, 2)
        rectified = cv2.perspectiveTransform(far_tag, image_to_rectified)
        far_width_px = float(np.linalg.norm(rectified[1, 0] - rectified[0, 0]))
        self.assertGreater(far_width_px, 30.0)
        restored = cv2.perspectiveTransform(rectified, rectified_to_image)
        self.assertTrue(np.allclose(restored, far_tag, atol=0.01))

    def test_rectified_pass_recovers_small_far_vehicle_in_same_frame(self):
        window = MainWindow.__new__(MainWindow)
        window.cfg = get_config()
        window.transformer = CoordinateTransformer()
        window.detector = ArUcoDetector()
        window.rectified_detection_geometry = None
        fixed = [
            marker(4, (600, 120), 42),
            marker(2, (1200, 120), 42),
            marker(3, (250, 1050), 90),
            marker(0, (1550, 1050), 90),
        ]
        self.assertTrue(window.transformer.calibrate(fixed))
        self.assertTrue(window._prepare_rectified_detection())

        image = np.full((1200, 1920, 3), 255, dtype=np.uint8)

        def paste_tag(marker_id, center, side):
            tag = cv2.aruco.generateImageMarker(window.detector.aruco_dict, marker_id, side)
            x = int(center[0] - side // 2)
            y = int(center[1] - side // 2)
            image[y:y + side, x:x + side] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)

        for item in fixed:
            side = int(round(np.linalg.norm(item["corners"][1] - item["corners"][0])))
            paste_tag(item["id"], item["center"], side)
        vehicle_center = np.asarray((900.0, 280.0), dtype=np.float32)
        paste_tag(0, vehicle_center, 12)

        _, direct_ids, _ = window.detector.detect_markers(image)
        self.assertEqual(list(direct_ids.flatten()).count(0), 1)
        rectified_instances = window._detect_rectified_marker_instances(image)
        recovered = [item for item in rectified_instances if item["id"] == 0]
        self.assertTrue(
            any(np.linalg.norm(item["center"] - vehicle_center) < 3.0 for item in recovered)
        )

    def test_locked_fixed_id_zero_is_not_selected_as_vehicle(self):
        window = MainWindow.__new__(MainWindow)
        window.vehicle_id = 0
        window.vehicle_track_center_px = None
        fixed_zero = marker(0, (880, 720))
        window.locked_fixed_markers = [
            {
                **fixed_zero,
                "match_radius": 20.0,
            }
        ]
        moving_zero = marker(0, (500, 400))

        self.assertIsNone(window._select_vehicle_marker([fixed_zero]))
        self.assertIs(window._select_vehicle_marker([fixed_zero, moving_zero]), moving_zero)
        self.assertIs(window._select_vehicle_marker([moving_zero]), moving_zero)


if __name__ == "__main__":
    unittest.main()
