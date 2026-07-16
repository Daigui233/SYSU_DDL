import tempfile
import unittest
from pathlib import Path

from steering_calibration_tool import (
    CalibrationStore,
    SteeringSession,
    aggregate_samples,
    build_fit_report,
    calculate_ackermann,
    polynomial_fit,
)


class AckermannCalculationTests(unittest.TestCase):
    def test_straight_angles_produce_zero_center_angle(self):
        result = calculate_ackermann(0.0, 0.0, 200.0, 140.0)
        self.assertEqual(result["center_angle_deg"], 0.0)
        self.assertEqual(result["curvature_per_m"], 0.0)
        self.assertIsNone(result["turn_radius_mm"])

    def test_consistent_right_turn_recovers_positive_center_angle(self):
        wheelbase = 200.0
        track = 140.0
        radius = 500.0
        left = __import__("math").degrees(__import__("math").atan(wheelbase / (radius + track / 2)))
        right = __import__("math").degrees(__import__("math").atan(wheelbase / (radius - track / 2)))
        result = calculate_ackermann(left, right, wheelbase, track)
        expected = __import__("math").degrees(__import__("math").atan(wheelbase / radius))
        self.assertAlmostEqual(result["center_angle_deg"], expected, places=6)
        self.assertAlmostEqual(result["turn_radius_mm"], radius, places=6)
        self.assertAlmostEqual(result["radius_mismatch_mm"], 0.0, places=6)

    def test_left_turn_uses_negative_sign(self):
        result = calculate_ackermann(-25.0, -30.0, 200.0, 140.0)
        self.assertLess(result["center_angle_deg"], 0.0)
        self.assertLess(result["curvature_per_m"], 0.0)

    def test_rejects_non_acute_angle(self):
        with self.assertRaises(ValueError):
            calculate_ackermann(90.0, 20.0, 200.0, 140.0)


class CalibrationStoreTests(unittest.TestCase):
    def test_store_autosaves_and_reloads(self):
        with tempfile.TemporaryDirectory() as directory:
            store = CalibrationStore(Path(directory))
            sample = store.add_sample(40.0, 690, 12.0, 10.0, "first")
            self.assertEqual(sample["id"], 1)
            self.assertTrue(store.paths.json_path.exists())
            self.assertTrue(store.paths.csv_path.exists())
            self.assertTrue(store.paths.header_path.exists())
            self.assertTrue(store.paths.fit_path.exists())

            loaded = CalibrationStore(Path(directory))
            self.assertEqual(len(loaded.samples), 1)
            self.assertEqual(loaded.samples[0]["note"], "first")
            self.assertEqual(loaded.next_id, 2)

    def test_aggregate_samples_averages_repeated_error(self):
        rows = aggregate_samples(
            [
                {
                    "error": 10.0,
                    "left_angle_deg": 2.0,
                    "right_angle_deg": 1.0,
                    "center_angle_deg": 1.5,
                },
                {
                    "error": 10.0,
                    "left_angle_deg": 4.0,
                    "right_angle_deg": 3.0,
                    "center_angle_deg": 3.5,
                },
            ]
        )
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["left_angle_deg"], 3.0)
        self.assertEqual(rows[0]["right_angle_deg"], 2.0)
        self.assertEqual(rows[0]["sample_count"], 2.0)

    def test_header_exports_inverse_control_function(self):
        with tempfile.TemporaryDirectory() as directory:
            store = CalibrationStore(Path(directory))
            store.add_sample(-80.0, 810, -18.0, -22.0)
            store.add_sample(0.0, 730, 0.0, 0.0)
            store.add_sample(80.0, 650, 22.0, 18.0)
            header = store.header_text()
            self.assertIn("steering_error_from_center_angle", header)
            self.assertIn("STEERING_INVERSE_POINT_COUNT (3U)", header)


class PolynomialFitTests(unittest.TestCase):
    def test_exact_quadratic_fit(self):
        x_values = [-2.0, -1.0, 0.0, 1.0, 2.0]
        y_values = [2.0 + 3.0 * x + 0.5 * x * x for x in x_values]
        model = polynomial_fit(x_values, y_values)
        self.assertTrue(model["available"])
        self.assertAlmostEqual(model["rmse"], 0.0, places=8)
        self.assertAlmostEqual(model["coefficients"][0], 2.0, places=8)
        self.assertAlmostEqual(model["coefficients"][1], 3.0, places=8)
        self.assertAlmostEqual(model["coefficients"][2], 0.5, places=8)

    def test_fit_report_contains_forward_and_inverse_models(self):
        samples = [
            {
                "error": error,
                "left_angle_deg": error * 0.11,
                "right_angle_deg": error * 0.09,
                "center_angle_deg": error * 0.10,
            }
            for error in (-100.0, -50.0, 0.0, 50.0, 100.0)
        ]
        report = build_fit_report(samples)
        self.assertEqual(report["unique_error_count"], 5)
        self.assertTrue(report["models"]["center_angle_from_error"]["available"])
        self.assertTrue(report["models"]["error_from_center_angle"]["available"])


class SteeringAcknowledgementTests(unittest.TestCase):
    def setUp(self):
        self.session = SteeringSession()

    def tearDown(self):
        self.session.shutdown()

    @staticmethod
    def feedback(motor_output=0):
        return {
            "online": True,
            "input_track_error": 40.0,
            "input_target_speed": 0.0,
            "motor_target": 0.0,
            "motor_output": motor_output,
            "age": 0.01,
            "input_age_ms": 10,
            "safety_flags": 0,
            "state": 1,
            "feedback_seq": 12,
        }

    def test_ack_requires_matching_error_and_stopped_motor(self):
        self.assertTrue(self.session._observe_feedback(self.feedback(), 40.0, True))
        self.assertFalse(self.session._observe_feedback(self.feedback(), 41.0, True))
        self.assertFalse(self.session._observe_feedback(self.feedback(motor_output=100), 40.0, True))


if __name__ == "__main__":
    unittest.main()
