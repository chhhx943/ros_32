import math
import unittest


from tools.vehicle.ackermann import MotionSafetyStatus, solve_ackermann
from tools.vehicle.geometry import R3X_GEOMETRY


class AckermannSolverTest(unittest.TestCase):
    def test_straight_motion_preserves_speed_and_has_infinite_radius(self):
        result = solve_ackermann(500.0, 0.0, R3X_GEOMETRY, 600.0)

        self.assertTrue(result.valid)
        self.assertEqual(result.safety_status, MotionSafetyStatus.OK)
        self.assertEqual(result.left_wheel_speed_mm_s, 500.0)
        self.assertEqual(result.right_wheel_speed_mm_s, 500.0)
        self.assertEqual(result.curvature_per_mm, 0.0)
        self.assertTrue(math.isinf(result.rear_axle_radius_signed_mm))
        self.assertTrue(math.isinf(result.rear_axle_radius_abs_mm))
        self.assertIsNone(result.inner_side)
        self.assertIsNone(result.outer_side)

    def test_zero_speed_preserves_nonzero_steering(self):
        result = solve_ackermann(0.0, 200.0, R3X_GEOMETRY, 600.0)

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.applied_steering_rad, 0.2)
        self.assertEqual(
            (result.left_wheel_speed_mm_s, result.right_wheel_speed_mm_s),
            (0.0, 0.0),
        )

    def test_nonfinite_runtime_inputs_return_non_sendable_safe_result(self):
        cases = (
            (math.nan, 0.0, R3X_GEOMETRY, 600.0, "speed_not_finite"),
            (0.0, math.inf, R3X_GEOMETRY, 600.0, "steering_not_finite"),
            (0.0, 0.0, None, 600.0, "invalid_geometry"),
            (0.0, 0.0, R3X_GEOMETRY, 0.0, "invalid_max_wheel_speed"),
            (0.0, 0.0, R3X_GEOMETRY, -1.0, "invalid_max_wheel_speed"),
        )

        for speed, steering, geometry, limit, reason in cases:
            with self.subTest(reason=reason):
                result = solve_ackermann(speed, steering, geometry, limit)
                self.assertFalse(result.valid)
                self.assertEqual(result.reason, reason)
                self.assertEqual(result.left_wheel_speed_mm_s, 0.0)
                self.assertEqual(result.right_wheel_speed_mm_s, 0.0)
                self.assertEqual(result.wheel_speed_scale, 0.0)

    def test_left_and_right_turns_select_the_correct_inner_wheel(self):
        left = solve_ackermann(500.0, 200.0, R3X_GEOMETRY, 600.0)
        right = solve_ackermann(500.0, -200.0, R3X_GEOMETRY, 600.0)

        self.assertGreater(left.curvature_per_mm, 0.0)
        self.assertLess(left.left_wheel_speed_mm_s, left.right_wheel_speed_mm_s)
        self.assertEqual((left.inner_side, left.outer_side), ("left", "right"))
        self.assertLess(right.curvature_per_mm, 0.0)
        self.assertLess(right.right_wheel_speed_mm_s, right.left_wheel_speed_mm_s)
        self.assertEqual((right.inner_side, right.outer_side), ("right", "left"))

    def test_reverse_keeps_geometric_inner_outer_meaning(self):
        result = solve_ackermann(-400.0, 200.0, R3X_GEOMETRY, 600.0)

        self.assertLess(result.left_wheel_speed_mm_s, 0.0)
        self.assertLess(result.right_wheel_speed_mm_s, 0.0)
        self.assertLess(
            abs(result.left_wheel_speed_mm_s),
            abs(result.right_wheel_speed_mm_s),
        )
        self.assertEqual((result.inner_side, result.outer_side), ("left", "right"))

    def test_twenty_degree_limit_has_389mm_radius_and_clamps_both_signs(self):
        for steering in (500.0, -500.0):
            result = solve_ackermann(100.0, steering, R3X_GEOMETRY, 600.0)

            self.assertTrue(result.steering_limited)
            self.assertAlmostEqual(
                abs(result.applied_steering_rad),
                math.radians(20.0),
            )
            self.assertTrue(
                math.isclose(
                    result.rear_axle_radius_abs_mm,
                    389.32,
                    rel_tol=2e-4,
                    abs_tol=0.05,
                )
            )
            self.assertFalse(
                math.isclose(
                    result.rear_axle_radius_abs_mm,
                    350.0,
                    rel_tol=1e-3,
                    abs_tol=0.1,
                )
            )

    def test_349_mrad_is_accepted_and_350_mrad_is_limited(self):
        accepted = solve_ackermann(100.0, 349.0, R3X_GEOMETRY, 600.0)
        limited = solve_ackermann(100.0, 350.0, R3X_GEOMETRY, 600.0)

        self.assertFalse(accepted.steering_limited)
        self.assertAlmostEqual(accepted.applied_steering_rad, 0.349)
        self.assertTrue(limited.steering_limited)
        self.assertAlmostEqual(limited.applied_steering_rad, math.radians(20.0))

    def test_peak_wheel_limit_scales_both_wheels_and_applied_body_speed(self):
        unlimited = solve_ackermann(600.0, 300.0, R3X_GEOMETRY, 1000.0)
        limited = solve_ackermann(600.0, 300.0, R3X_GEOMETRY, 600.0)

        self.assertTrue(limited.wheel_speed_limited)
        self.assertAlmostEqual(
            max(
                abs(limited.left_wheel_speed_mm_s),
                abs(limited.right_wheel_speed_mm_s),
            ),
            600.0,
        )
        self.assertTrue(
            math.isclose(
                unlimited.left_wheel_speed_mm_s
                / unlimited.right_wheel_speed_mm_s,
                limited.left_wheel_speed_mm_s
                / limited.right_wheel_speed_mm_s,
                rel_tol=1e-12,
            )
        )
        self.assertAlmostEqual(
            limited.applied_speed_mm_s,
            limited.requested_speed_mm_s * limited.wheel_speed_scale,
        )

    def test_limit_status_distinguishes_steering_speed_and_both(self):
        steering_only = solve_ackermann(100.0, 500.0, R3X_GEOMETRY, 600.0)
        speed_only = solve_ackermann(600.0, 200.0, R3X_GEOMETRY, 600.0)
        both = solve_ackermann(600.0, 500.0, R3X_GEOMETRY, 600.0)

        self.assertEqual(
            steering_only.safety_status,
            MotionSafetyStatus.STEERING_LIMITED,
        )
        self.assertEqual(speed_only.safety_status, MotionSafetyStatus.SPEED_LIMITED)
        self.assertEqual(both.safety_status, MotionSafetyStatus.LIMITED)

    def test_all_nonfinite_commands_and_speed_limits_are_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(field="speed", value=value):
                self.assertEqual(
                    solve_ackermann(value, 0.0, R3X_GEOMETRY, 600.0).reason,
                    "speed_not_finite",
                )
            with self.subTest(field="steering", value=value):
                self.assertEqual(
                    solve_ackermann(0.0, value, R3X_GEOMETRY, 600.0).reason,
                    "steering_not_finite",
                )
            with self.subTest(field="limit", value=value):
                self.assertEqual(
                    solve_ackermann(0.0, 0.0, R3X_GEOMETRY, value).reason,
                    "invalid_max_wheel_speed",
                )


if __name__ == "__main__":
    unittest.main()
