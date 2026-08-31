import math
import unittest


from tools.pid.analysis import analyze_response, score_result
from tools.pid.model import PIDGains, Sample


class PidAutotuneAnalysisTest(unittest.TestCase):
    def test_step_metrics_capture_rise_overshoot_settling_and_integrals(self):
        samples = [
            Sample(0.00, 100.0, 0.0, 500.0, 0.2, 0.6, 0.0),
            Sample(0.10, 100.0, 50.0, 400.0, 0.2, 0.6, 0.0),
            Sample(0.20, 100.0, 90.0, 300.0, 0.2, 0.6, 0.0),
            Sample(0.30, 100.0, 110.0, 200.0, 0.2, 0.6, 0.0),
            Sample(0.40, 100.0, 97.0, 120.0, 0.2, 0.6, 0.0),
            Sample(0.50, 100.0, 102.0, 100.0, 0.2, 0.6, 0.0),
            Sample(0.60, 100.0, 100.0, 100.0, 0.2, 0.6, 0.0),
        ]

        metrics = analyze_response(samples)

        self.assertAlmostEqual(metrics.rise_time_s, 0.20)
        self.assertAlmostEqual(metrics.overshoot_percent, 10.0)
        self.assertAlmostEqual(metrics.settling_time_s, 0.50)
        self.assertAlmostEqual(metrics.steady_state_error, 0.0)
        self.assertGreater(metrics.iae, 0.0)
        self.assertGreater(metrics.ise, 0.0)
        self.assertEqual(metrics.oscillation, 2)

    def test_score_prefers_fast_low_error_non_oscillatory_response(self):
        good = analyze_response([
            Sample(0.0, 100.0, 0.0, 400.0, 0.2, 0.4, 0.0),
            Sample(0.1, 100.0, 92.0, 180.0, 0.2, 0.4, 0.0),
            Sample(0.2, 100.0, 99.0, 110.0, 0.2, 0.4, 0.0),
            Sample(0.3, 100.0, 100.0, 100.0, 0.2, 0.4, 0.0),
        ])
        bad = analyze_response([
            Sample(0.0, 100.0, 0.0, 1000.0, 0.2, 0.4, 0.0),
            Sample(0.1, 100.0, 140.0, 1000.0, 0.2, 0.4, 0.0),
            Sample(0.2, 100.0, 60.0, -1000.0, 0.2, 0.4, 0.0),
            Sample(0.3, 100.0, 130.0, 1000.0, 0.2, 0.4, 0.0),
        ])

        self.assertTrue(math.isfinite(score_result(good)))
        self.assertLess(score_result(good), score_result(bad))

    def test_pid_gains_are_non_negative_and_serializable(self):
        gains = PIDGains(0.2, 0.6, 0.0)
        self.assertEqual(gains.as_dict(), {"kp": 0.2, "ki": 0.6, "kd": 0.0})
        with self.assertRaises(ValueError):
            PIDGains(-0.1, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
