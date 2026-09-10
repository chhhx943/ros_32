import unittest

from tools.pid.experiment import ExperimentRunner, Scenario
from tools.pid.model import PIDGains
from tools.pid.transport import SimulationTransport


class AutotuneSafeRunnerTest(unittest.TestCase):
    def test_l1_runner_ramps_target_caps_pwm_and_records_required_safety_fields(self):
        initial = PIDGains(0.2, 0.6, 0.0)
        runner = ExperimentRunner(SimulationTransport(), initial)

        result = runner.run_candidate(
            PIDGains(0.3, 0.7, 0.0),
            [Scenario("l1_forward", 100.0, 0.4)],
        )

        self.assertFalse(result.aborted)
        self.assertGreater(len(result.samples), 10)
        self.assertGreater(result.samples[0].target, 0.0)
        self.assertLess(result.samples[0].target, 100.0)
        self.assertLessEqual(max(abs(row.control_output or 0.0) for row in result.samples), 150.0)
        record = result.as_dict()
        for field in (
            "abort_reason", "max_pwm", "max_speed", "saturation_time",
            "stall_time", "oscillation_count", "speed_limit_hit", "safety_state",
            "level", "requested_target", "effective_target", "pwm_limit",
            "session_id", "experiment_id",
        ):
            self.assertIn(field, record)

    def test_reverse_and_high_speed_requests_are_rejected_before_motion(self):
        runner = ExperimentRunner(SimulationTransport(), PIDGains(0.2, 0.6, 0.0))

        reverse = runner.run_candidate(
            PIDGains(0.2, 0.6, 0.0), [Scenario("reverse", -100.0, 0.2)]
        )
        high = runner.run_candidate(
            PIDGains(0.2, 0.6, 0.0), [Scenario("high", 300.0, 0.2)]
        )

        self.assertTrue(reverse.aborted)
        self.assertEqual(reverse.abort_reason, "target_direction_not_allowed")
        self.assertTrue(high.aborted)
        self.assertEqual(high.abort_reason, "target_limit")

    def test_failed_candidate_never_replaces_best_safe_or_enters_leaderboard(self):
        initial = PIDGains(0.2, 0.6, 0.0)
        runner = ExperimentRunner(SimulationTransport(), initial)

        result = runner.run_candidate(
            PIDGains(2.0, 0.6, 0.0), [Scenario("unsafe_pid", 100.0, 0.2)]
        )

        self.assertTrue(result.aborted)
        self.assertEqual(result.abort_reason, "pid_step_limit")
        self.assertEqual(runner.best_pid, initial)
        self.assertEqual(runner.current_pid, initial)
        self.assertNotIn("best_score", result.as_dict())


if __name__ == "__main__":
    unittest.main()
