import unittest


from tools.pid.experiment import ExperimentRunner, Scenario
from tools.pid.model import PIDGains, Sample
from tools.pid.transport import SimulationTransport, TransportError


class FaultTransport(SimulationTransport):
    def run_step(self, target, duration_s):
        rows = super().run_step(target, duration_s)
        return [Sample(row.time_s, row.target, row.actual, row.control_output,
                       row.kp, row.ki, row.kd, safety_fault="mcu_stall")
                for row in rows]


class CommunicationTransport(SimulationTransport):
    def run_step(self, target, duration_s):
        raise TransportError("feedback timeout")


class SetPidCommunicationTransport(SimulationTransport):
    def set_pid(self, gains):
        if gains == PIDGains(0.3, 0.7, 0.0):
            raise TransportError("PID acknowledgement timeout")
        super().set_pid(gains)


class PidExperimentTest(unittest.TestCase):
    def test_candidate_trial_records_required_fields_and_restores_safe_pid(self):
        initial = PIDGains(0.2, 0.6, 0.0)
        transport = SimulationTransport()
        runner = ExperimentRunner(transport, initial)

        result = runner.run_candidate(
            PIDGains(0.3, 0.7, 0.0),
            [Scenario("low_speed_step", 100.0, 0.4)],
        )

        self.assertFalse(result.aborted)
        self.assertIsNotNone(result.metrics)
        self.assertGreater(len(result.samples), 10)
        row = result.samples[0].as_dict()
        for field in ("kp", "ki", "kd", "target", "actual", "error", "control_output", "time_s"):
            self.assertIn(field, row)
        self.assertEqual(runner.best_pid, PIDGains(0.3, 0.7, 0.0))
        self.assertEqual(runner.current_pid, runner.best_pid)
        self.assertEqual(len(runner.experiment_history), 1)

    def test_safety_fault_aborts_and_does_not_replace_last_safe_pid(self):
        initial = PIDGains(0.2, 0.6, 0.0)
        runner = ExperimentRunner(FaultTransport(), initial)

        result = runner.run_candidate(
            PIDGains(0.9, 0.9, 0.0),
            [Scenario("fault_injection", 100.0, 0.2)],
        )

        self.assertTrue(result.aborted)
        self.assertEqual(result.abort_reason, "mcu_fault:mcu_stall")
        self.assertEqual(runner.current_pid, initial)
        self.assertEqual(runner.best_pid, initial)
        self.assertEqual(runner.experiment_history[0]["status"], "aborted")

    def test_transport_error_aborts_and_is_recorded(self):
        initial = PIDGains(0.2, 0.6, 0.0)
        runner = ExperimentRunner(CommunicationTransport(), initial)

        result = runner.run_candidate(
            PIDGains(0.3, 0.7, 0.0),
            [Scenario("feedback_timeout", 100.0, 0.2)],
        )

        self.assertTrue(result.aborted)
        self.assertEqual(result.abort_reason, "transport:feedback timeout")
        self.assertEqual(runner.experiment_history[0]["status"], "aborted")

    def test_pid_update_error_aborts_and_is_recorded(self):
        initial = PIDGains(0.2, 0.6, 0.0)
        runner = ExperimentRunner(SetPidCommunicationTransport(), initial)

        result = runner.run_candidate(
            PIDGains(0.3, 0.7, 0.0),
            [Scenario("pid_ack_timeout", 100.0, 0.2)],
        )

        self.assertTrue(result.aborted)
        self.assertEqual(result.abort_reason, "transport:PID acknowledgement timeout")
        self.assertEqual(runner.current_pid, initial)
        self.assertEqual(runner.experiment_history[0]["status"], "aborted")


if __name__ == "__main__":
    unittest.main()
