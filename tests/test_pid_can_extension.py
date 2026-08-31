import unittest


from tools.pid.model import PIDGains
from tools.pid.protocol import (
    CMD_PID_D,
    CMD_PID_GAINS,
    decode_pid_ack,
    decode_pid_transaction,
    decode_control_output,
    encode_pid_transaction,
    encode_velocity_group,
)
from tools.pid.transport import SimulationTransport, TransportSafetyError


class PidCanExtensionTest(unittest.TestCase):
    def test_pid_ack_decodes_sequence_status_and_axis(self):
        self.assertEqual(
            decode_pid_ack(bytes([1, 7, 0, 1, 3, 0, 0, 0])),
            (7, 1, 3),
        )

    def test_control_output_feedback_decodes_signed_pwm_pair(self):
        self.assertEqual(
            decode_control_output(bytes([1, 4, 0, 3, 0xE8, 0x03, 0x18, 0xFC])),
            (1000, -1000),
        )

    def test_velocity_group_uses_frozen_v1_frame_layout(self):
        frames = encode_velocity_group(9, 600, -300)

        self.assertEqual([frame.arbitration_id for frame in frames], [0x120, 0x121])
        self.assertEqual(frames[0].data, bytes([1, 9, 0, 1, 0, 0, 0, 0]))
        self.assertEqual(frames[1].data, bytes([1, 9, 0, 1, 0x58, 0x02, 0xD4, 0xFE]))

    def test_pid_transaction_round_trips_q8_8_gains(self):
        gains = PIDGains(0.25, 0.75, 0.125)

        frames = encode_pid_transaction(7, gains)

        self.assertEqual([frame.arbitration_id for frame in frames], [CMD_PID_GAINS, CMD_PID_D])
        self.assertEqual(frames[0].data[0:4], bytes([1, 7, 0, 3]))
        self.assertEqual(decode_pid_transaction(frames), gains)

    def test_simulator_exposes_closed_loop_samples_and_requires_stop_before_gain_change(self):
        transport = SimulationTransport(dt_s=0.01)
        transport.set_pid(PIDGains(0.2, 0.6, 0.0))
        transport.start_trial()
        with self.assertRaises(TransportSafetyError):
            transport.set_pid(PIDGains(0.3, 0.6, 0.0))

        samples = transport.run_step(target=100.0, duration_s=0.5)
        transport.stop()

        self.assertGreater(len(samples), 10)
        self.assertGreater(samples[-1].actual, samples[0].actual)
        self.assertEqual(samples[-1].target, 100.0)
        self.assertEqual(samples[-1].kp, 0.2)
        self.assertIsNone(samples[-1].current_a)
        transport.set_pid(PIDGains(0.3, 0.6, 0.0))


if __name__ == "__main__":
    unittest.main()
