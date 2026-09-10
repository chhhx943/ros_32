import unittest

from tools.pid.protocol import (
    AUTOTUNE_OPCODE_HEARTBEAT,
    AUTOTUNE_OPCODE_REQUEST_PROMOTE,
    AUTOTUNE_OPCODE_START,
    AUTOTUNE_OPCODE_STOP,
    CMD_AUTOTUNE_CONTROL,
    FB_AUTOTUNE_IDENTITY,
    AutotuneTelemetry,
    CanFrame,
    decode_autotune_control,
    decode_autotune_telemetry,
    encode_autotune_control,
    encode_autotune_telemetry,
)


class AutotuneSafeProtocolTest(unittest.TestCase):
    def test_control_frames_round_trip_session_experiment_and_argument(self):
        for opcode, argument in (
            (AUTOTUNE_OPCODE_START, 100),
            (AUTOTUNE_OPCODE_STOP, 0),
            (AUTOTUNE_OPCODE_HEARTBEAT, 17),
            (AUTOTUNE_OPCODE_REQUEST_PROMOTE, 0),
        ):
            frame = encode_autotune_control(opcode, 0x1234, 0x5678, argument)
            self.assertEqual(frame.arbitration_id, CMD_AUTOTUNE_CONTROL)
            self.assertEqual(len(frame.data), 8)
            self.assertEqual(
                decode_autotune_control(frame.data),
                (opcode, 0x1234, 0x5678, argument),
            )

    def test_snapshot_uses_consistent_sequence_and_preserves_required_fields(self):
        telemetry = AutotuneTelemetry(
            profile_id=0xA1,
            session_id=0x1234,
            experiment_id=0x5678,
            snapshot_seq=9,
            autotune_state=4,
            level=1,
            requested_target=100,
            effective_target=80,
            pwm_limit=150,
            actual_left=76,
            actual_right=74,
            pwm_left=92,
            pwm_right=94,
            pid_left=(0.25, 0.10, 0.0, 92.0),
            pid_right=(0.26, 0.11, 0.0, 94.0),
            stall=0,
            saturation=0,
            overspeed=0,
            oscillation=0,
            abort_reason=0,
            speed_limit_hit=0,
            safety_state=1,
            fault_code=0,
            last_valid_command_age=20,
            stall_time=0,
            saturation_time=0,
            oscillation_count=0,
        )
        frames = encode_autotune_telemetry(telemetry)
        self.assertIn(FB_AUTOTUNE_IDENTITY, [frame.arbitration_id for frame in frames])
        self.assertGreaterEqual(len(frames), 7)
        decoded = decode_autotune_telemetry(frames)
        self.assertEqual(decoded.profile_id, 0xA1)
        self.assertEqual(decoded.session_id, 0x1234)
        self.assertEqual(decoded.experiment_id, 0x5678)
        self.assertEqual(decoded.requested_target, 100)
        self.assertEqual(decoded.effective_target, 80)
        self.assertEqual(decoded.pwm_limit, 150)
        self.assertEqual(decoded.actual_left, 76)
        self.assertEqual(decoded.pwm_right, 94)
        self.assertAlmostEqual(decoded.pid_right[0], 0.26, places=2)
        self.assertEqual(decoded.last_valid_command_age, 20)

    def test_snapshot_rejects_mixed_snapshot_sequence(self):
        telemetry = AutotuneTelemetry(
            profile_id=1, session_id=1, experiment_id=1, snapshot_seq=1,
            autotune_state=1, level=1, requested_target=0, effective_target=0,
            pwm_limit=150, actual_left=0, actual_right=0, pwm_left=0, pwm_right=0,
            pid_left=(0, 0, 0, 0), pid_right=(0, 0, 0, 0), stall=0, saturation=0,
            overspeed=0, oscillation=0, abort_reason=0, speed_limit_hit=0,
            safety_state=1, fault_code=0, last_valid_command_age=0, stall_time=0,
            saturation_time=0, oscillation_count=0,
        )
        frames = encode_autotune_telemetry(telemetry)
        corrupted = list(frames)
        corrupted[1] = CanFrame(corrupted[1].arbitration_id,
                                bytes([corrupted[1].data[0], 2]) + corrupted[1].data[2:])
        with self.assertRaises(ValueError):
            decode_autotune_telemetry(corrupted)


if __name__ == "__main__":
    unittest.main()
