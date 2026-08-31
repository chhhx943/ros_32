import math
import unittest


from tools.vehicle.ackermann import solve_ackermann
from tools.vehicle.command import (
    CommandCycleStatus,
    build_motion_frames,
    send_motion_command,
    send_safe_stop,
)
from tools.vehicle.geometry import R3X_GEOMETRY


class VehicleCommandTest(unittest.TestCase):
    def test_frames_reuse_frozen_v1_layout_and_one_result(self):
        result = solve_ackermann(500.0, 200.0, R3X_GEOMETRY, 600.0)

        steering, wheels = build_motion_frames(result, sequence=0x1234)

        self.assertEqual(
            (steering.arbitration_id, wheels.arbitration_id),
            (0x120, 0x121),
        )
        self.assertEqual(
            steering.data,
            bytes([1, 0x34, 0x12, 1, 200, 0, 0, 0]),
        )
        self.assertEqual(steering.data[:4], wheels.data[:4])
        self.assertEqual(
            int.from_bytes(wheels.data[4:6], "little", signed=True),
            round(result.left_wheel_speed_mm_s),
        )
        self.assertEqual(
            int.from_bytes(wheels.data[6:8], "little", signed=True),
            round(result.right_wheel_speed_mm_s),
        )

    def test_twenty_degree_protocol_quantization_never_exceeds_349_mrad(self):
        positive = solve_ackermann(0.0, 1000.0, R3X_GEOMETRY, 600.0)
        negative = solve_ackermann(0.0, -1000.0, R3X_GEOMETRY, 600.0)

        positive_frame, positive_wheels = build_motion_frames(positive, sequence=1)
        negative_frame, negative_wheels = build_motion_frames(negative, sequence=2)

        self.assertEqual(
            int.from_bytes(positive_frame.data[4:6], "little", signed=True),
            349,
        )
        self.assertEqual(
            int.from_bytes(negative_frame.data[4:6], "little", signed=True),
            -349,
        )
        self.assertEqual(positive_wheels.data[4:], bytes(4))
        self.assertEqual(negative_wheels.data[4:], bytes(4))

    def test_invalid_result_and_invalid_sequence_create_no_normal_frames(self):
        invalid = solve_ackermann(float("nan"), 0.0, R3X_GEOMETRY, 600.0)
        valid = solve_ackermann(0.0, 0.0, R3X_GEOMETRY, 600.0)

        with self.assertRaises(ValueError):
            build_motion_frames(invalid, sequence=1)
        for sequence in (-1, 0x10000, True):
            with self.subTest(sequence=sequence):
                with self.assertRaises(ValueError):
                    build_motion_frames(valid, sequence=sequence)

    def test_successful_motion_cycle_sends_steering_then_wheels_and_commits(self):
        sent = []
        result = solve_ackermann(300.0, 100.0, R3X_GEOMETRY, 600.0)

        cycle = send_motion_command(result, 9, sent.append)

        self.assertEqual([frame.arbitration_id for frame in sent], [0x120, 0x121])
        self.assertEqual(cycle.status, CommandCycleStatus.COMMAND_COMMITTED)
        self.assertTrue(cycle.compute_ok)
        self.assertTrue(cycle.steering_tx_ok)
        self.assertTrue(cycle.wheels_tx_ok)
        self.assertTrue(cycle.command_committed)
        self.assertFalse(cycle.safe_stop_requested)

    def test_partial_send_is_not_committed_and_requests_safe_stop(self):
        sent = []

        def sender(frame):
            sent.append(frame.arbitration_id)
            if frame.arbitration_id == 0x121:
                raise OSError("CAN TX failed")

        result = solve_ackermann(300.0, 100.0, R3X_GEOMETRY, 600.0)

        cycle = send_motion_command(result, 10, sender)

        self.assertEqual(sent, [0x120, 0x121])
        self.assertEqual(cycle.status, CommandCycleStatus.WHEELS_TX_FAILED)
        self.assertTrue(cycle.compute_ok)
        self.assertTrue(cycle.steering_tx_ok)
        self.assertFalse(cycle.wheels_tx_ok)
        self.assertFalse(cycle.command_committed)
        self.assertTrue(cycle.safe_stop_requested)

    def test_steering_failure_stops_normal_cycle_and_requests_safe_stop(self):
        sent = []

        def sender(frame):
            sent.append(frame.arbitration_id)
            raise OSError("CAN controller unavailable")

        result = solve_ackermann(300.0, 100.0, R3X_GEOMETRY, 600.0)

        cycle = send_motion_command(result, 10, sender)

        self.assertEqual(sent, [0x120])
        self.assertEqual(cycle.status, CommandCycleStatus.STEERING_TX_FAILED)
        self.assertFalse(cycle.command_committed)
        self.assertTrue(cycle.safe_stop_requested)

    def test_invalid_result_sends_no_motion_frames_and_requests_safe_stop(self):
        sent = []
        invalid = solve_ackermann(math.nan, 0.0, R3X_GEOMETRY, 600.0)

        cycle = send_motion_command(invalid, 10, sent.append)

        self.assertEqual(sent, [])
        self.assertEqual(cycle.status, CommandCycleStatus.INVALID_RESULT)
        self.assertFalse(cycle.compute_ok)
        self.assertFalse(cycle.command_committed)
        self.assertTrue(cycle.safe_stop_requested)

    def test_safe_stop_uses_fresh_zero_group(self):
        sent = []

        cycle = send_safe_stop(11, sent.append)

        self.assertEqual(cycle.status, CommandCycleStatus.SAFE_STOP_COMMITTED)
        self.assertTrue(cycle.safe_stop_tx_ok)
        self.assertTrue(cycle.command_committed)
        self.assertEqual(
            [frame.data for frame in sent],
            [
                bytes([1, 11, 0, 0, 0, 0, 0, 0]),
                bytes([1, 11, 0, 0, 0, 0, 0, 0]),
            ],
        )

    def test_safe_stop_failure_reports_watchdog_fallback(self):
        attempts = []

        def sender(frame):
            attempts.append(frame.arbitration_id)
            if frame.arbitration_id == 0x121:
                raise OSError("bus off")

        cycle = send_safe_stop(12, sender)

        self.assertEqual(attempts, [0x120, 0x121])
        self.assertEqual(cycle.status, CommandCycleStatus.WATCHDOG_FALLBACK)
        self.assertFalse(cycle.safe_stop_tx_ok)
        self.assertFalse(cycle.command_committed)
        self.assertTrue(cycle.watchdog_fallback)

    def test_every_cycle_emits_requested_applied_and_tx_audit_fields(self):
        records = []
        result = solve_ackermann(700.0, 500.0, R3X_GEOMETRY, 600.0)

        send_motion_command(
            result,
            13,
            lambda frame: None,
            audit_sink=records.append,
            timestamp=123.5,
        )

        self.assertEqual(len(records), 1)
        record = records[0]
        required = (
            "timestamp",
            "requested_speed_mm_s",
            "requested_steering_mrad",
            "applied_speed_mm_s",
            "applied_steering_mrad",
            "curvature_per_mm",
            "rear_axle_radius_mm",
            "left_target_mm_s",
            "right_target_mm_s",
            "inner_side",
            "outer_side",
            "steering_limited",
            "wheel_speed_limited",
            "wheel_speed_scale",
            "result_valid",
            "safety_status",
            "reason",
            "steering_tx_ok",
            "wheels_tx_ok",
            "command_committed",
        )
        for key in required:
            with self.subTest(key=key):
                self.assertIn(key, record)
        self.assertEqual(record["timestamp"], 123.5)
        self.assertEqual(record["requested_speed_mm_s"], 700.0)
        self.assertEqual(record["requested_steering_mrad"], 500.0)
        self.assertEqual(record["applied_steering_mrad"], 349)
        self.assertEqual(record["safety_status"], "limited")

    def test_cycle_without_custom_sink_uses_module_logger(self):
        result = solve_ackermann(100.0, 0.0, R3X_GEOMETRY, 600.0)

        with self.assertLogs("tools.vehicle.command", level="INFO") as captured:
            send_motion_command(result, 14, lambda frame: None)

        self.assertEqual(len(captured.output), 1)
        self.assertIn("command_committed", captured.output[0])

    def test_public_package_exports_complete_control_path(self):
        from tools.vehicle import (
            R3X_GEOMETRY as public_geometry,
            build_motion_frames as public_build_motion_frames,
            load_vehicle_config,
            send_motion_command as public_send_motion_command,
            send_safe_stop as public_send_safe_stop,
            solve_ackermann as public_solve_ackermann,
        )

        config = load_vehicle_config()
        result = public_solve_ackermann(
            300.0,
            100.0,
            config.geometry,
            config.max_wheel_speed_mm_s,
        )

        self.assertEqual(public_geometry, R3X_GEOMETRY)
        self.assertEqual(len(public_build_motion_frames(result, 1)), 2)
        self.assertTrue(callable(public_send_motion_command))
        self.assertTrue(callable(public_send_safe_stop))


if __name__ == "__main__":
    unittest.main()
