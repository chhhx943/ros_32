import tempfile
import unittest
from pathlib import Path

from tools.pid.automated_runner import (
    AutomatedRun,
    HardwarePreflight,
    RunBlocked,
    run_self_loop,
    write_run_report,
)


class AutomatedRunTest(unittest.TestCase):
    def test_missing_can_blocks_before_flash_or_motion(self):
        preflight = HardwarePreflight(
            programmer_available=True,
            target_connected=True,
            can_available=False,
            can_detail="no CAN backend",
            independent_driver_interlock=False,
            profile_id=None,
            level=None,
            pwm_limit=None,
        )
        run = AutomatedRun(preflight)
        with self.assertRaises(RunBlocked):
            run.require_hardware_motion()
        self.assertEqual(run.actions, [])
        self.assertFalse(run.flash_allowed)

    def test_automation_plan_is_l1_only_and_never_promotes(self):
        preflight = HardwarePreflight(
            programmer_available=True,
            target_connected=True,
            can_available=True,
            can_detail="test CAN",
            independent_driver_interlock=False,
            profile_id=0xA1,
            level=1,
            pwm_limit=150,
        )
        run = AutomatedRun(preflight)
        plan = run.motion_plan()
        self.assertEqual(plan, (25, 50, 75, 100))
        self.assertTrue(run.low_energy_only)
        self.assertFalse(run.can_request_promotion)
        self.assertNotIn(300, plan)

    def test_report_contains_blocking_preflight_and_no_false_success(self):
        preflight = HardwarePreflight(
            programmer_available=True,
            target_connected=True,
            can_available=False,
            can_detail="no CAN backend",
            independent_driver_interlock=False,
            profile_id=None,
            level=None,
            pwm_limit=None,
        )
        run = AutomatedRun(preflight)
        with tempfile.TemporaryDirectory() as directory:
            path = write_run_report(Path(directory), run)
            text = path.read_text(encoding="utf-8")
        self.assertIn("BLOCKED", text)
        self.assertIn("no CAN backend", text)
        self.assertNotIn("PID tuning PASS", text)

    def test_self_loop_runs_bounded_pid_search_without_flash_or_hardware_motion(self):
        preflight = HardwarePreflight(
            programmer_available=False,
            target_connected=False,
            can_available=False,
            can_detail="self-loop only",
            independent_driver_interlock=False,
            profile_id=None,
            level=None,
            pwm_limit=None,
        )
        run = AutomatedRun(preflight)
        with tempfile.TemporaryDirectory() as directory:
            run_self_loop(run, Path(directory))
            report = (Path(directory) / "run.json").read_text(encoding="utf-8")
            report_md = (Path(directory) / "AUTOTUNE_SAFE_AUTOMATED_RUN.md").read_text(encoding="utf-8")
        self.assertEqual(run.status, "SELF_LOOP_PASS")
        self.assertFalse(run.flash_allowed)
        self.assertFalse(run.hardware_motion_started)
        self.assertIn("self_loop", report)
        self.assertIn("simulated_motion", report)
        self.assertIn("不证明 MCU", report_md)


if __name__ == "__main__":
    unittest.main()
