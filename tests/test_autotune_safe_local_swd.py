import pathlib
import struct
import subprocess
import sys
import unittest
from unittest import mock

import tools.pid.local_swd_runner as local_swd_runner


ROOT = pathlib.Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools" / "pid" / "local_swd_runner.py"


class AutotuneSafeLocalSwdTest(unittest.TestCase):
    def test_dry_run_is_default_and_only_targets_request_fields(self):
        result = subprocess.run(
            [sys.executable, str(RUNNER), "--request", "start", "--wheel", "1"],
            cwd=ROOT, capture_output=True, text=True, check=True,
        )
        self.assertIn("DRY_RUN", result.stdout)
        self.assertIn("request_magic", result.stdout)
        self.assertIn("request_command", result.stdout)
        self.assertIn("request_wheel", result.stdout)
        self.assertNotIn("--write", result.stdout)

    def test_runner_is_l1_only_and_does_not_expose_level_or_limit_writes(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("L1_TARGET_LIMIT_MMPS = 100", source)
        self.assertIn("L1_PWM_LIMIT = 150", source)
        self.assertIn("g_autotune_safe_local_control", source)
        self.assertIn("--execute", source)
        self.assertIn("boot_handoff_and_resume", source)
        self.assertIn("-r32", source)
        self.assertNotIn("request_level", source)
        self.assertNotIn("request_pwm_limit", source)

    def test_programmer_path_uses_persistent_boot_handoff_before_run_reset(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("g_autotune_safe_local_boot_request", source)
        self.assertIn("BOOT_MAGIC", source)
        self.assertIn("resolve_boot_request_address", source)
        self.assertIn("boot_handoff_and_resume", source)

    def test_stop_plan_writes_zero_request_and_no_motion_command(self):
        result = subprocess.run(
            [sys.executable, str(RUNNER), "--request", "stop"],
            cwd=ROOT, capture_output=True, text=True, check=True,
        )
        self.assertIn("request_command = 2", result.stdout)
        self.assertIn("request_magic = 0x41544C52", result.stdout)

    def test_recover_plan_is_a_zero_stop_request(self):
        result = subprocess.run(
            [sys.executable, str(RUNNER), "--request", "recover", "--sequence", "9"],
            cwd=ROOT, capture_output=True, text=True, check=True,
        )
        self.assertIn("request_command = 3", result.stdout)
        self.assertIn("request_wheel = 0", result.stdout)

    def test_gdb_monitor_uses_async_interrupt_and_bounded_polling(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("class GdbMiSession", source)
        self.assertIn("-gdb-set mi-async on", source)
        self.assertIn("-exec-interrupt --all", source)
        self.assertIn("--monitor-seconds", source)
        self.assertIn("read(1)", source)
        self.assertIn("self._pending.clear()", source)
        self.assertNotIn("monitor reset halt", source)

    def test_monitor_timeout_writes_stop_before_closing_session(self):
        writes = []
        interrupts = []

        class FakeSession:
            def __init__(self, *args, **kwargs):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return None

            def write_word(self, address, value):
                writes.append((address, value))

            def continue_target(self):
                pass

            def interrupt_target(self):
                interrupts.append(True)

            def read_bytes(self, address, count):
                self.assert_count = count
                return struct.pack("<III6B6xI", 0, 0, 9, 0, 0, 0, 0, 0, 0, 0)

        plan = local_swd_runner.build_request_plan("start", 9, 1)
        with mock.patch.object(local_swd_runner, "GdbMiSession", FakeSession):
            local_swd_runner.monitor_local_request(
                pathlib.Path("gdb"), pathlib.Path("elf"), 0x20000550,
                plan, None, 0.06, 50, "localhost", 61234,
            )

        self.assertIn((0x20000554, local_swd_runner.COMMAND_STOP), writes)
        self.assertIn((0x20000550, local_swd_runner.REQUEST_MAGIC), writes)
        self.assertEqual(len(interrupts), 1)

    def test_decode_local_control_header_matches_c_alignment(self):
        data = struct.pack(
            "<III6B6xI", local_swd_runner.REQUEST_MAGIC, 1, 17,
            1, 3, 1, 0, 2, 6, 0x4C52534C,
        )
        decoded = local_swd_runner.decode_local_control_header(data)
        self.assertEqual(decoded["state"], 3)
        self.assertEqual(decoded["candidate_count"], 6)
        self.assertEqual(decoded["status_magic"], 0x4C52534C)


if __name__ == "__main__":
    unittest.main()
