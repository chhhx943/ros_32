import tempfile
import unittest
import subprocess
from pathlib import Path
from unittest import mock

import tools.pid.local_swd_runner as runner


class FakeBackend:
    def __init__(self, attach_generation=None, fail_at=None):
        self.events = []
        self.attach_generation = attach_generation
        self.fail_at = fail_at
        self.read_count = 0

    def build(self):
        self.events.append("build")

    def program_flash_verify_run(self):
        self.events.append("programmer_start")
        self.events.append("verify")
        self.events.append("run")
        self.events.append("programmer_exit")

    def attach_no_reset(self):
        self.events.append("gdb_attach")

    def read_boot_identity(self):
        self.read_count += 1
        generation = self.attach_generation if self.read_count > 1 else 7
        return {"boot_generation": generation, "boot_state": runner.BOOT_READY}

    def wait_ready(self):
        self.events.append("ready")

    def create_session(self):
        self.events.append("create")

    def start_experiment(self):
        self.events.append("start")

    def poll_terminal(self):
        self.events.append("terminal")
        if self.fail_at == "poll":
            raise RuntimeError("synthetic poll failure")

    def capture_result(self):
        self.events.append("capture")

    def ack_result(self):
        self.events.append("ack")

    def restore_debug(self):
        self.events.append("restore")


class AutotuneSafeRunnerOrchestrationTest(unittest.TestCase):
    def test_restore_failure_classifier_is_stage_specific(self):
        self.assertEqual(
            runner.classify_restore_failure("gdb_process"),
            "RESTORE_DEBUG_GDB_DEAD",
        )
        self.assertEqual(
            runner.classify_restore_failure("target_connection"),
            "RESTORE_DEBUG_TARGET_DISCONNECTED",
        )
        self.assertEqual(
            runner.classify_restore_failure("halt"),
            "RESTORE_DEBUG_HALT_FAIL",
        )
        self.assertEqual(
            runner.classify_restore_failure("stlink_release"),
            "RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT",
        )
        self.assertEqual(
            runner.classify_restore_failure("programmer", "verify failed"),
            "RESTORE_DEBUG_VERIFY_FAIL",
        )

    def test_command_timeout_is_artifacted_as_orchestration_failure(self):
        command_runner = runner.CommandRunner()
        timeout = subprocess.TimeoutExpired(
            ["tool.exe", "--run"], 1.0, output="partial stdout",
            stderr="partial stderr",
        )
        with mock.patch.object(runner.subprocess, "run", side_effect=timeout):
            with self.assertRaises(runner.OrchestrationFailure) as caught:
                command_runner.run(["tool.exe", "--run"], timeout_s=1.0)
        self.assertEqual(caught.exception.code, "COMMAND_TIMEOUT")
        self.assertEqual(len(command_runner.evidence), 1)
        record = command_runner.evidence[0]
        self.assertTrue(record.timed_out)
        self.assertEqual(record.timeout_s, 1.0)
        self.assertEqual(record.stdout, "partial stdout")
        self.assertEqual(record.stderr, "partial stderr")

    def test_hardware_artifact_contains_reconstructable_stage_logs(self):
        backend = runner.HardwareSmokeBackend(
            Path("missing-smoke.elf"), Path("missing-debug.elf"),
            Path("missing-programmer.exe"), Path("missing-gdb.exe"),
            Path("missing-server.exe"),
        )
        result = runner.OrchestrationResult(
            "RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT",
            "ORCHESTRATION_FAILURE",
            "probe failed",
        )
        with tempfile.TemporaryDirectory() as directory:
            artifact = runner.save_hardware_artifact(
                Path(directory), 1, backend, result,
            )
            for filename in (
                "restore_debug.json", "gdb_stdout.log", "gdb_stderr.log",
                "gdb_server_stdout.log", "gdb_server_stderr.log",
                "programmer_stdout.log", "programmer_stderr.log",
                "command_timeline.json", "process_snapshot.txt",
            ):
                self.assertTrue((artifact / filename).exists(), filename)

    def test_restore_retries_stlink_reopen_then_writes_debug_once(self):
        class FakeCommandRunner:
            def __init__(self):
                self.evidence = []
                self.calls = []
                self.results = [
                    runner.ProcessEvidence(["STM32_Programmer_CLI.exe"],
                                           returncode=1, stdout="DEV_USB_COMM_ERR"),
                    runner.ProcessEvidence(["STM32_Programmer_CLI.exe"],
                                           returncode=0),
                    runner.ProcessEvidence(["STM32_Programmer_CLI.exe"],
                                           returncode=0,
                                           stdout="Download verified successfully\nCore run"),
                ]

            def run(self, argv, timeout_s=60.0):
                self.calls.append((list(argv), timeout_s))
                result = self.results.pop(0)
                result.argv = list(argv)
                self.evidence.append(result)
                return result

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            programmer = root / "STM32_Programmer_CLI.exe"
            debug = root / "debug.elf"
            programmer.touch()
            debug.touch()
            backend = runner.HardwareSmokeBackend(
                root / "smoke.elf", debug, programmer, root / "gdb.exe",
                root / "server.exe",
            )
            fake = FakeCommandRunner()
            backend.commands = fake
            backend.restore_debug()
            restore = backend.evidence["restore_debug"]
            self.assertEqual(restore["status"], "OK")
            self.assertEqual(len(restore["attempts"]), 2)
            self.assertEqual(restore["attempts"][0]["failure"]["code"],
                             "RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT")
            self.assertEqual(sum("-w" in call[0] for call in fake.calls), 1)

    def test_restore_all_reopen_attempts_fail_with_release_timeout(self):
        class FakeCommandRunner:
            def __init__(self):
                self.evidence = []

            def run(self, argv, timeout_s=60.0):
                record = runner.ProcessEvidence(list(argv), returncode=1,
                                                stdout="DEV_USB_COMM_ERR")
                self.evidence.append(record)
                return record

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            programmer = root / "STM32_Programmer_CLI.exe"
            debug = root / "debug.elf"
            programmer.touch()
            debug.touch()
            backend = runner.HardwareSmokeBackend(
                root / "smoke.elf", debug, programmer, root / "gdb.exe",
                root / "server.exe",
            )
            backend.commands = FakeCommandRunner()
            with self.assertRaises(runner.OrchestrationFailure) as caught:
                backend.restore_debug()
            self.assertEqual(caught.exception.code,
                             "RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT")
            self.assertEqual(len(backend.evidence["restore_debug"]["attempts"]), 3)
            self.assertEqual(backend.evidence["restore_debug"]["status"], "FAIL")

    def test_restore_does_not_write_when_gdb_died(self):
        class DeadGdb:
            target_attached = True

            def is_alive(self):
                return False

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            programmer = root / "STM32_Programmer_CLI.exe"
            debug = root / "debug.elf"
            programmer.touch()
            debug.touch()
            backend = runner.HardwareSmokeBackend(
                root / "smoke.elf", debug, programmer, root / "gdb.exe",
                root / "server.exe",
            )
            backend.gdb_session = DeadGdb()
            with mock.patch.object(backend, "close") as close:
                with self.assertRaises(runner.OrchestrationFailure) as caught:
                    backend.restore_debug()
            self.assertEqual(caught.exception.code, "RESTORE_DEBUG_GDB_DEAD")
            close.assert_called_once()

    def test_detach_is_explicit_and_idempotent(self):
        session = runner.GdbMiSession(Path("gdb"), Path("elf"))
        session.process = mock.Mock()
        session.process.poll.return_value = None
        session.target_attached = True
        session._send = mock.Mock(return_value="1^done")
        session.detach_target()
        session.detach_target()
        session._send.assert_called_once_with("-target-detach")

    def test_integrity_failure_report_cannot_claim_motor_failure(self):
        text = runner.render_report(failure_code="DATA_INTEGRITY_FAIL")
        self.assertIn("ORCHESTRATION_FAILURE", text)
        self.assertNotIn("NO_MOTION", text)
        self.assertNotIn("ENCODER_FAIL", text)
        self.assertNotIn("PID_FAIL", text)

    def test_runner_waits_for_programmer_exit_before_attach(self):
        backend = FakeBackend()
        runner.SessionOrchestrator(backend).run_smoke_once()
        self.assertLess(backend.events.index("programmer_exit"),
                        backend.events.index("gdb_attach"))

    def test_attach_generation_change_is_orchestration_failure(self):
        backend = FakeBackend(attach_generation=8)
        result = runner.SessionOrchestrator(backend).run_smoke_once()
        self.assertEqual(result.code, "ATTACH_RESET_DETECTED")
        self.assertEqual(result.category, "ORCHESTRATION_FAILURE")
        self.assertIn("restore", backend.events)

    def test_runner_restores_debug_after_capture_failure(self):
        backend = FakeBackend(fail_at="poll")
        result = runner.SessionOrchestrator(backend).run_smoke_once()
        self.assertEqual(result.category, "ORCHESTRATION_FAILURE")
        self.assertIn("restore", backend.events)


if __name__ == "__main__":
    unittest.main()
