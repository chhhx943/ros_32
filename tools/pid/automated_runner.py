"""AutotuneSafe automated build/flash/run/restore orchestration.

The MCU remains the final actuator gate.  This module is deliberately a host
orchestrator, not a replacement for MCU safety checks.  Hardware execution is
refused until the programmer, CAN path, profile identity, and L1 envelope are
verified.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[2]
PROGRAMMER_DEFAULT = Path(
    r"D:\stm32cubeclt\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)
L1_TARGET_LIMIT_MMPS = 100
L1_PWM_LIMIT = 150

if __package__ in (None, ""):
    sys.path.insert(0, str(ROOT))

from tools.pid.transport import CAN_V1_BITRATE


class RunBlocked(RuntimeError):
    """A hard preflight or recovery condition prevented hardware execution."""


@dataclass(frozen=True)
class HardwarePreflight:
    programmer_available: bool
    target_connected: bool
    can_available: bool
    can_detail: str
    independent_driver_interlock: bool
    profile_id: Optional[int]
    level: Optional[int]
    pwm_limit: Optional[int]

    @property
    def safe_for_l1(self) -> bool:
        return (
            self.programmer_available
            and self.target_connected
            and self.can_available
            and self.profile_id == 0xA1
            and self.level == 1
            and self.pwm_limit is not None
            and self.pwm_limit <= L1_PWM_LIMIT
        )


class AutomatedRun:
    """Stateful evidence record and hard policy for one automated run."""

    def __init__(self, preflight: HardwarePreflight):
        self.preflight = preflight
        self.actions: List[str] = []
        self.flash_allowed = False
        self.hardware_motion_started = False
        self.safe_stop_confirmed = False
        self.status = "BLOCKED"
        self.evidence: Dict[str, Any] = {}

    @property
    def low_energy_only(self) -> bool:
        return not self.preflight.independent_driver_interlock

    @property
    def can_request_promotion(self) -> bool:
        # No L2+ compile-time envelope is currently configured or authorized.
        return False

    def require_hardware_motion(self) -> None:
        if not self.preflight.can_available:
            raise RunBlocked(self.preflight.can_detail or "CAN transport unavailable")
        if not self.preflight.safe_for_l1:
            raise RunBlocked("MCU AutotuneSafe L1 identity/envelope preflight failed")
        self.flash_allowed = True

    def motion_plan(self) -> Tuple[int, ...]:
        # Fixed, one-direction, ramped L1 probe. Never add H5/L2 targets here.
        return (25, 50, 75, 100)

    def mark(self, action: str) -> None:
        self.actions.append(action)

    def mark_motion_started(self) -> None:
        self.require_hardware_motion()
        self.hardware_motion_started = True
        self.status = "RUNNING_L1"
        self.mark("L1 motion probe started")

    def mark_safe_stop(self) -> None:
        self.safe_stop_confirmed = True
        self.mark("STOP sent and MCU stillness confirmed")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _run(command: Sequence[str], cwd: Path = ROOT, timeout_s: float = 180.0) -> Dict[str, Any]:
    completed = subprocess.run(
        list(command), cwd=str(cwd), capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=timeout_s, check=False,
    )
    return {
        "command": list(command),
        "returncode": completed.returncode,
        "stdout": completed.stdout[-12000:],
        "stderr": completed.stderr[-12000:],
    }


def run_software_validation() -> Dict[str, Any]:
    results = {
        "python_regression": _run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-q"]),
        "debug_build": _run(["cmake", "--build", "--preset", "Debug"], timeout_s=240.0),
        "autotune_safe_build": _run(["cmake", "--build", "--preset", "AutotuneSafe"], timeout_s=240.0),
    }
    results["passed"] = all(row["returncode"] == 0 for row in results.values() if isinstance(row, dict))
    for image in (
        ROOT / "build" / "Debug" / "ros.elf",
        ROOT / "build" / "AutotuneSafe" / "ros.elf",
        ROOT / "build" / "AutotuneSafe" / "ros_autotune_safe.bin",
    ):
        if image.exists():
            results.setdefault("images", {})[str(image.relative_to(ROOT))] = {
                "bytes": image.stat().st_size,
                "sha256": _sha256(image),
            }
    return results


def probe_stlink(programmer: Path) -> Tuple[bool, str]:
    if not programmer.exists():
        return False, "STM32_Programmer_CLI not found"
    result = _run([str(programmer), "-c", "port=SWD"], timeout_s=30.0)
    output = result["stdout"] + "\n" + result["stderr"]
    connected = result["returncode"] == 0 and "Device ID" in output and "Flash size" in output
    return connected, output[-4000:]


def probe_can(interface: str, channel: str, bitrate: int) -> Tuple[bool, str]:
    try:
        import can  # type: ignore
    except ImportError:
        return False, "python-can is not installed"
    bus = None
    try:
        bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
        return True, "CAN bus opened: %s/%s" % (interface, channel)
    except Exception as exc:  # backend-specific exceptions vary
        return False, "%s/%s: %s" % (interface, channel, exc)
    finally:
        if bus is not None:
            shutdown = getattr(bus, "shutdown", None)
            if shutdown is not None:
                shutdown()


def write_run_report(output_dir: Path, run: AutomatedRun,
                     software: Optional[Dict[str, Any]] = None) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "status": run.status,
        "preflight": asdict(run.preflight),
        "actions": list(run.actions),
        "flash_allowed": run.flash_allowed,
        "hardware_motion_started": run.hardware_motion_started,
        "safe_stop_confirmed": run.safe_stop_confirmed,
        "low_energy_only": run.low_energy_only,
        "motion_plan_mmps": list(run.motion_plan()),
        "evidence": run.evidence,
        "software": software,
    }
    (output_dir / "run.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    report = [
        "# AUTOTUNE_SAFE_AUTOMATED_RUN",
        "",
        "状态：**%s**" % run.status,
        "",
        "本报告由自动执行器生成。MCU AutotuneSafe 仍是最终 actuator gate；" \
        "host/AI 不能关闭、放宽或直接设置 level。",
        "",
        "## Preflight",
        "",
        "```json",
        json.dumps(asdict(run.preflight), indent=2, ensure_ascii=False),
        "```",
        "",
        "## 硬策略",
        "",
        "- L1-only probe: `0 → 25 → 50 → 75 → 100 mm/s → 0`。",
        "- PWM hard limit: `≤150‰`；target 只允许正方向；不请求 H5 或 L2+。",
        "- 无独立 driver-enable 互锁时保持 low-energy-only，不扩大 envelope。",
        "- 未确认 STOP + stillness，不恢复 Debug。",
        "- `SELF_LOOP_PASS` 仅证明 host 模型/协议流程；不证明 MCU、编码器、TB6612、电流或温升安全。",
        "",
        "## Actions",
        "",
    ]
    report.extend("- " + item for item in run.actions)
    if not run.actions:
        report.append("- （尚未执行硬件动作）")
    if software is not None:
        report.extend(["", "## Software evidence", "", "```json",
                       json.dumps(software, indent=2, ensure_ascii=False), "```"])
    path = output_dir / "AUTOTUNE_SAFE_AUTOMATED_RUN.md"
    path.write_text("\n".join(report) + "\n", encoding="utf-8")
    return path


def _timestamp_dir(root: Path) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    candidate = root / stamp
    suffix = 0
    while candidate.exists():
        suffix += 1
        candidate = root / (stamp + "-" + str(suffix))
    return candidate


def flash_image(programmer: Path, image: Path) -> Dict[str, Any]:
    """Flash and verify one image; no erase/option-byte operation is issued."""
    if not image.exists():
        raise RunBlocked("firmware image missing: " + str(image))
    result = _run([
        str(programmer), "-c", "port=SWD", "freq=4000",
        "-w", str(image), "-v", "-rst",
    ], timeout_s=60.0)
    if result["returncode"] != 0:
        raise RunBlocked("flash/verify failed: " + result["stderr"][-1000:])
    return result


def run_self_loop(run: AutomatedRun, output_dir: Path) -> None:
    """Exercise the host PID/search stack with a deterministic no-hardware model."""
    from tools.pid.experiment import ExperimentRunner, Scenario
    from tools.pid.model import PIDGains
    from tools.pid.search import generate_candidates
    from tools.pid.transport import SimulationTransport

    transport = SimulationTransport()
    runner = ExperimentRunner(transport, PIDGains(0.2, 0.6, 0.0))
    scenarios = [Scenario("self_loop_%d" % target, float(target), 0.25)
                 for target in run.motion_plan()]
    results = []
    run.status = "SELF_LOOP_RUNNING"
    run.mark("self-loop: no Flash, no CAN, no physical actuator")
    for candidate in generate_candidates(runner.best_pid, {
            "kp": [0.0, 1.0], "ki": [0.0, 2.0], "kd": [0.0, 0.5]}, 5, 7):
        result = runner.run_candidate(candidate, scenarios)
        results.append(result.as_dict())
        if result.aborted:
            run.status = "SELF_LOOP_FAIL"
            run.evidence["self_loop"] = {"simulated_motion": True, "experiments": results}
            write_run_report(output_dir, run)
            return
    run.evidence["self_loop"] = {
        "simulated_motion": True,
        "experiments": results,
        "best_pid": runner.best_pid.as_dict(),
        "best_score": runner.best_score,
        "simulated_stop_confirmed": True,
        "mcu_safety_proven": False,
    }
    run.mark("self-loop: bounded P-only → PI → optional D search completed")
    run.mark("self-loop: simulated target ramp/PWM limit/slew and STOP completed")
    run.status = "SELF_LOOP_PASS"
    write_run_report(output_dir, run)
    (output_dir / "self_loop_results.json").write_text(
        json.dumps(run.evidence["self_loop"], indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def run_hardware_session(run: AutomatedRun, programmer: Path, interface: str,
                         channel: str, bitrate: int) -> None:
    """Run only the fixed L1 probe and bounded PID search, then restore Debug."""
    from tools.pid.experiment import ExperimentRunner, Scenario
    from tools.pid.model import PIDGains
    from tools.pid.protocol import AUTOTUNE_PROFILE_ID, encode_autotune_control, \
        AUTOTUNE_OPCODE_START
    from tools.pid.search import generate_candidates
    from tools.pid.transport import PythonCanTransport, TransportError

    safe_image = ROOT / "build" / "AutotuneSafe" / "ros.elf"
    debug_image = ROOT / "build" / "Debug" / "ros.elf"
    flashed_safe = False
    transport = None
    try:
        run.mark("flash AutotuneSafe image with SWD and verify")
        run.evidence["safe_flash"] = flash_image(programmer, safe_image)
        flashed_safe = True
        transport = PythonCanTransport(channel=channel, interface=interface, bitrate=bitrate)
        snapshot = transport.receive_autotune_snapshot(timeout_s=2.0)
        run.evidence["post_flash_snapshot"] = asdict(snapshot)
        run.preflight = replace(run.preflight, profile_id=snapshot.profile_id,
                                level=snapshot.level, pwm_limit=snapshot.pwm_limit)
        if snapshot.profile_id != AUTOTUNE_PROFILE_ID or snapshot.level != 1 or \
                snapshot.pwm_limit > L1_PWM_LIMIT or snapshot.fault_code != 0:
            raise RunBlocked("post-flash AutotuneSafe L1 preflight failed")
        run.require_hardware_motion()

        # Malicious high target must be rejected before any actuator authorization.
        probe_session, probe_experiment = 0x6001, 0x6001
        transport._send(encode_autotune_control(
            AUTOTUNE_OPCODE_START, probe_session, probe_experiment, 300))
        rejected = transport.receive_autotune_snapshot(timeout_s=1.0)
        run.evidence["high_target_rejection"] = {
            "target": 300, "state": rejected.autotune_state,
            "level": rejected.level, "pwm_limit": rejected.pwm_limit,
        }
        if rejected.session_id == probe_session and rejected.experiment_id == probe_experiment:
            raise RunBlocked("MCU accepted illegal 300 mm/s START")

        initial = PIDGains(0.2, 0.6, 0.0)
        runner = ExperimentRunner(transport, initial)
        scenarios = [Scenario("l1_probe_%d" % target, float(target), 0.25)
                     for target in run.motion_plan()]
        run.mark_motion_started()
        run.evidence["motion_plan"] = list(run.motion_plan())
        for candidate in generate_candidates(initial, {
                "kp": [0.0, 1.0], "ki": [0.0, 2.0], "kd": [0.0, 0.5]}, 2, 7):
            result = runner.run_candidate(candidate, scenarios)
            run.evidence.setdefault("experiments", []).append(result.as_dict())
            if result.aborted:
                raise RunBlocked("MCU aborted candidate: " + str(result.abort_reason))
        run.mark_safe_stop()
        run.status = "L1_COMPLETE"
    except (RunBlocked, TransportError) as exc:
        run.status = "BLOCKED_RUNTIME: " + str(exc)
        raise
    finally:
        if transport is not None:
            try:
                transport.stop()
                if transport.wait_until_still(timeout_s=1.0):
                    run.mark_safe_stop()
            except Exception as exc:  # preserve the original failure and refuse restore
                run.evidence["stop_error"] = str(exc)
        if flashed_safe and run.safe_stop_confirmed:
            run.mark("restore Debug image with SWD and verify")
            run.evidence["debug_restore"] = flash_image(programmer, debug_image)
        elif flashed_safe:
            run.status = "BLOCKED_RECOVERY: MCU stillness was not confirmed; Debug not flashed"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="AutotuneSafe automated build/flash/tune/restore runner")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=CAN_V1_BITRATE)
    parser.add_argument("--programmer", type=Path, default=PROGRAMMER_DEFAULT)
    parser.add_argument("--output-root", type=Path, default=ROOT / "autotune_runs")
    parser.add_argument("--skip-software", action="store_true")
    parser.add_argument("--self-loop", action="store_true",
                        help="run deterministic host simulation only; never flash or drive hardware")
    args = parser.parse_args(argv)

    output_dir = _timestamp_dir(args.output_root)
    software = None if args.skip_software else run_software_validation()
    programmer_ok, programmer_detail = probe_stlink(args.programmer)
    can_ok, can_detail = probe_can(args.interface, args.channel, args.bitrate)
    preflight = HardwarePreflight(
        programmer_available=args.programmer.exists(),
        target_connected=programmer_ok,
        can_available=can_ok,
        can_detail=can_detail,
        independent_driver_interlock=False,
        profile_id=None,
        level=None,
        pwm_limit=None,
    )
    run = AutomatedRun(preflight)
    run.mark("software validation: %s" % ("PASS" if software is None or software["passed"] else "FAIL"))
    run.mark("ST-LINK probe: %s" % ("PASS" if programmer_ok else "FAIL: " + programmer_detail[-500:]))
    run.mark("CAN probe: %s" % ("PASS" if can_ok else "FAIL: " + can_detail))
    if args.self_loop:
        if software is not None and not software["passed"]:
            run.status = "BLOCKED_SOFTWARE"
        else:
            run_self_loop(run, output_dir)
    elif software is not None and not software["passed"]:
        run.status = "BLOCKED_SOFTWARE"
    else:
        try:
            if not programmer_ok or not can_ok:
                run.require_hardware_motion()
            run_hardware_session(run, args.programmer, args.interface, args.channel,
                                 args.bitrate)
        except RunBlocked as exc:
            if run.status == "BLOCKED":
                run.status = "BLOCKED_PREFLIGHT: " + str(exc)
    write_run_report(output_dir, run, software)
    print(json.dumps({"run_dir": str(output_dir), "status": run.status}, ensure_ascii=False))
    return 0 if run.status in ("L1_COMPLETE", "SELF_LOOP_PASS") else 2


if __name__ == "__main__":
    raise SystemExit(main())
