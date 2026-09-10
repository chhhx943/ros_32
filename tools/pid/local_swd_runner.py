"""Issue bounded AutotuneSafe local-experiment requests through ST-LINK.

The local executor owns the experiment and all safety decisions on the MCU.
This host utility only writes the debugger request mailbox; it cannot write
the level, PWM envelope, or safety thresholds.  It is a dry run unless
``--execute`` is explicitly supplied.
"""

from __future__ import annotations

import argparse
import binascii
import hashlib
import json
import queue
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[2]
PROGRAMMER_DEFAULT = Path(
    r"D:\stm32cubeclt\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)
ELF_DEFAULT = ROOT / "build" / "AutotuneSafe" / "ros.elf"
SMOKE_ELF_DEFAULT = ROOT / "build" / "LocalSessionSmoke" / "ros.elf"
DEBUG_ELF_DEFAULT = ROOT / "build" / "Debug" / "ros.elf"
GDB_DEFAULT = Path(
    r"D:\stm32cubeclt\STM32CubeCLT_1.19.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe"
)
GDB_SERVER_DEFAULT = Path(
    r"D:\stm32cubeclt\STM32CubeCLT_1.19.0\STLink-gdb-server\bin\ST-LINK_gdbserver.exe"
)
CUBE_PROGRAMMER_DIR = Path(
    r"D:\stm32cubeclt\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin"
)
REQUEST_MAGIC = 0x41544C52
BOOT_MAGIC = 0x41544254
COMMAND_START = 1
COMMAND_STOP = 2
COMMAND_RECOVER = 3
REQUEST_MAGIC_OFFSET = 0
REQUEST_COMMAND_OFFSET = 4
REQUEST_SEQUENCE_OFFSET = 8
REQUEST_WHEEL_OFFSET = 12
L1_TARGET_LIMIT_MMPS = 100
L1_PWM_LIMIT = 150
LOCAL_STATUS_HEADER_BYTES = 28
LOCAL_RESULT_MAGIC = 0x4C52534C

# Versioned AutotuneSafe session contract.  These values are deliberately
# independent from the legacy local executor state machine below.
SESSION_READY = 2
SESSION_ARMED = 3
SESSION_RUNNING = 4
SESSION_COMPLETE_LATCHED = 5
SESSION_ABORT_LATCHED = 6
SESSION_STATUS_MAGIC = 0x53544154
SESSION_REQUEST_MAGIC = 0x53525154
SESSION_RESULT_FINAL_MAGIC = 0x52534C54
SESSION_STATUS_SIZE = 68
SESSION_RESULT_SIZE = 68
BOOT_READY = 2


@dataclass(frozen=True)
class OrchestrationResult:
    code: str
    category: str
    detail: str = ""


class OrchestrationFailure(RuntimeError):
    def __init__(self, code: str, detail: str = ""):
        super().__init__(detail or code)
        self.code = code
        self.category = "ORCHESTRATION_FAILURE"


class SessionOrchestrator:
    """Execute one injectable, fail-closed session lifecycle.

    The backend owns tool-specific build/flash/GDB operations.  This class
    owns ordering and the invariant that every path attempts Debug restore.
    Keeping this seam injectable lets protocol races be tested without a board.
    """

    def __init__(self, backend):
        self.backend = backend

    def run_smoke_once(self) -> OrchestrationResult:
        try:
            self.backend.build()
            self.backend.program_flash_verify_run()
            before = self.backend.read_boot_identity()
            self.backend.attach_no_reset()
            after = self.backend.read_boot_identity()
            if after.get("boot_generation") != before.get("boot_generation"):
                raise OrchestrationFailure("ATTACH_RESET_DETECTED")
            if after.get("boot_state") != BOOT_READY:
                raise OrchestrationFailure("BOOT_HANDSHAKE_FAIL")
            self.backend.wait_ready()
            self.backend.create_session()
            self.backend.start_experiment()
            self.backend.poll_terminal()
            self.backend.capture_result()
            self.backend.ack_result()
            return OrchestrationResult("OK", "SUCCESS")
        except OrchestrationFailure as exc:
            return OrchestrationResult(exc.code, exc.category, str(exc))
        except Exception as exc:  # Every tool/protocol exception is fail-closed.
            return OrchestrationResult("ORCHESTRATION_FAIL",
                                       "ORCHESTRATION_FAILURE", str(exc))
        finally:
            try:
                self.backend.restore_debug()
            except Exception:
                # The concrete backend records RESTORE_DEBUG_FAIL.  The pure
                # contract layer must not mask the original failure.
                pass


@dataclass
class ProcessEvidence:
    argv: List[str]
    returncode: Optional[int] = None
    stdout: str = ""
    stderr: str = ""
    started_at: str = ""
    exited_at: str = ""
    timeout_s: Optional[float] = None
    timed_out: bool = False


def classify_restore_failure(stage: str, detail: str = "") -> str:
    """Map one restore lifecycle boundary to a stable fail-closed code."""
    normalized = stage.lower()
    if normalized == "gdb_process":
        return "RESTORE_DEBUG_GDB_DEAD"
    if normalized == "target_connection":
        return "RESTORE_DEBUG_TARGET_DISCONNECTED"
    if normalized == "halt":
        return "RESTORE_DEBUG_HALT_FAIL"
    if normalized == "read":
        return "RESTORE_DEBUG_READ_FAIL"
    if normalized == "write":
        return "RESTORE_DEBUG_WRITE_FAIL"
    if normalized == "verify":
        return "RESTORE_DEBUG_VERIFY_FAIL"
    if normalized == "detach":
        return "RESTORE_DEBUG_DETACH_FAIL"
    if normalized == "server_exit":
        return "RESTORE_DEBUG_SERVER_EXIT_FAIL"
    if normalized == "stlink_release":
        return "RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT"
    if normalized == "programmer":
        if "verify" in detail.lower():
            return "RESTORE_DEBUG_VERIFY_FAIL"
        return "RESTORE_DEBUG_WRITE_FAIL"
    return "RESTORE_DEBUG_FAIL"


class CommandRunner:
    def __init__(self):
        self.evidence: List[ProcessEvidence] = []

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()

    def run(self, argv: Sequence[str], timeout_s: float = 60.0) -> ProcessEvidence:
        record = ProcessEvidence([str(item) for item in argv],
                                 started_at=self._timestamp(),
                                 timeout_s=timeout_s)
        try:
            result = subprocess.run(record.argv, cwd=ROOT, capture_output=True,
                                    text=True, check=False, timeout=timeout_s)
            record.returncode = result.returncode
            record.stdout = result.stdout
            record.stderr = result.stderr
        except subprocess.TimeoutExpired as exc:
            record.timed_out = True
            record.stdout = (exc.stdout or "") if isinstance(
                exc.stdout, str) else (exc.stdout or b"").decode(errors="replace")
            record.stderr = (exc.stderr or "") if isinstance(
                exc.stderr, str) else (exc.stderr or b"").decode(errors="replace")
            record.exited_at = self._timestamp()
            self.evidence.append(record)
            raise OrchestrationFailure("COMMAND_TIMEOUT",
                                       "command timed out: " + " ".join(record.argv))
        record.exited_at = self._timestamp()
        self.evidence.append(record)
        return record


class GdbServerProcess:
    """A single ST-LINK GDB server with an explicit no-reset attach proof."""

    def __init__(self, executable: Path, host: str, port: int,
                 serial: Optional[str] = None, frequency_khz: int = 4000,
                 cube_programmer_path: Path = CUBE_PROGRAMMER_DIR,
                 command_runner: Optional[CommandRunner] = None):
        self.executable = executable
        self.host = host
        self.port = port
        self.serial = serial
        self.frequency_khz = frequency_khz
        self.cube_programmer_path = cube_programmer_path
        self.command_runner = command_runner or CommandRunner()
        self.argv: List[str] = []
        self.process: Optional[subprocess.Popen[str]] = None
        self.output: List[str] = []
        self.error_output: List[str] = []
        self._reader: Optional[threading.Thread] = None
        self._error_reader: Optional[threading.Thread] = None
        self.returncode: Optional[int] = None

    def no_reset_argv(self) -> List[str]:
        argv = [str(self.executable), "--swd", "--attach",
                "--port-number", str(self.port),
                "--frequency", str(self.frequency_khz),
                "--stm32cubeprogrammer-path", str(self.cube_programmer_path)]
        if self.serial:
            argv.extend(["--serial-number", self.serial])
        return argv

    def prove_no_reset_capability(self) -> ProcessEvidence:
        if not self.executable.exists():
            raise OrchestrationFailure("ATTACH_FAIL",
                                       "ST-LINK GDB server not found: " +
                                       str(self.executable))
        evidence = self.command_runner.run([str(self.executable), "--help"],
                                           timeout_s=10.0)
        help_text = (evidence.stdout + "\n" + evidence.stderr).lower()
        if evidence.returncode not in (0, None) or "--attach" not in help_text:
            raise OrchestrationFailure(
                "ATTACH_FAIL",
                "GDB server help does not prove --attach/no-reset capability")
        return evidence

    @staticmethod
    def _reject_reset_args(argv: Sequence[str]) -> None:
        forbidden = ("reset", "initialize-reset", "erase-all", "download",
                     "halt")
        joined = " ".join(argv).lower()
        if any(token in joined for token in forbidden):
            raise OrchestrationFailure("ATTACH_FAIL",
                                       "reset-capable GDB server argument rejected")

    def start(self, wait_s: float = 5.0) -> None:
        self.prove_no_reset_capability()
        self.argv = self.no_reset_argv()
        self._reject_reset_args(self.argv)
        self.process = subprocess.Popen(
            self.argv, cwd=ROOT, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        assert self.process.stdout is not None and self.process.stderr is not None

        def read_output(stream, target) -> None:
            for line in stream:
                target.append(line.rstrip("\r\n"))

        self._reader = threading.Thread(
            target=read_output, args=(self.process.stdout, self.output), daemon=True)
        self._reader.start()
        self._error_reader = threading.Thread(
            target=read_output, args=(self.process.stderr, self.error_output), daemon=True)
        self._error_reader.start()
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if any("waiting for debugger connection" in line.lower()
                   for line in self.output):
                return
            if self.process.poll() is not None:
                self.returncode = self.process.returncode
                raise OrchestrationFailure(
                    "ATTACH_FAIL",
                    "GDB server exited before opening its TCP port")
            time.sleep(0.05)
        raise OrchestrationFailure("ATTACH_FAIL",
                                   "GDB server did not open its TCP port")

    def is_alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def stop(self) -> bool:
        if self.process is None:
            return True
        process = self.process
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    return False
        self.returncode = process.returncode
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self._error_reader is not None:
            self._error_reader.join(timeout=1.0)
        self.process = None
        return process.poll() is not None


def crc32_ieee(data: bytes) -> int:
    """Return CRC-32/IEEE reflected with the session contract parameters."""
    return binascii.crc32(data) & 0xFFFFFFFF


def generation_delta_u32(newer: int, older: int) -> int:
    return ((newer & 0xFFFFFFFF) - (older & 0xFFFFFFFF)) & 0xFFFFFFFF


def generation_is_forward(newer: int, older: int) -> bool:
    delta = generation_delta_u32(newer, older)
    return delta != 0 and delta < 0x80000000


def decode_status_snapshot(data: bytes) -> dict:
    if len(data) != SESSION_STATUS_SIZE:
        raise ValueError("session status snapshot has incorrect size")
    commit_seq, magic, version, size = struct.unpack_from("<IIHH", data, 0)
    if commit_seq & 1:
        raise ValueError("session status snapshot is being written")
    if magic != SESSION_STATUS_MAGIC or version != 1 or size != SESSION_STATUS_SIZE:
        raise ValueError("session status header mismatch")
    expected_crc = crc32_ieee(data[4:64])
    actual_crc = struct.unpack_from("<I", data, 64)[0]
    if actual_crc != expected_crc:
        raise ValueError("session status CRC mismatch")
    fields = struct.unpack_from("<IIIIIIIIIIIII", data, 12)
    return {
        "commit_seq": commit_seq,
        "accepted_request_id": fields[0],
        "accepted_session_id": fields[1],
        "active_experiment_id": fields[2],
        "state": fields[3],
        "boot_generation": fields[4],
        "session_generation": fields[5],
        "sample_generation": fields[6],
        "start_sample_generation": fields[7],
        "sample_count": fields[8],
        "expected_sample_count": fields[9],
        "last_rejected_request_id": fields[10],
        "last_reject_reason": fields[11],
        "result_ack_request_id": fields[12],
        "status_crc32": actual_crc,
    }


def decode_boot_identity(data: bytes) -> dict:
    if len(data) != 44:
        raise ValueError("boot identity snapshot has incorrect size")
    commit_seq, magic, version, size = struct.unpack_from("<IIHH", data, 0)
    if commit_seq & 1:
        raise ValueError("boot identity snapshot is being written")
    if magic != 0x41544249 or version != 1 or size != 44:
        raise ValueError("boot identity header mismatch")
    expected_crc = crc32_ieee(data[4:40])
    actual_crc = struct.unpack_from("<I", data, 40)[0]
    if actual_crc != expected_crc:
        raise ValueError("boot identity CRC mismatch")
    profile, build_id, boot_generation, boot_reason, boot_state, mailbox_version, boot_count = struct.unpack_from(
        "<IIIIIII", data, 12
    )
    return {
        "commit_seq": commit_seq,
        "profile": profile,
        "firmware_build_id": build_id,
        "boot_generation": boot_generation,
        "boot_reason": boot_reason,
        "boot_state": boot_state,
        "mailbox_version": mailbox_version,
        "boot_count": boot_count,
        "boot_crc32": actual_crc,
    }


def read_stable_boot_identity(snapshots, retry_budget: int = 3) -> Optional[dict]:
    if retry_budget <= 0:
        return None
    for index, item in enumerate(snapshots):
        if index >= retry_budget:
            break
        try:
            if isinstance(item, tuple):
                seq1, raw, seq2 = item
                if (seq1 & 1) or seq1 != seq2:
                    continue
                decoded = decode_boot_identity(raw)
                if decoded["commit_seq"] != seq1:
                    continue
                return decoded
            return decode_boot_identity(item)
        except (TypeError, ValueError, struct.error):
            continue
    return None


def read_stable_snapshot(snapshots, retry_budget: int = 3) -> Optional[dict]:
    """Read a bounded sequence of status attempts using seqlock semantics.

    A test/provider item may be raw bytes (the two sequence reads are the same
    stable bus transaction) or ``(seq1, bytes, seq2)`` to model a torn read.
    Odd, changing, or CRC-invalid attempts are transient until the budget is
    exhausted.
    """
    if retry_budget <= 0:
        return None
    for index, item in enumerate(snapshots):
        if index >= retry_budget:
            break
        try:
            if isinstance(item, tuple):
                seq1, raw, seq2 = item
                if (seq1 & 1) or seq1 != seq2:
                    continue
                decoded = decode_status_snapshot(raw)
                if decoded["commit_seq"] != seq1:
                    continue
                return decoded
            return decode_status_snapshot(item)
        except (TypeError, ValueError, struct.error):
            continue
    return None


def decode_result(data: bytes) -> dict:
    if len(data) != SESSION_RESULT_SIZE:
        raise ValueError("session result has incorrect size")
    values = struct.unpack_from("<IHHIIIIIIIIIIIIIII", data, 0)
    return {
        "magic": values[0],
        "version": values[1],
        "size": values[2],
        "session_id": values[3],
        "experiment_id": values[4],
        "boot_generation": values[5],
        "session_generation": values[6],
        "start_sample_generation": values[7],
        "first_sample_generation": values[8],
        "last_sample_generation": values[9],
        "sample_count": values[10],
        "ring_start_index": values[11],
        "ring_count": values[12],
        "final_state": values[13],
        "abort_reason": values[14],
        "result_flags": values[15],
        "ring_crc32": values[16],
        "result_crc32": values[17],
    }


def validate_joint_identity(actual: dict, expected: dict) -> bool:
    fields = ("build_id", "boot_generation", "session_generation",
              "session_id", "experiment_id")
    return all(field in actual and field in expected and
               (actual[field] & 0xFFFFFFFF) == (expected[field] & 0xFFFFFFFF)
               for field in fields)


def validate_running_handshake(status: dict, generation_before_start: int,
                               session_id: int, experiment_id: int) -> bool:
    return (
        status.get("state") == SESSION_RUNNING and
        status.get("accepted_session_id") == session_id and
        status.get("active_experiment_id") == experiment_id and
        status.get("start_sample_generation") ==
        (generation_before_start & 0xFFFFFFFF) and
        (status.get("sample_generation") == status.get("start_sample_generation") or
         generation_is_forward(status.get("sample_generation", 0),
                               status.get("start_sample_generation", 0)))
    )


def validate_result(result, expected_identity: dict) -> str:
    if isinstance(result, (bytes, bytearray)):
        if len(result) < 4 or struct.unpack_from("<I", result, 0)[0] == 0:
            return "RESULT_COMMIT_INCOMPLETE"
        try:
            decoded = decode_result(bytes(result))
        except (ValueError, struct.error):
            return "DATA_INTEGRITY_FAIL"
        if decoded["magic"] != SESSION_RESULT_FINAL_MAGIC:
            return "RESULT_MAGIC_INVALID"
        if decoded["version"] != 1 or decoded["size"] != SESSION_RESULT_SIZE:
            return "DATA_INTEGRITY_FAIL"
        if crc32_ieee(bytes(result[:60])) != decoded["result_crc32"]:
            return "RESULT_CRC_FAIL"
        actual = decoded
    else:
        actual = dict(result)
        if actual.get("magic") == 0:
            return "RESULT_COMMIT_INCOMPLETE"
        if actual.get("magic") != SESSION_RESULT_FINAL_MAGIC:
            return "RESULT_MAGIC_INVALID"
    for field in ("boot_generation", "session_generation", "session_id",
                  "experiment_id"):
        if actual.get(field) != expected_identity.get(field):
            return "RESULT_GENERATION_MISMATCH" if "generation" in field else (
                "RESULT_SESSION_MISMATCH" if field == "session_id" else
                "RESULT_EXPERIMENT_MISMATCH")
    if actual.get("build_id", expected_identity.get("build_id")) != expected_identity.get("build_id"):
        return "BOOT_BUILD_ID_MISMATCH"
    return "OK"


def decode_samples(data: bytes) -> List[dict]:
    if len(data) % 40 != 0:
        raise ValueError("sample ring byte count is not aligned")
    rows = []
    for offset in range(0, len(data), 40):
        values = struct.unpack_from("<IIIIIiiiII", data, offset)
        rows.append({
            "sample_generation": values[0],
            "timestamp_ms": values[1],
            "session_id": values[2],
            "experiment_id": values[3],
            "sample_index": values[4],
            "target": values[5],
            "actual": values[6],
            "pwm": values[7],
            "state": values[8],
            "flags": values[9],
        })
    return rows


def validate_complete_result(result, ring: bytes,
                             expected_identity: dict) -> str:
    code = validate_result(result, expected_identity)
    if code != "OK":
        return code
    try:
        decoded = decode_result(bytes(result))
        rows = decode_samples(ring)
    except (TypeError, ValueError, struct.error):
        return "DATA_INTEGRITY_FAIL"
    if decoded["final_state"] != SESSION_COMPLETE_LATCHED:
        return "INCOMPLETE_EXPERIMENT_EVIDENCE"
    if decoded["sample_count"] != decoded["ring_count"] or \
            decoded["ring_count"] != len(rows):
        return "INCOMPLETE_EXPERIMENT_EVIDENCE"
    if crc32_ieee(ring) != decoded["ring_crc32"]:
        return "RING_CRC_FAIL"
    if not rows:
        return "INCOMPLETE_EXPERIMENT_EVIDENCE"
    if not generation_is_forward(decoded["first_sample_generation"],
                                 decoded["start_sample_generation"]):
        return "RESULT_GENERATION_MISMATCH"
    if decoded["last_sample_generation"] != rows[-1]["sample_generation"]:
        return "RESULT_GENERATION_MISMATCH"
    for index, row in enumerate(rows):
        if (row["session_id"] != expected_identity["session_id"] or
                row["experiment_id"] != expected_identity["experiment_id"] or
                row["sample_index"] != index):
            return "RESULT_SESSION_MISMATCH"
        if index and not generation_is_forward(
                row["sample_generation"], rows[index - 1]["sample_generation"]):
            return "SAMPLE_GENERATION_INVALID"
    return "OK"


def render_report(*, failure_code: Optional[str] = None,
                  evidence: Optional[dict] = None) -> str:
    """Render a conservative validation report from captured evidence."""
    failure = failure_code or "NONE"
    category = "ORCHESTRATION_FAILURE" if failure != "NONE" else "VALID"
    payload = evidence if evidence is not None else {}
    lines = [
        "# AUTOTUNE_SAFE_SWD_SESSION_VALIDATION",
        "",
        "- Scope: ST-LINK/SWD session orchestration only",
        "- Terminal states: `COMPLETE_LATCHED` / `ABORT_LATCHED`",
        "- Generation ordering: uint32 wrap-aware delta",
        "- Snapshot rule: odd/changing sequence or transient CRC mismatch retries",
        "- Result rule: final magic is committed last; magic=0 is incomplete",
        "- Failure category: `%s`" % category,
        "- Failure code: `%s`" % failure,
        "",
        "```json",
        json.dumps(payload, indent=2, ensure_ascii=False, default=str),
        "```",
        "",
    ]
    if failure != "NONE":
        lines.extend([
            "No motor, encoder, or PID conclusion is permitted from this artifact.",
            "The result is not actuator evidence until the complete session contract is proven.",
            "",
        ])
    return "\n".join(lines)


def _tool(name: str) -> Optional[str]:
    found = shutil.which(name)
    if found:
        return found
    if name == "arm-none-eabi-nm":
        common = Path(r"C:\Program Files (x86)\Arm GNU Toolchain\12.2 mpacbti-rel1\bin") / name
        if common.exists():
            return str(common)
    return None


def resolve_symbol(elf: Path, symbol: str) -> int:
    nm = _tool("arm-none-eabi-nm")
    if nm is None:
        raise RuntimeError("arm-none-eabi-nm is required to resolve the local request mailbox")
    if not elf.exists():
        raise RuntimeError("AutotuneSafe ELF not found: " + str(elf))
    result = subprocess.run(
        [nm, "-C", "--defined-only", str(elf)],
        cwd=ROOT, capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        raise RuntimeError("nm failed: " + result.stderr[-500:])
    for line in result.stdout.splitlines():
        match = re.match(r"^\s*([0-9A-Fa-f]+)\s+[Bb]\s+%s\s*$" %
                         re.escape(symbol), line)
        if match:
            address = int(match.group(1), 16)
            if not 0x20000000 <= address < 0x20020000:
                raise RuntimeError("local request mailbox is not in STM32F4 SRAM")
            return address
    raise RuntimeError("%s was not found in AutotuneSafe ELF" % symbol)


def resolve_control_address(elf: Path) -> int:
    return resolve_symbol(elf, "g_autotune_safe_local_control")


def resolve_boot_request_address(elf: Path) -> int:
    return resolve_symbol(elf, "g_autotune_safe_local_boot_request")


def resolve_session_address(elf: Path, symbol: str) -> int:
    return resolve_symbol(elf, symbol)


def pack_session_request(request_id: int, request_type: int, session_id: int,
                         experiment_id: int, payload: Sequence[int] = (),
                         boot_generation_hint: int = 0) -> bytes:
    values = list(payload[:3]) + [0, 0, 0]
    data = bytearray(48)
    struct.pack_into(
        "<IHHIIIIIIIII", data, 0,
        0, 1, 48, request_id & 0xFFFFFFFF, request_type,
        session_id & 0xFFFFFFFF, experiment_id & 0xFFFFFFFF,
        values[0] & 0xFFFFFFFF, values[1] & 0xFFFFFFFF,
        values[2] & 0xFFFFFFFF, boot_generation_hint & 0xFFFFFFFF, 0,
    )
    struct.pack_into("<I", data, 44, crc32_ieee(bytes(data[4:44])))
    return bytes(data)


def write_session_request(session: GdbMiSession, address: int, data: bytes) -> None:
    if len(data) != 48:
        raise ValueError("session request has incorrect size")
    # Runtime request magic is the commit marker: all payload bytes and CRC
    # are visible before the final aligned magic write.
    session.write_bytes(address + 4, data[4:])
    session.write_word(address, SESSION_REQUEST_MAGIC)


def _hex(value: int) -> str:
    return "0x%08X" % (value & 0xFFFFFFFF)


def decode_local_control_header(data: bytes) -> dict:
    """Decode only the stable request/status prefix of the MCU mailbox."""
    if len(data) < LOCAL_STATUS_HEADER_BYTES:
        raise ValueError("local control snapshot is shorter than its header")
    magic, command, sequence = struct.unpack_from("<III", data, 0)
    wheel, state, active_wheel, candidate_id, point_index, candidate_count = struct.unpack_from(
        "<BBBBBB", data, 12
    )
    status_magic = struct.unpack_from("<I", data, 24)[0]
    return {
        "request_magic": magic,
        "request_command": command,
        "request_sequence": sequence,
        "request_wheel": wheel,
        "state": state,
        "active_wheel": active_wheel,
        "candidate_id": candidate_id,
        "point_index": point_index,
        "candidate_count": candidate_count,
        "status_magic": status_magic,
    }


class GdbMiSession:
    """One bounded, interruptible GDB/MI connection to a running STM32."""

    def __init__(self, gdb: Path, elf: Path, host: str = "localhost",
                 port: int = 61234, timeout_s: float = 3.0):
        self.gdb = gdb
        self.elf = elf
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.process: Optional[subprocess.Popen[str]] = None
        self._lines: "queue.Queue[Optional[str]]" = queue.Queue()
        self._reader: Optional[threading.Thread] = None
        self._error_reader: Optional[threading.Thread] = None
        self.stdout_lines: List[str] = []
        self.stderr_lines: List[str] = []
        self._token = 0
        self._pending: List[str] = []
        self.target_attached = False
        self.target_halted = False

    def __enter__(self) -> "GdbMiSession":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _read_output(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        # GDB emits the MI prompt without a trailing newline when stdout is a
        # pipe. Read characters so ``(gdb)`` is observable in both pipe and
        # PTY environments; line iteration would block forever at startup.
        buffer = ""
        while True:
            char = self.process.stdout.read(1)
            if char == "":
                break
            buffer += char
            if char == "\n":
                line = buffer.rstrip("\r\n")
                self.stdout_lines.append(line)
                self._lines.put(line)
                buffer = ""
            elif re.search(r"\(gdb\)\s*$", buffer):
                self.stdout_lines.append(buffer)
                self._lines.put(buffer)
                buffer = ""
        if buffer:
            line = buffer.rstrip("\r\n")
            self.stdout_lines.append(line)
            self._lines.put(line)
        self._lines.put(None)

    def _read_stderr(self) -> None:
        assert self.process is not None and self.process.stderr is not None
        for line in self.process.stderr:
            self.stderr_lines.append(line.rstrip("\r\n"))

    def _next_line(self, timeout_s: Optional[float] = None) -> str:
        try:
            line = self._lines.get(timeout=self.timeout_s if timeout_s is None else timeout_s)
        except queue.Empty as exc:
            raise RuntimeError("GDB/MI response timeout") from exc
        if line is None:
            raise RuntimeError("GDB/MI process closed its output")
        return line

    def _wait_token(self, token: int, allow_running: bool = False) -> str:
        deadline = time.monotonic() + self.timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("GDB/MI command response timeout")
            line = self._next_line(remaining)
            if not line.strip() or line.strip() == "(gdb)":
                continue
            if line.startswith("*stopped") or line.startswith("=thread"):
                self._pending.append(line)
                continue
            if line.startswith("%d^" % token):
                if "^error" in line:
                    raise RuntimeError("GDB/MI command failed: " + line)
                if (not allow_running) and line.startswith("%d^running" % token):
                    raise RuntimeError("GDB/MI target unexpectedly remained running")
                return line

    def _send(self, command: str, allow_running: bool = False) -> str:
        if self.process is None or self.process.stdin is None:
            raise RuntimeError("GDB/MI session is not running")
        self._token += 1
        token = self._token
        self.process.stdin.write("%d%s\n" % (token, command))
        self.process.stdin.flush()
        return self._wait_token(token, allow_running=allow_running)

    def _wait_stopped(self) -> str:
        for index, pending in enumerate(self._pending):
            if pending.startswith("*stopped"):
                return self._pending.pop(index)
        deadline = time.monotonic() + self.timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("GDB/MI stop notification timeout")
            line = self._next_line(remaining)
            if not line.strip() or line.strip() == "(gdb)":
                continue
            if line.startswith("*stopped"):
                return line
            self._pending.append(line)

    def start(self) -> None:
        if not self.gdb.exists():
            raise RuntimeError("arm-none-eabi-gdb not found: " + str(self.gdb))
        if not self.elf.exists():
            raise RuntimeError("AutotuneSafe ELF not found: " + str(self.elf))
        self.process = subprocess.Popen(
            [str(self.gdb), "-q", "--interpreter=mi3", str(self.elf)],
            cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        assert self.process.stderr is not None
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()
        self._error_reader = threading.Thread(target=self._read_stderr, daemon=True)
        self._error_reader.start()
        self._wait_prompt()
        self._send("-gdb-set mi-async on")
        self._send("-target-select remote %s:%d" % (self.host, self.port))
        self.target_attached = True
        self.target_halted = True
        # target-select reports the initial halt asynchronously. It is not
        # the stop notification belonging to a later interrupt command.
        self._pending.clear()

    def _wait_prompt(self) -> None:
        deadline = time.monotonic() + self.timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("GDB/MI initial prompt timeout")
            line = self._next_line(remaining)
            if line.strip() == "(gdb)":
                return

    def write_word(self, address: int, value: int) -> None:
        # Little-endian raw bytes avoid architecture-dependent GDB expressions.
        payload = struct.pack("<I", value & 0xFFFFFFFF).hex()
        self._send("-data-write-memory-bytes %s %s" % (_hex(address), payload))

    def write_bytes(self, address: int, data: bytes) -> None:
        if not data:
            return
        self._send("-data-write-memory-bytes %s %s" %
                   (_hex(address), data.hex()))

    def read_bytes(self, address: int, count: int) -> bytes:
        response = self._send("-data-read-memory-bytes %s %d" % (_hex(address), count))
        match = re.search(r'contents="([0-9A-Fa-f]*)"', response)
        if match is None:
            raise RuntimeError("GDB/MI memory response has no contents: " + response)
        data = bytes.fromhex(match.group(1))
        if len(data) != count:
            raise RuntimeError("GDB/MI memory response has incomplete contents")
        return data

    def continue_target(self) -> None:
        self._send("-exec-continue --all", allow_running=True)
        self.target_halted = False

    def interrupt_target(self) -> None:
        try:
            self._send("-exec-interrupt --all", allow_running=True)
        except RuntimeError as exc:
            if "not running" not in str(exc):
                raise
            self.target_halted = True
            return
        self._wait_stopped()
        self.target_halted = True

    def is_alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def read_debug_state(self) -> dict:
        if not self.is_alive():
            raise OrchestrationFailure(classify_restore_failure("gdb_process"))
        if not self.target_attached:
            raise OrchestrationFailure(
                classify_restore_failure("target_connection"))
        try:
            response = self._send('-data-evaluate-expression "$pc"')
        except RuntimeError as exc:
            raise OrchestrationFailure(
                classify_restore_failure("read"), str(exc)) from exc
        match = re.search(r'value="([^"]+)"', response)
        if match is None:
            raise OrchestrationFailure(
                classify_restore_failure("read"), response)
        return {"pc": match.group(1), "halted": self.target_halted}

    def detach_target(self) -> None:
        if not self.target_attached:
            return
        if not self.is_alive():
            raise OrchestrationFailure(classify_restore_failure("gdb_process"))
        try:
            self._send("-target-detach")
        except RuntimeError as exc:
            raise OrchestrationFailure(
                classify_restore_failure("detach"), str(exc)) from exc
        self.target_attached = False

    def close(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None and self.process.stdin is not None:
            try:
                self.process.stdin.write("-gdb-exit\n")
                self.process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
        try:
            self.process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2.0)
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self._error_reader is not None:
            self._error_reader.join(timeout=1.0)
        self.target_attached = False
        self.process = None


class HardwareSmokeBackend:
    """Concrete ST-LINK backend for one LocalSessionSmoke transaction."""

    def __init__(self, smoke_elf: Path, debug_elf: Path, programmer: Path,
                 gdb: Path, gdb_server: Path, gdb_host: str = "localhost",
                 gdb_port: int = 61234, serial: Optional[str] = None,
                 poll_ms: int = 50, deadline_s: float = 10.0):
        self.smoke_elf = smoke_elf
        self.debug_elf = debug_elf
        self.programmer = programmer
        self.gdb = gdb
        self.gdb_server = gdb_server
        self.gdb_host = gdb_host
        self.gdb_port = gdb_port
        self.serial = serial
        self.poll_s = poll_ms / 1000.0
        self.deadline_s = deadline_s
        self.commands = CommandRunner()
        self.server: Optional[GdbServerProcess] = None
        self.gdb_session: Optional[GdbMiSession] = None
        self.addresses = {}
        self.expected_identity = {}
        self.request_id = int(time.time_ns()) & 0xFFFFFFFF
        self.session_id = (self.request_id ^ 0x53455353) & 0xFFFFFFFF
        self.experiment_id = (self.request_id ^ 0x45585031) & 0xFFFFFFFF
        self.events: List[str] = []
        self.timeline: List[dict] = []
        self.gdb_expected = False
        self.last_status: Optional[dict] = None
        self.last_boot: Optional[dict] = None
        self.evidence = {
            "profile": "LOCAL_SESSION_SMOKE",
            "smoke_elf": str(self.smoke_elf),
            "debug_elf": str(self.debug_elf),
            "serial": self.serial,
            "gdb_host": self.gdb_host,
            "gdb_port": self.gdb_port,
            "poll_ms": poll_ms,
            "events": self.events,
            "timeline": self.timeline,
            "boot_identity": None,
            "armed_status": None,
            "running_status": None,
            "terminal_status": None,
            "result": None,
            "ring": None,
            "debug_restore": None,
            "restore_debug": None,
        }

    def _event(self, name: str, **details) -> None:
        self.events.append(name)
        self.timeline.append({
            "event": name,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            **details,
        })

    def build(self) -> None:
        if not self.smoke_elf.exists():
            raise OrchestrationFailure("BUILD_FAIL", "LocalSessionSmoke ELF missing")
        self.evidence["smoke_elf_sha256"] = hashlib.sha256(
            self.smoke_elf.read_bytes()).hexdigest()
        if self.debug_elf.exists():
            self.evidence["debug_elf_sha256"] = hashlib.sha256(
                self.debug_elf.read_bytes()).hexdigest()
        self._event("build")

    def _connect_command(self) -> str:
        connect = "port=SWD"
        if self.serial:
            connect += " sn=" + self.serial
        return connect

    def program_flash_verify_run(self) -> None:
        if not self.programmer.exists():
            raise OrchestrationFailure("FLASH_FAIL", "CubeProgrammer not found")
        record = self.commands.run([
            str(self.programmer), "-c", self._connect_command(), "freq=4000",
            "-w", str(self.smoke_elf), "-v", "-rst", "-run",
        ])
        self._event("programmer_exit", returncode=record.returncode)
        if record.returncode != 0:
            raise OrchestrationFailure("VERIFY_FAIL", record.stderr[-1000:])

    def attach_no_reset(self) -> None:
        self.server = GdbServerProcess(
            self.gdb_server, self.gdb_host, self.gdb_port, self.serial,
            command_runner=self.commands,
        )
        self.gdb_expected = True
        self._event("gdb_server_start", argv=self.server.no_reset_argv())
        self.server.start()
        self.evidence["gdb_server"] = {
            "argv": list(self.server.argv),
            "output": list(self.server.output),
            "stderr": list(self.server.error_output),
            "no_reset_attach": True,
        }
        self.gdb_session = GdbMiSession(
            self.gdb, self.smoke_elf, self.gdb_host, self.gdb_port,
        )
        self.gdb_session.start()
        self._event("gdb_attach")
        for name in (
            "g_autotune_session_boot_identity",
            "g_autotune_session_request",
            "g_autotune_session_status",
            "g_autotune_session_result",
            "g_autotune_session_samples",
        ):
            self.addresses[name] = resolve_session_address(self.smoke_elf, name)
        # GDB target-select initially stops the CPU; continue is explicit and
        # follows attach, never a fixed sleep assumption.
        self.gdb_session.continue_target()

    def _read_boot(self) -> Optional[dict]:
        assert self.gdb_session is not None
        self.gdb_session.interrupt_target()
        try:
            address = self.addresses["g_autotune_session_boot_identity"]
            raw1 = self.gdb_session.read_bytes(address, 4)
            raw = self.gdb_session.read_bytes(address, 44)
            raw2 = self.gdb_session.read_bytes(address, 4)
            seq1 = struct.unpack("<I", raw1)[0]
            seq2 = struct.unpack("<I", raw2)[0]
            snapshot = read_stable_boot_identity([(seq1, raw, seq2)], 1)
            if snapshot is not None:
                self.last_boot = snapshot
            return snapshot
        finally:
            self.gdb_session.continue_target()

    def _read_status(self) -> Optional[dict]:
        assert self.gdb_session is not None
        self.gdb_session.interrupt_target()
        try:
            address = self.addresses["g_autotune_session_status"]
            raw1 = self.gdb_session.read_bytes(address, 4)
            raw = self.gdb_session.read_bytes(address, SESSION_STATUS_SIZE)
            raw2 = self.gdb_session.read_bytes(address, 4)
            seq1 = struct.unpack("<I", raw1)[0]
            seq2 = struct.unpack("<I", raw2)[0]
            snapshot = read_stable_snapshot([(seq1, raw, seq2)], 1)
            if snapshot is not None:
                self.last_status = snapshot
            return snapshot
        finally:
            self.gdb_session.continue_target()

    def _poll(self, read_fn, predicate, code: str):
        deadline = time.monotonic() + self.deadline_s
        while time.monotonic() < deadline:
            snapshot = read_fn()
            if snapshot is not None and predicate(snapshot):
                return snapshot
            time.sleep(self.poll_s)
        detail = {"last_boot": self.last_boot, "last_status": self.last_status}
        raise OrchestrationFailure(code, json.dumps(detail, ensure_ascii=False))

    def wait_ready(self) -> dict:
        identity = self._poll(
            self._read_boot,
            lambda row: row["boot_state"] == BOOT_READY,
            "BOOT_HANDSHAKE_TIMEOUT",
        )
        if identity["profile"] != 1 or identity["mailbox_version"] != 1:
            raise OrchestrationFailure("BOOT_PROFILE_MISMATCH")
        self.expected_identity = {
            "build_id": identity["firmware_build_id"],
            "boot_generation": identity["boot_generation"],
        }
        self.evidence["boot_identity"] = identity
        return identity

    def _send_request(self, request_type: int, session_id: int,
                      experiment_id: int, payload: Sequence[int] = ()) -> int:
        assert self.gdb_session is not None
        self.request_id = (self.request_id + 1) & 0xFFFFFFFF
        if self.request_id == 0:
            self.request_id = 1
        request = pack_session_request(
            self.request_id, request_type, session_id, experiment_id,
            payload, self.expected_identity["boot_generation"],
        )
        self.gdb_session.interrupt_target()
        write_session_request(
            self.gdb_session, self.addresses["g_autotune_session_request"], request
        )
        self.gdb_session.continue_target()
        return self.request_id

    def create_session(self) -> dict:
        request_id = self._send_request(1, self.session_id, 0)
        armed = self._poll(
            self._read_status,
            lambda row: row["state"] == SESSION_ARMED and
            row["accepted_request_id"] == request_id and
            row["accepted_session_id"] == self.session_id,
            "ARM_TIMEOUT",
        )
        self.evidence["armed_status"] = armed
        return armed

    def start_experiment(self) -> dict:
        armed = self._poll(
            self._read_status,
            lambda row: row["state"] == SESSION_ARMED and
            row["accepted_session_id"] == self.session_id,
            "ARM_TIMEOUT",
        )
        generation_before_start = armed["sample_generation"]
        request_id = self._send_request(
            2, self.session_id, self.experiment_id,
            [generation_before_start],
        )
        running = self._poll(
            self._read_status,
            lambda row: row["state"] == SESSION_RUNNING and
            row["accepted_request_id"] == request_id,
            "START_HANDSHAKE_FAIL",
        )
        if not validate_running_handshake(
                running, generation_before_start,
                self.session_id, self.experiment_id):
            raise OrchestrationFailure("START_HANDSHAKE_FAIL")
        self.expected_identity.update({
            "session_generation": running["session_generation"],
            "session_id": self.session_id,
            "experiment_id": self.experiment_id,
        })
        self.evidence["generation_before_start"] = generation_before_start
        self.evidence["running_status"] = running
        return running

    def poll_terminal(self) -> dict:
        terminal = self._poll(
            self._read_status,
            lambda row: row["state"] in
            (SESSION_COMPLETE_LATCHED, SESSION_ABORT_LATCHED),
            "STATUS_SEQLOCK_TIMEOUT",
        )
        self.evidence["terminal_status"] = terminal
        return terminal

    def capture_result(self) -> dict:
        assert self.gdb_session is not None
        self.gdb_session.interrupt_target()
        result_address = self.addresses["g_autotune_session_result"]
        try:
            result = self.gdb_session.read_bytes(result_address, SESSION_RESULT_SIZE)
            if len(result) < 4 or struct.unpack_from("<I", result, 0)[0] != SESSION_RESULT_FINAL_MAGIC:
                raise OrchestrationFailure("RESULT_COMMIT_INCOMPLETE")
            decoded = decode_result(result)
            ring = self.gdb_session.read_bytes(
                self.addresses["g_autotune_session_samples"],
                decoded["ring_count"] * 40,
            )
            code = validate_complete_result(result, ring, self.expected_identity)
            if code != "OK":
                raise OrchestrationFailure(code)
            captured = {"result": decoded, "ring": decode_samples(ring)}
            self.evidence["result"] = decoded
            self.evidence["ring"] = captured["ring"]
            return captured
        finally:
            self.gdb_session.continue_target()

    def ack_result(self) -> None:
        self._send_request(3, self.session_id, self.experiment_id)
        self._poll(
            self._read_status,
            lambda row: row["state"] == SESSION_READY and
            row["result_ack_request_id"] == self.request_id,
            "ACK_RESULT_TIMEOUT",
        )

    def close(self) -> None:
        if self.gdb_session is not None:
            self.evidence["gdb"] = {
                "stdout": list(self.gdb_session.stdout_lines),
                "stderr": list(self.gdb_session.stderr_lines),
                "alive_before_close": self.gdb_session.is_alive(),
                "target_attached_before_close": self.gdb_session.target_attached,
            }
            self.gdb_session.close()
            self.gdb_session = None
        if self.server is not None:
            server_stopped = self.server.stop()
            self.evidence.setdefault("gdb_server", {}).update({
                "output": list(self.server.output),
                "stderr": list(self.server.error_output),
                "alive_after_stop": not server_stopped,
                "returncode": self.server.returncode,
            })
            self.server = None

    def restore_debug(self) -> None:
        previous = self.evidence.get("restore_debug")
        if isinstance(previous, dict) and previous.get("status") == "OK":
            return
        restore = {
            "stage": "RESTORE_DEBUG_BEGIN",
            "max_attempts": 3,
            "deadline_s": 30.0,
            "attempts": [],
        }
        self.evidence["restore_debug"] = restore
        self.evidence["debug_restore"] = restore

        # Reconcile the live debug lifecycle before closing it. A missing GDB
        # object is valid when the primary failure happened before attach; a
        # dead GDB process is recorded separately and cannot be mistaken for a
        # ST-LINK release failure.
        try:
            restore["stage"] = "CHECK_GDB_PROCESS"
            if self.gdb_session is not None:
                restore["gdb_alive"] = self.gdb_session.is_alive()
                restore["target_connected"] = self.gdb_session.target_attached
                if not self.gdb_session.is_alive():
                    raise OrchestrationFailure(
                        classify_restore_failure("gdb_process"))
                if not self.gdb_session.target_attached:
                    raise OrchestrationFailure(
                        classify_restore_failure("target_connection"))
                restore["stage"] = "HALT_TARGET"
                try:
                    self.gdb_session.interrupt_target()
                except (RuntimeError, OrchestrationFailure) as exc:
                    raise OrchestrationFailure(
                        classify_restore_failure("halt"), str(exc)) from exc
                restore["stage"] = "READ_CURRENT_DEBUG_STATE"
                state_before = self.gdb_session.read_debug_state()
                restore["target_state_before"] = state_before
                restore["target_state_after"] = state_before
                restore["stage"] = "DETACH"
                self.gdb_session.detach_target()
                restore["detached"] = True
            else:
                restore["gdb_alive"] = None
                restore["target_connected"] = None
                restore["target_state_before"] = None
        except OrchestrationFailure as exc:
            restore["failure"] = {
                "code": exc.code,
                "detail": str(exc),
            }
            # Preserve this primary restore-boundary failure; the finally
            # block releases any process handles exactly once.
            restore["stage"] = "RESTORE_DEBUG_FAIL"
            raise
        finally:
            if self.gdb_session is not None or self.server is not None:
                self.close()

        if not self.programmer.exists() or not self.debug_elf.exists():
            code = classify_restore_failure("write")
            restore["failure"] = {"code": code, "detail": "Debug image/tool missing"}
            restore["stage"] = "RESTORE_DEBUG_FAIL"
            raise OrchestrationFailure(code, "Debug image/tool missing")

        argv = [
            str(self.programmer), "-c", self._connect_command(), "freq=4000",
            "-w", str(self.debug_elf), "-v", "-rst", "-run",
        ]
        probe_argv = [str(self.programmer), "-c", self._connect_command(),
                      "freq=4000"]
        deadline = time.monotonic() + restore["deadline_s"]
        last_failure = None
        for attempt in range(1, restore["max_attempts"] + 1):
            if time.monotonic() >= deadline:
                break
            item = {
                "attempt": attempt,
                "stage": "WAIT_STLINK_RELEASE",
                "started_at": datetime.now(timezone.utc).isoformat(),
                "gdb_alive": False,
                "gdb_server_alive": False,
                "target_connected": False,
                "target_state_before": restore.get("target_state_before"),
            }
            started = time.monotonic()
            try:
                probe = self.commands.run(probe_argv, timeout_s=10.0)
                item["probe"] = probe.__dict__
            except OrchestrationFailure as exc:
                code = classify_restore_failure("stlink_release")
                last_failure = OrchestrationFailure(code, str(exc))
                item["failure"] = {"code": code, "detail": str(last_failure)}
                item["elapsed_ms"] = int((time.monotonic() - started) * 1000)
                restore["attempts"].append(item)
                continue
            item["elapsed_ms"] = int((time.monotonic() - started) * 1000)
            if probe.returncode != 0:
                last_failure = OrchestrationFailure(
                    classify_restore_failure("stlink_release"),
                    probe.stdout[-1000:] + probe.stderr[-1000:],
                )
                item["failure"] = {
                    "code": last_failure.code,
                    "detail": str(last_failure),
                }
                restore["attempts"].append(item)
                if attempt < restore["max_attempts"] and time.monotonic() < deadline:
                    # This is a bounded ownership poll after an observed
                    # failed reopen, not a readiness assumption for the MCU.
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                continue

            item["stage"] = "RESTORE_DEBUG_STATE"
            try:
                record = self.commands.run(argv, timeout_s=60.0)
                item["programmer"] = record.__dict__
            except OrchestrationFailure as exc:
                code = classify_restore_failure("write")
                last_failure = OrchestrationFailure(code, str(exc))
                item["failure"] = {"code": code, "detail": str(last_failure)}
                restore["attempts"].append(item)
                continue
            combined = (record.stdout + "\n" + record.stderr).lower()
            if record.returncode != 0:
                code = classify_restore_failure("programmer", combined)
                last_failure = OrchestrationFailure(code,
                                                    record.stdout[-1000:] + record.stderr[-1000:])
                item["failure"] = {"code": code, "detail": str(last_failure)}
                restore["attempts"].append(item)
                continue
            if "verified successfully" not in combined:
                code = classify_restore_failure("verify")
                last_failure = OrchestrationFailure(
                    code, "CubeProgrammer did not emit verify success")
                item["failure"] = {"code": code, "detail": str(last_failure)}
                restore["attempts"].append(item)
                continue
            item["stage"] = "VERIFY_DEBUG_STATE"
            item["target_state_after"] = (
                "running" if "core run" in combined else "unknown")
            item["elapsed_ms"] = int((time.monotonic() - started) * 1000)
            restore["attempts"].append(item)
            restore.update({
                "stage": "RESTORE_DEBUG_OK",
                "status": "OK",
                "target_state_after": item["target_state_after"],
            })
            return

        restore["stage"] = "RESTORE_DEBUG_FAIL"
        restore["status"] = "FAIL"
        if last_failure is None:
            last_failure = OrchestrationFailure(
                classify_restore_failure("stlink_release"),
                "restore deadline expired",
            )
        restore["failure"] = {
            "code": last_failure.code,
            "detail": str(last_failure),
        }
        raise last_failure

    def run_hardware_smoke(self) -> OrchestrationResult:
        outcome: Optional[OrchestrationResult] = None
        try:
            self.build()
            self.program_flash_verify_run()
            self.attach_no_reset()
            self.wait_ready()
            self.create_session()
            self.start_experiment()
            self.poll_terminal()
            self.capture_result()
            self.ack_result()
            outcome = OrchestrationResult("OK", "SUCCESS")
        except OrchestrationFailure as exc:
            outcome = OrchestrationResult(exc.code, exc.category, str(exc))
        except Exception as exc:
            outcome = OrchestrationResult("ORCHESTRATION_FAIL",
                                          "ORCHESTRATION_FAILURE", str(exc))
        finally:
            try:
                self.restore_debug()
            except OrchestrationFailure as exc:
                if outcome is None or outcome.code == "OK":
                    outcome = OrchestrationResult(exc.code, exc.category,
                                                  str(exc))
                else:
                    outcome = OrchestrationResult(
                        outcome.code, outcome.category,
                        outcome.detail + " | restore=" + str(exc),
                    )
            except Exception as exc:
                if outcome is None or outcome.code == "OK":
                    outcome = OrchestrationResult(
                        "RESTORE_DEBUG_FAIL", "ORCHESTRATION_FAILURE", str(exc))
                else:
                    outcome = OrchestrationResult(
                        outcome.code, outcome.category,
                        outcome.detail + " | restore=" + str(exc),
                    )
        assert outcome is not None
        return outcome


def save_hardware_artifact(output_root: Path, index: int,
                           backend: HardwareSmokeBackend,
                           result: OrchestrationResult) -> Path:
    """Persist one complete or fail-closed hardware transaction."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    suffix = "%08x" % (time.time_ns() & 0xFFFFFFFF)
    directory = output_root / (stamp + "_session_smoke_%03d_%s" %
                               (index, suffix))
    directory.mkdir(parents=True, exist_ok=False)
    payload = dict(backend.evidence)
    payload["outcome"] = {
        "code": result.code,
        "category": result.category,
        "detail": result.detail,
    }
    payload["commands"] = [record.__dict__ for record in backend.commands.evidence]
    payload["created_at"] = datetime.now(timezone.utc).isoformat()

    def write_text(filename: str, content: str) -> None:
        (directory / filename).write_text(content, encoding="utf-8")

    def process_log(kind: str, stream: str) -> str:
        chunks = []
        for record in backend.commands.evidence:
            executable = Path(record.argv[0]).name.lower()
            if kind not in executable:
                continue
            chunks.append("[%s] %s\n%s" % (
                record.started_at, " ".join(record.argv),
                getattr(record, stream),
            ))
        return "\n\n".join(chunks) + ("\n" if chunks else "")

    gdb = payload.get("gdb") or {}
    server = payload.get("gdb_server") or {}
    restore = payload.get("restore_debug") or payload.get("debug_restore") or {}
    write_text("gdb_stdout.log", "\n".join(gdb.get("stdout", [])) +
               ("\n" if gdb.get("stdout") else ""))
    write_text("gdb_stderr.log", "\n".join(gdb.get("stderr", [])) +
               ("\n" if gdb.get("stderr") else ""))
    write_text("gdb_server_stdout.log", "\n".join(server.get("output", [])) +
               ("\n" if server.get("output") else ""))
    write_text("gdb_server_stderr.log", "\n".join(server.get("stderr", [])) +
               ("\n" if server.get("stderr") else ""))
    write_text("programmer_stdout.log", process_log("stm32_programmer_cli", "stdout"))
    write_text("programmer_stderr.log", process_log("stm32_programmer_cli", "stderr"))
    write_text("restore_debug.json",
               json.dumps(restore, indent=2, ensure_ascii=False, default=str) + "\n")
    write_text("command_timeline.json", json.dumps({
        "lifecycle": payload.get("timeline", []),
        "commands": payload["commands"],
    }, indent=2, ensure_ascii=False, default=str) + "\n")
    write_text("process_snapshot.txt", json.dumps({
        "gdb": gdb,
        "gdb_server": server,
        "restore_debug": restore,
    }, indent=2, ensure_ascii=False, default=str) + "\n")
    (directory / "run.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, default=str) + "\n",
        encoding="utf-8",
    )
    (directory / "AUTOTUNE_SAFE_SWD_SESSION_VALIDATION.md").write_text(
        render_report(
            failure_code=None if result.code == "OK" else result.code,
            evidence=payload,
        ),
        encoding="utf-8",
    )
    return directory


def monitor_local_request(gdb: Path, elf: Path, address: int,
                          plan: Sequence[Tuple[str, int]], serial: Optional[str],
                          monitor_seconds: float, poll_ms: int,
                          gdb_host: str, gdb_port: int) -> List[dict]:
    """Run a request with a live MI monitor and fail closed on monitor errors."""
    del serial  # SWD serial selection belongs to the GDB server invocation.
    offsets = {
        "request_command": REQUEST_COMMAND_OFFSET,
        "request_sequence": REQUEST_SEQUENCE_OFFSET,
        "request_wheel": REQUEST_WHEEL_OFFSET,
        "request_magic": REQUEST_MAGIC_OFFSET,
    }
    snapshots: List[dict] = []
    stop_sequence = (next(value for field, value in plan if field == "request_sequence") + 1) & 0xFFFFFFFF
    stop_plan = build_request_plan("stop", stop_sequence, 0)
    target_running = False
    last_snapshot: Optional[dict] = None
    with GdbMiSession(gdb, elf, gdb_host, gdb_port) as session:
        for field, value in plan:
            session.write_word(address + offsets[field], value)
        session.continue_target()
        target_running = True
        deadline = time.monotonic() + monitor_seconds
        try:
            # Do not halt the MCU periodically: pausing the 10 ms control
            # scheduler creates a false control-overrun fault. The MCU-local
            # safety layer remains live for the whole experiment. Wait in
            # short slices only to keep the host process interruptible, then
            # take one final snapshot in the same debugger session.
            while time.monotonic() < deadline:
                time.sleep(min(1.0, max(0.05, deadline - time.monotonic())))
            session.interrupt_target()
            target_running = False
            snapshot = decode_local_control_header(
                session.read_bytes(address, LOCAL_STATUS_HEADER_BYTES)
            )
            last_snapshot = snapshot
            snapshot["timestamp_monotonic"] = time.monotonic()
            snapshots.append(snapshot)
            print("LOCAL_STATUS " + repr(snapshot))
        except Exception:
            # The target is expected to be halted after a failed poll. If it
            # is still reachable, write an ordinary STOP through this same
            # connection. If the debugger itself failed, MCU-local watchdogs
            # remain the final safety authority.
            try:
                if target_running:
                    session.interrupt_target()
                    target_running = False
                for field, value in stop_plan:
                    session.write_word(address + offsets[field], value)
                session.continue_target()
                target_running = True
            except Exception:
                pass
            raise
        finally:
            try:
                if target_running:
                    session.interrupt_target()
                    target_running = False
                if last_snapshot is None or last_snapshot["state"] != 10:
                    for field, value in stop_plan:
                        session.write_word(address + offsets[field], value)
                session.continue_target()
            except Exception:
                pass
    return snapshots


def build_request_plan(command: str, sequence: int, wheel: int) -> List[Tuple[str, int]]:
    if command == "start":
        if wheel not in (1, 2):
            raise ValueError("START requires --wheel 1 or 2")
        command_value = COMMAND_START
        wheel_value = wheel
    elif command == "stop":
        command_value = COMMAND_STOP
        wheel_value = 0
    elif command == "recover":
        command_value = COMMAND_RECOVER
        wheel_value = 0
    else:
        raise ValueError("request must be start or stop")

    # Write the magic last so a running MCU never observes a half-written
    # request as valid.  No entry exists for level/PWM/safety configuration.
    return [
        ("request_command", command_value),
        ("request_sequence", sequence & 0xFFFFFFFF),
        ("request_wheel", wheel_value),
        ("request_magic", REQUEST_MAGIC),
    ]


def _write_command(programmer: Path, address: int, field_offset: int, value: int,
                   serial: Optional[str]) -> Sequence[str]:
    connect = "port=SWD"
    if serial:
        connect += " sn=" + serial
    return [str(programmer), "-c", connect, "freq=4000", "-halt",
            "-w32", _hex(address + field_offset), _hex(value)]


def _run_command(programmer: Path, serial: Optional[str]) -> Sequence[str]:
    connect = "port=SWD"
    if serial:
        connect += " sn=" + serial
    return [str(programmer), "-c", connect, "freq=4000", "-run"]


def _connect_command(programmer: Path, serial: Optional[str]) -> List[str]:
    connect = "port=SWD"
    if serial:
        connect += " sn=" + serial
    return [str(programmer), "-c", connect, "freq=4000"]


def _read_command(programmer: Path, address: int, words: int,
                  serial: Optional[str]) -> Sequence[str]:
    connect = "port=SWD"
    if serial:
        connect += " sn=" + serial
    return [str(programmer), "-c", connect, "freq=4000",
            "-r32", _hex(address), str(words)]


def execute_request(programmer: Path, address: int, plan: Sequence[Tuple[str, int]],
                    serial: Optional[str]) -> List[dict]:
    if not programmer.exists():
        raise RuntimeError("STM32_Programmer_CLI not found: " + str(programmer))
    offsets = {
        "request_command": REQUEST_COMMAND_OFFSET,
        "request_sequence": REQUEST_SEQUENCE_OFFSET,
        "request_wheel": REQUEST_WHEEL_OFFSET,
        "request_magic": REQUEST_MAGIC_OFFSET,
    }
    # CubeProgrammer's -run performs a software reset. Write the request into
    # the retained boot handoff, then let the profile consume it after reset;
    # the ordinary .bss mailbox would be cleared before the app sees it.
    command = _connect_command(programmer, serial) + ["-halt"]
    for field, value in plan:
        if field == "request_magic":
            value = BOOT_MAGIC
        command.extend(["-w32", _hex(address + offsets[field]), _hex(value)])
    command.append("-run")
    result = subprocess.run(command, cwd=ROOT, capture_output=True,
                            text=True, check=False)
    if result.returncode != 0:
        raise RuntimeError("ST-LINK request write/resume failed: " +
                           result.stderr[-500:])
    return [{"field": "boot_handoff_and_resume", "value": None,
             "returncode": result.returncode,
             "stdout": result.stdout[-2000:],
             "stderr": result.stderr[-1000:]}]


def read_status(programmer: Path, address: int, serial: Optional[str]) -> dict:
    if not programmer.exists():
        raise RuntimeError("STM32_Programmer_CLI not found: " + str(programmer))
    result = subprocess.run(
        _read_command(programmer, address, 16, serial),
        cwd=ROOT, capture_output=True, text=True, check=False,
    )
    if result.returncode != 0:
        raise RuntimeError("ST-LINK RAM read failed")
    return {"returncode": result.returncode,
            "stdout": result.stdout[-12000:],
            "stderr": result.stderr[-2000:]}


def dry_probe_tools(programmer: Path, gdb: Path, gdb_server: Path,
                    serial: Optional[str]) -> dict:
    runner = CommandRunner()
    evidence = {"paths": {"programmer": str(programmer),
                           "gdb": str(gdb),
                           "gdb_server": str(gdb_server)},
                "commands": []}
    for tool, argv in (
        ("programmer", [str(programmer), "--version"]),
        ("gdb", [str(gdb), "--version"]),
    ):
        if not Path(argv[0]).exists():
            evidence[tool] = {"available": False}
            continue
        record = runner.run(argv, timeout_s=10.0)
        evidence[tool] = {"available": record.returncode == 0,
                          "returncode": record.returncode,
                          "stdout": record.stdout[-2000:],
                          "stderr": record.stderr[-1000:]}
    server_process = GdbServerProcess(
        gdb_server, "localhost", 61234, serial, command_runner=runner,
    )
    help_record = server_process.prove_no_reset_capability()
    debugger_record = runner.run([
        str(gdb_server), "--debuggers",
        "--stm32cubeprogrammer-path", str(CUBE_PROGRAMMER_DIR),
    ], timeout_s=10.0)
    evidence["gdb_server_help"] = {
        "returncode": help_record.returncode,
        "stdout": help_record.stdout[-6000:],
        "stderr": help_record.stderr[-1000:],
        "no_reset_attach_proven": True,
        "argv": server_process.no_reset_argv(),
    }
    evidence["target_enum"] = {
        "returncode": debugger_record.returncode,
        "stdout": debugger_record.stdout[-2000:],
        "stderr": debugger_record.stderr[-1000:],
        "serial_requested": serial,
    }
    evidence["commands"] = [record.__dict__ for record in runner.evidence]
    return evidence


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", choices=("start", "stop", "recover"))
    parser.add_argument("--wheel", type=int, default=0, help="1=left, 2=right; required for start")
    parser.add_argument("--sequence", type=int,
                        default=(int(time.time() * 1000) & 0xFFFFFFFF))
    parser.add_argument("--elf", type=Path, default=ELF_DEFAULT)
    parser.add_argument("--programmer", type=Path, default=PROGRAMMER_DEFAULT)
    parser.add_argument("--gdb", type=Path, default=GDB_DEFAULT)
    parser.add_argument("--gdb-server", type=Path, default=GDB_SERVER_DEFAULT)
    parser.add_argument("--smoke-elf", type=Path, default=SMOKE_ELF_DEFAULT)
    parser.add_argument("--debug-elf", type=Path, default=DEBUG_ELF_DEFAULT)
    parser.add_argument("--gdb-host", default="localhost")
    parser.add_argument("--gdb-port", type=int, default=61234)
    parser.add_argument("--monitor-seconds", type=float, default=0.0,
                        help="monitor through one GDB/MI session; requires a running ST-LINK GDB server")
    parser.add_argument("--poll-ms", type=int, default=250,
                        help="GDB/MI status poll interval while monitoring")
    parser.add_argument("--serial", default=None)
    parser.add_argument("--read-status", action="store_true",
                        help="after an executed request, read the local RAM mailbox")
    parser.add_argument("--execute", action="store_true",
                        help="actually halt MCU and write the request mailbox")
    parser.add_argument("--session-smoke", action="store_true",
                        help="run the deterministic actuator-free SWD session")
    parser.add_argument("--session-count", type=int, default=1,
                        help="number of independent smoke transactions")
    parser.add_argument("--output-root", type=Path, default=ROOT / "autotune_runs",
                        help="root directory for per-session evidence artifacts")
    parser.add_argument("--dry-probe", action="store_true",
                        help="probe tool help/version and no-reset capability only")
    args = parser.parse_args(argv)

    if args.dry_probe:
        try:
            print(json.dumps(dry_probe_tools(args.programmer, args.gdb,
                                              args.gdb_server, args.serial),
                             indent=2, ensure_ascii=False))
            return 0
        except (RuntimeError, OrchestrationFailure) as exc:
            print("ATTACH_FAIL: " + str(exc))
            return 2

    if args.session_smoke:
        if args.request is not None:
            parser.error("--session-smoke cannot be combined with --request")
        if args.session_count < 1:
            parser.error("--session-count must be positive")
        if not args.execute:
            print("DRY_RUN: session smoke would require --execute; no flash performed")
            return 0
        for index in range(args.session_count):
            backend = HardwareSmokeBackend(
                args.smoke_elf, args.debug_elf, args.programmer, args.gdb,
                args.gdb_server, args.gdb_host, args.gdb_port, args.serial,
                args.poll_ms,
            )
            result = backend.run_hardware_smoke()
            artifact = save_hardware_artifact(args.output_root, index + 1,
                                              backend, result)
            print("SESSION_SMOKE_%02d %s %s" %
                  (index + 1, result.category, result.code))
            print("SESSION_SMOKE_ARTIFACT %s" % artifact)
            if result.code != "OK":
                print(render_report(failure_code=result.code,
                                    evidence={"detail": result.detail,
                                              "events": backend.events}))
                return 1
        print("SESSION_SMOKE_PASS %d/%d" % (args.session_count,
                                             args.session_count))
        return 0

    if args.request is None:
        parser.error("--request is required unless --session-smoke or --dry-probe is used")

    try:
        plan = build_request_plan(args.request, args.sequence, args.wheel)
        address = resolve_control_address(args.elf) if args.execute else None
        boot_address = resolve_boot_request_address(args.elf) if args.execute else None
    except (RuntimeError, ValueError) as exc:
        parser.error(str(exc))

    mode = "EXECUTE" if args.execute else "DRY_RUN"
    print(mode + ": AutotuneSafe local request; MCU remains final safety authority")
    print("L1 policy: target <= %d mm/s, PWM <= %d per mille" %
          (L1_TARGET_LIMIT_MMPS, L1_PWM_LIMIT))
    if address is not None:
        print("g_autotune_safe_local_control = " + _hex(address))
        print("g_autotune_safe_local_boot_request = " + _hex(boot_address))
    for field, value in plan:
        print("%s = %s" % (field, _hex(value) if field == "request_magic" else str(value)))
    if args.execute:
        if args.monitor_seconds > 0.0:
            if args.request != "start":
                parser.error("--monitor-seconds is only valid with --request start")
            if args.read_status:
                parser.error("--monitor-seconds cannot be combined with --read-status")
            if args.poll_ms < 50:
                parser.error("--poll-ms must be at least 50 ms")
            snapshots = monitor_local_request(
                args.gdb, args.elf, address, plan, args.serial,
                args.monitor_seconds, args.poll_ms, args.gdb_host, args.gdb_port,
            )  # type: ignore[arg-type]
            print("monitor complete; snapshots=%d; MCU remains final safety authority" %
                  len(snapshots))
            return 0
        results = execute_request(args.programmer, boot_address, plan, args.serial)  # type: ignore[arg-type]
        if any(row["returncode"] != 0 for row in results):
            return 1
        print("request written; read MCU status/telemetry before any next request")
        if args.read_status:
            print(read_status(args.programmer, address, args.serial)["stdout"])
    else:
        if args.monitor_seconds > 0.0:
            parser.error("--monitor-seconds requires --execute")
        if args.read_status:
            parser.error("--read-status requires --execute")
        print("no ST-LINK write performed; add --execute only after physical L1 preflight")
    return 0


if __name__ == "__main__":
    sys.exit(main())
