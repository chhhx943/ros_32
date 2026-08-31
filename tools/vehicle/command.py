import logging
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, Optional, Tuple


from tools.pid.protocol import CanFrame, encode_velocity_group

from .ackermann import AckermannResult, MotionSafetyStatus


MAX_STEERING_MRAD_PROTOCOL = 349
LOGGER = logging.getLogger(__name__)

FrameSender = Callable[[CanFrame], None]
AuditSink = Callable[[Dict[str, object]], None]


class CommandCycleStatus(str, Enum):
    COMMAND_COMMITTED = "command_committed"
    INVALID_RESULT = "invalid_result"
    STEERING_TX_FAILED = "steering_tx_failed"
    WHEELS_TX_FAILED = "wheels_tx_failed"
    SAFE_STOP_COMMITTED = "safe_stop_committed"
    WATCHDOG_FALLBACK = "watchdog_fallback"


@dataclass(frozen=True)
class CommandCycleResult:
    status: CommandCycleStatus
    compute_ok: bool
    steering_tx_ok: bool
    wheels_tx_ok: bool
    command_committed: bool
    safe_stop_requested: bool
    safe_stop_tx_ok: Optional[bool]
    watchdog_fallback: bool
    error: Optional[str]


def _validate_sequence(sequence: object) -> int:
    if (
        isinstance(sequence, bool)
        or not isinstance(sequence, int)
        or sequence < 0
        or sequence > 0xFFFF
    ):
        raise ValueError("command sequence must fit u16")
    return sequence


def build_motion_frames(
    result: AckermannResult,
    sequence: int,
) -> Tuple[CanFrame, CanFrame]:
    if not isinstance(result, AckermannResult) or not result.valid:
        raise ValueError("invalid Ackermann result cannot form a motion command")
    sequence = _validate_sequence(sequence)

    steering_mrad = round(result.applied_steering_rad * 1000.0)
    steering_mrad = max(
        -MAX_STEERING_MRAD_PROTOCOL,
        min(MAX_STEERING_MRAD_PROTOCOL, steering_mrad),
    )
    frames = encode_velocity_group(
        sequence=sequence,
        left_velocity_mmps=round(result.left_wheel_speed_mm_s),
        right_velocity_mmps=round(result.right_wheel_speed_mm_s),
        mode_flags=0x01,
        steering_mrad=steering_mrad,
    )
    return frames[0], frames[1]


def _motion_cycle(
    status: CommandCycleStatus,
    compute_ok: bool,
    steering_tx_ok: bool = False,
    wheels_tx_ok: bool = False,
    committed: bool = False,
    error: Optional[str] = None,
) -> CommandCycleResult:
    return CommandCycleResult(
        status=status,
        compute_ok=compute_ok,
        steering_tx_ok=steering_tx_ok,
        wheels_tx_ok=wheels_tx_ok,
        command_committed=committed,
        safe_stop_requested=not committed,
        safe_stop_tx_ok=None,
        watchdog_fallback=False,
        error=error,
    )


def _safe_stop_cycle(
    steering_tx_ok: bool,
    wheels_tx_ok: bool,
    error: Optional[str] = None,
) -> CommandCycleResult:
    committed = steering_tx_ok and wheels_tx_ok
    return CommandCycleResult(
        status=(
            CommandCycleStatus.SAFE_STOP_COMMITTED
            if committed
            else CommandCycleStatus.WATCHDOG_FALLBACK
        ),
        compute_ok=True,
        steering_tx_ok=steering_tx_ok,
        wheels_tx_ok=wheels_tx_ok,
        command_committed=committed,
        safe_stop_requested=not committed,
        safe_stop_tx_ok=committed,
        watchdog_fallback=not committed,
        error=error,
    )


def _quantized_steering_mrad(result: AckermannResult) -> int:
    value = round(result.applied_steering_rad * 1000.0)
    return max(-MAX_STEERING_MRAD_PROTOCOL, min(MAX_STEERING_MRAD_PROTOCOL, value))


def build_audit_record(
    result: AckermannResult,
    cycle: CommandCycleResult,
    timestamp: Optional[float] = None,
) -> Dict[str, object]:
    return {
        "timestamp": time.time() if timestamp is None else timestamp,
        "requested_speed_mm_s": result.requested_speed_mm_s,
        "requested_steering_mrad": result.requested_steering_rad * 1000.0,
        "applied_speed_mm_s": result.applied_speed_mm_s,
        "applied_steering_mrad": _quantized_steering_mrad(result),
        "curvature_per_mm": result.curvature_per_mm,
        "rear_axle_radius_mm": result.rear_axle_radius_signed_mm,
        "rear_axle_radius_abs_mm": result.rear_axle_radius_abs_mm,
        "left_target_mm_s": result.left_wheel_speed_mm_s,
        "right_target_mm_s": result.right_wheel_speed_mm_s,
        "inner_side": result.inner_side,
        "outer_side": result.outer_side,
        "steering_limited": result.steering_limited,
        "wheel_speed_limited": result.wheel_speed_limited,
        "wheel_speed_scale": result.wheel_speed_scale,
        "result_valid": result.valid,
        "safety_status": result.safety_status.value,
        "reason": result.reason,
        "steering_tx_ok": cycle.steering_tx_ok,
        "wheels_tx_ok": cycle.wheels_tx_ok,
        "command_committed": cycle.command_committed,
        "command_status": cycle.status.value,
        "safe_stop_requested": cycle.safe_stop_requested,
        "safe_stop_tx_ok": cycle.safe_stop_tx_ok,
        "watchdog_fallback": cycle.watchdog_fallback,
        "error": cycle.error,
    }


def _emit_audit(
    result: AckermannResult,
    cycle: CommandCycleResult,
    audit_sink: Optional[AuditSink],
    timestamp: Optional[float],
) -> None:
    record = build_audit_record(result, cycle, timestamp)
    if audit_sink is None:
        LOGGER.info("vehicle_command %s", record)
    else:
        audit_sink(record)


def send_motion_command(
    result: AckermannResult,
    sequence: int,
    send_frame: FrameSender,
    audit_sink: Optional[AuditSink] = None,
    timestamp: Optional[float] = None,
) -> CommandCycleResult:
    if not isinstance(result, AckermannResult) or not result.valid:
        cycle = _motion_cycle(
            status=CommandCycleStatus.INVALID_RESULT,
            compute_ok=False,
        )
        if isinstance(result, AckermannResult):
            _emit_audit(result, cycle, audit_sink, timestamp)
        return cycle

    steering_frame, wheels_frame = build_motion_frames(result, sequence)
    try:
        send_frame(steering_frame)
    except Exception as exc:
        cycle = _motion_cycle(
            status=CommandCycleStatus.STEERING_TX_FAILED,
            compute_ok=True,
            error=str(exc),
        )
    else:
        try:
            send_frame(wheels_frame)
        except Exception as exc:
            cycle = _motion_cycle(
                status=CommandCycleStatus.WHEELS_TX_FAILED,
                compute_ok=True,
                steering_tx_ok=True,
                error=str(exc),
            )
        else:
            cycle = _motion_cycle(
                status=CommandCycleStatus.COMMAND_COMMITTED,
                compute_ok=True,
                steering_tx_ok=True,
                wheels_tx_ok=True,
                committed=True,
            )

    _emit_audit(result, cycle, audit_sink, timestamp)
    return cycle


def _safe_stop_result() -> AckermannResult:
    return AckermannResult(
        valid=True,
        requested_speed_mm_s=0.0,
        requested_steering_rad=0.0,
        applied_speed_mm_s=0.0,
        applied_steering_rad=0.0,
        curvature_per_mm=0.0,
        rear_axle_radius_signed_mm=math.inf,
        rear_axle_radius_abs_mm=math.inf,
        left_wheel_speed_mm_s=0.0,
        right_wheel_speed_mm_s=0.0,
        inner_side=None,
        outer_side=None,
        wheel_speed_scale=1.0,
        steering_limited=False,
        wheel_speed_limited=False,
        safety_status=MotionSafetyStatus.OK,
        reason=None,
    )


def send_safe_stop(
    sequence: int,
    send_frame: FrameSender,
    audit_sink: Optional[AuditSink] = None,
    timestamp: Optional[float] = None,
) -> CommandCycleResult:
    sequence = _validate_sequence(sequence)
    frames = encode_velocity_group(
        sequence=sequence,
        left_velocity_mmps=0,
        right_velocity_mmps=0,
        mode_flags=0x00,
        steering_mrad=0,
    )
    try:
        send_frame(frames[0])
    except Exception as exc:
        cycle = _safe_stop_cycle(False, False, str(exc))
    else:
        try:
            send_frame(frames[1])
        except Exception as exc:
            cycle = _safe_stop_cycle(True, False, str(exc))
        else:
            cycle = _safe_stop_cycle(True, True)

    _emit_audit(_safe_stop_result(), cycle, audit_sink, timestamp)
    return cycle
