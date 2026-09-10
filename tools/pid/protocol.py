from dataclasses import dataclass
from typing import Iterable, List, Tuple

from .model import PIDGains


PROTOCOL_VERSION = 1
CMD_PID_GAINS = 0x123
CMD_PID_D = 0x124
CMD_STEERING = 0x120
CMD_REAR_WHEELS = 0x121
FB_CONTROL_OUTPUT = 0x188
FB_STATUS = 0x180
FB_HEALTH = 0x181
FB_REAR_VELOCITY = 0x182
FB_DIAGNOSTICS = 0x186
FB_PID_GAINS = 0x187
CMD_AUTOTUNE_CONTROL = 0x125
FB_AUTOTUNE_IDENTITY = 0x189
FB_AUTOTUNE_STATE = 0x18A
FB_AUTOTUNE_LIMITS = 0x18B
FB_AUTOTUNE_WHEEL = 0x18C
FB_AUTOTUNE_PID_PI = 0x18D
FB_AUTOTUNE_PID_DO = 0x18E
FB_AUTOTUNE_SAFETY = 0x18F
FB_AUTOTUNE_COUNTERS = 0x190
AUTOTUNE_PROFILE_ID = 0xA1
AUTOTUNE_OPCODE_START = 1
AUTOTUNE_OPCODE_STOP = 2
AUTOTUNE_OPCODE_HEARTBEAT = 3
AUTOTUNE_OPCODE_QUERY = 4
AUTOTUNE_OPCODE_REQUEST_PROMOTE = 5
PID_AXIS_LEFT = 0x01
PID_AXIS_RIGHT = 0x02
PID_AXIS_BOTH = PID_AXIS_LEFT | PID_AXIS_RIGHT
PID_COMMIT_COOKIE = 0xC35A


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes


@dataclass(frozen=True)
class AutotuneTelemetry:
    profile_id: int
    session_id: int
    experiment_id: int
    snapshot_seq: int
    autotune_state: int
    level: int
    requested_target: int
    effective_target: int
    pwm_limit: int
    actual_left: int
    actual_right: int
    pwm_left: int
    pwm_right: int
    pid_left: Tuple[float, float, float, float]
    pid_right: Tuple[float, float, float, float]
    stall: int
    saturation: int
    overspeed: int
    oscillation: int
    abort_reason: int
    speed_limit_hit: int
    safety_state: int
    fault_code: int
    last_valid_command_age: int
    stall_time: int
    saturation_time: int
    oscillation_count: int


def _u16(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError("value must fit u16")
    return value.to_bytes(2, "little")


def _i16(value: int) -> bytes:
    if not -0x8000 <= value <= 0x7FFF:
        raise ValueError("value must fit signed i16")
    return value.to_bytes(2, "little", signed=True)


def _read_u16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "little")


def _read_i16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=True)


def _encode_signed_q8_8(value: float) -> int:
    if value < -128.0 or value > 127.996:
        raise ValueError("signed telemetry value is outside Q8.8 range")
    return int(round(value * 256.0))


def _decode_signed_q8_8(raw: int) -> float:
    if raw & 0x8000:
        raw -= 0x10000
    return raw / 256.0


def encode_autotune_control(opcode: int, session_id: int,
                            experiment_id: int, argument: int) -> CanFrame:
    if opcode not in (AUTOTUNE_OPCODE_START, AUTOTUNE_OPCODE_STOP,
                      AUTOTUNE_OPCODE_HEARTBEAT, AUTOTUNE_OPCODE_QUERY,
                      AUTOTUNE_OPCODE_REQUEST_PROMOTE):
        raise ValueError("unknown autotune opcode")
    if not 0 <= session_id <= 0xFFFF or not 0 <= experiment_id <= 0xFFFF:
        raise ValueError("autotune identifiers must fit u16")
    argument_bytes = _i16(argument) if opcode == AUTOTUNE_OPCODE_START else _u16(argument)
    return CanFrame(
        CMD_AUTOTUNE_CONTROL,
        bytes((PROTOCOL_VERSION, opcode)) + _u16(session_id) +
        _u16(experiment_id) + argument_bytes,
    )


def decode_autotune_control(data: bytes):
    if len(data) != 8 or data[0] != PROTOCOL_VERSION:
        raise ValueError("invalid autotune control frame")
    opcode = data[1]
    if opcode not in (AUTOTUNE_OPCODE_START, AUTOTUNE_OPCODE_STOP,
                      AUTOTUNE_OPCODE_HEARTBEAT, AUTOTUNE_OPCODE_QUERY,
                      AUTOTUNE_OPCODE_REQUEST_PROMOTE):
        raise ValueError("unknown autotune opcode")
    argument = _read_i16(data, 6) if opcode == AUTOTUNE_OPCODE_START else _read_u16(data, 6)
    return opcode, _read_u16(data, 2), _read_u16(data, 4), argument


def _frame_seq(frame: CanFrame) -> int:
    if frame.arbitration_id == FB_AUTOTUNE_IDENTITY:
        return frame.data[2]
    return frame.data[1]


def encode_autotune_telemetry(telemetry: AutotuneTelemetry) -> List[CanFrame]:
    if not 0 <= telemetry.snapshot_seq <= 0xFF:
        raise ValueError("snapshot sequence must fit u8")
    identity = bytes((PROTOCOL_VERSION, telemetry.profile_id & 0xFF,
                      telemetry.snapshot_seq)) + _u16(telemetry.session_id) + \
        _u16(telemetry.experiment_id) + bytes(1)
    state = bytes((PROTOCOL_VERSION, telemetry.snapshot_seq,
                   telemetry.autotune_state & 0xFF, telemetry.level & 0xFF,
                   telemetry.safety_state & 0xFF, telemetry.abort_reason & 0xFF)) + \
        _u16(telemetry.fault_code)
    limits = bytes((PROTOCOL_VERSION, telemetry.snapshot_seq)) + \
        _i16(telemetry.requested_target) + _i16(telemetry.effective_target) + \
        _u16(telemetry.pwm_limit)

    def wheel(axis: int, actual: int, pwm: int) -> CanFrame:
        return CanFrame(FB_AUTOTUNE_WHEEL,
                        bytes((PROTOCOL_VERSION, telemetry.snapshot_seq, axis)) +
                        _i16(actual) + _i16(pwm) + bytes(1))

    def pid(axis: int, values: Tuple[float, float, float, float]):
        first = bytes((PROTOCOL_VERSION, telemetry.snapshot_seq, axis)) + \
            _i16(_encode_signed_q8_8(values[0])) + \
            _i16(_encode_signed_q8_8(values[1])) + bytes(1)
        second = bytes((PROTOCOL_VERSION, telemetry.snapshot_seq, axis)) + \
            _i16(_encode_signed_q8_8(values[2])) + \
            _i16(_encode_signed_q8_8(values[3])) + bytes(1)
        return CanFrame(FB_AUTOTUNE_PID_PI, first), CanFrame(FB_AUTOTUNE_PID_DO, second)

    flags = ((1 if telemetry.stall else 0) |
             (2 if telemetry.saturation else 0) |
             (4 if telemetry.overspeed else 0) |
             (8 if telemetry.oscillation else 0) |
             (16 if telemetry.speed_limit_hit else 0))
    safety = bytes((PROTOCOL_VERSION, telemetry.snapshot_seq, flags,
                    telemetry.abort_reason & 0xFF)) + \
        _u16(telemetry.stall_time) + _u16(telemetry.saturation_time)
    counters = bytes((PROTOCOL_VERSION, telemetry.snapshot_seq)) + \
        _u16(telemetry.oscillation_count) + _u16(telemetry.last_valid_command_age) + bytes(2)
    left_pi, left_do = pid(1, telemetry.pid_left)
    right_pi, right_do = pid(2, telemetry.pid_right)
    return [
        CanFrame(FB_AUTOTUNE_IDENTITY, identity), CanFrame(FB_AUTOTUNE_STATE, state),
        CanFrame(FB_AUTOTUNE_LIMITS, limits), wheel(1, telemetry.actual_left, telemetry.pwm_left),
        wheel(2, telemetry.actual_right, telemetry.pwm_right), left_pi, left_do,
        right_pi, right_do, CanFrame(FB_AUTOTUNE_SAFETY, safety),
        CanFrame(FB_AUTOTUNE_COUNTERS, counters),
    ]


def decode_autotune_telemetry(frames: Iterable[CanFrame]) -> AutotuneTelemetry:
    rows = list(frames)
    required = {FB_AUTOTUNE_IDENTITY, FB_AUTOTUNE_STATE, FB_AUTOTUNE_LIMITS,
                FB_AUTOTUNE_WHEEL, FB_AUTOTUNE_PID_PI, FB_AUTOTUNE_PID_DO,
                FB_AUTOTUNE_SAFETY, FB_AUTOTUNE_COUNTERS}
    by_id = {}
    for frame in rows:
        if frame.arbitration_id in required:
            if len(frame.data) != 8 or frame.arbitration_id in by_id:
                if frame.arbitration_id not in (FB_AUTOTUNE_WHEEL, FB_AUTOTUNE_PID_PI,
                                                FB_AUTOTUNE_PID_DO):
                    raise ValueError("duplicate or malformed autotune frame")
            by_id.setdefault(frame.arbitration_id, []).append(frame)
    if not required.issubset(by_id):
        raise ValueError("incomplete autotune telemetry snapshot")
    identity = by_id[FB_AUTOTUNE_IDENTITY][0]
    identity_data = identity.data
    seq = _frame_seq(identity)
    if any(_frame_seq(frame) != seq for frame in rows if frame.arbitration_id in required):
        raise ValueError("mixed autotune telemetry snapshot sequence")
    if len(by_id[FB_AUTOTUNE_WHEEL]) != 2 or len(by_id[FB_AUTOTUNE_PID_PI]) != 2 or \
            len(by_id[FB_AUTOTUNE_PID_DO]) != 2:
        raise ValueError("autotune telemetry must contain both wheels")

    def axis_frame(frame_id: int, axis: int) -> CanFrame:
        return next(frame for frame in by_id[frame_id] if frame.data[2] == axis)

    state = by_id[FB_AUTOTUNE_STATE][0].data
    limits = by_id[FB_AUTOTUNE_LIMITS][0].data
    safety = by_id[FB_AUTOTUNE_SAFETY][0].data
    counters = by_id[FB_AUTOTUNE_COUNTERS][0].data
    left_wheel = axis_frame(FB_AUTOTUNE_WHEEL, 1).data
    right_wheel = axis_frame(FB_AUTOTUNE_WHEEL, 2).data

    def pid_values(axis: int) -> Tuple[float, float, float, float]:
        pi = axis_frame(FB_AUTOTUNE_PID_PI, axis).data
        do = axis_frame(FB_AUTOTUNE_PID_DO, axis).data
        return (_decode_signed_q8_8(_read_u16(pi, 3)),
                _decode_signed_q8_8(_read_u16(pi, 5)),
                _decode_signed_q8_8(_read_u16(do, 3)),
                _decode_signed_q8_8(_read_u16(do, 5)))

    flags = safety[2]
    return AutotuneTelemetry(
        profile_id=identity_data[1], session_id=_read_u16(identity_data, 3),
        experiment_id=_read_u16(identity_data, 5), snapshot_seq=seq,
        autotune_state=state[2], level=state[3], requested_target=_read_i16(limits, 2),
        effective_target=_read_i16(limits, 4), pwm_limit=_read_u16(limits, 6),
        actual_left=_read_i16(left_wheel, 3), actual_right=_read_i16(right_wheel, 3),
        pwm_left=_read_i16(left_wheel, 5), pwm_right=_read_i16(right_wheel, 5),
        pid_left=pid_values(1), pid_right=pid_values(2), stall=int(bool(flags & 1)),
        saturation=int(bool(flags & 2)), overspeed=int(bool(flags & 4)),
        oscillation=int(bool(flags & 8)), abort_reason=safety[3],
        speed_limit_hit=int(bool(flags & 16)), safety_state=state[4],
        fault_code=_read_u16(state, 6), last_valid_command_age=_read_u16(counters, 4),
        stall_time=_read_u16(safety, 4), saturation_time=_read_u16(safety, 6),
        oscillation_count=_read_u16(counters, 2),
    )


def _encode_gain(value: float) -> int:
    if value < 0.0 or value > 255.996:
        raise ValueError("PID gain is outside Q8.8 range")
    return int(round(value * 256.0))


def _decode_gain(raw: int) -> float:
    return raw / 256.0


def _header(sequence: int, axis_mask: int) -> bytearray:
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("PID transaction sequence must fit u16")
    if axis_mask not in (PID_AXIS_LEFT, PID_AXIS_RIGHT, PID_AXIS_BOTH):
        raise ValueError("PID axis mask must select left, right, or both wheels")
    return bytearray((PROTOCOL_VERSION, sequence & 0xFF, sequence >> 8, axis_mask))


def encode_pid_transaction(sequence: int, gains: PIDGains, axis_mask: int = PID_AXIS_BOTH) -> List[CanFrame]:
    first = _header(sequence, axis_mask)
    kp = _encode_gain(gains.kp)
    ki = _encode_gain(gains.ki)
    first.extend((kp & 0xFF, kp >> 8, ki & 0xFF, ki >> 8))

    second = _header(sequence, axis_mask)
    kd = _encode_gain(gains.kd)
    second.extend((kd & 0xFF, kd >> 8, PID_COMMIT_COOKIE & 0xFF, PID_COMMIT_COOKIE >> 8))
    return [CanFrame(CMD_PID_GAINS, bytes(first)), CanFrame(CMD_PID_D, bytes(second))]


def decode_pid_transaction(frames: Iterable[CanFrame]) -> PIDGains:
    rows = list(frames)
    if len(rows) != 2 or rows[0].arbitration_id != CMD_PID_GAINS or rows[1].arbitration_id != CMD_PID_D:
        raise ValueError("PID transaction must contain CMD_PID_GAINS followed by CMD_PID_D")
    if len(rows[0].data) != 8 or len(rows[1].data) != 8:
        raise ValueError("PID frames must use DLC 8")
    if rows[0].data[0] != PROTOCOL_VERSION or rows[1].data[0] != PROTOCOL_VERSION:
        raise ValueError("unsupported PID protocol version")
    if rows[0].data[1:4] != rows[1].data[1:4]:
        raise ValueError("PID transaction headers do not match")
    cookie = rows[1].data[6] | (rows[1].data[7] << 8)
    if cookie != PID_COMMIT_COOKIE:
        raise ValueError("invalid PID transaction cookie")
    kp = rows[0].data[4] | (rows[0].data[5] << 8)
    ki = rows[0].data[6] | (rows[0].data[7] << 8)
    kd = rows[1].data[4] | (rows[1].data[5] << 8)
    return PIDGains(_decode_gain(kp), _decode_gain(ki), _decode_gain(kd))


def decode_pid_ack(data: bytes):
    if len(data) != 8 or data[0] != PROTOCOL_VERSION or data[5:] != bytes(3):
        raise ValueError("invalid PID acknowledgement frame")
    if data[3] > 4 or data[4] not in (PID_AXIS_LEFT, PID_AXIS_RIGHT, PID_AXIS_BOTH, 0):
        raise ValueError("invalid PID acknowledgement status or axis")
    return (data[1] | (data[2] << 8), data[3], data[4])


def encode_velocity_group(sequence: int, left_velocity_mmps: int, right_velocity_mmps: int,
                          mode_flags: int = 0x01, steering_mrad: int = 0) -> List[CanFrame]:
    if not -3000 <= left_velocity_mmps <= 3000 or not -3000 <= right_velocity_mmps <= 3000:
        raise ValueError("wheel velocity is outside the frozen V1 envelope")
    if not -1000 <= steering_mrad <= 1000:
        raise ValueError("steering target is outside the frozen V1 envelope")
    header = bytes((PROTOCOL_VERSION, sequence & 0xFF, (sequence >> 8) & 0xFF, mode_flags & 0xFF))
    steering = header + int(steering_mrad).to_bytes(2, "little", signed=True) + bytes(2)
    wheels = header + int(left_velocity_mmps).to_bytes(2, "little", signed=True) + \
        int(right_velocity_mmps).to_bytes(2, "little", signed=True)
    return [CanFrame(CMD_STEERING, steering), CanFrame(CMD_REAR_WHEELS, wheels)]


def decode_control_output(data: bytes):
    if len(data) != 8 or data[0] != PROTOCOL_VERSION or data[3] & 0xFC:
        raise ValueError("invalid control-output feedback frame")
    if (data[3] & 0x03) != 0x03:
        raise ValueError("control-output feedback does not contain both wheels")
    return (int.from_bytes(data[4:6], "little", signed=True),
            int.from_bytes(data[6:8], "little", signed=True))
