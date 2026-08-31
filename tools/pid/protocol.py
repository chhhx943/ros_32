from dataclasses import dataclass
from typing import Iterable, List

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
PID_AXIS_LEFT = 0x01
PID_AXIS_RIGHT = 0x02
PID_AXIS_BOTH = PID_AXIS_LEFT | PID_AXIS_RIGHT
PID_COMMIT_COOKIE = 0xC35A


@dataclass(frozen=True)
class CanFrame:
    arbitration_id: int
    data: bytes


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
