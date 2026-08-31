import math
import time
from typing import List, Optional

from .model import PIDGains, Sample
from .protocol import (CMD_REAR_WHEELS, CMD_STEERING, FB_CONTROL_OUTPUT, FB_DIAGNOSTICS,
                       FB_HEALTH, FB_PID_GAINS, FB_REAR_VELOCITY, FB_STATUS, CanFrame,
                       decode_control_output, decode_pid_ack, encode_pid_transaction,
                       encode_velocity_group)


class TransportError(RuntimeError):
    pass


class TransportSafetyError(TransportError):
    pass


class SimulationTransport:
    """Small deterministic first-order wheel model with the same PID shape as MCU control."""

    def __init__(self, dt_s: float = 0.01, time_constant_s: float = 0.18, max_speed_mmps: float = 2200.0):
        if dt_s <= 0.0 or time_constant_s <= 0.0 or max_speed_mmps <= 0.0:
            raise ValueError("simulation parameters must be positive")
        self.dt_s = dt_s
        self.time_constant_s = time_constant_s
        self.max_speed_mmps = max_speed_mmps
        self.gains: Optional[PIDGains] = None
        self.actual = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.time_s = 0.0
        self.running = False

    def set_pid(self, gains: PIDGains) -> None:
        if self.running:
            raise TransportSafetyError("PID gains may only change while stopped")
        self.gains = gains
        self.integral = 0.0
        self.previous_error = 0.0

    def start_trial(self) -> None:
        if self.gains is None:
            raise TransportError("PID gains must be set before starting a trial")
        self.running = True
        self.actual = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.time_s = 0.0

    def run_step(self, target: float, duration_s: float) -> List[Sample]:
        if not self.running or self.gains is None:
            raise TransportError("trial is not running")
        if duration_s <= 0.0:
            raise ValueError("duration must be positive")
        samples: List[Sample] = []
        count = int(math.ceil(duration_s / self.dt_s))
        for _ in range(count):
            error = target - self.actual
            self.integral += error * self.dt_s
            derivative = (error - self.previous_error) / self.dt_s
            raw_output = (self.gains.kp * error + self.gains.ki * self.integral + self.gains.kd * derivative)
            control_output = max(-1000.0, min(1000.0, raw_output))
            acceleration = ((control_output / 1000.0) * self.max_speed_mmps - self.actual) / self.time_constant_s
            self.actual += acceleration * self.dt_s
            self.previous_error = error
            self.time_s += self.dt_s
            samples.append(Sample(self.time_s, target, self.actual, control_output,
                                   self.gains.kp, self.gains.ki, self.gains.kd))
        return samples

    def stop(self) -> None:
        self.running = False


class PythonCanTransport:
    """python-can transport for the frozen V1 command/feedback frames."""

    def __init__(self, channel: str = "can0", interface: str = "socketcan", bitrate: int = 1_000_000,
                 ack_timeout_s: float = 0.25):
        try:
            import can  # type: ignore
        except ImportError as exc:
            raise TransportError("live CAN transport requires optional package 'python-can'") from exc
        self._can = can
        self.bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
        self.running = False
        self.gains: Optional[PIDGains] = None
        self.pid_sequence = 0
        self.command_sequence = 0
        self.dt_s = 0.02
        self.ack_timeout_s = ack_timeout_s

    def _send(self, frame: CanFrame) -> None:
        message = self._can.Message(arbitration_id=frame.arbitration_id,
                                    data=frame.data, is_extended_id=False)
        self.bus.send(message)

    def _send_stop_group(self) -> None:
        for frame in encode_velocity_group(self.command_sequence, 0, 0, mode_flags=0x00):
            self._send(frame)
        self.command_sequence = (self.command_sequence + 1) & 0xFFFF

    def _wait_for_pid_ack(self, sequence: int) -> None:
        deadline = time.monotonic() + self.ack_timeout_s
        while time.monotonic() < deadline:
            message = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if message is None:
                break
            if message.arbitration_id != FB_PID_GAINS or len(message.data) != 8:
                continue
            try:
                ack_sequence, status, axis = decode_pid_ack(bytes(message.data))
            except ValueError:
                continue
            if ack_sequence != sequence:
                continue
            if status != 1 or axis != 3:
                raise TransportSafetyError("MCU rejected PID transaction status=%d" % status)
            return
        raise TransportError("no accepted FB_PID_GAINS acknowledgement from MCU")

    def set_pid(self, gains: PIDGains) -> None:
        if self.running:
            raise TransportSafetyError("live PID gains may only change while stopped")
        self._send_stop_group()
        sequence = self.pid_sequence
        for frame in encode_pid_transaction(sequence, gains):
            self._send(frame)
        self._wait_for_pid_ack(sequence)
        self.gains = gains
        self.pid_sequence = (self.pid_sequence + 1) & 0xFFFF

    def start_trial(self) -> None:
        self.running = True

    def run_step(self, target: float, duration_s: float) -> List[Sample]:
        if not self.running:
            raise TransportError("live trial is not running")
        samples: List[Sample] = []
        fault: Optional[str] = None
        safety_state: Optional[str] = None
        drive_allowed = False
        health_seen = False
        diagnostics_seen = False
        for _ in range(int(math.ceil(duration_s / self.dt_s))):
            for frame in encode_velocity_group(self.command_sequence, int(target), int(target)):
                self._send(frame)
            self.command_sequence = (self.command_sequence + 1) & 0xFFFF
            deadline = time.monotonic() + self.dt_s
            velocity: Optional[float] = None
            control_output: Optional[float] = None
            output_received = False
            while time.monotonic() < deadline:
                message = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
                if message is None:
                    break
                if message.arbitration_id == FB_STATUS and len(message.data) >= 8:
                    raw_fault = int.from_bytes(bytes(message.data[6:8]), "little")
                    fault = ("0x%04x" % raw_fault) if raw_fault else fault
                elif message.arbitration_id == FB_HEALTH and len(message.data) >= 8:
                    health_seen = True
                    health_flags = message.data[3]
                    if health_flags & 0x20:
                        fault = fault or "estop_active"
                    elif health_flags & 0x40:
                        fault = fault or "safe_stop_active"
                    elif health_flags & 0x80:
                        fault = fault or "fault_latched"
                elif message.arbitration_id == FB_DIAGNOSTICS and len(message.data) >= 8:
                    diagnostics_seen = True
                    diagnostic_flags = message.data[3]
                    safety_state = str(message.data[4])
                    drive_allowed = bool((diagnostic_flags & 0x02) and message.data[4] == 4)
                    if diagnostic_flags & 0x01:
                        fault = fault or "calibration_required"
                    elif diagnostic_flags & 0x20:
                        fault = fault or "control_overrun"
                    elif diagnostic_flags & 0x40:
                        fault = fault or "motor_stall"
                    elif diagnostic_flags & 0x80:
                        fault = fault or "can_error"
                elif message.arbitration_id == FB_REAR_VELOCITY and len(message.data) >= 8:
                    flags = message.data[3]
                    if (flags & 0x03) == 0x03:
                        left = int.from_bytes(bytes(message.data[4:6]), "little", signed=True)
                        right = int.from_bytes(bytes(message.data[6:8]), "little", signed=True)
                        velocity = (left + right) / 2.0
                elif message.arbitration_id == FB_CONTROL_OUTPUT and len(message.data) >= 8:
                    left_output, right_output = decode_control_output(bytes(message.data))
                    control_output = (left_output + right_output) / 2.0
                    output_received = True
                if velocity is not None and output_received and health_seen and diagnostics_seen:
                    break
            if velocity is None or not output_received or not health_seen or not diagnostics_seen:
                raise TransportError("no complete safe feedback group received during live trial")
            if self.gains is None:
                raise TransportError("live PID gains are not set")
            samples.append(Sample((len(samples) + 1) * self.dt_s, target, velocity, control_output,
                                  self.gains.kp, self.gains.ki, self.gains.kd,
                                  safety_state=safety_state, safety_fault=fault or
                                  (None if drive_allowed else "drive_not_allowed")))
            if fault:
                break
            if not drive_allowed:
                break
        return samples

    def stop(self) -> None:
        if hasattr(self, "bus"):
            self._send_stop_group()
        self.running = False
