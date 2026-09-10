from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class PIDGains:
    kp: float
    ki: float
    kd: float

    def __post_init__(self) -> None:
        if self.kp < 0.0 or self.ki < 0.0 or self.kd < 0.0:
            raise ValueError("PID gains must be non-negative")
        if not all(map(lambda value: value == value and abs(value) != float("inf"), (self.kp, self.ki, self.kd))):
            raise ValueError("PID gains must be finite")

    def as_dict(self) -> Dict[str, float]:
        return {"kp": self.kp, "ki": self.ki, "kd": self.kd}


@dataclass(frozen=True)
class Sample:
    time_s: float
    target: float
    actual: float
    control_output: Optional[float]
    kp: float
    ki: float
    kd: float
    current_a: Optional[float] = None
    voltage_v: Optional[float] = None
    safety_state: Optional[str] = None
    safety_fault: Optional[str] = None
    requested_target: Optional[float] = None
    effective_target: Optional[float] = None
    pwm_limit: Optional[float] = None
    session_id: Optional[int] = None
    experiment_id: Optional[int] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            "time_s": self.time_s,
            "target": self.target,
            "actual": self.actual,
            "error": self.target - self.actual,
            "control_output": self.control_output,
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "current_a": self.current_a,
            "voltage_v": self.voltage_v,
            "safety_state": self.safety_state,
            "safety_fault": self.safety_fault,
            "requested_target": self.requested_target,
            "effective_target": self.effective_target,
            "pwm_limit": self.pwm_limit,
            "session_id": self.session_id,
            "experiment_id": self.experiment_id,
        }


@dataclass(frozen=True)
class ResponseMetrics:
    rise_time_s: Optional[float]
    overshoot_percent: float
    settling_time_s: Optional[float]
    steady_state_error: float
    iae: float
    ise: float
    oscillation: int

    def as_dict(self) -> Dict[str, Any]:
        return {
            "rise_time_s": self.rise_time_s,
            "overshoot_percent": self.overshoot_percent,
            "settling_time_s": self.settling_time_s,
            "steady_state_error": self.steady_state_error,
            "iae": self.iae,
            "ise": self.ise,
            "oscillation": self.oscillation,
        }
