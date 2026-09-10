from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

from .analysis import analyze_response, safety_violation, score_result
from .model import PIDGains, ResponseMetrics, Sample
from .transport import TransportError


class HostSafetyPolicy:
    """Second-layer checks; MCU limits remain authoritative on real hardware."""

    TARGET_LIMIT_MMPS = 100.0
    PWM_LIMIT = 150.0
    RELATIVE_STEP = 0.25
    BOOTSTRAP_STEP = PIDGains(0.10, 0.15, 0.05)
    ABSOLUTE_MIN = PIDGains(0.0, 0.0, 0.0)
    ABSOLUTE_MAX = PIDGains(4.0, 4.0, 1.0)

    def validate_target(self, target: float) -> Optional[str]:
        if target <= 0.0:
            return "target_direction_not_allowed"
        if target > self.TARGET_LIMIT_MMPS:
            return "target_limit"
        return None

    def validate_candidate(self, candidate: PIDGains, baseline: PIDGains,
                           bootstrap: bool) -> Optional[str]:
        for value, lower, upper in zip(
                (candidate.kp, candidate.ki, candidate.kd),
                (self.ABSOLUTE_MIN.kp, self.ABSOLUTE_MIN.ki, self.ABSOLUTE_MIN.kd),
                (self.ABSOLUTE_MAX.kp, self.ABSOLUTE_MAX.ki, self.ABSOLUTE_MAX.kd)):
            if value < lower or value > upper:
                return "pid_absolute_limit"
        steps = self.BOOTSTRAP_STEP if bootstrap else PIDGains(
            baseline.kp * self.RELATIVE_STEP,
            baseline.ki * self.RELATIVE_STEP,
            baseline.kd * self.RELATIVE_STEP if baseline.kd > 0.0 else self.BOOTSTRAP_STEP.kd,
        )
        for value, reference, limit in (
            (candidate.kp, baseline.kp, steps.kp),
            (candidate.ki, baseline.ki, steps.ki),
            (candidate.kd, baseline.kd, steps.kd),
        ):
            if abs(value - reference) > limit + 1e-9:
                return "pid_step_limit"
        return None


@dataclass(frozen=True)
class Scenario:
    name: str
    target: float
    duration_s: float


@dataclass
class TrialResult:
    candidate_pid: PIDGains
    samples: List[Sample]
    metrics: Optional[ResponseMetrics]
    score: Optional[float]
    aborted: bool
    abort_reason: Optional[str]
    scenario_scores: Dict[str, float]
    level: int = 1
    requested_target: Optional[float] = None
    effective_target: Optional[float] = None
    pwm_limit: float = 150.0
    session_id: int = 0
    experiment_id: int = 0
    max_pwm: float = 0.0
    max_speed: float = 0.0
    saturation_time: float = 0.0
    stall_time: float = 0.0
    oscillation_count: int = 0
    speed_limit_hit: bool = False
    safety_state: str = "READY"

    def as_dict(self) -> Dict[str, Any]:
        return {
            "candidate_pid": self.candidate_pid.as_dict(),
            "metrics": self.metrics.as_dict() if self.metrics else None,
            "score": self.score,
            "aborted": self.aborted,
            "abort_reason": self.abort_reason,
            "scenario_scores": self.scenario_scores,
            "sample_count": len(self.samples),
            "level": self.level,
            "requested_target": self.requested_target,
            "effective_target": self.effective_target,
            "pwm_limit": self.pwm_limit,
            "session_id": self.session_id,
            "experiment_id": self.experiment_id,
            "max_pwm": self.max_pwm,
            "max_speed": self.max_speed,
            "saturation_time": self.saturation_time,
            "stall_time": self.stall_time,
            "oscillation_count": self.oscillation_count,
            "speed_limit_hit": self.speed_limit_hit,
            "safety_state": self.safety_state,
        }


class ExperimentRunner:
    def __init__(self, transport: Any, initial_pid: PIDGains,
                 max_actual: float = 3500.0, max_output: float = 1000.0):
        self.transport = transport
        self.current_pid = initial_pid
        self.candidate_pid: Optional[PIDGains] = None
        self.best_pid = initial_pid
        self.best_score = float("inf")
        self.experiment_history: List[Dict[str, Any]] = []
        self.sample_history: List[Sample] = []
        self.max_actual = max_actual
        self.max_output = max_output
        self.safety_policy = HostSafetyPolicy()
        self.bootstrap_active = True
        self.level = 1
        self.session_id = 1
        self.experiment_id = 0
        self.transport.stop()
        self.transport.set_pid(initial_pid)

    def _aggregate_metrics(self, metrics: Iterable[ResponseMetrics]) -> ResponseMetrics:
        rows = list(metrics)
        if not rows:
            raise ValueError("at least one scenario metric is required")
        rise = [row.rise_time_s for row in rows]
        settling = [row.settling_time_s for row in rows]
        return ResponseMetrics(
            sum(value for value in rise if value is not None) / len(rise) if all(value is not None for value in rise) else None,
            sum(row.overshoot_percent for row in rows) / len(rows),
            sum(value for value in settling if value is not None) / len(settling) if all(value is not None for value in settling) else None,
            sum(row.steady_state_error for row in rows) / len(rows),
            sum(row.iae for row in rows),
            sum(row.ise for row in rows),
            sum(row.oscillation for row in rows),
        )

    def run_trial(self, candidate: PIDGains, scenario: Scenario) -> TrialResult:
        samples: List[Sample] = []
        abort_reason: Optional[str] = None
        target_reason = self.safety_policy.validate_target(scenario.target)
        pid_reason = self.safety_policy.validate_candidate(
            candidate, self.best_pid, self.bootstrap_active)
        self.experiment_id += 1
        experiment_id = self.experiment_id
        if target_reason:
            return TrialResult(candidate, samples, None, None, True, target_reason, {},
                               self.level, scenario.target, 0.0,
                               self.safety_policy.PWM_LIMIT, self.session_id, experiment_id,
                               safety_state="ABORT")
        if pid_reason:
            return TrialResult(candidate, samples, None, None, True, pid_reason, {},
                               self.level, scenario.target, 0.0,
                               self.safety_policy.PWM_LIMIT, self.session_id, experiment_id,
                               safety_state="ABORT")
        self.transport.stop()
        try:
            self.transport.set_pid(candidate)
            self.transport.start_trial(self.session_id, experiment_id)
            samples = self.transport.run_step(scenario.target, scenario.duration_s)
            for sample in samples:
                abort_reason = safety_violation(sample, self.max_actual, self.max_output)
                if abort_reason:
                    break
        except TransportError as exc:
            abort_reason = "transport:" + str(exc)
        finally:
            self.transport.stop()
            try:
                if not self.transport.wait_until_still():
                    abort_reason = abort_reason or "stillness_timeout"
            except TransportError as exc:
                abort_reason = abort_reason or "transport:" + str(exc)
            self.transport.set_pid(self.current_pid)

        if abort_reason:
            return self._trial_result(candidate, scenario, experiment_id, samples,
                                      None, None, True, abort_reason, {})
        metrics = analyze_response(samples)
        return self._trial_result(candidate, scenario, experiment_id, samples,
                                  metrics, score_result(metrics), False, None,
                                  {scenario.name: score_result(metrics)})

    def _trial_result(self, candidate: PIDGains, scenario: Scenario, experiment_id: int,
                      samples: List[Sample], metrics: Optional[ResponseMetrics],
                      score: Optional[float], aborted: bool, abort_reason: Optional[str],
                      scenario_scores: Dict[str, float]) -> TrialResult:
        effective = max((sample.effective_target or sample.target for sample in samples), default=0.0)
        return TrialResult(
            candidate, samples, metrics, score, aborted, abort_reason, scenario_scores,
            self.level, scenario.target, effective, self.safety_policy.PWM_LIMIT,
            self.session_id, experiment_id,
            max((abs(sample.control_output or 0.0) for sample in samples), default=0.0),
            max((abs(sample.actual) for sample in samples), default=0.0),
            safety_state="ABORT" if aborted else "COMPLETE",
        )

    def run_candidate(self, candidate: PIDGains, scenarios: Iterable[Scenario]) -> TrialResult:
        self.candidate_pid = candidate
        all_samples: List[Sample] = []
        all_metrics: List[ResponseMetrics] = []
        scenario_scores: Dict[str, float] = {}
        trial_results: List[TrialResult] = []
        aborted = False
        abort_reason: Optional[str] = None
        for scenario in scenarios:
            result = self.run_trial(candidate, scenario)
            trial_results.append(result)
            all_samples.extend(result.samples)
            if result.aborted:
                aborted = True
                abort_reason = result.abort_reason
                break
            assert result.metrics is not None and result.score is not None
            all_metrics.append(result.metrics)
            scenario_scores.update(result.scenario_scores)

        if aborted:
            failed = trial_results[-1]
            final = TrialResult(
                candidate, all_samples, None, None, True, abort_reason, scenario_scores,
                failed.level, failed.requested_target, failed.effective_target,
                failed.pwm_limit, failed.session_id, failed.experiment_id,
                failed.max_pwm, failed.max_speed, failed.saturation_time,
                failed.stall_time, failed.oscillation_count, failed.speed_limit_hit,
                failed.safety_state,
            )
        else:
            metrics = self._aggregate_metrics(all_metrics)
            total_score = sum(scenario_scores.values())
            final = TrialResult(
                candidate, all_samples, metrics, total_score, False, None, scenario_scores,
                self.level,
                trial_results[-1].requested_target if trial_results else None,
                max((trial.effective_target or 0.0 for trial in trial_results), default=0.0),
                self.safety_policy.PWM_LIMIT,
                self.session_id,
                trial_results[-1].experiment_id if trial_results else 0,
                max((trial.max_pwm for trial in trial_results), default=0.0),
                max((trial.max_speed for trial in trial_results), default=0.0),
                max((trial.saturation_time for trial in trial_results), default=0.0),
                max((trial.stall_time for trial in trial_results), default=0.0),
                max((trial.oscillation_count for trial in trial_results), default=0),
                any(trial.speed_limit_hit for trial in trial_results),
                "COMPLETE",
            )
            if total_score < self.best_score:
                self.best_score = total_score
                self.best_pid = candidate
                self.current_pid = candidate
                self.transport.set_pid(self.current_pid)
            self.bootstrap_active = False

        self.sample_history.extend(all_samples)

        history = final.as_dict()
        history["status"] = "aborted" if final.aborted else "completed"
        self.experiment_history.append(history)
        return final
