from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

from .analysis import analyze_response, safety_violation, score_result
from .model import PIDGains, ResponseMetrics, Sample
from .transport import TransportError


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

    def as_dict(self) -> Dict[str, Any]:
        return {
            "candidate_pid": self.candidate_pid.as_dict(),
            "metrics": self.metrics.as_dict() if self.metrics else None,
            "score": self.score,
            "aborted": self.aborted,
            "abort_reason": self.abort_reason,
            "scenario_scores": self.scenario_scores,
            "sample_count": len(self.samples),
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
        self.transport.stop()
        try:
            self.transport.set_pid(candidate)
            self.transport.start_trial()
            samples = self.transport.run_step(scenario.target, scenario.duration_s)
            for sample in samples:
                abort_reason = safety_violation(sample, self.max_actual, self.max_output)
                if abort_reason:
                    break
        except TransportError as exc:
            abort_reason = "transport:" + str(exc)
        finally:
            self.transport.stop()
            self.transport.set_pid(self.current_pid)

        if abort_reason:
            return TrialResult(candidate, samples, None, None, True, abort_reason, {})
        metrics = analyze_response(samples)
        return TrialResult(candidate, samples, metrics, score_result(metrics), False, None,
                           {scenario.name: score_result(metrics)})

    def run_candidate(self, candidate: PIDGains, scenarios: Iterable[Scenario]) -> TrialResult:
        self.candidate_pid = candidate
        all_samples: List[Sample] = []
        all_metrics: List[ResponseMetrics] = []
        scenario_scores: Dict[str, float] = {}
        aborted = False
        abort_reason: Optional[str] = None
        for scenario in scenarios:
            result = self.run_trial(candidate, scenario)
            all_samples.extend(result.samples)
            if result.aborted:
                aborted = True
                abort_reason = result.abort_reason
                break
            assert result.metrics is not None and result.score is not None
            all_metrics.append(result.metrics)
            scenario_scores.update(result.scenario_scores)

        if aborted:
            final = TrialResult(candidate, all_samples, None, None, True, abort_reason, scenario_scores)
        else:
            metrics = self._aggregate_metrics(all_metrics)
            total_score = sum(scenario_scores.values())
            final = TrialResult(candidate, all_samples, metrics, total_score, False, None, scenario_scores)
            if total_score < self.best_score:
                self.best_score = total_score
                self.best_pid = candidate
                self.current_pid = candidate
                self.transport.set_pid(self.current_pid)

        self.sample_history.extend(all_samples)

        history = final.as_dict()
        history["status"] = "aborted" if final.aborted else "completed"
        self.experiment_history.append(history)
        return final
