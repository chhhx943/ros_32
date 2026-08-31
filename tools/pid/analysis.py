import math
from typing import Iterable, List, Optional

from .model import ResponseMetrics, Sample


def _trapz(values: List[float], times: List[float]) -> float:
    return sum((times[index] - times[index - 1]) * (values[index] + values[index - 1]) / 2.0
               for index in range(1, len(values)))


def _crossed_rise_threshold(actual: float, target: float) -> bool:
    if target >= 0.0:
        return actual >= target * 0.9
    return actual <= target * 0.9


def analyze_response(samples: Iterable[Sample], settling_band: float = 0.02) -> ResponseMetrics:
    rows = list(samples)
    if len(rows) < 2:
        raise ValueError("at least two samples are required")
    times = [row.time_s for row in rows]
    if any(times[index] <= times[index - 1] for index in range(1, len(times))):
        raise ValueError("sample times must be strictly increasing")

    target = rows[-1].target
    scale = abs(target)
    if scale <= 0.0:
        raise ValueError("step target must be non-zero")
    errors = [row.target - row.actual for row in rows]
    rise_time = next((row.time_s for row in rows if _crossed_rise_threshold(row.actual, target)), None)
    if target >= 0.0:
        peak = max(row.actual for row in rows)
        overshoot = max(0.0, (peak - target) / scale * 100.0)
    else:
        trough = min(row.actual for row in rows)
        overshoot = max(0.0, (target - trough) / scale * 100.0)

    band = scale * settling_band
    settling = None
    for index, row in enumerate(rows):
        if all(abs(future.target - future.actual) <= band for future in rows[index:]):
            settling = row.time_s
            break

    tail_count = max(1, len(rows) // 10)
    steady_state_error = abs(sum(errors[-tail_count:]) / tail_count)
    signs = [1 if error > band else -1 if error < -band else 0 for error in errors]
    nonzero_signs = [sign for sign in signs if sign]
    oscillation = sum(nonzero_signs[index] != nonzero_signs[index - 1]
                      for index in range(1, len(nonzero_signs)))
    iae = _trapz([abs(error) for error in errors], times)
    ise = _trapz([error * error for error in errors], times)
    return ResponseMetrics(rise_time, overshoot, settling, steady_state_error, iae, ise, oscillation)


def score_result(metrics: ResponseMetrics) -> float:
    """Lower is better; missing settling/rise times are deliberately penalized."""
    rise = metrics.rise_time_s if metrics.rise_time_s is not None else 10.0
    settling = metrics.settling_time_s if metrics.settling_time_s is not None else 20.0
    return (rise + settling + 0.05 * metrics.overshoot_percent +
            2.0 * metrics.steady_state_error + metrics.iae +
            0.1 * metrics.ise + 0.5 * metrics.oscillation)


def safety_violation(sample: Sample, max_actual: float, max_output: float) -> Optional[str]:
    if sample.safety_fault:
        return "mcu_fault:" + sample.safety_fault
    if abs(sample.actual) > max_actual:
        return "actual_limit"
    if sample.control_output is not None and abs(sample.control_output) > max_output:
        return "output_limit"
    return None
