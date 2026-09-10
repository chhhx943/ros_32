from typing import Dict, Iterator, Tuple

from .model import PIDGains


def generate_candidates(initial: PIDGains, bounds: Dict[str, Tuple[float, float]],
                        count: int, seed: int = 0) -> Iterator[PIDGains]:
    if count <= 0:
        return
    for name in ("kp", "ki", "kd"):
        lower, upper = bounds[name]
        if lower < 0.0 or upper < lower:
            raise ValueError("invalid PID search bounds")
    # Keep the seed argument for CLI compatibility, but deliberately do not
    # use random exploration on a real actuator.
    del seed

    def clipped(value: float, name: str) -> float:
        lower, upper = bounds[name]
        return max(lower, min(upper, value))

    yield initial
    # Stage 1: tune P around the safe baseline while retaining its I term and
    # keeping D disabled. Stage 2: tune I. Stage 3: introduce D only if the
    # caller asks for enough iterations.
    staged = [
        PIDGains(clipped(initial.kp * 0.75, "kp"), initial.ki, 0.0),
        PIDGains(clipped(initial.kp * 1.25, "kp"), initial.ki, 0.0),
        PIDGains(initial.kp, clipped(initial.ki * 0.75, "ki"), 0.0),
        PIDGains(initial.kp, clipped(initial.ki * 1.25, "ki"), 0.0),
        PIDGains(initial.kp, initial.ki, clipped(0.05, "kd")),
    ]
    for candidate in staged[:count - 1]:
        yield candidate
