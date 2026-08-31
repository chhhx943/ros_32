import random
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
    rng = random.Random(seed)
    yield initial
    for _ in range(count - 1):
        yield PIDGains(
            rng.uniform(*bounds["kp"]),
            rng.uniform(*bounds["ki"]),
            rng.uniform(*bounds["kd"]),
        )
