import json
import math
from dataclasses import dataclass
from numbers import Real
from pathlib import Path
from typing import Optional, Union


from .geometry import DEFAULT_CONFIG_PATH, VehicleGeometry, geometry_from_mapping


PROTOCOL_MAX_WHEEL_SPEED_MM_S = 3000.0


@dataclass(frozen=True)
class VehicleConfig:
    geometry: VehicleGeometry
    max_wheel_speed_mm_s: float


def load_vehicle_config(
    path: Optional[Union[str, Path]] = None,
) -> VehicleConfig:
    source = DEFAULT_CONFIG_PATH if path is None else Path(path)
    data = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("vehicle config must contain a JSON object")

    required = (
        "wheelbase_mm",
        "track_mm",
        "tire_diameter_mm",
        "max_steering_deg",
        "max_wheel_speed_mm_s",
    )
    if any(name not in data for name in required):
        raise ValueError("vehicle config is missing required fields")

    geometry = geometry_from_mapping(data)
    limit = data["max_wheel_speed_mm_s"]
    if isinstance(limit, bool) or not isinstance(limit, Real):
        raise ValueError("max_wheel_speed_mm_s must be numeric")
    limit = float(limit)
    if (
        not math.isfinite(limit)
        or limit <= 0.0
        or limit > PROTOCOL_MAX_WHEEL_SPEED_MM_S
    ):
        raise ValueError(
            "max_wheel_speed_mm_s is outside the frozen protocol envelope"
        )
    return VehicleConfig(geometry=geometry, max_wheel_speed_mm_s=limit)
