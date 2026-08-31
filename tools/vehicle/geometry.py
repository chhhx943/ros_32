import json
import math
from dataclasses import dataclass
from numbers import Real
from pathlib import Path
from typing import Mapping


DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.json")


def _require_positive_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("%s must be a finite positive number" % name)
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError("%s must be a finite positive number" % name)
    return number


@dataclass(frozen=True)
class VehicleGeometry:
    wheelbase_mm: float
    track_mm: float
    tire_diameter_mm: float
    max_steering_rad: float

    def __post_init__(self) -> None:
        for name in (
            "wheelbase_mm",
            "track_mm",
            "tire_diameter_mm",
            "max_steering_rad",
        ):
            object.__setattr__(
                self,
                name,
                _require_positive_finite(name, getattr(self, name)),
            )
        if self.max_steering_rad >= math.pi / 2.0:
            raise ValueError("max_steering_rad must be less than pi/2")

    @property
    def tire_radius_mm(self) -> float:
        return self.tire_diameter_mm / 2.0


def geometry_from_mapping(data: Mapping[str, object]) -> VehicleGeometry:
    try:
        max_steering_deg = _require_positive_finite(
            "max_steering_deg",
            data["max_steering_deg"],
        )
        return VehicleGeometry(
            wheelbase_mm=data["wheelbase_mm"],
            track_mm=data["track_mm"],
            tire_diameter_mm=data["tire_diameter_mm"],
            max_steering_rad=math.radians(max_steering_deg),
        )
    except (KeyError, TypeError) as exc:
        raise ValueError("vehicle geometry fields are invalid") from exc


def _load_default_geometry() -> VehicleGeometry:
    data = json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("default vehicle config must contain a JSON object")
    return geometry_from_mapping(data)


R3X_GEOMETRY = _load_default_geometry()
