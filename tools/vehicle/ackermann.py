import math
from dataclasses import dataclass
from enum import Enum
from numbers import Real
from typing import Optional


from .geometry import VehicleGeometry


class MotionSafetyStatus(str, Enum):
    OK = "ok"
    STEERING_LIMITED = "steering_limited"
    SPEED_LIMITED = "speed_limited"
    LIMITED = "limited"
    INVALID_INPUT = "invalid_input"
    INVALID_GEOMETRY = "invalid_geometry"
    INVALID_SPEED_LIMIT = "invalid_speed_limit"


@dataclass(frozen=True)
class AckermannResult:
    valid: bool
    requested_speed_mm_s: float
    requested_steering_rad: float
    applied_speed_mm_s: float
    applied_steering_rad: float
    curvature_per_mm: float
    rear_axle_radius_signed_mm: float
    rear_axle_radius_abs_mm: float
    left_wheel_speed_mm_s: float
    right_wheel_speed_mm_s: float
    inner_side: Optional[str]
    outer_side: Optional[str]
    wheel_speed_scale: float
    steering_limited: bool
    wheel_speed_limited: bool
    safety_status: MotionSafetyStatus
    reason: Optional[str]


def _finite_real(value: object) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _requested_value(value: object, scale: float = 1.0) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        return 0.0
    return float(value) * scale


def _invalid_result(
    speed_mm_s: object,
    steering_mrad: object,
    status: MotionSafetyStatus,
    reason: str,
) -> AckermannResult:
    return AckermannResult(
        valid=False,
        requested_speed_mm_s=_requested_value(speed_mm_s),
        requested_steering_rad=_requested_value(steering_mrad, 0.001),
        applied_speed_mm_s=0.0,
        applied_steering_rad=0.0,
        curvature_per_mm=0.0,
        rear_axle_radius_signed_mm=math.inf,
        rear_axle_radius_abs_mm=math.inf,
        left_wheel_speed_mm_s=0.0,
        right_wheel_speed_mm_s=0.0,
        inner_side=None,
        outer_side=None,
        wheel_speed_scale=0.0,
        steering_limited=False,
        wheel_speed_limited=False,
        safety_status=status,
        reason=reason,
    )


def _geometry_is_valid(geometry: object) -> bool:
    if not isinstance(geometry, VehicleGeometry):
        return False
    values = (
        geometry.wheelbase_mm,
        geometry.track_mm,
        geometry.tire_diameter_mm,
        geometry.max_steering_rad,
    )
    return (
        all(math.isfinite(value) and value > 0.0 for value in values)
        and geometry.max_steering_rad < math.pi / 2.0
    )


def solve_ackermann(
    speed_mm_s: object,
    steering_mrad: object,
    geometry: object,
    max_wheel_speed_mm_s: object,
) -> AckermannResult:
    speed = _finite_real(speed_mm_s)
    if speed is None:
        return _invalid_result(
            speed_mm_s,
            steering_mrad,
            MotionSafetyStatus.INVALID_INPUT,
            "speed_not_finite",
        )

    steering = _finite_real(steering_mrad)
    if steering is None:
        return _invalid_result(
            speed_mm_s,
            steering_mrad,
            MotionSafetyStatus.INVALID_INPUT,
            "steering_not_finite",
        )

    if not _geometry_is_valid(geometry):
        return _invalid_result(
            speed_mm_s,
            steering_mrad,
            MotionSafetyStatus.INVALID_GEOMETRY,
            "invalid_geometry",
        )

    speed_limit = _finite_real(max_wheel_speed_mm_s)
    if speed_limit is None or speed_limit <= 0.0:
        return _invalid_result(
            speed_mm_s,
            steering_mrad,
            MotionSafetyStatus.INVALID_SPEED_LIMIT,
            "invalid_max_wheel_speed",
        )

    requested_steering_rad = steering * 0.001
    applied_steering_rad = max(
        -geometry.max_steering_rad,
        min(geometry.max_steering_rad, requested_steering_rad),
    )
    steering_limited = applied_steering_rad != requested_steering_rad
    curvature = math.tan(applied_steering_rad) / geometry.wheelbase_mm

    if curvature == 0.0:
        radius_signed = math.inf
        radius_abs = math.inf
        inner_side = None
        outer_side = None
    else:
        radius_signed = 1.0 / curvature
        radius_abs = abs(radius_signed)
        if curvature > 0.0:
            inner_side = "left"
            outer_side = "right"
        else:
            inner_side = "right"
            outer_side = "left"

    half_track = geometry.track_mm / 2.0
    left_raw = speed * (1.0 - curvature * half_track)
    right_raw = speed * (1.0 + curvature * half_track)
    peak = max(abs(left_raw), abs(right_raw))
    wheel_speed_scale = 1.0 if peak <= speed_limit else speed_limit / peak
    wheel_speed_limited = wheel_speed_scale < 1.0

    if steering_limited and wheel_speed_limited:
        safety_status = MotionSafetyStatus.LIMITED
    elif steering_limited:
        safety_status = MotionSafetyStatus.STEERING_LIMITED
    elif wheel_speed_limited:
        safety_status = MotionSafetyStatus.SPEED_LIMITED
    else:
        safety_status = MotionSafetyStatus.OK

    return AckermannResult(
        valid=True,
        requested_speed_mm_s=speed,
        requested_steering_rad=requested_steering_rad,
        applied_speed_mm_s=speed * wheel_speed_scale,
        applied_steering_rad=applied_steering_rad,
        curvature_per_mm=curvature,
        rear_axle_radius_signed_mm=radius_signed,
        rear_axle_radius_abs_mm=radius_abs,
        left_wheel_speed_mm_s=left_raw * wheel_speed_scale,
        right_wheel_speed_mm_s=right_raw * wheel_speed_scale,
        inner_side=inner_side,
        outer_side=outer_side,
        wheel_speed_scale=wheel_speed_scale,
        steering_limited=steering_limited,
        wheel_speed_limited=wheel_speed_limited,
        safety_status=safety_status,
        reason=None,
    )
