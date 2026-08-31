# R3X Ackermann Chassis Motion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python 3.9-compatible, host-side R3X Ackermann motion layer that validates and limits commands, computes rear-wheel targets, reuses the frozen CAN V1 encoder, records auditable outcomes, and requests deterministic safe stopping on failures.

**Architecture:** `geometry.py` owns immutable dimensions, `config.py` loads one auditable vehicle profile, and `ackermann.py` remains a pure hardware-independent solver. `command.py` is the only new CAN-facing module: it converts one immutable solver result into the existing two-frame protocol group, reports per-frame outcomes, and exposes a separate fresh-sequence safe-stop operation. No STM32 source or protocol layout changes are required.

**Tech Stack:** Python 3.9 standard library (`dataclasses`, `enum`, `json`, `logging`, `math`, `time`, `typing`), existing `tools.pid.protocol`, and `unittest` discovery.

---

## File map

| Path | Responsibility |
| --- | --- |
| `tools/vehicle/__init__.py` | Stable public imports only. |
| `tools/vehicle/geometry.py` | Immutable vehicle dimensions, validation, derived tire radius, and `R3X_GEOMETRY`. |
| `tools/vehicle/config.py` | Strict JSON-to-domain conversion and `VehicleConfig`. |
| `tools/vehicle/config.json` | Auditable R3X geometry plus the conservative 600 mm/s current software ceiling. |
| `tools/vehicle/ackermann.py` | Pure validation, steering clamp, curvature/radius, wheel split, uniform scaling, result/status types. |
| `tools/vehicle/command.py` | Existing-protocol frame construction, ordered sending, safe-stop/fallback status, and audit-record emission. |
| `tests/test_vehicle_geometry.py` | Geometry/config validation and single-source-of-truth tests. |
| `tests/test_vehicle_ackermann.py` | Solver behavior, limits, invalid inputs, reverse semantics, and numerical properties. |
| `tests/test_vehicle_command.py` | Exact CAN payload, same-result pairing, partial failure, safe-stop, watchdog fallback, and log tests. |

### Task 1: Immutable geometry and auditable configuration

**Files:**
- Create: `tests/test_vehicle_geometry.py`
- Create: `tools/vehicle/geometry.py`
- Create: `tools/vehicle/config.py`
- Create: `tools/vehicle/config.json`

- [ ] **Step 1: Write the failing geometry/config tests**

```python
import json
import math
import tempfile
import unittest
from pathlib import Path

from tools.vehicle.config import load_vehicle_config
from tools.vehicle.geometry import R3X_GEOMETRY, VehicleGeometry


class VehicleGeometryTest(unittest.TestCase):
    def test_r3x_geometry_has_one_diameter_source_and_derived_radius(self):
        self.assertEqual(R3X_GEOMETRY.wheelbase_mm, 141.7)
        self.assertEqual(R3X_GEOMETRY.track_mm, 120.0)
        self.assertEqual(R3X_GEOMETRY.tire_diameter_mm, 65.0)
        self.assertEqual(R3X_GEOMETRY.tire_radius_mm, 32.5)
        self.assertAlmostEqual(R3X_GEOMETRY.max_steering_rad, math.radians(20.0))

    def test_geometry_rejects_nonfinite_nonpositive_and_singular_values(self):
        valid = dict(wheelbase_mm=141.7, track_mm=120.0,
                     tire_diameter_mm=65.0, max_steering_rad=math.radians(20.0))
        for field, value in (("wheelbase_mm", 0.0), ("track_mm", -1.0),
                             ("tire_diameter_mm", math.inf),
                             ("max_steering_rad", math.pi / 2)):
            values = dict(valid)
            values[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                VehicleGeometry(**values)

    def test_default_config_is_explicit_and_protocol_bounded(self):
        config = load_vehicle_config()
        self.assertEqual(config.geometry, R3X_GEOMETRY)
        self.assertEqual(config.max_wheel_speed_mm_s, 600.0)

    def test_config_rejects_missing_or_out_of_protocol_wheel_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            for value in (None, 0.0, math.inf, 3000.1):
                body = {"wheelbase_mm": 141.7, "track_mm": 120.0,
                        "tire_diameter_mm": 65.0, "max_steering_deg": 20.0}
                if value is not None:
                    body["max_wheel_speed_mm_s"] = value
                path.write_text(json.dumps(body), encoding="utf-8")
                with self.subTest(value=value), self.assertRaises(ValueError):
                    load_vehicle_config(path)
```

- [ ] **Step 2: Run the tests and verify RED**

Run: `python -m unittest discover -s tests -p test_vehicle_geometry.py -v`

Expected: import failure for missing `tools.vehicle`.

- [ ] **Step 3: Implement the geometry domain object**

```python
# tools/vehicle/geometry.py
import json
import math
from dataclasses import dataclass
from numbers import Real
from pathlib import Path


def _require_positive_finite(name, value):
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

    def __post_init__(self):
        for name in ("wheelbase_mm", "track_mm", "tire_diameter_mm", "max_steering_rad"):
            object.__setattr__(self, name, _require_positive_finite(name, getattr(self, name)))
        if self.max_steering_rad >= math.pi / 2.0:
            raise ValueError("max_steering_rad must be less than pi/2")

    @property
    def tire_radius_mm(self):
        return self.tire_diameter_mm / 2.0


def geometry_from_mapping(data):
    return VehicleGeometry(data["wheelbase_mm"], data["track_mm"],
                           data["tire_diameter_mm"],
                           math.radians(data["max_steering_deg"]))


DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.json")
R3X_GEOMETRY = geometry_from_mapping(
    json.loads(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")))
```

- [ ] **Step 4: Implement strict configuration loading**

```python
# tools/vehicle/config.py
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Union

from .geometry import DEFAULT_CONFIG_PATH, VehicleGeometry, geometry_from_mapping

PROTOCOL_MAX_WHEEL_SPEED_MM_S = 3000.0
@dataclass(frozen=True)
class VehicleConfig:
    geometry: VehicleGeometry
    max_wheel_speed_mm_s: float


def load_vehicle_config(path=None):
    source = DEFAULT_CONFIG_PATH if path is None else Path(path)
    data = json.loads(source.read_text(encoding="utf-8"))
    required = ("wheelbase_mm", "track_mm", "tire_diameter_mm",
                "max_steering_deg", "max_wheel_speed_mm_s")
    if any(name not in data for name in required):
        raise ValueError("vehicle config is missing required fields")
    geometry = geometry_from_mapping(data)
    limit = data["max_wheel_speed_mm_s"]
    if isinstance(limit, bool) or not isinstance(limit, (int, float)):
        raise ValueError("max_wheel_speed_mm_s must be numeric")
    limit = float(limit)
    if not math.isfinite(limit) or not 0.0 < limit <= PROTOCOL_MAX_WHEEL_SPEED_MM_S:
        raise ValueError("max_wheel_speed_mm_s is outside the protocol envelope")
    return VehicleConfig(geometry, limit)
```

```json
{
  "wheelbase_mm": 141.7,
  "track_mm": 120.0,
  "tire_diameter_mm": 65.0,
  "max_steering_deg": 20.0,
  "max_wheel_speed_mm_s": 600.0
}
```

- [ ] **Step 5: Run the focused tests and verify GREEN**

Run: `python -m unittest discover -s tests -p test_vehicle_geometry.py -v`

Expected: 4 tests pass with no warnings.

- [ ] **Step 6: Commit the geometry/config slice**

```powershell
git add -- tools/vehicle/geometry.py tools/vehicle/config.py tools/vehicle/config.json tests/test_vehicle_geometry.py
git commit -m "feat: add R3X vehicle geometry configuration"
```

### Task 2: Pure solver validation and straight-line behavior

**Files:**
- Create: `tests/test_vehicle_ackermann.py`
- Create: `tools/vehicle/ackermann.py`

- [ ] **Step 1: Write failing tests for safe invalid results and straight motion**

```python
import math
import unittest

from tools.vehicle.ackermann import MotionSafetyStatus, solve_ackermann
from tools.vehicle.geometry import R3X_GEOMETRY


class AckermannSolverTest(unittest.TestCase):
    def test_straight_motion_preserves_speed_and_has_infinite_radius(self):
        result = solve_ackermann(500.0, 0.0, R3X_GEOMETRY, 600.0)
        self.assertTrue(result.valid)
        self.assertEqual(result.safety_status, MotionSafetyStatus.OK)
        self.assertEqual(result.left_wheel_speed_mm_s, 500.0)
        self.assertEqual(result.right_wheel_speed_mm_s, 500.0)
        self.assertEqual(result.curvature_per_mm, 0.0)
        self.assertTrue(math.isinf(result.rear_axle_radius_signed_mm))
        self.assertIsNone(result.inner_side)
        self.assertIsNone(result.outer_side)

    def test_zero_speed_preserves_nonzero_steering(self):
        result = solve_ackermann(0.0, 200.0, R3X_GEOMETRY, 600.0)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.applied_steering_rad, 0.2)
        self.assertEqual((result.left_wheel_speed_mm_s, result.right_wheel_speed_mm_s),
                         (0.0, 0.0))

    def test_nonfinite_runtime_inputs_return_non_sendable_safe_result(self):
        cases = ((math.nan, 0.0, R3X_GEOMETRY, 600.0, "speed_not_finite"),
                 (0.0, math.inf, R3X_GEOMETRY, 600.0, "steering_not_finite"),
                 (0.0, 0.0, None, 600.0, "invalid_geometry"),
                 (0.0, 0.0, R3X_GEOMETRY, 0.0, "invalid_max_wheel_speed"))
        for speed, steering, geometry, limit, reason in cases:
            with self.subTest(reason=reason):
                result = solve_ackermann(speed, steering, geometry, limit)
                self.assertFalse(result.valid)
                self.assertEqual(result.reason, reason)
                self.assertEqual(result.left_wheel_speed_mm_s, 0.0)
                self.assertEqual(result.right_wheel_speed_mm_s, 0.0)
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run: `python -m unittest discover -s tests -p test_vehicle_ackermann.py -v`

Expected: import failure for missing `tools.vehicle.ackermann`.

- [ ] **Step 3: Implement result/status types, ordered validation, and the straight path**

Create `MotionSafetyStatus` with `OK`, `STEERING_LIMITED`, `SPEED_LIMITED`, `LIMITED`, `INVALID_INPUT`, `INVALID_GEOMETRY`, and `INVALID_SPEED_LIMIT`. Create the frozen `AckermannResult` with every field in specification section 17. Implement `_invalid_result(reason, status, requested values)` so applied speed/steering, curvature, and wheel speeds are zero; radii are `math.inf`; sides are `None`; scale is zero; and no CAN-facing module can mistake it for a valid command.

The public function must retain this exact signature and validation order:

```python
def solve_ackermann(speed_mm_s, steering_mrad, geometry, max_wheel_speed_mm_s):
    # 1 speed, 2 steering, 3 geometry, 4 speed limit
    # convert mrad once, clamp, calculate, scale, build immutable result
```

Use a real-number validator that rejects booleans, wrong types, NaN, and infinities without leaking `TypeError` from `math.isfinite`.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run: `python -m unittest discover -s tests -p test_vehicle_ackermann.py -v`

Expected: 3 tests pass.

- [ ] **Step 5: Commit the validated pure solver skeleton**

```powershell
git add -- tools/vehicle/ackermann.py tests/test_vehicle_ackermann.py
git commit -m "feat: add safe Ackermann solver contract"
```

### Task 3: Turning geometry, steering clamp, reverse semantics, and uniform wheel scaling

**Files:**
- Modify: `tests/test_vehicle_ackermann.py`
- Modify: `tools/vehicle/ackermann.py`

- [ ] **Step 1: Add failing numerical behavior tests**

```python
def test_left_and_right_turns_select_the_correct_inner_wheel(self):
    left = solve_ackermann(500.0, 200.0, R3X_GEOMETRY, 600.0)
    right = solve_ackermann(500.0, -200.0, R3X_GEOMETRY, 600.0)
    self.assertLess(left.left_wheel_speed_mm_s, left.right_wheel_speed_mm_s)
    self.assertEqual((left.inner_side, left.outer_side), ("left", "right"))
    self.assertLess(right.right_wheel_speed_mm_s, right.left_wheel_speed_mm_s)
    self.assertEqual((right.inner_side, right.outer_side), ("right", "left"))

def test_reverse_keeps_geometric_inner_outer_meaning(self):
    result = solve_ackermann(-400.0, 200.0, R3X_GEOMETRY, 600.0)
    self.assertLess(abs(result.left_wheel_speed_mm_s), abs(result.right_wheel_speed_mm_s))
    self.assertEqual((result.inner_side, result.outer_side), ("left", "right"))

def test_twenty_degree_limit_has_389mm_radius_and_clamps_both_signs(self):
    for steering in (500.0, -500.0):
        result = solve_ackermann(100.0, steering, R3X_GEOMETRY, 600.0)
        self.assertTrue(result.steering_limited)
        self.assertAlmostEqual(abs(result.applied_steering_rad), math.radians(20.0))
        self.assertTrue(math.isclose(result.rear_axle_radius_abs_mm, 389.32,
                                     rel_tol=2e-4, abs_tol=0.05))
        self.assertFalse(math.isclose(result.rear_axle_radius_abs_mm, 350.0,
                                      rel_tol=1e-3, abs_tol=0.1))

def test_peak_wheel_limit_scales_both_wheels_and_applied_body_speed(self):
    unlimited = solve_ackermann(600.0, 300.0, R3X_GEOMETRY, 1000.0)
    limited = solve_ackermann(600.0, 300.0, R3X_GEOMETRY, 600.0)
    self.assertTrue(limited.wheel_speed_limited)
    self.assertEqual(max(abs(limited.left_wheel_speed_mm_s),
                         abs(limited.right_wheel_speed_mm_s)), 600.0)
    self.assertTrue(math.isclose(
        unlimited.left_wheel_speed_mm_s / unlimited.right_wheel_speed_mm_s,
        limited.left_wheel_speed_mm_s / limited.right_wheel_speed_mm_s,
        rel_tol=1e-12,
    ))
    self.assertAlmostEqual(limited.applied_speed_mm_s,
                           limited.requested_speed_mm_s * limited.wheel_speed_scale)
```

- [ ] **Step 2: Run the new tests and verify RED for missing turn/limit behavior**

Run: `python -m unittest discover -s tests -p test_vehicle_ackermann.py -v`

Expected: the newly added turn, radius, and scaling assertions fail while Task 2 tests stay green.

- [ ] **Step 3: Implement the complete solver equations**

```python
requested_rad = float(steering_mrad) / 1000.0
applied_rad = max(-geometry.max_steering_rad,
                  min(geometry.max_steering_rad, requested_rad))
curvature = math.tan(applied_rad) / geometry.wheelbase_mm
if curvature == 0.0:
    radius_signed = radius_abs = math.inf
    inner_side = outer_side = None
else:
    radius_signed = 1.0 / curvature
    radius_abs = abs(radius_signed)
    inner_side, outer_side = (("left", "right") if curvature > 0.0
                              else ("right", "left"))
left_raw = float(speed_mm_s) * (1.0 - curvature * geometry.track_mm / 2.0)
right_raw = float(speed_mm_s) * (1.0 + curvature * geometry.track_mm / 2.0)
peak = max(abs(left_raw), abs(right_raw))
scale = 1.0 if peak <= max_wheel_speed_mm_s else max_wheel_speed_mm_s / peak
left = left_raw * scale
right = right_raw * scale
```

Choose safety status deterministically: neither limit `OK`, steering only `STEERING_LIMITED`, speed only `SPEED_LIMITED`, both `LIMITED`. Set `applied_speed_mm_s = requested_speed_mm_s * scale` and keep inner/outer based only on curvature, never speed sign.

- [ ] **Step 4: Run the solver suite and verify GREEN**

Run: `python -m unittest discover -s tests -p test_vehicle_ackermann.py -v`

Expected: all solver tests pass.

- [ ] **Step 5: Commit the complete Ackermann mathematics**

```powershell
git add -- tools/vehicle/ackermann.py tests/test_vehicle_ackermann.py
git commit -m "feat: compute limited Ackermann wheel targets"
```

### Task 4: Exact CAN V1 command adaptation

**Files:**
- Create: `tests/test_vehicle_command.py`
- Create: `tools/vehicle/command.py`

- [ ] **Step 1: Write failing exact-frame and invalid-result tests**

```python
import unittest

from tools.vehicle.ackermann import solve_ackermann
from tools.vehicle.command import build_motion_frames
from tools.vehicle.geometry import R3X_GEOMETRY


class VehicleCommandTest(unittest.TestCase):
    def test_frames_reuse_frozen_v1_layout_and_one_result(self):
        result = solve_ackermann(500.0, 200.0, R3X_GEOMETRY, 600.0)
        steering, wheels = build_motion_frames(result, sequence=0x1234)
        self.assertEqual((steering.arbitration_id, wheels.arbitration_id), (0x120, 0x121))
        self.assertEqual(steering.data, bytes([1, 0x34, 0x12, 1, 200, 0, 0, 0]))
        self.assertEqual(steering.data[:4], wheels.data[:4])
        self.assertEqual(int.from_bytes(wheels.data[4:6], "little", signed=True),
                         round(result.left_wheel_speed_mm_s))
        self.assertEqual(int.from_bytes(wheels.data[6:8], "little", signed=True),
                         round(result.right_wheel_speed_mm_s))

    def test_twenty_degree_protocol_quantization_never_exceeds_349_mrad(self):
        result = solve_ackermann(0.0, 1000.0, R3X_GEOMETRY, 600.0)
        steering, _ = build_motion_frames(result, sequence=1)
        self.assertEqual(int.from_bytes(steering.data[4:6], "little", signed=True), 349)

    def test_invalid_result_and_invalid_sequence_create_no_normal_frames(self):
        invalid = solve_ackermann(float("nan"), 0.0, R3X_GEOMETRY, 600.0)
        with self.assertRaises(ValueError):
            build_motion_frames(invalid, sequence=1)
        with self.assertRaises(ValueError):
            build_motion_frames(
                solve_ackermann(0.0, 0.0, R3X_GEOMETRY, 600.0),
                sequence=0x10000,
            )
```

- [ ] **Step 2: Run the command tests and verify RED**

Run: `python -m unittest discover -s tests -p test_vehicle_command.py -v`

Expected: import failure for missing `tools.vehicle.command`.

- [ ] **Step 3: Implement protocol reuse and boundary quantization**

```python
from typing import Tuple

from tools.pid.protocol import CanFrame, encode_velocity_group

MAX_STEERING_MRAD_PROTOCOL = 349


def build_motion_frames(result, sequence):
    if not result.valid:
        raise ValueError("invalid Ackermann result cannot form a motion command")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence <= 0xFFFF:
        raise ValueError("command sequence must fit u16")
    steering = round(result.applied_steering_rad * 1000.0)
    steering = max(-MAX_STEERING_MRAD_PROTOCOL,
                   min(MAX_STEERING_MRAD_PROTOCOL, steering))
    frames = encode_velocity_group(
        sequence,
        round(result.left_wheel_speed_mm_s),
        round(result.right_wheel_speed_mm_s),
        mode_flags=0x01,
        steering_mrad=steering,
    )
    return frames[0], frames[1]
```

Do not reproduce the header, byte order, IDs, or integer serialization in `command.py`.

- [ ] **Step 4: Run the exact CAN tests and existing protocol tests**

Run: `python -m unittest discover -s tests -p test_vehicle_command.py -v`

Run: `python -m unittest discover -s tests -p test_pid_can_extension.py -v`

Expected: both commands pass; exact payload agrees with `docs/CAN_PROTOCOL.md` and the byte positions read by `BSP/bsp_bxcan.c`.

- [ ] **Step 5: Commit the frame adapter**

```powershell
git add -- tools/vehicle/command.py tests/test_vehicle_command.py
git commit -m "feat: adapt Ackermann results to CAN V1"
```

### Task 5: Ordered transmission, safe-stop status, watchdog fallback, and audit records

**Files:**
- Modify: `tests/test_vehicle_command.py`
- Modify: `tools/vehicle/command.py`

- [ ] **Step 1: Add failing transmission outcome tests**

```python
def test_partial_send_is_not_committed_and_requests_safe_stop(self):
    sent = []
    def sender(frame):
        sent.append(frame.arbitration_id)
        if frame.arbitration_id == 0x121:
            raise OSError("CAN TX failed")
    result = solve_ackermann(300.0, 100.0, R3X_GEOMETRY, 600.0)
    cycle = send_motion_command(result, 10, sender)
    self.assertEqual(sent, [0x120, 0x121])
    self.assertTrue(cycle.compute_ok)
    self.assertTrue(cycle.steering_tx_ok)
    self.assertFalse(cycle.wheels_tx_ok)
    self.assertFalse(cycle.command_committed)
    self.assertTrue(cycle.safe_stop_requested)

def test_invalid_result_sends_no_motion_frames_and_requests_safe_stop(self):
    sent = []
    invalid = solve_ackermann(math.nan, 0.0, R3X_GEOMETRY, 600.0)
    cycle = send_motion_command(invalid, 10, sent.append)
    self.assertEqual(sent, [])
    self.assertFalse(cycle.compute_ok)
    self.assertTrue(cycle.safe_stop_requested)

def test_safe_stop_uses_zero_group_and_reports_watchdog_fallback_on_tx_failure(self):
    sent = []
    ok = send_safe_stop(11, sent.append)
    self.assertTrue(ok.command_committed)
    self.assertEqual([frame.data for frame in sent], [
        bytes([1, 11, 0, 0, 0, 0, 0, 0]),
        bytes([1, 11, 0, 0, 0, 0, 0, 0]),
    ])
    failed = send_safe_stop(12, lambda frame: (_ for _ in ()).throw(OSError("down")))
    self.assertFalse(failed.safe_stop_tx_ok)
    self.assertTrue(failed.watchdog_fallback)

def test_every_cycle_emits_requested_applied_and_tx_audit_fields(self):
    records = []
    result = solve_ackermann(700.0, 500.0, R3X_GEOMETRY, 600.0)
    cycle = send_motion_command(result, 13, lambda frame: None, audit_sink=records.append,
                                timestamp=123.5)
    record = records[0]
    for key in ("timestamp", "requested_speed_mm_s", "requested_steering_mrad",
                "applied_speed_mm_s", "applied_steering_mrad", "curvature_per_mm",
                "rear_axle_radius_mm", "left_target_mm_s", "right_target_mm_s",
                "inner_side", "outer_side", "steering_limited", "wheel_speed_limited",
                "wheel_speed_scale", "result_valid", "safety_status", "reason",
                "steering_tx_ok", "wheels_tx_ok", "command_committed"):
        self.assertIn(key, record)
```

- [ ] **Step 2: Run the command tests and verify RED**

Run: `python -m unittest discover -s tests -p test_vehicle_command.py -v`

Expected: failures for missing cycle result/status and send functions.

- [ ] **Step 3: Implement immutable cycle status and ordered exception-safe sending**

Add `CommandCycleStatus` values `COMMAND_COMMITTED`, `INVALID_RESULT`, `STEERING_TX_FAILED`, `WHEELS_TX_FAILED`, `SAFE_STOP_COMMITTED`, and `WATCHDOG_FALLBACK`. Add a frozen `CommandCycleResult` with `compute_ok`, `steering_tx_ok`, `wheels_tx_ok`, `command_committed`, `safe_stop_requested`, `safe_stop_tx_ok`, `watchdog_fallback`, and `error`.

```python
def send_motion_command(result, sequence, send_frame, audit_sink=None, timestamp=None):
    if not result.valid:
        cycle = CommandCycleResult.invalid_result()
        _emit_audit(result, cycle, audit_sink, timestamp)
        return cycle
    steering, wheels = build_motion_frames(result, sequence)
    try:
        send_frame(steering)
    except Exception as exc:
        cycle = CommandCycleResult.steering_failed(str(exc))
    else:
        try:
            send_frame(wheels)
        except Exception as exc:
            cycle = CommandCycleResult.wheels_failed(str(exc))
        else:
            cycle = CommandCycleResult.committed()
    _emit_audit(result, cycle, audit_sink, timestamp)
    return cycle
```

`send_safe_stop(sequence, send_frame, ...)` must call the existing `encode_velocity_group(sequence, 0, 0, mode_flags=0x00, steering_mrad=0)` and use the same steering-then-wheels order. Any exception returns `WATCHDOG_FALLBACK`; it must never claim STOP was transmitted successfully.

- [ ] **Step 4: Implement complete audit record emission**

`build_audit_record()` must convert requested steering back to mrad, applied steering to quantized mrad, use the absolute/signed radius field consistently, serialize enum values with `.value`, and include all fields listed in specification section 37. `_emit_audit()` calls the supplied sink exactly once; when no sink is supplied it emits one `logging.getLogger(__name__).info(...)` record so production calls are still observable.

- [ ] **Step 5: Run command and protocol suites and verify GREEN**

Run: `python -m unittest discover -s tests -p test_vehicle_command.py -v`

Run: `python -m unittest discover -s tests -p 'test_*protocol.py' -v`

Expected: all command and protocol tests pass; partial failures never set committed.

- [ ] **Step 6: Commit cycle safety and audit behavior**

```powershell
git add -- tools/vehicle/command.py tests/test_vehicle_command.py
git commit -m "feat: report Ackermann command cycle safety"
```

### Task 6: Stable package API and integration-level acceptance tests

**Files:**
- Create: `tools/vehicle/__init__.py`
- Modify: `tests/test_vehicle_geometry.py`
- Modify: `tests/test_vehicle_ackermann.py`
- Modify: `tests/test_vehicle_command.py`

- [ ] **Step 1: Add a failing public API smoke test**

```python
def test_public_package_exports_complete_control_path(self):
    from tools.vehicle import (R3X_GEOMETRY, build_motion_frames,
                               load_vehicle_config, send_motion_command,
                               send_safe_stop, solve_ackermann)
    config = load_vehicle_config()
    result = solve_ackermann(300.0, 100.0, config.geometry,
                             config.max_wheel_speed_mm_s)
    self.assertEqual(len(build_motion_frames(result, 1)), 2)
```

- [ ] **Step 2: Run the smoke test and verify RED**

Run: `python -m unittest discover -s tests -p test_vehicle_command.py -v`

Expected: import failure because the package has not exported the stable API.

- [ ] **Step 3: Export only the supported API**

```python
from .ackermann import AckermannResult, MotionSafetyStatus, solve_ackermann
from .command import (CommandCycleResult, CommandCycleStatus, build_motion_frames,
                      send_motion_command, send_safe_stop)
from .config import VehicleConfig, load_vehicle_config
from .geometry import R3X_GEOMETRY, VehicleGeometry

__all__ = [
    "AckermannResult", "CommandCycleResult", "CommandCycleStatus",
    "MotionSafetyStatus", "R3X_GEOMETRY", "VehicleConfig", "VehicleGeometry",
    "build_motion_frames", "load_vehicle_config", "send_motion_command",
    "send_safe_stop", "solve_ackermann",
]
```

- [ ] **Step 4: Run all vehicle tests**

Run: `python -m unittest discover -s tests -p 'test_vehicle_*.py' -v`

Expected: all vehicle tests pass with no errors or warnings.

- [ ] **Step 5: Commit the public package boundary**

```powershell
git add -- tools/vehicle/__init__.py tests/test_vehicle_geometry.py tests/test_vehicle_ackermann.py tests/test_vehicle_command.py
git commit -m "feat: expose R3X vehicle motion API"
```

### Task 7: Full regression, firmware build, and specification audit

**Files:**
- Modify only if verification finds an Ackermann-scope defect.

- [ ] **Step 1: Run the complete Python regression suite**

Run: `python -m unittest discover -s tests -v`

Expected: all existing and new tests pass; zero failures and zero errors.

- [ ] **Step 2: Build the ordinary non-bench firmware**

Run: `cmake --build build\Debug --target ros`

Expected: exit code 0. This proves the host-only additions did not disturb the existing STM32 build. Do not flash or start any hardware mode.

- [ ] **Step 3: Run source and whitespace checks**

Run: `python -m compileall -q tools\vehicle tests\test_vehicle_geometry.py tests\test_vehicle_ackermann.py tests\test_vehicle_command.py`

Run: `git diff --check`

Expected: both exit 0.

- [ ] **Step 4: Audit every software acceptance item from specification section 69**

Record evidence for geometry centralization, derived radius, pure solver, straight/turn/reverse behavior, 20-degree clamp, 389.3 mm radius, uniform scaling, requested/applied split, zero-speed steering, nonfinite rejection, invalid-result send inhibition, MCU-compatible encoding, same-result frame construction, partial-send failure, safe-stop fallback, new tests, and full regression. Mark hardware stages 58–63 as pending operator-controlled validation rather than claiming them complete.

- [ ] **Step 5: Request code review and address all critical/important findings**

Review only the Ackermann commits/files against the approved design and this plan. Re-run focused and full verification after any correction.

- [ ] **Step 6: Commit any verification-driven corrections**

```powershell
git add -- tools/vehicle tests/test_vehicle_geometry.py tests/test_vehicle_ackermann.py tests/test_vehicle_command.py
git commit -m "fix: address Ackermann motion review findings"
```

Skip this commit when review finds no required correction.
