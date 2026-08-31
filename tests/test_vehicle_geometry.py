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
        self.assertAlmostEqual(
            R3X_GEOMETRY.max_steering_rad,
            math.radians(20.0),
        )

    def test_geometry_rejects_nonfinite_nonpositive_and_singular_values(self):
        valid = {
            "wheelbase_mm": 141.7,
            "track_mm": 120.0,
            "tire_diameter_mm": 65.0,
            "max_steering_rad": math.radians(20.0),
        }
        invalid_values = (
            ("wheelbase_mm", 0.0),
            ("track_mm", -1.0),
            ("tire_diameter_mm", math.inf),
            ("max_steering_rad", math.pi / 2.0),
        )

        for field, value in invalid_values:
            values = dict(valid)
            values[field] = value
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    VehicleGeometry(**values)

    def test_default_config_is_explicit_and_protocol_bounded(self):
        config = load_vehicle_config()

        self.assertEqual(config.geometry, R3X_GEOMETRY)
        self.assertEqual(config.max_wheel_speed_mm_s, 600.0)

    def test_config_rejects_missing_or_out_of_protocol_wheel_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            for value in (None, 0.0, math.inf, 3000.1):
                body = {
                    "wheelbase_mm": 141.7,
                    "track_mm": 120.0,
                    "tire_diameter_mm": 65.0,
                    "max_steering_deg": 20.0,
                }
                if value is not None:
                    body["max_wheel_speed_mm_s"] = value
                path.write_text(json.dumps(body), encoding="utf-8")
                with self.subTest(value=value):
                    with self.assertRaises(ValueError):
                        load_vehicle_config(path)


if __name__ == "__main__":
    unittest.main()
