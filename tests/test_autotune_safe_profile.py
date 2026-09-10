import json
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


def read_text(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class AutotuneSafeProfileTest(unittest.TestCase):
    def test_cmake_declares_dedicated_autotune_safe_profile_and_source(self):
        cmake = read_text("CMakeLists.txt")
        self.assertIn("AUTOTUNE_SAFE_PROFILE", cmake)
        self.assertIn("BSP/autotune_safe.c", cmake)

    def test_dedicated_preset_enables_profile_and_keeps_motion_bench_off(self):
        presets = json.loads(read_text("CMakePresets.json"))
        configure = {item["name"]: item for item in presets["configurePresets"]}
        self.assertIn("AutotuneSafe", configure)
        safe = configure["AutotuneSafe"]["cacheVariables"]
        self.assertEqual(safe["AUTOTUNE_SAFE_PROFILE"], "ON")
        self.assertEqual(safe["CAN_MOTOR_BENCH_TEST"], "OFF")
        self.assertEqual(safe["ACKERMANN_BENCH_TEST"], "OFF")

    def test_normal_profiles_do_not_define_runtime_autotune_switch(self):
        cmake = read_text("CMakeLists.txt")
        presets = json.loads(read_text("CMakePresets.json"))
        for name in ("Debug", "Release"):
            cache = next(item for item in presets["configurePresets"]
                         if item["name"] == name)["cacheVariables"]
            self.assertNotIn("AUTOTUNE_SAFE_PROFILE", cache)
        self.assertIn("if(AUTOTUNE_SAFE_PROFILE)", cmake)
        self.assertIn("AUTOTUNE_SAFE_PROFILE", cmake)

    def test_local_profile_uses_only_frozen_calibration_baseline(self):
        cmake = read_text("CMakeLists.txt")
        safe_block = cmake.split("if(AUTOTUNE_SAFE_PROFILE)", 1)[1].split("endif()", 1)[0]
        self.assertIn("CALIBRATION_BENCH_DEFAULTS", safe_block)
        for name in ("Debug", "Release"):
            cache = next(item for item in json.loads(read_text("CMakePresets.json"))["configurePresets"]
                         if item["name"] == name)["cacheVariables"]
            self.assertNotIn("CALIBRATION_BENCH_DEFAULTS", cache)

    def test_final_motor_drive_path_declares_autotune_gate(self):
        motor = read_text("BSP/bsp_motor.c")
        self.assertIn("autotune_safe.h", motor)
        self.assertIn("AutotuneSafe_", motor)
        self.assertIn("Motor_Drive", motor)

    def test_main_guards_autotune_initialization_at_compile_time(self):
        main = read_text("Core/Src/main.c")
        self.assertIn("#ifdef AUTOTUNE_SAFE_PROFILE", main)
        self.assertIn("AutotuneSafe_", main)


if __name__ == "__main__":
    unittest.main()
