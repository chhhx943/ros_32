import json
import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as handle:
        return handle.read()


class SteeringBenchStructureTest(unittest.TestCase):
    def test_steering_bench_is_default_off_and_has_a_build_preset(self):
        cmake = read_rel("CMakeLists.txt")
        presets = json.loads(read_rel("CMakePresets.json"))
        configure = {item["name"]: item for item in presets["configurePresets"]}
        build = {item["name"]: item for item in presets["buildPresets"]}

        self.assertRegex(
            cmake,
            r'option\(STEERING_BENCH_TEST\s+"[^"]+"\s+OFF\)',
        )
        self.assertIn("BSP/steering_bench.c", cmake)
        self.assertIn("Steering_Bench_Run();", read_rel("Core/Src/main.c"))
        self.assertIn("STEERING_BENCH_TEST", read_rel("Core/Src/main.c"))
        self.assertIn("SteeringBench", configure)
        self.assertEqual(
            configure["SteeringBench"]["cacheVariables"].get("STEERING_BENCH_TEST"),
            "ON",
        )
        self.assertIn("SteeringBench", build)

    def test_steering_bench_has_five_zero_and_plus_minus_five_degree_phases(self):
        header = read_rel("BSP/steering_bench.h")
        source = read_rel("BSP/steering_bench.c")

        self.assertIn("STEERING_BENCH_PHASE_COUNT         5U", header)
        self.assertIn("STEERING_BENCH_HOLD_MS", header)
        self.assertIn("STEERING_BENCH_TARGET_MRAD         87", header)
        self.assertIn("STEERING_BENCH_RESULT_MAGIC", header)
        self.assertIn("g_steering_bench_result", header)
        self.assertIn("CAN_MODE_LOOPBACK", source)
        self.assertIn("CAN_Motor_Bench_BuildSteeringFrame", source)
        self.assertIn("CAN_Motor_Bench_BuildWheelsFrame", source)
        self.assertIn("STEERING_BENCH_HOLD_MS", source)
        self.assertIn("STEERING_BENCH_GROUP_PERIOD_MS", source)
        self.assertIn("Servo_GetPulseUs", source)
        self.assertIn("BSP_BXCAN_ID_CMD_STEERING", source)
        self.assertIn("BSP_BXCAN_ID_CMD_REAR_WHEELS", source)

        for target in (
            "0,",
            "STEERING_BENCH_TARGET_MRAD",
            "-STEERING_BENCH_TARGET_MRAD",
        ):
            self.assertIn(target, source)

    def test_steering_bench_never_requests_nonzero_rear_wheel_speed(self):
        source = read_rel("BSP/steering_bench.c")

        self.assertNotIn("Motor_Drive(", source)
        self.assertIn("CAN_Motor_Bench_BuildWheelsFrame", source)
        self.assertIn(", 0, 0, wheels)", source)

    def test_zero_speed_steering_is_preserved_in_standby(self):
        source = read_rel("BSP/chassis_control.c")

        self.assertIn("Chassis_ControlApplyStaticSteering", source)
        self.assertIn("SAFETY_STATE_STANDBY", source)
        self.assertIn("Servo_SetAngleMrad(g_target_command.equivalent_steering_mrad)", source)


if __name__ == "__main__":
    unittest.main()
