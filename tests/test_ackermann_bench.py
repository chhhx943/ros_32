import json
import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as handle:
        return handle.read()


class AckermannBenchStructureTest(unittest.TestCase):
    def test_ackermann_bench_is_default_off_and_has_a_build_preset(self):
        cmake = read_rel("CMakeLists.txt")
        presets = json.loads(read_rel("CMakePresets.json"))
        configure = {item["name"]: item for item in presets["configurePresets"]}
        build = {item["name"]: item for item in presets["buildPresets"]}

        self.assertRegex(cmake, r'option\(ACKERMANN_BENCH_TEST\s+"[^"]+"\s+OFF\)')
        self.assertIn("BSP/ackermann_bench.c", cmake)
        self.assertIn("Ackermann_Bench_Run();", read_rel("Core/Src/main.c"))
        self.assertIn("ACKERMANN_BENCH_TEST", read_rel("Core/Src/main.c"))
        self.assertIn("AckermannBench", configure)
        self.assertEqual(
            configure["AckermannBench"]["cacheVariables"].get("ACKERMANN_BENCH_TEST"),
            "ON",
        )
        self.assertIn("AckermannBench", build)

    def test_bench_vectors_match_conservative_host_ackermann_outputs(self):
        header = read_rel("BSP/ackermann_bench.h")
        source = read_rel("BSP/ackermann_bench.c")

        self.assertIn("ACKERMANN_BENCH_PHASE_COUNT         5U", header)
        self.assertIn("ACKERMANN_BENCH_BODY_SPEED_MMPS    200", header)
        self.assertIn("ACKERMANN_BENCH_TURN_STEERING_MRAD 200", header)
        self.assertIn("ACKERMANN_BENCH_RESULT_MAGIC", header)
        self.assertIn("CAN_Motor_Bench_BuildSteeringFrame", source)
        self.assertIn("CAN_Motor_Bench_BuildWheelsFrame", source)
        self.assertIn("ACKERMANN_BENCH_TURN_LEFT_MMPS", source)
        self.assertIn("ACKERMANN_BENCH_TURN_RIGHT_MMPS", source)
        self.assertIn("Encoder_Sample", source)
        self.assertIn("__HAL_TIM_GET_COMPARE(&htim3", source)

    def test_bench_keeps_test_speed_conservative_and_checks_directional_response(self):
        header = read_rel("BSP/ackermann_bench.h")
        source = read_rel("BSP/ackermann_bench.c")

        self.assertIn("ACKERMANN_BENCH_HOLD_MS", header)
        self.assertNotIn("CAN_MOTOR_BENCH_TARGET_MMPS", source)
        self.assertIn("left_encoder_delta_after", header)
        self.assertIn("right_encoder_delta_after", header)
        self.assertIn("BSP_BXCAN_FAULT_NONE", source)
        self.assertIn("Motor_EmergencyBrakeAll", source)

    def test_bench_initializes_safety_inputs_before_sampling_estop(self):
        source = read_rel("BSP/ackermann_bench.c")

        self.assertLess(source.index("Chassis_ControlInit();"),
                        source.index("Physical_EStop_IsAsserted()"))


if __name__ == "__main__":
    unittest.main()
