import os
import re
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as f:
        return f.read()


class MotorBenchTestStructure(unittest.TestCase):
    def test_bench_option_is_default_off_and_wired_in_cmake(self):
        cmake = read_rel("CMakeLists.txt")
        self.assertRegex(
            cmake,
            r'option\(MOTOR_BENCH_TEST\s+"[^"]+"\s+OFF\)',
        )
        self.assertIn("MOTOR_BENCH_TEST", cmake)

    def test_main_runs_motors_sequentially_then_coasts(self):
        main = read_rel("Core/Src/main.c")
        self.assertIn("#ifdef MOTOR_BENCH_TEST", main)
        self.assertIn("Motor_Init();", main)
        bench = main.split("#ifdef MOTOR_BENCH_TEST", 1)[1].split(
            "#elif defined(BSP_BXCAN_RUN_LOOPBACK_SELF_TEST)", 1
        )[0]
        self.assertIn("Motor_CoastAll();", bench)

        ordered_steps = (
            "Motor_Drive(1U, 200);",
            "HAL_Delay(3000);",
            "Motor_Coast(1U);",
            "HAL_Delay(1000);",
            "Motor_Drive(2U, 200);",
            "HAL_Delay(3000);",
            "Motor_Coast(2U);",
        )
        positions = []
        search_from = 0
        for step in ordered_steps:
            position = bench.index(step, search_from)
            positions.append(position)
            search_from = position + len(step)
        self.assertEqual(positions, sorted(positions))
        self.assertRegex(bench, r"Motor_Coast\(2U\);\s*while \(1\)")
        self.assertIn("Chassis_ControlInit();", main)


if __name__ == "__main__":
    unittest.main()
