import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as f:
        return f.read()


class BxcanLoopbackSelfTestStructure(unittest.TestCase):
    def test_loopback_module_exposes_gdb_readable_result(self):
        self.assertTrue(os.path.exists(os.path.join(ROOT, "BSP", "bsp_bxcan_loopback.h")))
        self.assertTrue(os.path.exists(os.path.join(ROOT, "BSP", "bsp_bxcan_loopback.c")))

        header = read_rel("BSP/bsp_bxcan_loopback.h")
        source = read_rel("BSP/bsp_bxcan_loopback.c")

        self.assertIn("BSP_BXCAN_LoopbackResult_t", header)
        self.assertIn("g_bxcan_loopback_result", header)
        self.assertIn("BSP_BXCAN_RunLoopbackSelfTest", header)
        self.assertIn("CAN_MODE_LOOPBACK", source)
        self.assertIn("HAL_CAN_AddTxMessage", source)
        self.assertIn("HAL_CAN_GetRxMessage", source)
        self.assertIn("BSP_BXCAN_LOOPBACK_TEST_STD_ID", source)

    def test_loopback_selftest_is_default_off_and_enabled_by_cmake_option(self):
        cmake = read_rel("CMakeLists.txt")

        self.assertIn("option(BSP_BXCAN_RUN_LOOPBACK_SELF_TEST", cmake)
        self.assertIn("BSP/bsp_bxcan_loopback.c", cmake)
        self.assertIn("target_compile_definitions", cmake)
        self.assertIn("BSP_BXCAN_RUN_LOOPBACK_SELF_TEST", cmake)

    def test_main_runs_loopback_selftest_only_under_macro(self):
        main = read_rel("Core/Src/main.c")

        self.assertIn("#ifdef BSP_BXCAN_RUN_LOOPBACK_SELF_TEST", main)
        self.assertIn('#include "bsp_bxcan_loopback.h"', main)
        self.assertIn("BSP_BXCAN_RunLoopbackSelfTest(&hcan1)", main)
        self.assertIn("#else", main)
        self.assertIn("Chassis_ControlInit();", main)


if __name__ == "__main__":
    unittest.main()
