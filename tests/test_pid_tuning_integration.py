import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as handle:
        return handle.read()


class PidTuningIntegrationTest(unittest.TestCase):
    def test_pid_tuning_module_is_built_and_wired_into_control_loop(self):
        cmake = read_rel("CMakeLists.txt")
        control = read_rel("BSP/chassis_control.c")
        self.assertIn("BSP/pid_tuning.c", cmake)
        self.assertIn("PID_Tuning_Init()", control)
        self.assertIn("PID_Tuning_GetGains", control)

    def test_can_routes_pid_commands_and_periodic_ack(self):
        source = read_rel("BSP/bsp_bxcan.c")
        header = read_rel("BSP/bsp_bxcan.h")
        self.assertIn("PID_TUNING_ID_CMD_GAINS", header)
        self.assertIn("PID_TUNING_ID_CMD_D", header)
        self.assertIn("PID_TUNING_ID_FB_GAINS", header)
        self.assertIn("PID_Tuning_OnFrame", source)
        self.assertIn("PID_Tuning_BuildFeedbackFrame", source)

    def test_can_feedback_exposes_actual_control_output_for_live_logging(self):
        source = read_rel("BSP/bsp_bxcan.c")
        header = read_rel("BSP/bsp_bxcan.h")
        control = read_rel("BSP/chassis_control.c")
        self.assertIn("BSP_BXCAN_ID_FB_CONTROL_OUTPUT", header)
        self.assertIn("BSP_BXCAN_ID_FB_CONTROL_OUTPUT", source)
        self.assertIn("left_control_output", control)
        self.assertIn("right_control_output", control)

    def test_autotune_profile_routes_runtime_gains_through_mcu_validator(self):
        tuning = read_rel("BSP/pid_tuning.c")
        self.assertIn("autotune_safe.h", tuning)
        self.assertIn("AutotuneSafe_ValidateCandidate", tuning)
        self.assertIn("AutotuneSafe_SetCandidate", tuning)
        self.assertIn("AutotuneSafe_GetRecoveryGains", tuning)


if __name__ == "__main__":
    unittest.main()
