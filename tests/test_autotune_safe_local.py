import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class AutotuneSafeLocalContractTest(unittest.TestCase):
    def read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8")

    def test_local_executor_is_profile_only_and_has_explicit_state_machine(self):
        cmake = self.read("CMakeLists.txt")
        header = self.read("BSP/autotune_safe_local.h")
        source = self.read("BSP/autotune_safe_local.c")
        self.assertIn("AUTOTUNE_SAFE_PROFILE", cmake)
        self.assertIn("autotune_safe_local.c", cmake)
        for state in ("WAIT_STILL", "RAMP_UP", "HOLD", "RAMP_DOWN", "EVALUATE", "COOLING"):
            self.assertIn(state, header + source)
        self.assertIn("g_autotune_safe_local_control", header + source)
        self.assertIn("Encoder_Sample", source)
        self.assertIn("PID_UpdateDt", source)
        self.assertIn("Motor_Drive", source)
        self.assertNotIn("HAL_Delay", source)

    def test_local_cooling_and_idle_keep_a_fresh_zero_stop_command(self):
        source = self.read("BSP/autotune_safe_local.c")
        self.assertIn("LocalSetCommand(g_autotune_safe_local_control.active_wheel, 0, now_ms);", source)
        self.assertIn("LocalSetCommand(0U, 0, now_ms);", source)

    def test_local_pid_flow_requires_open_loop_start_scan_before_candidates(self):
        header = self.read("BSP/autotune_safe_local.h")
        source = self.read("BSP/autotune_safe_local.c")
        self.assertIn("AUTOTUNE_SAFE_LOCAL_OPEN_LOOP_SCAN", header)
        self.assertIn("g_open_loop_pwm", source)
        self.assertIn("open_loop_start_pwm", header + source)
        self.assertIn("g_open_loop_start_pwm", source)

    def test_local_start_is_debug_request_only_and_does_not_change_safety_envelope(self):
        header = self.read("BSP/autotune_safe_local.h")
        source = self.read("BSP/autotune_safe_local.c")
        self.assertIn("AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC", header)
        self.assertIn("request_magic", source)
        self.assertIn("AutotuneSafe_ConfirmPreflight", source)
        self.assertIn("Motor_Drive", source)
        self.assertNotIn("pwm_limit =", source)
        self.assertNotIn("target_limit_mmps =", source)
        self.assertNotIn("level =", source)

    def test_local_executor_is_called_only_in_safe_profile_and_normal_boot_does_not_start_it(self):
        cmake = self.read("CMakeLists.txt")
        main = self.read("Core/Src/main.c")
        chassis = self.read("BSP/chassis_control.c")
        self.assertIn("if(AUTOTUNE_SAFE_PROFILE)", cmake)
        self.assertIn("AutotuneSafe_LocalExperiment_Process", chassis)
        self.assertIn("AUTOTUNE_SAFE_PROFILE", chassis)
        self.assertNotIn("AutotuneSafe_LocalExperiment_RequestStart", main)


if __name__ == "__main__":
    unittest.main()
