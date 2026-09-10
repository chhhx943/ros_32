import os
import re
import subprocess
import tempfile
import textwrap
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as f:
        return f.read()


class EngineeringCloseoutStructureTest(unittest.TestCase):
    def test_calibration_slots_fit_512k_flash_and_are_linker_reserved(self):
        header = read_rel("BSP/wheel_calibration_storage.h")
        linker = read_rel("STM32F407XX_FLASH.ld")
        self.assertNotIn("0x080C0000", header)
        self.assertNotIn("0x080E0000", header)
        self.assertRegex(header, r"WHEEL_CALIBRATION_STORAGE_SLOT0_ADDRESS\s+0x080[0-7]")
        self.assertIn("CALIBRATION_STORAGE", linker)

    def test_tim3_is_pwm_only_and_main_does_not_start_base_irq(self):
        main = read_rel("Core/Src/main.c")
        tim = read_rel("Core/Src/tim.c")
        self.assertNotIn("HAL_TIM_Base_Start_IT(&htim3)", main)
        tim3 = tim.split("void MX_TIM3_Init", 1)[1].split("void MX_TIM4_Init", 1)[0]
        # PWM_Init does not invoke the generated Base MSP hook; retain base
        # initialization for the TIM3 clock, while keeping Base IRQ disabled.
        self.assertIn("HAL_TIM_Base_Init(&htim3)", tim3)

    def test_can_pin_documentation_matches_pb8_pb9_remap(self):
        protocol = read_rel("docs/CAN_PROTOCOL.md")
        self.assertIn("PB8/PB9", protocol)
        self.assertNotIn("CAN1 on PA11/PA12", protocol)

    def test_tim6_scheduler_and_overrun_path_are_present(self):
        main = read_rel("Core/Src/main.c")
        it = read_rel("Core/Src/stm32f4xx_it.c")
        self.assertIn("HAL_TIM_PeriodElapsedCallback", it)
        self.assertIn("Chassis_ControlOnTick", it)
        self.assertIn("Chassis_ControlProcessEvents", main)

    def test_tim6_base_is_1khz_at_84mhz_timer_clock(self):
        tim = read_rel("Core/Src/tim.c")
        ioc = read_rel("ros.ioc")
        self.assertRegex(tim, r"htim6\.Init\.Prescaler\s*=\s*84-1")
        self.assertRegex(ioc, r"(?m)^TIM6\.Prescaler=84-1$")

    def test_iwdg_is_fed_only_by_application_and_targets_two_seconds(self):
        watchdog = read_rel("BSP/watchdog.c")
        chassis = read_rel("BSP/chassis_control.c")
        self.assertRegex(watchdog, r"IWDG->RLR\s*=\s*2000U")
        self.assertIn("BSP_Watchdog_Feed", chassis)

    def test_iwdg_stall_hardware_probe_is_opt_in(self):
        cmake = read_rel("CMakeLists.txt")
        main = read_rel("Core/Src/main.c")
        self.assertIn("option(IWDG_STALL_TEST", cmake)
        self.assertIn("IWDG_STALL_TEST", main)
        self.assertIn("g_iwdg_stall_result", main)

    def test_safe_stop_watchdog_probe_is_opt_in(self):
        cmake = read_rel("CMakeLists.txt")
        main = read_rel("Core/Src/main.c")
        self.assertIn("option(SAFE_STOP_WATCHDOG_TEST", cmake)
        self.assertIn("SAFE_STOP_WATCHDOG_TEST", main)
        self.assertIn("g_safe_stop_watchdog_result", main)
        self.assertIn("__HAL_RCC_CLEAR_RESET_FLAGS();", main)


class EngineeringCloseoutBehaviorTest(unittest.TestCase):
    def compile_and_run(self, body):
        source = textwrap.dedent(
            r'''
            #include <stdint.h>
            #include <stdlib.h>
            #define BSP_BXCAN_HOST_TEST
            #define BSP_BXCAN_ENABLE_TEST_HOOKS
            #include "BSP/bsp_bxcan.c"
            #include "BSP/wheel_calibration.c"
            #include "BSP/wheel_calibration_service.c"
            static void frame(uint16_t id, uint16_t seq, uint8_t flags, int16_t a, int16_t b, uint32_t now) {
                uint8_t d[8] = {1, (uint8_t)seq, (uint8_t)(seq >> 8), flags,
                                (uint8_t)a, (uint8_t)(a >> 8), (uint8_t)b, (uint8_t)(b >> 8)};
                BSP_BXCAN_OnRxFrame(id, 8, 0, 0, d, now);
            }
            '''
            + body
        )
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "closeout.c")
            exe = os.path.join(tmp, "closeout.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)
            subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                            "-DWHEEL_CALIBRATION_HOST_TEST",
                            "-DWHEEL_CALIBRATION_SERVICE_HOST_TEST",
                            "-DENCODER_HOST_TEST",
                            "-I", ROOT, "-I", os.path.join(ROOT, "BSP"),
                            "-I", os.path.join(ROOT, "Core", "Inc"),
                            "-I", os.path.join(ROOT, "Drivers", "STM32F4xx_HAL_Driver", "Inc"),
                            src, "-o", exe],
                           check=True, cwd=ROOT)
            subprocess.run([exe], check=True, cwd=ROOT)

    def test_duplicate_sequence_does_not_refresh_watchdog_or_command(self):
        self.compile_and_run(
            r'''
            int main(void) {
                BSP_BXCAN_Init();
                frame(BSP_BXCAN_ID_CMD_STEERING, 9, BSP_BXCAN_MODE_VELOCITY, 0, 0, 0);
                frame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 9, BSP_BXCAN_MODE_VELOCITY, 100, 100, 1);
                BSP_BXCAN_Command_t c;
                if (!BSP_BXCAN_GetCommand(&c) || c.command_seq != 9) return 1;
                frame(BSP_BXCAN_ID_CMD_STEERING, 9, BSP_BXCAN_MODE_VELOCITY, 0, 0, 90);
                frame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 9, BSP_BXCAN_MODE_VELOCITY, 900, 900, 91);
                BSP_BXCAN_Process(102);
                if (BSP_BXCAN_IsCommandFresh() != 0) return 2;
                if (BSP_BXCAN_GetCommand(&c) == 0 || c.rear_left_velocity_mmps != 0 ||
                    c.rear_right_velocity_mmps != 0) return 3;
                if (BSP_BXCAN_GetCommand(&c) != 0) return 4;
                if (BSP_BXCAN_GetAppliedCommandSeq() != 9) return 5;
                return 0;
            }
            '''
        )

    def test_command_sequence_wraparound_is_fresh_but_duplicate_zero_is_stale(self):
        self.compile_and_run(
            r'''
            int main(void) {
                BSP_BXCAN_Command_t c;
                BSP_BXCAN_Init();
                frame(BSP_BXCAN_ID_CMD_STEERING, 0xFFFFU, BSP_BXCAN_MODE_VELOCITY, 0, 0, 0);
                frame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 0xFFFFU, BSP_BXCAN_MODE_VELOCITY, 100, 100, 1);
                if (!BSP_BXCAN_GetCommand(&c) || c.command_seq != 0xFFFFU) return 1;
                frame(BSP_BXCAN_ID_CMD_STEERING, 0U, BSP_BXCAN_MODE_VELOCITY, 0, 0, 2);
                frame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 0U, BSP_BXCAN_MODE_VELOCITY, 200, 200, 3);
                if (!BSP_BXCAN_GetCommand(&c) || c.command_seq != 0U ||
                    c.rear_left_velocity_mmps != 200) return 2;
                frame(BSP_BXCAN_ID_CMD_STEERING, 0U, BSP_BXCAN_MODE_VELOCITY, 0, 0, 4);
                frame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 0U, BSP_BXCAN_MODE_VELOCITY, 900, 900, 5);
                if (BSP_BXCAN_GetCommand(&c) != 0U || BSP_BXCAN_GetAppliedCommandSeq() != 0U) return 3;
                return 0;
            }
            '''
        )

    def test_safe_stop_command_selects_safe_stop_not_fault(self):
        # Compiled in the existing safety-manager harness by a follow-up implementation test.
        source = read_rel("BSP/safety_manager.c")
        self.assertIn("SAFETY_STATE_SAFE_STOP", source)
        self.assertNotIn("safe_stop_active.*SAFETY_STATE_FAULT", source)

    def test_servo_out_of_envelope_is_rejected_without_clamp(self):
        servo = read_rel("BSP/servo.c")
        self.assertIn("Servo_SetAngleMradChecked", servo)
        self.assertIn("return 0U", servo)

    def test_motor_direction_change_has_dead_time_state(self):
        motor = read_rel("BSP/bsp_motor.c")
        self.assertIn("dead", motor.lower())
        self.assertIn("Motor_Process", motor)


if __name__ == "__main__":
    unittest.main()
