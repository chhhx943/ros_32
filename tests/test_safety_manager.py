import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


COMMON_STUBS = textwrap.dedent(
    r"""
    #include <stdint.h>
    #include <stdlib.h>

    #include "BSP/bsp_bxcan.h"

    uint8_t g_physical_estop_pin_level = 1U;
    static uint16_t fake_fault = BSP_BXCAN_FAULT_NONE;
    static uint16_t set_fault_code;
    static uint8_t set_fault_latched;

    uint16_t BSP_BXCAN_GetFault(void) { return fake_fault; }
    void BSP_BXCAN_SetFault(uint16_t fault_code, uint8_t latched)
    {
        set_fault_code = fault_code;
        set_fault_latched = latched;
        fake_fault = fault_code;
    }

    #include "BSP/physical_estop.c"
    #include "BSP/wheel_calibration.c"
    #include "BSP/wheel_calibration_service.c"
    #include "BSP/safety_manager.c"

    static void install_valid_calibration(void)
    {
        CalibrationData_t data = {1U, 1, -1, 100U, 100U, 1U, 0U};
        Wheel_Calibration_Install(&data);
    }

    static BSP_BXCAN_Command_t make_command(uint8_t mode_flags,
                                            int16_t left,
                                            int16_t right)
    {
        BSP_BXCAN_Command_t command = {0};
        command.mode_flags = mode_flags;
        command.rear_left_velocity_mmps = left;
        command.rear_right_velocity_mmps = right;
        return command;
    }
    """
)


class SafetyManagerHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_safety_manager.c")
            exe = os.path.join(tmp, "test_safety_manager.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DPHYSICAL_ESTOP_HOST_TEST",
                    "-DWHEEL_CALIBRATION_HOST_TEST",
                    "-DWHEEL_CALIBRATION_SERVICE_HOST_TEST",
                    "-DENCODER_HOST_TEST",
                    "-Wno-unused-function",
                    "-DSAFETY_MANAGER_HOST_TEST",
                    "-DBSP_BXCAN_HOST_TEST",
                    "-I",
                    ROOT,
                    "-I",
                    os.path.join(ROOT, "BSP"),
                    src,
                    "-o",
                    exe,
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([exe], check=True, cwd=ROOT)

    def test_physical_estop_requires_release_and_explicit_reset_before_drive(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t velocity = make_command(BSP_BXCAN_MODE_VELOCITY, 300, 300);
                BSP_BXCAN_Command_t reset_stop = make_command(
                    BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_RESET_FAULT, 0, 0);

                Safety_Manager_Init();
                install_valid_calibration();
                Safety_Manager_Process(0U);
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetState() != SAFETY_STATE_DRIVE) return 1;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_DRIVE) return 2;

                g_physical_estop_pin_level = 0U;
                Safety_Manager_Process(10U);
                if (Safety_Manager_GetState() != SAFETY_STATE_ESTOP) return 3;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_BRAKE) return 4;
                if (Safety_Manager_DriveAllowed() != 0U) return 5;

                g_physical_estop_pin_level = 1U;
                Safety_Manager_Process(20U);
                if (Safety_Manager_GetState() != SAFETY_STATE_ESTOP) return 6;
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetAction() != SAFETY_ACTION_BRAKE) return 7;

                Safety_Manager_AcceptCommand(&reset_stop);
                if (Safety_Manager_GetState() != SAFETY_STATE_ESTOP) return 8;
                Safety_Manager_Process(70U);
                Safety_Manager_AcceptCommand(&reset_stop);
                if (Safety_Manager_GetState() != SAFETY_STATE_STANDBY) return 9;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 10;
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetState() != SAFETY_STATE_DRIVE) return 11;
                return 0;
            }
            """
        )

    def test_estop_beats_fault_and_safe_stop_beats_velocity(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t velocity = make_command(BSP_BXCAN_MODE_VELOCITY, 300, 300);
                BSP_BXCAN_Command_t stop = make_command(BSP_BXCAN_MODE_STOP, 0, 0);
                BSP_BXCAN_Command_t safe_stop = make_command(
                    BSP_BXCAN_MODE_VELOCITY | BSP_BXCAN_FLAG_SAFE_STOP, 300, 300);
                BSP_BXCAN_Command_t estop = make_command(BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_ESTOP, 0, 0);

                Safety_Manager_Init();
                install_valid_calibration();
                fake_fault = BSP_BXCAN_FAULT_COMMAND_TIMEOUT;
                Safety_Manager_Process(100U);
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetState() != SAFETY_STATE_SAFE_STOP) return 1;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 2;

                Safety_Manager_AcceptCommand(&safe_stop);
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 3;
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 4;

                fake_fault = BSP_BXCAN_FAULT_NONE;
                Safety_Manager_AcceptCommand(&stop);
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetAction() != SAFETY_ACTION_DRIVE) return 5;

                Safety_Manager_AcceptCommand(&estop);
                if (Safety_Manager_GetState() != SAFETY_STATE_ESTOP) return 6;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_BRAKE) return 7;
                if (set_fault_code != BSP_BXCAN_FAULT_ESTOP_ACTIVE || set_fault_latched != 1U) return 8;
                return 0;
            }
            """
        )

    def test_invalid_calibration_blocks_drive_until_valid_data_is_installed(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t velocity = make_command(BSP_BXCAN_MODE_VELOCITY, 300, 300);

                Safety_Manager_Init();
                Safety_Manager_Process(0U);
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetState() != SAFETY_STATE_CALIBRATION_REQUIRED) return 1;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 2;
                if (Safety_Manager_DriveAllowed() != 0U) return 3;
                if (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_NONE) return 4;

                install_valid_calibration();
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetState() != SAFETY_STATE_DRIVE) return 5;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_DRIVE) return 6;
                return 0;
            }
            """
        )

    def test_active_calibration_selects_calibration_safety_action(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t stop = make_command(BSP_BXCAN_MODE_STOP, 0, 0);
                Wheel_Calibration_ServiceRequest_t request = {0};

                request.service_seq = 7U;
                request.opcode = WHEEL_CALIBRATION_OPCODE_START;
                request.options = WHEEL_CALIBRATION_OPTION_MASK;
                request.service_cookie = WHEEL_CALIBRATION_SERVICE_COOKIE;

                Wheel_Calibration_Service_Init();
                Safety_Manager_Init();
                Safety_Manager_Process(0U);
                Wheel_Calibration_Service_OnRequest(&request, 0U, &stop, 1U, 0U, 0U);
                Safety_Manager_Process(1U);

                if (Safety_Manager_GetState() != SAFETY_STATE_CALIBRATION) return 1;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_CALIBRATION) return 2;
                if (Safety_Manager_DriveAllowed() != 0U) return 3;
                return 0;
            }
        """)

    def test_latched_fault_priority_and_reset_rearbitration(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t velocity = make_command(BSP_BXCAN_MODE_VELOCITY, 300, 300);
                BSP_BXCAN_Command_t reset_stop = make_command(
                    BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_RESET_FAULT, 0, 0);

                Safety_Manager_Init();
                install_valid_calibration();
                Safety_Manager_Process(0U);
                Safety_Manager_ReportFault(BSP_BXCAN_FAULT_MOTOR_STALL);
                Safety_Manager_Process(10U);
                if (Safety_Manager_GetState() != SAFETY_STATE_FAULT) return 1;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 2;
                if (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_MOTOR_STALL) return 3;

                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 4;
                Safety_Manager_AcceptCommand(&reset_stop);
                if (Safety_Manager_GetState() != SAFETY_STATE_STANDBY) return 5;
                Safety_Manager_AcceptCommand(&velocity);
                if (Safety_Manager_GetState() != SAFETY_STATE_DRIVE) return 6;
                return 0;
            }
        """)

    def test_control_overrun_is_latched_and_brakes(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                Safety_Manager_Init();
                install_valid_calibration();
                Safety_Manager_Process(0U);
                Safety_Manager_ReportControlTiming(101U);
                Safety_Manager_Process(10U);
                if (Safety_Manager_GetState() != SAFETY_STATE_FAULT) return 1;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_BRAKE) return 2;
                if (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_CONTROL_OVERRUN) return 3;
                return 0;
            }
        """)

    def test_encoder_invalid_samples_latch_after_threshold_and_recover(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t reset_stop = make_command(
                    BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_RESET_FAULT, 0, 0);
                uint32_t now;

                Safety_Manager_Init();
                install_valid_calibration();
                Safety_Manager_Process(0U);
                for (now = 0U; now < 100U; now += 20U) {
                    Safety_Manager_ReportEncoderSample(1U, 0U, 0, 20U);
                }
                Safety_Manager_Process(100U);
                if (Safety_Manager_GetState() != SAFETY_STATE_FAULT) return 1;
                if (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_REAR_ENCODER) return 2;

                for (now = 100U; now < 220U; now += 20U) {
                    Safety_Manager_ReportEncoderSample(1U, 1U, 0, 20U);
                }
                Safety_Manager_Process(220U);
                Safety_Manager_AcceptCommand(&reset_stop);
                if (Safety_Manager_GetState() != SAFETY_STATE_STANDBY) return 3;
                return 0;
            }
        """)

    def test_watchdog_reset_is_latched_until_explicit_reset(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t reset_stop = make_command(
                    BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_RESET_FAULT, 0, 0);

                Safety_Manager_Init();
                install_valid_calibration();
                Safety_Manager_NotifyWatchdogReset();
                Safety_Manager_Process(0U);
                if (Safety_Manager_GetState() != SAFETY_STATE_FAULT) return 1;
                if (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_MCU_WATCHDOG_RESET) return 2;
                Safety_Manager_AcceptCommand(&reset_stop);
                if (Safety_Manager_GetState() != SAFETY_STATE_STANDBY) return 3;
                return 0;
            }
        """)

    def test_sustained_opposite_encoder_direction_latches_encoder_fault(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                Safety_Manager_Init();
                install_valid_calibration();
                Safety_Manager_Process(0U);
                Safety_Manager_ReportDriveObservation(1U, 600, -60, 500, 0U);
                Safety_Manager_ReportDriveObservation(1U, 600, -60, 500, 100U);
                if (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_REAR_ENCODER) return 1;
                if (Safety_Manager_GetState() != SAFETY_STATE_FAULT) return 2;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 3;
                return 0;
            }
        """)

    def test_sustained_high_pwm_without_speed_latches_motor_stall(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                Safety_Manager_Init();
                install_valid_calibration();
                Safety_Manager_Process(0U);
                Safety_Manager_ReportDriveObservation(1U, 600, 0, 800, 0U);
                Safety_Manager_ReportDriveObservation(1U, 600, 0, 800, 1000U);
                Safety_Manager_ReportDriveObservation(1U, 600, 0, 800, 1500U);
                if (Safety_Manager_GetFault() != BSP_BXCAN_FAULT_MOTOR_STALL) return 1;
                if (Safety_Manager_GetState() != SAFETY_STATE_FAULT) return 2;
                if (Safety_Manager_GetAction() != SAFETY_ACTION_COAST) return 3;
                return 0;
            }
        """)


if __name__ == "__main__":
    unittest.main()
