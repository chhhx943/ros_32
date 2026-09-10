import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


COMMON_STUBS = textwrap.dedent(
    r"""
    #include <stdint.h>

    #include "BSP/bsp_bxcan.h"
    #include "BSP/bsp_motor.h"
    #include "BSP/encoder.h"
    #include "BSP/PID.h"

    static BSP_BXCAN_Command_t pending_command;
    static uint8_t has_pending_command;
    static uint8_t command_fresh;
    static uint16_t fake_fault = BSP_BXCAN_FAULT_NONE;
    uint8_t g_physical_estop_pin_level = 1U;
    static EncoderSample_t left_sample = {0, 0, 0, 1U};
    static EncoderSample_t right_sample = {0, 0, 0, 1U};
    static BSP_BXCAN_Feedback_t last_feedback;
    static BSP_BXCAN_Diagnostics_t last_diagnostics;
    static int feedback_set_count;
    static int coast_all_count;

    static BSP_BXCAN_Command_t make_velocity_command(int16_t left, int16_t right)
    {
        BSP_BXCAN_Command_t command;
        command.command_seq = 7U;
        command.mode_flags = BSP_BXCAN_MODE_VELOCITY;
        command.equivalent_steering_mrad = 0;
        command.rear_left_velocity_mmps = left;
        command.rear_right_velocity_mmps = right;
        command.accepted_time_ms = 100U;
        return command;
    }

    static void queue_command(BSP_BXCAN_Command_t command)
    {
        pending_command = command;
        has_pending_command = 1U;
    }

    void BSP_BXCAN_Init(void) {}
    void BSP_BXCAN_Process(uint32_t now_ms) { (void)now_ms; }
    uint8_t BSP_BXCAN_IsCommandFresh(void) { return command_fresh; }
    uint8_t BSP_BXCAN_GetCommand(BSP_BXCAN_Command_t *out_command)
    {
        if (has_pending_command == 0U) {
            return 0U;
        }
        *out_command = pending_command;
        has_pending_command = 0U;
        command_fresh = 1U;
        return 1U;
    }
    uint16_t BSP_BXCAN_GetFault(void) { return fake_fault; }
    void BSP_BXCAN_SetFault(uint16_t fault_code, uint8_t latched)
    {
        (void)latched;
        fake_fault = fault_code;
    }
    void BSP_BXCAN_SetFeedback(const BSP_BXCAN_Feedback_t *feedback)
    {
        last_feedback = *feedback;
        feedback_set_count++;
    }
    void BSP_BXCAN_SetDiagnostics(const BSP_BXCAN_Diagnostics_t *diagnostics)
    {
        last_diagnostics = *diagnostics;
    }
    void BSP_BXCAN_GetDiagnostics(BSP_BXCAN_Diagnostics_t *diagnostics)
    {
        *diagnostics = last_diagnostics;
    }

    void Motor_Init(void) {}
    void Motor_Drive(uint8_t num, int16_t pwm)
    {
        (void)num;
        (void)pwm;
    }
    void Motor_Coast(uint8_t num) { (void)num; }
    void Motor_Brake(uint8_t num) { (void)num; }
    void Motor_CoastAll(void) { coast_all_count++; }
    void Motor_EmergencyBrakeAll(void) {}
    void Motor_SetPWM(uint8_t num, int16_t pwm) { Motor_Drive(num, pwm); }

    void Encoder_Init(void) {}
    void Encoder_Reset(void) {}
    EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms)
    {
        (void)dt_ms;
        return (num == 1U) ? left_sample : right_sample;
    }
    float Encoder_Get(uint8_t num)
    {
        (void)num;
        return 0.0f;
    }

    void PID_Reset(PID_t *p) { p->Out = 0.0f; }
    void PID_UpdateDt(PID_t *p, float dt_s)
    {
        (void)dt_s;
        p->Out = (p->Target - p->Actual) * 0.1f;
    }
    void PID_Update(PID_t *p) { PID_UpdateDt(p, 1.0f); }

    #include "BSP/physical_estop.c"
    #include "BSP/wheel_calibration.c"
    #include "BSP/wheel_calibration_service.c"
    #include "BSP/safety_manager.c"
    #include "BSP/chassis_control.c"
    """
)


class ChassisFeedbackHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_chassis_feedback.c")
            exe = os.path.join(tmp, "test_chassis_feedback.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-Wno-unused-function",
                    "-DBSP_BXCAN_HOST_TEST",
                    "-DCHASSIS_CONTROL_HOST_TEST",
                    "-DENCODER_HOST_TEST",
                    "-DPHYSICAL_ESTOP_HOST_TEST",
                    "-DWHEEL_CALIBRATION_HOST_TEST",
                    "-DWHEEL_CALIBRATION_SERVICE_HOST_TEST",
                    "-DCALIBRATION_BENCH_DEFAULTS",
                    "-DSAFETY_MANAGER_HOST_TEST",
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

    def test_control_step_publishes_measured_velocity_and_position_feedback(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                Chassis_ControlInit();
                queue_command(make_velocity_command(1000, -1000));
                Chassis_ControlProcess(100U);

                left_sample.velocity_mmps = 123;
                left_sample.accumulated_counts = 56000;
                left_sample.trusted = 1U;
                right_sample.velocity_mmps = -456;
                right_sample.accumulated_counts = -28000;
                right_sample.trusted = 1U;
                Chassis_ControlProcess(110U);

                if (feedback_set_count != 1) return 1;
                if (last_feedback.velocity_flags !=
                    (BSP_BXCAN_VELOCITY_LEFT_VALID | BSP_BXCAN_VELOCITY_RIGHT_VALID)) return 2;
                if (last_feedback.rear_left_velocity_mmps != 123) return 3;
                if (last_feedback.rear_right_velocity_mmps != -456) return 4;
                if (last_feedback.rear_left_position_valid != 1U) return 5;
                if (last_feedback.rear_right_position_valid != 1U) return 6;
                if (last_feedback.rear_left_position_mrad < 6280 ||
                    last_feedback.rear_left_position_mrad > 6286) return 7;
                if (last_feedback.rear_right_position_mrad > -3140 ||
                    last_feedback.rear_right_position_mrad < -3146) return 8;
                return 0;
            }
            """
        )

    def test_safe_stop_still_samples_and_publishes_read_only_encoder_feedback(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                Chassis_ControlInit();
                left_sample.velocity_mmps = 0;
                left_sample.accumulated_counts = 56000;
                left_sample.trusted = 1U;
                right_sample.velocity_mmps = 0;
                right_sample.accumulated_counts = 28000;
                right_sample.trusted = 1U;

                /* No drive command: startup remains calibration-required /
                   safe-stop, but encoder feedback must still be observable. */
                Chassis_ControlProcess(0U);
                Chassis_ControlProcess(10U);

                if (feedback_set_count != 1) return 1;
                if (last_feedback.rear_left_position_valid != 1U) return 2;
                if (last_feedback.rear_right_position_valid != 1U) return 3;
                if (last_feedback.rear_left_position_mrad < 6280 ||
                    last_feedback.rear_left_position_mrad > 6286) return 4;
                if (last_feedback.rear_right_position_mrad < 3140 ||
                    last_feedback.rear_right_position_mrad > 3146) return 5;
                return 0;
            }
            """
        )

    def test_untrusted_sample_is_visible_in_feedback_flags_before_coast(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                Chassis_ControlInit();
                queue_command(make_velocity_command(800, 800));
                Chassis_ControlProcess(200U);

                left_sample.velocity_mmps = 75;
                left_sample.accumulated_counts = 1000;
                left_sample.trusted = 1U;
                right_sample.velocity_mmps = 0;
                right_sample.accumulated_counts = 0;
                right_sample.trusted = 0U;
                Chassis_ControlProcess(210U);

                if (feedback_set_count != 1) return 1;
                if (last_feedback.velocity_flags != BSP_BXCAN_VELOCITY_LEFT_VALID) return 2;
                if (last_feedback.rear_left_velocity_mmps != 75) return 3;
                if (last_feedback.rear_right_velocity_mmps != 0) return 4;
                if (last_feedback.rear_left_position_valid != 1U) return 5;
                if (last_feedback.rear_right_position_valid != 0U) return 6;
                if (coast_all_count == 0) return 7;
                return 0;
            }
            """
        )

    def test_physical_estop_is_visible_in_feedback_status(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                Chassis_ControlInit();
                queue_command(make_velocity_command(500, 500));
                Chassis_ControlProcess(300U);

                left_sample.trusted = 1U;
                right_sample.trusted = 1U;
                Chassis_ControlProcess(310U);

                g_physical_estop_pin_level = 0U;
                Chassis_ControlProcess(320U);

                if (feedback_set_count < 2) return 1;
                if ((last_feedback.status_flags & BSP_BXCAN_STATUS_ESTOP_ACTIVE) == 0U) return 2;
                return 0;
            }
            """
        )

    def test_feedback_publishes_safety_and_encoder_diagnostics(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                Chassis_ControlInit();
                queue_command(make_velocity_command(500, 500));
                Chassis_ControlProcess(400U);

                left_sample.trusted = 1U;
                right_sample.trusted = 0U;
                Chassis_ControlProcess(410U);

                if (last_diagnostics.safety_state != SAFETY_STATE_DRIVE) return 1;
                if (last_diagnostics.safety_action != SAFETY_ACTION_DRIVE) return 2;
                if ((last_diagnostics.flags & BSP_BXCAN_DIAG_CALIBRATION_REQUIRED) != 0U) return 3;
                if ((last_diagnostics.flags & BSP_BXCAN_DIAG_COMMAND_FRESH) == 0U) return 4;
                if ((last_diagnostics.flags & BSP_BXCAN_DIAG_LEFT_ENCODER_INVALID) != 0U) return 5;
                if ((last_diagnostics.flags & BSP_BXCAN_DIAG_RIGHT_ENCODER_INVALID) == 0U) return 6;
                if ((last_diagnostics.flags & BSP_BXCAN_DIAG_DRIVE_ALLOWED) == 0U) return 7;
                return 0;
            }
            """
        )

    def test_feedback_publishes_ready_state_before_first_control_sample(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void)
            {
                Chassis_ControlInit();
                Chassis_ControlProcess(100U);

                if (last_diagnostics.safety_state != SAFETY_STATE_STANDBY) return 1;
                if (last_diagnostics.safety_action != SAFETY_ACTION_COAST) return 2;
                if ((last_diagnostics.flags & BSP_BXCAN_DIAG_DRIVE_ALLOWED) != 0U) return 3;
                if ((last_diagnostics.flags & BSP_BXCAN_DIAG_COMMAND_FRESH) != 0U) return 4;
                return 0;
            }
            """
        )


if __name__ == "__main__":
    unittest.main()
