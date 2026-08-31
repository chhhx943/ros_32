import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class ChassisClosedLoopHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_chassis_closed_loop.c")
            exe = os.path.join(tmp, "test_chassis_closed_loop.exe")
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

    def test_init_starts_encoder_and_resets_wheel_controllers(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                Chassis_ControlInit();

                if (can_init_count != 1) return 1;
                if (motor_init_count != 1) return 2;
                if (encoder_init_count != 1) return 3;
                if (pid_reset_count != 2) return 4;
                if (coast_all_count != 1) return 5;
                return 0;
            }
        """)

    def test_velocity_command_waits_for_10ms_control_step(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t command = make_velocity_command(1000, 500);

                Chassis_ControlInit();
                queue_command(command);
                Chassis_ControlProcess(104U);
                if (drive_count != 0) return 1;

                left_sample.velocity_mmps = 100;
                right_sample.velocity_mmps = 50;
                Chassis_ControlProcess(114U);

                if (drive_count != 2) return 2;
                if (motor_pwm[1] <= 0 || motor_pwm[2] <= 0) return 3;
                if (last_encoder_dt_ms != 10U) return 4;
                return 0;
            }
        """)

    def test_stop_timeout_and_estop_clear_controller_outputs(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t velocity = make_velocity_command(1000, 1000);
                BSP_BXCAN_Command_t stop = make_stop_command();
                BSP_BXCAN_Command_t estop = make_stop_command();

                Chassis_ControlInit();
                queue_command(velocity);
                Chassis_ControlProcess(10U);
                Chassis_ControlProcess(20U);
                if (drive_count != 2) return 1;

                pid_reset_count = 0;
                queue_command(stop);
                Chassis_ControlProcess(21U);
                if (coast_all_count < 2) return 2;
                if (pid_reset_count != 2) return 3;

                pid_reset_count = 0;
                fake_fault = BSP_BXCAN_FAULT_COMMAND_TIMEOUT;
                Chassis_ControlProcess(119U);
                if (coast_all_count < 3) return 4;
                if (pid_reset_count != 2) return 5;

                pid_reset_count = 0;
                fake_fault = BSP_BXCAN_FAULT_NONE;
                estop.mode_flags = BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_ESTOP;
                queue_command(estop);
                Chassis_ControlProcess(120U);
                if (brake_all_count != 1) return 6;
                if (pid_reset_count != 2) return 7;
                return 0;
            }
        """)

    def test_missed_control_ticks_run_once_with_real_elapsed_dt(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                Chassis_ControlInit();
                queue_command(make_velocity_command(1200, 1200));
                Chassis_ControlProcess(100U);

                Chassis_ControlProcess(130U);

                if (encoder_sample_count != 2) return 1;
                if (pid_update_count != 2) return 2;
                if (last_encoder_dt_ms != 30U) return 3;
                if (last_pid_dt_s < 0.029f || last_pid_dt_s > 0.031f) return 4;
                return 0;
            }
        """)

    def test_untrusted_encoder_sample_coasts_and_resets_controllers(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                Chassis_ControlInit();
                queue_command(make_velocity_command(1200, 1200));
                Chassis_ControlProcess(100U);

                right_sample.trusted = 0U;
                pid_reset_count = 0;
                Chassis_ControlProcess(110U);

                if (coast_all_count < 2) return 1;
                if (pid_reset_count != 2) return 2;
                if (drive_count != 0) return 3;
                return 0;
            }
        """)

    def test_physical_estop_brakes_and_requires_reset_stop_before_drive(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void)
            {
                Chassis_ControlInit();
                queue_command(make_velocity_command(1200, 1200));
                Chassis_ControlProcess(100U);

                g_physical_estop_pin_level = 0U;
                Chassis_ControlProcess(101U);
                if (brake_all_count != 1) return 1;

                g_physical_estop_pin_level = 1U;
                Chassis_ControlProcess(102U);
                queue_command(make_velocity_command(1200, 1200));
                Chassis_ControlProcess(103U);
                if (drive_count != 0) return 2;

                {
                    BSP_BXCAN_Command_t reset_stop = make_stop_command();
                    reset_stop.mode_flags |= BSP_BXCAN_FLAG_RESET_FAULT;
                    queue_command(reset_stop);
                }
                Chassis_ControlProcess(152U);
                Chassis_ControlProcess(153U);
                queue_command(make_velocity_command(1200, 1200));
                Chassis_ControlProcess(154U);
                Chassis_ControlProcess(164U);
                if (drive_count != 2) return 3;
                return 0;
            }
            """)


COMMON_STUBS = textwrap.dedent(
    r"""
    #include <stdint.h>

    #include "BSP/bsp_bxcan.h"
    #include "BSP/bsp_motor.h"
    #include "BSP/encoder.h"
    #include "BSP/PID.h"

    static int can_init_count;
    static int motor_init_count;
    static int encoder_init_count;
    static int pid_reset_count;
    static int pid_update_count;
    static int coast_all_count;
    static int brake_all_count;
    static int drive_count;
    static int16_t motor_pwm[3];
    static uint32_t last_encoder_dt_ms;
    static float last_pid_dt_s;
    static uint16_t fake_fault = BSP_BXCAN_FAULT_NONE;
    uint8_t g_physical_estop_pin_level = 1U;
    static BSP_BXCAN_Command_t pending_command;
    static uint8_t has_pending_command;
    static EncoderSample_t left_sample = {0, 0, 0, 1U};
    static EncoderSample_t right_sample = {0, 0, 0, 1U};
    static int encoder_sample_count;
    static BSP_BXCAN_Diagnostics_t diagnostics;

    static BSP_BXCAN_Command_t make_velocity_command(int16_t left, int16_t right)
    {
        BSP_BXCAN_Command_t command;
        command.command_seq = 1U;
        command.mode_flags = BSP_BXCAN_MODE_VELOCITY;
        command.equivalent_steering_mrad = 0;
        command.rear_left_velocity_mmps = left;
        command.rear_right_velocity_mmps = right;
        command.accepted_time_ms = 0U;
        return command;
    }

    static BSP_BXCAN_Command_t make_stop_command(void)
    {
        BSP_BXCAN_Command_t command = make_velocity_command(0, 0);
        command.mode_flags = BSP_BXCAN_MODE_STOP;
        return command;
    }

    static void queue_command(BSP_BXCAN_Command_t command)
    {
        pending_command = command;
        has_pending_command = 1U;
    }

    void BSP_BXCAN_Init(void) { can_init_count++; }
    void BSP_BXCAN_Process(uint32_t now_ms) { (void)now_ms; }
    uint8_t BSP_BXCAN_IsCommandFresh(void) { return 1U; }
    uint8_t BSP_BXCAN_GetCommand(BSP_BXCAN_Command_t *out_command)
    {
        if (has_pending_command == 0U) {
            return 0U;
        }
        *out_command = pending_command;
        has_pending_command = 0U;
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
        (void)feedback;
    }
    void BSP_BXCAN_SetDiagnostics(const BSP_BXCAN_Diagnostics_t *value)
    {
        diagnostics = *value;
    }
    void BSP_BXCAN_GetDiagnostics(BSP_BXCAN_Diagnostics_t *value)
    {
        *value = diagnostics;
    }

    void Motor_Init(void) { motor_init_count++; }
    void Motor_Drive(uint8_t num, int16_t pwm)
    {
        drive_count++;
        motor_pwm[num] = pwm;
    }
    void Motor_Coast(uint8_t num) { (void)num; }
    void Motor_Brake(uint8_t num) { (void)num; }
    void Motor_CoastAll(void) { coast_all_count++; }
    void Motor_EmergencyBrakeAll(void) { brake_all_count++; }
    void Motor_SetPWM(uint8_t num, int16_t pwm) { Motor_Drive(num, pwm); }

    void Encoder_Init(void) { encoder_init_count++; }
    void Encoder_Reset(void) {}
    EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms)
    {
        last_encoder_dt_ms = dt_ms;
        encoder_sample_count++;
        return (num == 1U) ? left_sample : right_sample;
    }
    float Encoder_Get(uint8_t num)
    {
        (void)num;
        return 0.0f;
    }

    void PID_Reset(PID_t *p)
    {
        pid_reset_count++;
        p->Out = 0.0f;
        p->Error0 = 0.0f;
        p->Error1 = 0.0f;
        p->ErrorInt = 0.0f;
    }
    void PID_UpdateDt(PID_t *p, float dt_s)
    {
        pid_update_count++;
        last_pid_dt_s = dt_s;
        p->Error0 = p->Target - p->Actual;
        p->Out = p->Error0 * 0.2f;
    }
    void PID_Update(PID_t *p) { PID_UpdateDt(p, 1.0f); }

    #include "BSP/physical_estop.c"
    #include "BSP/wheel_calibration.c"
    #include "BSP/wheel_calibration_service.c"
    #include "BSP/safety_manager.c"
    #include "BSP/chassis_control.c"
    """
)


if __name__ == "__main__":
    unittest.main()
