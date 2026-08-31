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
    #include "BSP/bsp_motor.h"
    #include "BSP/encoder.h"
    #include "BSP/PID.h"

    enum { MOTOR_NONE = 0, MOTOR_DRIVE = 1, MOTOR_COAST = 2, MOTOR_BRAKE = 3 };
    static int motor_mode[3];
    static int16_t motor_pwm[3];
    static EncoderSample_t left_sample = {0, 0, 0, 1U};
    static EncoderSample_t right_sample = {0, 0, 0, 1U};
    static uint32_t last_encoder_dt_ms;
    uint8_t g_physical_estop_pin_level = 1U;

    void Motor_Init(void) {}
    void Motor_Drive(uint8_t num, int16_t pwm) { motor_mode[num] = MOTOR_DRIVE; motor_pwm[num] = pwm; }
    void Motor_Coast(uint8_t num) { motor_mode[num] = MOTOR_COAST; motor_pwm[num] = 0; }
    void Motor_Brake(uint8_t num) { motor_mode[num] = MOTOR_BRAKE; motor_pwm[num] = 0; }
    void Motor_CoastAll(void) { Motor_Coast(1); Motor_Coast(2); }
    void Motor_EmergencyBrakeAll(void) { Motor_Brake(1); Motor_Brake(2); }
    void Motor_SetPWM(uint8_t num, int16_t pwm) { Motor_Drive(num, pwm); }

    void Encoder_Init(void) {}
    void Encoder_Reset(void) {}
    EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms)
    {
        last_encoder_dt_ms = dt_ms;
        return (num == 1U) ? left_sample : right_sample;
    }
    float Encoder_Get(uint8_t num)
    {
        (void)num;
        return 0.0f;
    }

    void PID_Reset(PID_t *p)
    {
        p->Out = 0.0f;
        p->Error0 = 0.0f;
        p->Error1 = 0.0f;
        p->ErrorInt = 0.0f;
    }
    void PID_UpdateDt(PID_t *p, float dt_s)
    {
        (void)dt_s;
        p->Error0 = p->Target - p->Actual;
        p->Out = p->Error0 * 0.2f;
    }
    void PID_Update(PID_t *p) { PID_UpdateDt(p, 1.0f); }

    #include "BSP/bsp_bxcan.c"
    #include "BSP/physical_estop.c"
    #include "BSP/wheel_calibration.c"
    #include "BSP/wheel_calibration_service.c"
    #include "BSP/safety_manager.c"
    #include "BSP/chassis_control.c"
    """
)


class ChassisControlIntegrationTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_chassis_control.c")
            exe = os.path.join(tmp, "test_chassis_control.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DBSP_BXCAN_HOST_TEST",
                    "-DBSP_BXCAN_ENABLE_TEST_HOOKS",
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

    def test_velocity_command_drives_left_and_right_wheels_on_control_step(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void) {
                const uint8_t steering[8] = {0x01, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00, 0x00};
                const uint8_t wheels[8] = {0x01, 0x34, 0x12, 0x01, 0xDC, 0x05, 0x48, 0xF4};

                Chassis_ControlInit();
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                Chassis_ControlProcess(104);

                if (motor_mode[1] == MOTOR_DRIVE || motor_mode[2] == MOTOR_DRIVE) abort();

                Chassis_ControlProcess(114);

                if (last_encoder_dt_ms != 10U) abort();
                if (motor_mode[1] != MOTOR_DRIVE || motor_pwm[1] != 300) abort();
                if (motor_mode[2] != MOTOR_DRIVE || motor_pwm[2] != -600) abort();
                return 0;
            }
            """
        )

    def test_stop_command_coasts_both_wheels(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void) {
                const uint8_t steering[8] = {0x01, 0x35, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00};
                const uint8_t wheels[8] = {0x01, 0x35, 0x12, 0x00, 0xDC, 0x05, 0xDC, 0x05};

                Chassis_ControlInit();
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                Chassis_ControlProcess(104);

                if (motor_mode[1] != MOTOR_COAST || motor_pwm[1] != 0) abort();
                if (motor_mode[2] != MOTOR_COAST || motor_pwm[2] != 0) abort();
                return 0;
            }
            """
        )

    def test_estop_frame_brakes_immediately(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void) {
                const uint8_t estop[8] = {0x01, 0x36, 0x12, 0x04, 0x00, 0x00, 0x00, 0x00};

                Chassis_ControlInit();
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, estop, 100);
                Chassis_ControlProcess(100);

                if (motor_mode[1] != MOTOR_BRAKE || motor_pwm[1] != 0) abort();
                if (motor_mode[2] != MOTOR_BRAKE || motor_pwm[2] != 0) abort();
                return 0;
            }
            """
        )

    def test_command_timeout_coasts_both_wheels(self):
        self.compile_and_run(
            COMMON_STUBS
            + r"""
            int main(void) {
                const uint8_t steering[8] = {0x01, 0x37, 0x12, 0x01, 0x00, 0x00, 0x00, 0x00};
                const uint8_t wheels[8] = {0x01, 0x37, 0x12, 0x01, 0xDC, 0x05, 0xDC, 0x05};

                Chassis_ControlInit();
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                Chassis_ControlProcess(104);
                Chassis_ControlProcess(114);
                Chassis_ControlProcess(204);

                if (motor_mode[1] != MOTOR_COAST || motor_pwm[1] != 0) abort();
                if (motor_mode[2] != MOTOR_COAST || motor_pwm[2] != 0) abort();
                return 0;
            }
            """
        )

    def test_calibration_recommendation_drives_only_current_wheel(self):
        self.compile_and_run(COMMON_STUBS + r"""
            int main(void) {
                const uint8_t steering[8] = {0x01, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                const uint8_t wheels[8] = {0x01, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                Wheel_Calibration_ServiceRequest_t request = {0};
                BSP_BXCAN_Command_t stop = {0};
                stop.mode_flags = BSP_BXCAN_MODE_STOP;

                request.service_seq = 9U;
                request.opcode = WHEEL_CALIBRATION_OPCODE_START;
                request.options = WHEEL_CALIBRATION_OPTION_MASK;
                request.service_cookie = WHEEL_CALIBRATION_SERVICE_COOKIE;

                Chassis_ControlInit();
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 0);
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 0);
                Chassis_ControlProcess(0U);
                Wheel_Calibration_Service_OnRequest(&request, 0U, &stop, 1U, 0U, 0U);

                Chassis_ControlProcess(10U);
                for (uint32_t now = 20U; now <= 300U; now += 10U) {
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, now);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, now);
                    Chassis_ControlProcess(now);
                }
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 300);
                BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 300);
                Chassis_ControlProcess(310U);

                if (Wheel_Calibration_Service_GetStage() != WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) return 1;
                if (motor_mode[1] != MOTOR_DRIVE || motor_pwm[1] <= 0) return 2;
                if (motor_mode[2] != MOTOR_COAST || motor_pwm[2] != 0) return 3;
                return 0;
            }
        """)


if __name__ == "__main__":
    unittest.main()
