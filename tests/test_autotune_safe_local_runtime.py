import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class AutotuneSafeLocalRuntimeTest(unittest.TestCase):
    def test_session_contract_latches_terminal_results_and_acknowledges_once(self):
        source = textwrap.dedent(r"""
            #include <stdint.h>
            #include <string.h>
            #include "BSP/autotune_safe_session.h"
            #include "BSP/autotune_safe_session.c"
            #include "BSP/autotune_safe_local_smoke.c"

            static uint32_t request_id = 100U;

            static void send_request(uint32_t type, uint32_t session,
                                     uint32_t experiment, uint32_t hint) {
                AutotuneSession_Request_t request = {0};
                request.version = AUTOTUNE_SESSION_VERSION;
                request.size = sizeof(request);
                request.request_id = ++request_id;
                request.request_type = type;
                request.session_id = session;
                request.experiment_id = experiment;
                request.payload[0] = hint;
                request.boot_generation_hint = 1U;
                request.crc32 = AutotuneSession_Crc32(
                    ((const uint8_t *)&request) + 4U, 40U);
                g_autotune_session_request = request;
                g_autotune_session_request.magic = AUTOTUNE_SESSION_REQUEST_MAGIC;
            }

            int main(void) {
                AutotuneSession_Init(0x12345678U,
                    AUTOTUNE_SESSION_PROFILE_LOCAL_SMOKE,
                    AUTOTUNE_SESSION_BOOT_REASON_POWER_ON);
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_READY) return 1;

                send_request(AUTOTUNE_SESSION_REQUEST_CREATE_SESSION, 11U, 0U, 0U);
                AutotuneSession_Process(10U);
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_ARMED) return 2;
                uint32_t before = g_autotune_session_status.sample_generation;
                send_request(AUTOTUNE_SESSION_REQUEST_START_EXPERIMENT,
                             11U, 22U, before);
                AutotuneSession_LocalSmokeProcess(20U);
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_RUNNING) return 3;
                if (g_autotune_session_status.start_sample_generation != before) return 4;
                for (uint32_t now = 30U; now < 340U; now += 10U) {
                    AutotuneSession_LocalSmokeProcess(now);
                }
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED) return 5;
                if (g_autotune_session_result.magic !=
                    AUTOTUNE_SESSION_RESULT_FINAL_MAGIC) return 6;
                if (g_autotune_session_result.sample_count != 32U) return 7;
                AutotuneSession_LocalSmokeProcess(1000U);
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED) return 8;
                send_request(AUTOTUNE_SESSION_REQUEST_ACK_RESULT, 11U, 22U, 0U);
                AutotuneSession_LocalSmokeProcess(1010U);
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_READY) return 9;
                if (g_autotune_session_result.magic != 0U) return 10;

                send_request(AUTOTUNE_SESSION_REQUEST_CREATE_SESSION, 12U, 0U, 0U);
                AutotuneSession_LocalSmokeProcess(1020U);
                send_request(AUTOTUNE_SESSION_REQUEST_START_EXPERIMENT,
                             12U, 23U, g_autotune_session_status.sample_generation);
                AutotuneSession_LocalSmokeProcess(1030U);
                send_request(AUTOTUNE_SESSION_REQUEST_STOP, 12U, 23U, 0U);
                AutotuneSession_LocalSmokeProcess(1040U);
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_ABORT_LATCHED) return 11;
                AutotuneSession_LocalSmokeProcess(1050U);
                if (g_autotune_session_status.state !=
                    AUTOTUNE_SESSION_STATE_ABORT_LATCHED) return 12;
                return 0;
            }
        """)
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "session_runtime.c")
            exe = os.path.join(tmp, "session_runtime.exe")
            with open(src, "w", encoding="utf-8") as handle:
                handle.write(source)
            subprocess.run([
                "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", ROOT, src, "-o", exe,
            ], check=True, cwd=ROOT)
            result = subprocess.run([exe], cwd=ROOT)
            self.assertEqual(result.returncode, 0)

    def test_request_gated_local_executor_and_single_wheel_sequence(self):
        source = textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/safety_manager.h"
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            #include "BSP/autotune_safe_local.h"

            static uint8_t safety_state = SAFETY_STATE_STANDBY;
            static uint16_t fault_code;
            static uint8_t estop;
            static uint32_t drive_count;
            static int16_t last_left_pwm;
            static int16_t last_right_pwm;

            #include "BSP/autotune_safe_local.c"

            Safety_State_t Safety_Manager_GetState(void) { return (Safety_State_t)safety_state; }
            Safety_Action_t Safety_Manager_GetAction(void) {
                return (safety_state == SAFETY_STATE_DRIVE) ? SAFETY_ACTION_DRIVE : SAFETY_ACTION_COAST;
            }
            uint16_t Safety_Manager_GetFault(void) { return fault_code; }
            uint8_t Safety_Manager_IsEstopActive(void) { return estop; }
            void Safety_Manager_AcceptCommand(const BSP_BXCAN_Command_t *command) {
                safety_state = (command->mode_flags & BSP_BXCAN_MODE_MASK) == BSP_BXCAN_MODE_STOP
                                   ? SAFETY_STATE_STANDBY : SAFETY_STATE_DRIVE;
            }
            void Safety_Manager_ReportDriveObservation(uint8_t wheel, int16_t target,
                                                       int16_t actual, int16_t pwm, uint32_t now) {
                (void)wheel; (void)target; (void)actual; (void)pwm; (void)now;
            }
            void BSP_BXCAN_SetLocalVelocityCommand(int16_t left, int16_t right, uint32_t now) {
                (void)left; (void)right; (void)now;
            }
            EncoderSample_t Encoder_Sample(uint8_t wheel, uint32_t dt) {
                EncoderSample_t sample = {0, 0, 0, 1};
                (void)wheel; (void)dt;
                return sample;
            }
            void Encoder_GetSpeedEvidence(const EncoderSample_t *sample, uint32_t dt,
                                          EncoderSpeedEvidence_t *evidence) {
                evidence->raw_delta_counts = sample->delta_counts;
                evidence->dt_ms = dt;
                evidence->mcu_speed_mmps = sample->velocity_mmps;
                evidence->recomputed_speed_mmps = sample->velocity_mmps;
                evidence->counts_per_wheel_rev = 56000;
                evidence->circumference_mm_x1000 = 208900;
            }
            void Motor_CoastAll(void) { last_left_pwm = 0; last_right_pwm = 0; }
            void Motor_Coast(uint8_t wheel) { if (wheel == 1) last_left_pwm = 0; else last_right_pwm = 0; }
            void Motor_Drive(uint8_t wheel, int16_t pwm) {
                drive_count++;
                if (wheel == 1) last_left_pwm = pwm; else last_right_pwm = pwm;
            }
            void PID_Reset(PID_t *pid) { pid->Out = 0; pid->Error0 = 0; pid->Error1 = 0; pid->ErrorInt = 0; }
            void PID_UpdateDt(PID_t *pid, float dt) { pid->Error1 = pid->Error0; pid->Error0 = pid->Target - pid->Actual; pid->Out = pid->Kp * pid->Error0; (void)dt; }

            int main(void) {
                AutotuneSafe_Init();
                AutotuneSafe_LocalExperiment_Init();
                g_autotune_safe_local_boot_request.magic =
                    AUTOTUNE_SAFE_LOCAL_BOOT_MAGIC;
                g_autotune_safe_local_boot_request.command =
                    AUTOTUNE_SAFE_LOCAL_COMMAND_START;
                g_autotune_safe_local_boot_request.sequence = 1234U;
                g_autotune_safe_local_boot_request.wheel = 1U;
                AutotuneSafe_LocalExperiment_Init();
                if (g_autotune_safe_local_control.request_magic !=
                    AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC) return 8;
                if (g_autotune_safe_local_control.request_command !=
                    AUTOTUNE_SAFE_LOCAL_COMMAND_START) return 9;
                if (g_autotune_safe_local_control.request_sequence != 1234U) return 10;
                if (g_autotune_safe_local_boot_request.magic != 0U) return 11;
                g_autotune_safe_local_control.request_magic = 0U;
                g_autotune_safe_local_control.request_command =
                    AUTOTUNE_SAFE_LOCAL_COMMAND_NONE;
                g_autotune_safe_local_control.request_magic = AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC;
                g_autotune_safe_local_control.request_command = AUTOTUNE_SAFE_LOCAL_COMMAND_START;
                g_autotune_safe_local_control.request_wheel = 1;
                AutotuneSafe_LocalExperiment_Process(0);
                if (g_autotune_safe_local_control.request_command !=
                    AUTOTUNE_SAFE_LOCAL_COMMAND_START) return 6;
                for (uint32_t now = 10; now < 70; now += 10) {
                    AutotuneSafe_LocalExperiment_Process(now);
                }
                if (g_autotune_safe_local_control.active_wheel != 1) return 7;

                AutotuneSafe_Init();
                AutotuneSafe_LocalExperiment_Init();
                safety_state = SAFETY_STATE_STANDBY;
                drive_count = 0;
                g_autotune_safe_local_control.request_command = AUTOTUNE_SAFE_LOCAL_COMMAND_STOP;
                AutotuneSafe_LocalExperiment_Process(0);
                for (uint32_t now = 0; now < 100; now += 10) {
                    AutotuneSafe_LocalExperiment_Process(now);
                }
                if (drive_count != 0) return 1;
                g_autotune_safe_local_control.request_magic = AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC;
                g_autotune_safe_local_control.request_command = AUTOTUNE_SAFE_LOCAL_COMMAND_START;
                g_autotune_safe_local_control.request_wheel = 1;
                for (uint32_t now = 100; now < 220; now += 10) {
                    AutotuneSafe_LocalExperiment_Process(now);
                }
                if (g_autotune_safe_local_control.active_wheel != 1) return 2;
                if (g_autotune_safe_local_control.state == AUTOTUNE_SAFE_LOCAL_IDLE) return 3;
                if (last_right_pwm != 0) return 4;
                if (g_autotune_safe_local_control.candidate_id != 0) return 5;
                if (last_left_pwm != 40) return 6;
                return 0;
            }
        """)
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "local_runtime.c")
            exe = os.path.join(tmp, "local_runtime.exe")
            with open(src, "w", encoding="utf-8") as handle:
                handle.write(source)
            subprocess.run([
                "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                "-Wno-unused-function", "-DAUTOTUNE_SAFE_PROFILE",
                "-DBSP_BXCAN_HOST_TEST", "-DENCODER_HOST_TEST",
                "-DSAFETY_MANAGER_HOST_TEST", "-I", ROOT, src, "-o", exe,
            ], check=True, cwd=ROOT)
            result = subprocess.run([exe], cwd=ROOT)
            self.assertEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
