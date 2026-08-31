import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class BspBxcanProtocolTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_bsp_bxcan.c")
            exe = os.path.join(tmp, "test_bsp_bxcan.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DWHEEL_CALIBRATION_SERVICE_HOST_TEST",
                    "-DWHEEL_CALIBRATION_HOST_TEST",
                    "-DENCODER_HOST_TEST",
                    "-DBSP_BXCAN_HOST_TEST",
                    "-DBSP_BXCAN_ENABLE_TEST_HOOKS",
                    "-I",
                    ROOT,
                    "-I",
                    os.path.join(ROOT, "BSP"),
                    src,
                    os.path.join(ROOT, "BSP", "wheel_calibration.c"),
                    os.path.join(ROOT, "BSP", "wheel_calibration_service.c"),
                    "-o",
                    exe,
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([exe], check=True, cwd=ROOT)

    def test_command_group_decodes_only_matching_pair(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                static void expect_u16(uint16_t got, uint16_t want) { if (got != want) abort(); }
                static void expect_i16(int16_t got, int16_t want) { if (got != want) abort(); }
                static void expect_u8(uint8_t got, uint8_t want) { if (got != want) abort(); }

                int main(void) {
                    const uint8_t steering[8] = {0x01, 0x34, 0x12, 0x01, 0xFA, 0x00, 0x00, 0x00};
                    const uint8_t wheels[8] = {0x01, 0x34, 0x12, 0x01, 0x84, 0x03, 0x4C, 0x04};
                    BSP_BXCAN_Command_t command;

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                    if (BSP_BXCAN_GetCommand(&command)) abort();

                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                    if (!BSP_BXCAN_GetCommand(&command)) abort();

                    expect_u16(command.command_seq, 0x1234);
                    expect_u8(command.mode_flags, BSP_BXCAN_MODE_VELOCITY);
                    expect_i16(command.equivalent_steering_mrad, 250);
                    expect_i16(command.rear_left_velocity_mmps, 900);
                    expect_i16(command.rear_right_velocity_mmps, 1100);
                    return 0;
                }
                """
            )
        )

    def test_calibration_command_is_routed_and_feedback_is_encoded(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/wheel_calibration_service.h"
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    const uint8_t steering[8] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                    const uint8_t wheels[8] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                    const uint8_t calibration[8] = {0x01, 0x34, 0x12, 0x01, 0x03, 0x5A, 0xC3, 0x00};
                    uint8_t feedback[8] = {0};

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_CALIBRATION, 8, 0, 0, calibration, 110);

                    if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_PRECHECK) return 1;
                    if (Wheel_Calibration_Service_GetStage() != WHEEL_CALIBRATION_STAGE_PRECHECK) return 2;
                    BSP_BXCAN_TestBuildCalibrationFrame(feedback);
                    if (feedback[0] != 0x01 || feedback[1] != 0x34 || feedback[2] != 0x12 ||
                        feedback[3] != WHEEL_CALIBRATION_TX_PRECHECK ||
                        feedback[4] != WHEEL_CALIBRATION_STAGE_PRECHECK ||
                        feedback[5] != WHEEL_CALIBRATION_EXIT_NONE || feedback[6] != 0x04 ||
                        feedback[7] != 0x00) return 3;
                    return 0;
                }
                """
            )
        )

    def test_calibration_bad_cookie_returns_bad_format_feedback(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/wheel_calibration_service.h"
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    const uint8_t steering[8] = {0x01, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                    const uint8_t wheels[8] = {0x01, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                    const uint8_t calibration[8] = {0x01, 0x35, 0x12, 0x01, 0x03, 0x00, 0x00, 0x00};
                    uint8_t feedback[8] = {0};

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_CALIBRATION, 8, 0, 0, calibration, 110);

                    if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_REJECTED) return 1;
                    if (Wheel_Calibration_Service_GetExitReason() != WHEEL_CALIBRATION_EXIT_REJECTED_BAD_FORMAT) return 2;
                    BSP_BXCAN_TestBuildCalibrationFrame(feedback);
                    if (feedback[1] != 0x35 || feedback[2] != 0x12 ||
                        feedback[3] != WHEEL_CALIBRATION_TX_REJECTED || feedback[5] != 0x13 ||
                        feedback[6] != 0x05) return 3;
                    return 0;
                }
                """
            )
        )

    def test_single_frame_estop_does_not_update_applied_seq(self):
        """Protocol §6: a single-frame E-stop MUST NOT update applied_command_seq
        because no complete command group was committed. The 2026-08-30 CAN motor
        bench run caught the firmware violating this rule."""
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                static void expect_u16(uint16_t got, uint16_t want) { if (got != want) abort(); }

                int main(void) {
                    const uint8_t steering[8] = {0x01, 0x34, 0x12, 0x01, 0xFA, 0x00, 0x00, 0x00};
                    const uint8_t wheels[8] = {0x01, 0x34, 0x12, 0x01, 0x84, 0x03, 0x4C, 0x04};
                    const uint8_t estop[8] = {0x01, 0x77, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00};
                    BSP_BXCAN_Command_t command;

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                    if (!BSP_BXCAN_GetCommand(&command)) abort();
                    expect_u16(BSP_BXCAN_GetAppliedCommandSeq(), 0x1234);

                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, estop, 200);
                    expect_u16(BSP_BXCAN_GetFault(), BSP_BXCAN_FAULT_ESTOP_ACTIVE);
                    if (!BSP_BXCAN_GetCommand(&command)) abort();
                    expect_u16((uint16_t)(command.mode_flags & BSP_BXCAN_FLAG_ESTOP),
                               BSP_BXCAN_FLAG_ESTOP);
                    expect_u16(BSP_BXCAN_GetAppliedCommandSeq(), 0x1234);
                    return 0;
                }
                """
            )
        )

    def test_feedback_frames_match_protocol_vectors(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                static void expect_bytes(const uint8_t got[8], const uint8_t want[8]) {
                    for (int i = 0; i < 8; ++i) {
                        if (got[i] != want[i]) abort();
                    }
                }

                int main(void) {
                    uint8_t data[8];
                    BSP_BXCAN_Feedback_t feedback = {
                        0x0F, 900, 1100, 3142, 2718, 0x03, 1, 1, 0, 0
                    };

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_SetFault(BSP_BXCAN_FAULT_NONE, 0);
                    BSP_BXCAN_TestBuildFrame(BSP_BXCAN_ID_FB_STATUS, 0x0102, 0x7A, 0x1234, &feedback, 1000, data);
                    expect_bytes(data, (uint8_t[8]){0x01, 0x02, 0x01, 0x7A, 0x34, 0x12, 0x00, 0x00});

                    BSP_BXCAN_TestBuildFrame(BSP_BXCAN_ID_FB_HEALTH, 0x0102, 0x7A, 0x1234, &feedback, 1000, data);
                    expect_bytes(data, (uint8_t[8]){0x01, 0x02, 0x01, 0x0F, 0xE8, 0x03, 0x00, 0x00});

                    BSP_BXCAN_TestBuildFrame(BSP_BXCAN_ID_FB_REAR_VELOCITY, 0x0102, 0x7A, 0x1234, &feedback, 1000, data);
                    expect_bytes(data, (uint8_t[8]){0x01, 0x02, 0x01, 0x03, 0x84, 0x03, 0x4C, 0x04});

                    BSP_BXCAN_TestBuildFrame(BSP_BXCAN_ID_FB_REAR_LEFT_POSITION, 0x0102, 0x7A, 0x1234, &feedback, 1000, data);
                    expect_bytes(data, (uint8_t[8]){0x01, 0x02, 0x01, 0x01, 0x46, 0x0C, 0x00, 0x00});

                    BSP_BXCAN_TestBuildFrame(BSP_BXCAN_ID_FB_REAR_RIGHT_POSITION, 0x0102, 0x7A, 0x1234, &feedback, 1000, data);
                    expect_bytes(data, (uint8_t[8]){0x01, 0x02, 0x01, 0x01, 0x9E, 0x0A, 0x00, 0x00});
                    return 0;
                }
                """
            )
        )

    def test_diagnostics_frame_reports_execution_state_and_command_age(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    uint8_t data[8] = {0};
                    BSP_BXCAN_Diagnostics_t diagnostics = {0};

                    diagnostics.flags = BSP_BXCAN_DIAG_DRIVE_ALLOWED |
                                         BSP_BXCAN_DIAG_COMMAND_FRESH |
                                         BSP_BXCAN_DIAG_LEFT_ENCODER_INVALID;
                    diagnostics.safety_state = 4U;
                    diagnostics.safety_action = 1U;
                    diagnostics.command_age_ms = 37U;
                    diagnostics.rx_error_count = 12U;
                    diagnostics.tx_error_count = 3U;
                    diagnostics.can_error_class = BSP_BXCAN_CAN_ERROR_NONE;

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_SetDiagnostics(&diagnostics);
                    BSP_BXCAN_TestBuildFrame(BSP_BXCAN_ID_FB_DIAGNOSTICS,
                                             0x1234,
                                             0,
                                             0,
                                             &(BSP_BXCAN_Feedback_t){0},
                                             0,
                                             data);

                    if (data[0] != BSP_BXCAN_PROTOCOL_VERSION) return 1;
                    if (data[1] != 0x34 || data[2] != 0x12) return 2;
                    if (data[3] != diagnostics.flags) return 3;
                    if (data[4] != diagnostics.safety_state) return 4;
                    if (data[5] != diagnostics.safety_action) return 5;
                    if (data[6] != 3U) return 6;
                    if (data[7] != BSP_BXCAN_CAN_ERROR_NONE) return 7;
                    return 0;
                }
                """
            )
        )

    def test_can_diagnostic_counters_record_rx_tx_and_bus_off(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    BSP_BXCAN_Diagnostics_t diagnostics = {0};

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_ReportRxError();
                    BSP_BXCAN_ReportTxError();
                    BSP_BXCAN_ReportCanError(BSP_BXCAN_CAN_ERROR_BUS_OFF);
                    BSP_BXCAN_GetDiagnostics(&diagnostics);

                    if (diagnostics.rx_error_count != 1U) return 1;
                    if (diagnostics.tx_error_count != 1U) return 2;
                    if (diagnostics.can_error_class != BSP_BXCAN_CAN_ERROR_BUS_OFF) return 3;
                    if ((diagnostics.flags & BSP_BXCAN_DIAG_CAN_ERROR) == 0U) return 4;
                    return 0;
                }
                """
            )
        )

    def test_estop_frame_immediately_outputs_zero_command(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    const uint8_t estop[8] = {0x01, 0x36, 0x12, 0x04, 0x00, 0x00, 0x00, 0x00};
                    BSP_BXCAN_Command_t command;

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, estop, 100);

                    if (!BSP_BXCAN_GetCommand(&command)) abort();
                    if (command.command_seq != 0x1236) abort();
                    if (command.rear_left_velocity_mmps != 0) abort();
                    if (command.rear_right_velocity_mmps != 0) abort();
                    if (BSP_BXCAN_GetFault() != BSP_BXCAN_FAULT_ESTOP_ACTIVE) abort();
                    return 0;
                }
                """
            )
        )

    def test_inconsistent_command_pair_is_rejected(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    const uint8_t steering[8] = {0x01, 0x34, 0x12, 0x01, 0xFA, 0x00, 0x00, 0x00};
                    const uint8_t wheels_bad_seq[8] = {0x01, 0x35, 0x12, 0x01, 0x84, 0x03, 0x4C, 0x04};
                    BSP_BXCAN_Command_t command;

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels_bad_seq, 102);

                    if (BSP_BXCAN_GetCommand(&command)) abort();
                    if (BSP_BXCAN_GetFault() != BSP_BXCAN_FAULT_GROUP_INCONSISTENT) abort();
                    return 0;
                }
                """
            )
        )

    def test_command_timeout_outputs_zero_command(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    const uint8_t steering[8] = {0x01, 0x34, 0x12, 0x01, 0xFA, 0x00, 0x00, 0x00};
                    const uint8_t wheels[8] = {0x01, 0x34, 0x12, 0x01, 0x84, 0x03, 0x4C, 0x04};
                    BSP_BXCAN_Command_t command;

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 104);
                    if (!BSP_BXCAN_GetCommand(&command)) abort();

                    BSP_BXCAN_Process(204);
                    if (!BSP_BXCAN_GetCommand(&command)) abort();
                    if (command.rear_left_velocity_mmps != 0) abort();
                    if (command.rear_right_velocity_mmps != 0) abort();
                    if (BSP_BXCAN_GetFault() != BSP_BXCAN_FAULT_COMMAND_TIMEOUT) abort();
                    return 0;
                }
                """
            )
        )

    def test_command_pair_accepts_exact_window_boundary(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "BSP/bsp_bxcan.c"

                int main(void) {
                    const uint8_t steering[8] = {0x01, 0x34, 0x12, 0x01, 0xFA, 0x00, 0x00, 0x00};
                    const uint8_t wheels[8] = {0x01, 0x34, 0x12, 0x01, 0x84, 0x03, 0x4C, 0x04};
                    BSP_BXCAN_Command_t command;

                    BSP_BXCAN_ResetForTest();
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_STEERING, 8, 0, 0, steering, 100);
                    BSP_BXCAN_Process(110);
                    BSP_BXCAN_OnRxFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, 8, 0, 0, wheels, 110);

                    if (!BSP_BXCAN_GetCommand(&command)) abort();
                    if (command.command_seq != 0x1234) abort();
                    return 0;
                }
                """
            )
        )


if __name__ == "__main__":
    unittest.main()
