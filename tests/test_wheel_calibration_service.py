import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


COMMON = textwrap.dedent(
    r"""
    #include <stdint.h>
    #include <stdlib.h>

    #include "BSP/bsp_bxcan.h"
    #include "BSP/encoder.h"
    #include "BSP/wheel_calibration.h"

    #include "BSP/wheel_calibration.c"
    #include "BSP/wheel_calibration_service.c"

    static BSP_BXCAN_Command_t make_stop(void)
    {
        BSP_BXCAN_Command_t command = {0};
        command.mode_flags = BSP_BXCAN_MODE_STOP;
        command.accepted_time_ms = 0U;
        return command;
    }

    static Wheel_Calibration_ServiceRequest_t make_start(uint16_t seq)
    {
        Wheel_Calibration_ServiceRequest_t request = {0};
        request.service_seq = seq;
        request.opcode = WHEEL_CALIBRATION_OPCODE_START;
        request.options = WHEEL_CALIBRATION_OPTION_MAINTENANCE_CONFIRM |
                          WHEEL_CALIBRATION_OPTION_WHEELS_LIFTED_CONFIRM;
        request.service_cookie = WHEEL_CALIBRATION_SERVICE_COOKIE;
        request.reserved = 0U;
        return request;
    }

    static EncoderSample_t sample(int64_t counts, int32_t velocity)
    {
        EncoderSample_t value = {0};
        value.accumulated_counts = counts;
        value.velocity_mmps = velocity;
        value.trusted = 1U;
        return value;
    }
    """
)


class WheelCalibrationServiceHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_wheel_calibration_service.c")
            exe = os.path.join(tmp, "test_wheel_calibration_service.exe")
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
                    "-DWHEEL_CALIBRATION_HOST_TEST",
                    "-DWHEEL_CALIBRATION_SERVICE_HOST_TEST",
                    "-DBSP_BXCAN_HOST_TEST",
                    "-DENCODER_HOST_TEST",
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

    def test_request_and_feedback_codec_match_protocol(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                uint8_t data[8] = {0x01, 0x34, 0x12, 0x01, 0x03, 0x5A, 0xC3, 0x00};
                uint8_t feedback_data[8] = {0};
                Wheel_Calibration_ServiceRequest_t request = {0};
                Wheel_Calibration_ServiceFeedback_t feedback = {0};

                if (Wheel_Calibration_Service_DecodeRequest(8U, 0U, 0U, data, &request) == 0U) return 1;
                if (request.service_seq != 0x1234U ||
                    request.opcode != WHEEL_CALIBRATION_OPCODE_START ||
                    request.options != 0x03U ||
                    request.service_cookie != WHEEL_CALIBRATION_SERVICE_COOKIE ||
                    request.reserved != 0U) return 2;

                feedback.service_seq = 0x1234U;
                feedback.transaction_state = WHEEL_CALIBRATION_TX_PRECHECK;
                feedback.calibration_stage = WHEEL_CALIBRATION_STAGE_PRECHECK;
                feedback.exit_reason = WHEEL_CALIBRATION_EXIT_NONE;
                feedback.result_flags = WHEEL_CALIBRATION_RESULT_RESPONSE_TO_REQUEST;
                feedback.progress_percent = 7U;
                Wheel_Calibration_Service_EncodeFeedback(&feedback, feedback_data);

                if (feedback_data[0] != 0x01U || feedback_data[1] != 0x34U ||
                    feedback_data[2] != 0x12U || feedback_data[3] != 0x01U ||
                    feedback_data[4] != 0x01U || feedback_data[5] != 0x00U ||
                    feedback_data[6] != 0x04U || feedback_data[7] != 7U) return 3;
                return 0;
            }
            """
        )

    def test_start_requires_stop_and_confirmations_and_enters_precheck(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t stop = make_stop();
                BSP_BXCAN_Command_t velocity = stop;
                Wheel_Calibration_ServiceRequest_t start = make_start(1U);

                velocity.mode_flags = BSP_BXCAN_MODE_VELOCITY;
                velocity.rear_left_velocity_mmps = 100;
                velocity.rear_right_velocity_mmps = 100;

                Wheel_Calibration_Service_Init();
                Wheel_Calibration_Service_OnRequest(&start, 0U, &velocity, 1U, 0U, 0U);
                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_REJECTED) return 1;
                if (Wheel_Calibration_Service_GetExitReason() != WHEEL_CALIBRATION_EXIT_REJECTED_NOT_PERMITTED) return 2;

                Wheel_Calibration_Service_Init();
                start.options = WHEEL_CALIBRATION_OPTION_MAINTENANCE_CONFIRM;
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_REJECTED) return 3;

                Wheel_Calibration_Service_Init();
                start.options = WHEEL_CALIBRATION_OPTION_MAINTENANCE_CONFIRM |
                                 WHEEL_CALIBRATION_OPTION_WHEELS_LIFTED_CONFIRM;
                start.service_cookie = 0U;
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_REJECTED) return 4;
                if (Wheel_Calibration_Service_GetExitReason() != WHEEL_CALIBRATION_EXIT_REJECTED_BAD_FORMAT) return 5;

                Wheel_Calibration_Service_Init();
                start.service_cookie = WHEEL_CALIBRATION_SERVICE_COOKIE;
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_PRECHECK) return 6;
                Wheel_Calibration_Service_OnRequest(&start, 1U, &stop, 1U, 0U, 0U);
                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_PRECHECK) return 7;
                return 0;
            }
            """
        )

    def test_precheck_and_left_forward_are_nonblocking_and_recommend_low_pwm(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t stop = make_stop();
                Wheel_Calibration_ServiceRequest_t start = make_start(2U);
                Wheel_Calibration_Recommendation_t recommendation = {0};

                Wheel_Calibration_Service_Init();
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                EncoderSample_t left = sample(0, 0);
                EncoderSample_t right = sample(0, 0);
                Wheel_Calibration_Service_Process(0U, &stop, 1U, &left, &right, 0U, 0U);
                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_PRECHECK) return 1;

                Wheel_Calibration_Service_Process(300U, &stop, 1U, &left, &right, 0U, 0U);
                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_RUNNING) return 2;
                if (Wheel_Calibration_Service_GetStage() != WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) return 3;
                if (Wheel_Calibration_Service_GetRecommendation(&recommendation) == 0U) return 4;
                if (recommendation.left_pwm <= 0 || recommendation.left_pwm > 250 || recommendation.right_pwm != 0) return 5;
                return 0;
            }
            """
        )

    def test_start_pwm_is_ramped_and_first_response_is_persisted(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t stop = make_stop();
                Wheel_Calibration_ServiceRequest_t start = make_start(22U);
                Wheel_Calibration_Recommendation_t recommendation = {0};
                EncoderSample_t left = sample(0, 0);
                EncoderSample_t right = sample(0, 0);

                Wheel_Calibration_Service_Init();
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                Wheel_Calibration_Service_Process(0U, &stop, 1U, &left, &right, 0U, 0U);
                Wheel_Calibration_Service_Process(300U, &stop, 1U, &left, &right, 0U, 0U);
                if (Wheel_Calibration_Service_GetRecommendation(&recommendation) == 0U ||
                    recommendation.left_pwm <= 0 || recommendation.left_pwm >= 100) return 1;
                Wheel_Calibration_Service_Process(500U, &stop, 1U, &left, &right, 0U, 0U);
                if (Wheel_Calibration_Service_GetRecommendation(&recommendation) == 0U ||
                    recommendation.left_pwm <= 0 || recommendation.left_pwm <= 50) return 2;
                left.accumulated_counts = 300;
                Wheel_Calibration_Service_Process(600U, &stop, 1U, &left, &right, 0U, 0U);
                left.accumulated_counts = 600;
                Wheel_Calibration_Service_Process(1800U, &stop, 1U, &left, &right, 0U, 0U);
                if (Wheel_Calibration_Service_GetStage() != WHEEL_CALIBRATION_STAGE_LEFT_SETTLE) return 3;
                return 0;
            }
            """
        )

    def test_terminal_decision_latches_the_exact_left_sample(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t stop = make_stop();
                Wheel_Calibration_ServiceRequest_t start = make_start(23U);
                Wheel_Calibration_TerminalSample_t terminal = {0};
                EncoderSample_t left = sample(0, 0);
                EncoderSample_t right = sample(0, 0);

                Wheel_Calibration_Service_Init();
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                Wheel_Calibration_Service_Process(0U, &stop, 1U, &left, &right, 0U, 0U);
                Wheel_Calibration_Service_Process(300U, &stop, 1U, &left, &right, 0U, 0U);

                left.accumulated_counts = 100;
                left.delta_counts = 100;
                left.velocity_mmps = 100;
                left.trusted = 1U;
                Wheel_Calibration_Service_Process(1800U, &stop, 1U, &left, &right, 0U, 0U);

                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_FAILED_VALIDATION) return 1;
                if (Wheel_Calibration_Service_GetExitReason() !=
                    WHEEL_CALIBRATION_EXIT_FORWARD_INSUFFICIENT_MOTION) return 2;
                left.accumulated_counts = 9999;
                left.delta_counts = 9999;
                left.trusted = 0U;
                Wheel_Calibration_Service_GetTerminalSample(&terminal);
                if (terminal.valid == 0U) return 3;
                if (terminal.stage != WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) return 4;
                if (terminal.timestamp_ms != 1800U) return 5;
                if (terminal.left_sample.accumulated_counts != 100) return 6;
                if (terminal.left_sample.delta_counts != 100) return 7;
                if (terminal.left_sample.velocity_mmps != 100) return 8;
                if (terminal.left_sample.trusted != 1U) return 9;
                if (terminal.stage_delta_counts != 100) return 10;
                return 0;
            }
            """
        )

    def test_both_wheels_commit_only_after_forward_reverse_validation(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t stop = make_stop();
                Wheel_Calibration_ServiceRequest_t start = make_start(3U);
                EncoderSample_t left = sample(0, 0);
                EncoderSample_t right = sample(0, 0);

                Wheel_Calibration_Service_Init();
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                Wheel_Calibration_Service_Process(0U, &stop, 1U, &left, &right, 0U, 0U);
                Wheel_Calibration_Service_Process(300U, &stop, 1U, &left, &right, 0U, 0U);

                left.accumulated_counts = 600;
                Wheel_Calibration_Service_Process(1800U, &stop, 1U, &left, &right, 0U, 0U);
                Wheel_Calibration_Service_Process(2100U, &stop, 1U, &left, &right, 0U, 0U);
                Wheel_Calibration_Service_Process(2400U, &stop, 1U, &left, &right, 0U, 0U);
                left.accumulated_counts = 0;
                Wheel_Calibration_Service_Process(3900U, &stop, 1U, &left, &right, 0U, 0U);

                right.accumulated_counts = 600;
                Wheel_Calibration_Service_Process(5400U, &stop, 1U, &left, &right, 0U, 0U);
                Wheel_Calibration_Service_Process(5700U, &stop, 1U, &left, &right, 0U, 0U);
                Wheel_Calibration_Service_Process(6000U, &stop, 1U, &left, &right, 0U, 0U);
                right.accumulated_counts = 0;
                Wheel_Calibration_Service_Process(7500U, &stop, 1U, &left, &right, 0U, 0U);

                if (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_SUCCEEDED) return 1;
                if (Wheel_Calibration_Service_GetExitReason() != WHEEL_CALIBRATION_EXIT_SUCCESS) return 2;
                if (Wheel_Calibration_IsValid() == 0U) return 3;
                {
                    Wheel_Calibration_TerminalSample_t terminal = {0};
                    Wheel_Calibration_Service_GetTerminalSample(&terminal);
                    if (terminal.valid == 0U) return 4;
                    if (terminal.terminal_state != WHEEL_CALIBRATION_TX_SUCCEEDED) return 5;
                    if (terminal.stage != WHEEL_CALIBRATION_STAGE_RIGHT_REVERSE) return 6;
                    if (terminal.timestamp_ms != 7500U) return 7;
                    if (terminal.left_sample.accumulated_counts != 0) return 8;
                }
                return 0;
            }
            """
        )

    def test_event_response_is_one_shot_and_active_service_publishes_periodically(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                BSP_BXCAN_Command_t stop = make_stop();
                Wheel_Calibration_ServiceRequest_t start = make_start(4U);

                Wheel_Calibration_Service_Init();
                Wheel_Calibration_Service_OnRequest(&start, 0U, &stop, 1U, 0U, 0U);
                if (Wheel_Calibration_Service_HasResponsePending() == 0U) return 1;
                if (Wheel_Calibration_Service_ShouldPublish(0U) == 0U) return 2;
                Wheel_Calibration_Service_MarkPublished(0U);
                if (Wheel_Calibration_Service_HasResponsePending() != 0U) return 3;
                if (Wheel_Calibration_Service_ShouldPublish(19U) != 0U) return 4;
                if (Wheel_Calibration_Service_ShouldPublish(20U) == 0U) return 5;
                return 0;
            }
            """
        )


if __name__ == "__main__":
    unittest.main()
