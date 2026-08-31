import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class PidTuningProtocolHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_pid_tuning.c")
            exe = os.path.join(tmp, "test_pid_tuning.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)
            subprocess.run(
                ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", ROOT, src, "-o", exe],
                check=True, cwd=ROOT,
            )
            subprocess.run([exe], check=True, cwd=ROOT)

    def test_two_frame_transaction_applies_only_in_safe_stop(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include <stdlib.h>
            #include "BSP/pid_tuning.h"
            #include "BSP/pid_tuning.c"

            static void frame(uint16_t seq, uint8_t axis, uint16_t kp, uint16_t ki,
                              uint8_t data[8]) {
                data[0] = 1U; data[1] = (uint8_t)seq; data[2] = (uint8_t)(seq >> 8);
                data[3] = axis; data[4] = (uint8_t)kp; data[5] = (uint8_t)(kp >> 8);
                data[6] = (uint8_t)ki; data[7] = (uint8_t)(ki >> 8);
            }

            static void frame_d(uint16_t seq, uint8_t axis, uint16_t kd, uint8_t data[8]) {
                data[0] = 1U; data[1] = (uint8_t)seq; data[2] = (uint8_t)(seq >> 8);
                data[3] = axis; data[4] = (uint8_t)kd; data[5] = (uint8_t)(kd >> 8);
                data[6] = 0x5AU; data[7] = 0xC3U;
            }

            int main(void) {
                uint8_t kpki[8], kd[8], feedback[8];
                PID_Tuning_Gains_t left, right;

                PID_Tuning_Init();
                frame(3U, PID_TUNING_AXIS_BOTH, 77U, 154U, kpki);
                frame_d(3U, PID_TUNING_AXIS_BOTH, 26U, kd);

                PID_Tuning_OnFrame(PID_TUNING_ID_CMD_GAINS, 8, 0, 0, kpki, 10U,
                                   1U, 1U, 1U, 0U, 0U);
                PID_Tuning_OnFrame(PID_TUNING_ID_CMD_D, 8, 0, 0, kd, 11U,
                                   1U, 1U, 1U, 0U, 0U);
                PID_Tuning_GetGains(&left, &right);
                if (left.kp < 0.300f || left.kp > 0.302f) return 1;
                if (left.ki < 0.600f || left.ki > 0.603f) return 2;
                if (left.kd < 0.100f || left.kd > 0.103f) return 3;
                if (right.kp != left.kp || right.ki != left.ki || right.kd != left.kd) return 4;

                frame(4U, PID_TUNING_AXIS_RIGHT, 256U, 512U, kpki);
                frame_d(4U, PID_TUNING_AXIS_RIGHT, 64U, kd);
                PID_Tuning_OnFrame(PID_TUNING_ID_CMD_GAINS, 8, 0, 0, kpki, 12U,
                                   1U, 1U, 1U, 0U, 0U);
                PID_Tuning_OnFrame(PID_TUNING_ID_CMD_D, 8, 0, 0, kd, 13U,
                                   1U, 1U, 1U, 0U, 0U);
                PID_Tuning_GetGains(&left, &right);
                if (left.kp < 0.300f || left.kp > 0.302f) return 5;
                if (right.kp < 0.999f || right.kp > 1.001f) return 6;
                if (right.ki < 1.999f || right.ki > 2.001f) return 7;
                if (right.kd < 0.249f || right.kd > 0.251f) return 8;

                frame(5U, PID_TUNING_AXIS_BOTH, 128U, 128U, kpki);
                PID_Tuning_OnFrame(PID_TUNING_ID_CMD_GAINS, 8, 0, 0, kpki, 20U,
                                   1U, 0U, 0U, 0U, 0U);
                frame_d(5U, PID_TUNING_AXIS_BOTH, 128U, kd);
                PID_Tuning_OnFrame(PID_TUNING_ID_CMD_D, 8, 0, 0, kd, 21U,
                                   1U, 0U, 0U, 0U, 0U);
                PID_Tuning_GetGains(&left, &right);
                if (left.kp < 0.300f || left.kp > 0.302f) return 9;

                PID_Tuning_GetFeedback(NULL);
                PID_Tuning_BuildFeedbackFrame(feedback);
                if (feedback[3] != PID_TUNING_STATUS_REJECTED_UNSAFE) return 10;
                return 0;
            }
        """))


if __name__ == "__main__":
    unittest.main()
