import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class AutotuneSafeCoreHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_autotune_safe_core.c")
            exe = os.path.join(tmp, "test_autotune_safe_core.exe")
            with open(src, "w", encoding="utf-8") as handle:
                handle.write(source)
            subprocess.run(
                [
                    "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-DAUTOTUNE_SAFE_PROFILE", "-DAUTOTUNE_SAFE_HOST_TEST",
                    "-I", ROOT, src, "-o", exe,
                ],
                check=True,
                cwd=ROOT,
            )
            result = subprocess.run([exe], cwd=ROOT)
            self.assertEqual(result.returncode, 0)

    def test_bootstrap_limits_and_mcu_owned_level_promotion(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include <stdlib.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            static int near(float actual, float expected)
            {
                return (actual > expected - 0.0001f) && (actual < expected + 0.0001f);
            }

            int main(void)
            {
                AutotuneSafe_Status_t status;
                AutotuneSafe_Gains_t candidate = {0.25f, 0.75f, 0.05f};
                AutotuneSafe_Gains_t too_large = {16.0f, 0.75f, 0.05f};
                AutotuneSafe_Gains_t too_far = {0.313f, 0.75f, 0.05f};

                AutotuneSafe_Init();
                AutotuneSafe_GetStatus(&status);
                if (status.level != AUTOTUNE_SAFE_LEVEL_L0_LOCKED) return 1;
                if (status.state != AUTOTUNE_SAFE_STATE_LOCKED) return 2;
                if (status.pwm_limit != 0U || status.target_limit_mmps != 0U) return 3;

                if (AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U) == 0U) return 4;
                AutotuneSafe_GetStatus(&status);
                if (status.level != AUTOTUNE_SAFE_LEVEL_L1_INITIAL) return 5;
                if (status.pwm_limit != 150U || status.target_limit_mmps != 100U) return 6;
                if (AutotuneSafe_ValidateCandidate(&candidate) !=
                    AUTOTUNE_SAFE_GAIN_ACCEPTED) return 7;
                if (AutotuneSafe_ValidateCandidate(&too_large) ==
                    AUTOTUNE_SAFE_GAIN_ACCEPTED) return 8;

                AutotuneSafe_SetCandidate(&candidate);
                AutotuneSafe_RecordCompletePass();
                AutotuneSafe_GetStatus(&status);
                if (status.bootstrap_active != 0U) return 9;
                if (!near(status.best_known_safe.kp, 0.25f)) return 10;
                if (AutotuneSafe_ValidateCandidate(&too_far) ==
                    AUTOTUNE_SAFE_GAIN_ACCEPTED) return 11;

                if (AutotuneSafe_RequestPromote() != 0U) return 12;
                AutotuneSafe_RecordCompletePass();
                AutotuneSafe_RecordCompletePass();
                if (AutotuneSafe_RequestPromote() != 0U) return 13;
                AutotuneSafe_GetStatus(&status);
                if (status.level != AUTOTUNE_SAFE_LEVEL_L1_INITIAL) return 14;

                AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_STALL);
                AutotuneSafe_GetStatus(&status);
                if (status.level != AUTOTUNE_SAFE_LEVEL_L1_INITIAL) return 15;
                if (status.promotion_eligible != 0U) return 16;
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_STALL) return 17;
                return 0;
            }
        """))


if __name__ == "__main__":
    unittest.main()
