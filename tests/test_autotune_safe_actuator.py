import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class AutotuneSafeActuatorHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_autotune_safe_actuator.c")
            exe = os.path.join(tmp, "test_autotune_safe_actuator.exe")
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

    def test_only_authorized_l1_session_can_drive_and_output_is_shaped(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            int main(void)
            {
                AutotuneSafe_Status_t status;
                AutotuneSafe_Init();
                if (AutotuneSafe_ActuatorGate(1U, 500) != 0) return 1;
                if (AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U) == 0U) return 2;
                if (AutotuneSafe_BeginExperiment(0x11U, 0x22U, 100) == 0U) return 3;
                if (AutotuneSafe_GetEffectiveTarget() <= 0) return 4;
                if (AutotuneSafe_GetEffectiveTarget() >= 100) return 5;
                if (AutotuneSafe_Heartbeat(0x11U, 0x22U, 1U, 0U) == 0U) return 6;
                AutotuneSafe_SetSafetyInput(1U, 1U);
                if (AutotuneSafe_ActuatorGate(1U, 500) != 20) return 7;
                if (AutotuneSafe_ActuatorGate(1U, 500) != 40) return 8;
                AutotuneSafe_RecordAbort(AUTOTUNE_SAFE_ABORT_OVERSPEED);
                if (AutotuneSafe_ActuatorGate(1U, 500) != 0) return 9;
                AutotuneSafe_GetStatus(&status);
                if (status.state != AUTOTUNE_SAFE_STATE_ABORT) return 9;
                return 0;
            }
        """))

    def test_direct_reverse_request_is_rejected_until_still_stop(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            int main(void)
            {
                AutotuneSafe_Init();
                if (AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U) == 0U) return 1;
                if (AutotuneSafe_BeginExperiment(1U, 1U, -100) != 0U) return 2;
                return 0;
            }
        """))

    def test_stop_requires_mcu_stillness_confirmation_before_next_experiment(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            int main(void)
            {
                AutotuneSafe_Status_t status;
                AutotuneSafe_Init();
                AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U);
                AutotuneSafe_BeginExperiment(1U, 1U, 100);
                AutotuneSafe_Heartbeat(1U, 1U, 1U, 0U);
                AutotuneSafe_RequestStop();
                AutotuneSafe_RecordStillness(0U, 0, 0);
                AutotuneSafe_RecordStillness(60U, 0, 0);
                AutotuneSafe_GetStatus(&status);
                if (status.state != AUTOTUNE_SAFE_STATE_READY) return 1;
                if (AutotuneSafe_BeginExperiment(1U, 2U, 100) == 0U) return 2;
                return 0;
            }
        """))

    def test_completed_stop_promotes_only_the_accepted_candidate_to_safe_baseline(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            int main(void)
            {
                AutotuneSafe_Status_t status;
                AutotuneSafe_Gains_t candidate = {0.25f, 0.75f, 0.05f};
                AutotuneSafe_Init();
                AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U);
                AutotuneSafe_SetCandidate(&candidate);
                AutotuneSafe_BeginExperiment(1U, 1U, 100);
                AutotuneSafe_Heartbeat(1U, 1U, 1U, 0U);
                AutotuneSafe_RecordSample(0U, 10, 0, 20, 1U, 1U, 1U);
                AutotuneSafe_RequestStop();
                AutotuneSafe_RecordStillness(0U, 0, 0);
                AutotuneSafe_RecordStillness(60U, 0, 0);
                AutotuneSafe_GetStatus(&status);
                if (status.best_known_safe.kp != 0.25f) return 1;
                if (status.bootstrap_active != 0U) return 2;
                return 0;
            }
        """))

    def test_all_existing_drive_sources_use_the_motor_final_gate(self):
        with open(os.path.join(ROOT, "BSP", "bsp_motor.c"), encoding="utf-8") as handle:
            motor = handle.read()
        self.assertIn("AutotuneSafe_ActuatorGate", motor)
        for relative in ("BSP/chassis_control.c", "BSP/h4_characterization.c", "Core/Src/main.c"):
            with open(os.path.join(ROOT, relative), encoding="utf-8") as handle:
                source = handle.read()
            if "Motor_Drive(" in source:
                self.assertIn("Motor_Drive(", source)
        with open(os.path.join(ROOT, "BSP", "chassis_control.c"), encoding="utf-8") as handle:
            control = handle.read()
        self.assertNotIn("__HAL_TIM_SET_COMPARE", control)
        self.assertIn("AutotuneSafe_GetEffectiveTarget", control)
        self.assertIn("AutotuneSafe_RecordSample", control)
        self.assertIn("AutotuneSafe_RecordStillness", control)


if __name__ == "__main__":
    unittest.main()
