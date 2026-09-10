import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class AutotuneSafeAbortHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_autotune_safe_abort.c")
            exe = os.path.join(tmp, "test_autotune_safe_abort.exe")
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

    def test_local_stall_saturation_overspeed_oscillation_and_safety_abort(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            static void ready(void)
            {
                AutotuneSafe_Init();
                if (AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U) == 0U) return;
                (void)AutotuneSafe_BeginExperiment(7U, 8U, 100);
                AutotuneSafe_Heartbeat(7U, 8U, 1U, 0U);
            }

            int main(void)
            {
                AutotuneSafe_Status_t status;

                ready();
                AutotuneSafe_RecordSample(0U, 100, 0, 120, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(100U, 100, 0, 120, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(160U, 100, 0, 120, 1U, 1U, 1U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_STALL) return 1;
                if (status.stall_time_ms < 160U || status.abort_flag == 0U) return 2;

                ready();
                AutotuneSafe_RecordSample(0U, 100, 20, 145, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(100U, 100, 20, 145, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(260U, 100, 20, 145, 1U, 1U, 1U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_SATURATION) return 3;

                ready();
                AutotuneSafe_RecordSample(0U, 100, 160, 80, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(10U, 100, 160, 80, 1U, 1U, 1U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_OVERSPEED ||
                    status.speed_limit_hit == 0U) return 4;

                ready();
                AutotuneSafe_RecordSample(0U, 100, 120, 80, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(10U, 100, 80, 80, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(20U, 100, 120, 80, 1U, 1U, 1U);
                AutotuneSafe_RecordSample(30U, 100, 80, 80, 1U, 1U, 1U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_OSCILLATION ||
                    status.oscillation_count < 3U) return 5;

                ready();
                AutotuneSafe_RecordSample(0U, 100, 0, 20, 1U, 0U, 1U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_SAFETY) return 6;
                if (AutotuneSafe_ActuatorGate(1U, 100) != 0) return 7;
                return 0;
            }
        """))

    def test_session_watchdog_uses_only_fresh_ordered_heartbeat(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            int main(void)
            {
                AutotuneSafe_Status_t status;
                AutotuneSafe_Init();
                if (AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U) == 0U) return 1;
                if (AutotuneSafe_BeginExperiment(9U, 10U, 100) == 0U) return 2;
                if (AutotuneSafe_Heartbeat(9U, 10U, 1U, 0U) == 0U) return 3;
                if (AutotuneSafe_Heartbeat(9U, 10U, 1U, 50U) != 0U) return 4;
                if (AutotuneSafe_Heartbeat(9U, 10U, 0U, 50U) != 0U) return 5;
                AutotuneSafe_Process(99U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_flag != 0U) return 6;
                AutotuneSafe_Process(101U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_WATCHDOG) return 7;
                return 0;
            }
        """))

    def test_local_runtime_energy_proxy_aborts_and_keeps_cooling_lock(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            int main(void)
            {
                AutotuneSafe_Status_t status;
                uint32_t now;
                AutotuneSafe_Init();
                AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U);
                AutotuneSafe_BeginExperiment(1U, 1U, 100);
                for (now = 0U; now <= 3100U; now += 10U) {
                    AutotuneSafe_Heartbeat(1U, 1U, (uint16_t)(now / 10U + 1U), now);
                    (void)AutotuneSafe_ActuatorGate(1U, 150);
                    AutotuneSafe_Process(now);
                }
                AutotuneSafe_GetStatus(&status);
                if (status.abort_reason != AUTOTUNE_SAFE_ABORT_THERMAL) return 1;
                if (status.state != AUTOTUNE_SAFE_STATE_COOLING) return 2;
                if (AutotuneSafe_BeginExperiment(1U, 2U, 100) != 0U) return 3;
                return 0;
            }
        """))


if __name__ == "__main__":
    unittest.main()
