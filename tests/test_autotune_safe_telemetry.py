import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as handle:
        return handle.read()


class AutotuneSafeTelemetryIntegrationTest(unittest.TestCase):
    def test_mcu_builder_emits_every_classic_can_snapshot_frame(self):
        source = textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            int main(void)
            {
                const uint16_t ids[] = {
                    AUTOTUNE_SAFE_ID_FB_IDENTITY, AUTOTUNE_SAFE_ID_FB_STATE,
                    AUTOTUNE_SAFE_ID_FB_LIMITS, AUTOTUNE_SAFE_ID_FB_WHEEL,
                    AUTOTUNE_SAFE_ID_FB_PID_PI, AUTOTUNE_SAFE_ID_FB_PID_DO,
                    AUTOTUNE_SAFE_ID_FB_SAFETY, AUTOTUNE_SAFE_ID_FB_COUNTERS,
                };
                uint8_t data[8];
                uint8_t i;
                AutotuneSafe_Init();
                for (i = 0U; i < (uint8_t)(sizeof(ids) / sizeof(ids[0])); ++i) {
                    uint8_t axis = (ids[i] == AUTOTUNE_SAFE_ID_FB_WHEEL ||
                                    ids[i] == AUTOTUNE_SAFE_ID_FB_PID_PI ||
                                    ids[i] == AUTOTUNE_SAFE_ID_FB_PID_DO) ? 1U : 0U;
                    if (AutotuneSafe_BuildTelemetryFrame(ids[i], 7U, axis, data) == 0U) {
                        return (int)(i + 1U);
                    }
                    if (data[0] != 1U) return 20;
                }
                return 0;
            }
        """)
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "telemetry.c")
            exe = os.path.join(tmp, "telemetry.exe")
            with open(src, "w", encoding="utf-8") as handle:
                handle.write(source)
            subprocess.run([
                "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                "-DAUTOTUNE_SAFE_PROFILE", "-DAUTOTUNE_SAFE_HOST_TEST",
                "-I", ROOT, src, "-o", exe,
            ], check=True, cwd=ROOT)
            subprocess.run([exe], check=True, cwd=ROOT)

    def test_mcu_declares_snapshot_feedback_ids_and_builder(self):
        header = read_rel("BSP/autotune_safe.h")
        can_header = read_rel("BSP/bsp_bxcan.h")
        source = read_rel("BSP/autotune_safe.c")
        self.assertIn("AUTOTUNE_SAFE_ID_FB_IDENTITY", header)
        self.assertIn("AUTOTUNE_SAFE_ID_FB_COUNTERS", header)
        self.assertIn("AutotuneSafe_BuildTelemetryFrame", header)
        self.assertIn("AutotuneSafe_BuildTelemetryFrame", source)
        self.assertIn("AUTOTUNE_SAFE_ID_FB_IDENTITY", can_header)

    def test_profile_can_scheduler_publishes_a_consistent_autotune_snapshot(self):
        source = read_rel("BSP/bsp_bxcan.c")
        self.assertIn("AutotuneSafe_BuildTelemetryFrame", source)
        self.assertIn("g_autotune_tx_snapshot_seq", source)
        self.assertIn("AUTOTUNE_SAFE_ID_FB_COUNTERS", source)
        self.assertIn("#ifdef AUTOTUNE_SAFE_PROFILE", source)


if __name__ == "__main__":
    unittest.main()
