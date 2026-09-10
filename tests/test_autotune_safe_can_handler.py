import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class AutotuneSafeCanHandlerHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_autotune_safe_can_handler.c")
            exe = os.path.join(tmp, "test_autotune_safe_can_handler.exe")
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

    def test_start_heartbeat_stop_and_promote_are_session_bound(self):
        self.compile_and_run(textwrap.dedent(r"""
            #include <stdint.h>
            #include "BSP/autotune_safe.h"
            #include "BSP/autotune_safe.c"

            static void control(uint8_t opcode, uint16_t session,
                                uint16_t experiment, uint16_t argument,
                                uint32_t now_ms)
            {
                uint8_t data[8] = {1U, opcode,
                                   (uint8_t)session, (uint8_t)(session >> 8),
                                   (uint8_t)experiment, (uint8_t)(experiment >> 8),
                                   (uint8_t)argument, (uint8_t)(argument >> 8)};
                AutotuneSafe_OnCanFrame(8U, 0U, 0U, data, now_ms);
            }

            int main(void)
            {
                AutotuneSafe_Status_t status;
                AutotuneSafe_Init();
                if (AutotuneSafe_ConfirmPreflight(1U, 1U, 1U, 1U) == 0U) return 1;
                control(AUTOTUNE_SAFE_OPCODE_START, 0x1234U, 0x5678U, 100U, 0U);
                AutotuneSafe_GetStatus(&status);
                if (status.session_id != 0x1234U || status.experiment_id != 0x5678U) return 2;
                if (status.requested_target != 100) return 3;
                control(AUTOTUNE_SAFE_OPCODE_HEARTBEAT, 0x9999U, 0x5678U, 1U, 0U);
                AutotuneSafe_GetStatus(&status);
                if (status.abort_flag != 0U) return 4;
                control(AUTOTUNE_SAFE_OPCODE_HEARTBEAT, 0x1234U, 0x5678U, 1U, 20U);
                if (AutotuneSafe_Heartbeat(0x1234U, 0x5678U, 1U, 20U) != 0U) return 5;
                control(AUTOTUNE_SAFE_OPCODE_REQUEST_PROMOTE, 0x1234U, 0x5678U, 0U, 20U);
                AutotuneSafe_GetStatus(&status);
                if (status.level != AUTOTUNE_SAFE_LEVEL_L1_INITIAL) return 6;
                control(AUTOTUNE_SAFE_OPCODE_STOP, 0x1234U, 0x5678U, 0U, 30U);
                AutotuneSafe_GetStatus(&status);
                if (status.state != AUTOTUNE_SAFE_STATE_STOPPING) return 7;
                return 0;
            }
        """))


if __name__ == "__main__":
    unittest.main()
