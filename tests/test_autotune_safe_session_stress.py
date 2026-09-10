import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class AutotuneSafeSessionStressTest(unittest.TestCase):
    def test_repeated_lifecycle_rejects_stale_and_duplicate_messages(self):
        source = textwrap.dedent(r"""
            #include <stdint.h>
            #include <string.h>
            #include "BSP/autotune_safe_session.h"
            #include "BSP/autotune_safe_session.c"
            #include "BSP/autotune_safe_local_smoke.c"

            static uint32_t next_request_id = 100U;

            static void publish(uint32_t request_id, uint32_t type,
                                uint32_t session_id, uint32_t experiment_id,
                                uint32_t payload0) {
                AutotuneSession_Request_t request = {0};
                request.version = AUTOTUNE_SESSION_VERSION;
                request.size = sizeof(request);
                request.request_id = request_id;
                request.request_type = type;
                request.session_id = session_id;
                request.experiment_id = experiment_id;
                request.payload[0] = payload0;
                request.boot_generation_hint = 1U;
                request.crc32 = AutotuneSession_Crc32(
                    ((const uint8_t *)&request) + 4U, 40U);
                g_autotune_session_request = request;
                g_autotune_session_request.magic =
                    AUTOTUNE_SESSION_REQUEST_MAGIC;
            }

            static void process_request(uint32_t type, uint32_t session_id,
                                        uint32_t experiment_id,
                                        uint32_t payload0) {
                publish(++next_request_id, type, session_id, experiment_id,
                        payload0);
                AutotuneSession_Process(next_request_id);
            }

            int main(void) {
                uint32_t now = 0U;
                AutotuneSession_Init(0x53545253U,
                    AUTOTUNE_SESSION_PROFILE_LOCAL_SMOKE,
                    AUTOTUNE_SESSION_BOOT_REASON_POWER_ON);
                for (uint32_t cycle = 1U; cycle <= 50U; ++cycle) {
                    uint32_t session = 0x10000000U + cycle;
                    uint32_t experiment = 0x20000000U + cycle;
                    uint32_t before;
                    uint32_t result_magic;
                    uint32_t result_crc;

                    process_request(AUTOTUNE_SESSION_REQUEST_CREATE_SESSION,
                                    session, 0U, 0U);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_ARMED) return 1;
                    before = g_autotune_session_status.sample_generation;

                    /* Duplicate CREATE and stale request cannot re-arm or
                       increment session_generation. */
                    publish(next_request_id, AUTOTUNE_SESSION_REQUEST_CREATE_SESSION,
                            session + 1U, 0U, 0U);
                    AutotuneSession_Process(++now);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_ARMED ||
                        g_autotune_session_status.session_generation != cycle) return 2;
                    publish(1U, AUTOTUNE_SESSION_REQUEST_CREATE_SESSION,
                            session + 2U, 0U, 0U);
                    AutotuneSession_Process(++now);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_ARMED) return 3;

                    process_request(AUTOTUNE_SESSION_REQUEST_START_EXPERIMENT,
                                    session, experiment, before);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_RUNNING ||
                        g_autotune_session_status.start_sample_generation != before) return 4;

                    /* Duplicate START is rejected without changing the active
                       experiment or forcing an abort. */
                    publish(next_request_id, AUTOTUNE_SESSION_REQUEST_START_EXPERIMENT,
                            session, experiment + 1U, before);
                    AutotuneSession_Process(++now);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_RUNNING) return 5;

                    now += 10U;
                    for (uint32_t sample = 0U; sample < 32U; ++sample) {
                        AutotuneSession_LocalSmokeProcess(now);
                        now += 10U;
                    }
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED ||
                        g_autotune_session_result.sample_count != 32U) return 6;
                    result_magic = g_autotune_session_result.magic;
                    result_crc = g_autotune_session_result.result_crc32;

                    /* Delayed reads and duplicate START cannot mutate a
                       latched immutable result. */
                    AutotuneSession_LocalSmokeProcess(now + 1000U);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED ||
                        g_autotune_session_result.magic != result_magic ||
                        g_autotune_session_result.result_crc32 != result_crc) return 7;
                    publish(next_request_id, AUTOTUNE_SESSION_REQUEST_START_EXPERIMENT,
                            session, experiment, before);
                    AutotuneSession_Process(++now);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED ||
                        g_autotune_session_result.result_crc32 != result_crc) return 8;

                    process_request(AUTOTUNE_SESSION_REQUEST_ACK_RESULT,
                                    session, experiment, 0U);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_READY ||
                        g_autotune_session_result.magic != 0U) return 9;
                    /* ACK replay is a harmless stale request in READY. */
                    publish(next_request_id, AUTOTUNE_SESSION_REQUEST_ACK_RESULT,
                            session, experiment, 0U);
                    AutotuneSession_Process(++now);
                    if (g_autotune_session_status.state !=
                        AUTOTUNE_SESSION_STATE_READY) return 10;
                }
                return 0;
            }
        """)
        with tempfile.TemporaryDirectory() as tmp:
            source_path = os.path.join(tmp, "session_stress.c")
            executable = os.path.join(tmp, "session_stress.exe")
            with open(source_path, "w", encoding="utf-8") as handle:
                handle.write(source)
            subprocess.run([
                "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", ROOT, source_path, "-o", executable,
            ], check=True, cwd=ROOT)
            result = subprocess.run([executable], cwd=ROOT)
            self.assertEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
