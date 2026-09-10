#include "autotune_safe_session.h"

static uint32_t g_smoke_last_sample_ms;
static uint8_t g_smoke_sample_clock_valid;

void AutotuneSession_LocalSmokeProcess(uint32_t now_ms)
{
    AutotuneSession_Process(now_ms);
    if (g_autotune_session_status.state != AUTOTUNE_SESSION_STATE_RUNNING) {
        return;
    }
    if (g_autotune_session_status.sample_count <
        AUTOTUNE_SESSION_SMOKE_SAMPLE_COUNT) {
        if (g_smoke_sample_clock_valid == 0U) {
            g_smoke_last_sample_ms = now_ms;
            g_smoke_sample_clock_valid = 1U;
        } else if ((uint32_t)(now_ms - g_smoke_last_sample_ms) <
                   AUTOTUNE_SESSION_SMOKE_SAMPLE_PERIOD_MS) {
            return;
        }
        (void)AutotuneSession_AppendSample(now_ms, 100, 100, 0, 1U);
        g_smoke_last_sample_ms = now_ms;
    }
    if (g_autotune_session_status.sample_count >=
        AUTOTUNE_SESSION_SMOKE_SAMPLE_COUNT) {
        AutotuneSession_Complete();
    }
}
