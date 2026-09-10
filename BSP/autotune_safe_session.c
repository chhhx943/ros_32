#include "autotune_safe_session.h"

#include <string.h>

#if defined(__GNUC__)
#define AUTOTUNE_SESSION_NOINIT __attribute__((section(".noinit"), used))
#else
#define AUTOTUNE_SESSION_NOINIT
#endif

volatile AutotuneSession_Retained_t g_autotune_session_retained
    AUTOTUNE_SESSION_NOINIT;
volatile AutotuneSession_BootRequest_t g_autotune_session_boot_request
    AUTOTUNE_SESSION_NOINIT;
volatile AutotuneSession_BootIdentity_t g_autotune_session_boot_identity;
volatile AutotuneSession_Request_t g_autotune_session_request;
volatile AutotuneSession_Status_t g_autotune_session_status;
volatile AutotuneSession_Result_t g_autotune_session_result;
volatile AutotuneSession_Sample_t
    g_autotune_session_samples[AUTOTUNE_SESSION_SAMPLE_CAPACITY];

static uint32_t g_firmware_build_id;
static AutotuneSession_Profile_t g_profile;
static uint32_t g_boot_generation;
static uint32_t g_boot_count;
static uint32_t g_last_consumed_boot_request_id;
static uint32_t g_last_request_id;
static uint32_t g_session_generation;
static uint32_t g_session_id;
static uint32_t g_experiment_id;
static uint32_t g_sample_generation;
static uint32_t g_sample_count;
static uint32_t g_result_ack_request_id;

static void AutotuneSession_Barrier(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("dmb sy" ::: "memory");
#else
    __asm volatile("" ::: "memory");
#endif
}

uint32_t AutotuneSession_Crc32(const void *data, uint32_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;

    for (index = 0U; index < size; ++index) {
        uint32_t bit;
        crc ^= bytes[index];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 1U) != 0U) ?
                  ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

uint32_t AutotuneSession_GenerationDelta(uint32_t newer, uint32_t older)
{
    return newer - older;
}

uint8_t AutotuneSession_GenerationIsForward(uint32_t newer, uint32_t older)
{
    uint32_t delta = AutotuneSession_GenerationDelta(newer, older);
    return (delta != 0U) && (delta < 0x80000000UL);
}

static uint8_t RetainedValid(const AutotuneSession_Retained_t *retained)
{
    return (retained->magic == AUTOTUNE_SESSION_BOOT_MAGIC) &&
           (retained->boot_generation != 0U) &&
           (retained->crc32 ==
            AutotuneSession_Crc32(retained, 16U));
}

static void RetainedCommit(void)
{
    AutotuneSession_Retained_t retained = {0};

    retained.magic = AUTOTUNE_SESSION_BOOT_MAGIC;
    retained.boot_generation = g_boot_generation;
    retained.boot_count = g_boot_count;
    retained.last_consumed_request_id = g_last_consumed_boot_request_id;
    retained.crc32 = AutotuneSession_Crc32(&retained, 16U);
    g_autotune_session_retained = retained;
}

static void BootIdentityCommit(uint32_t state, uint32_t boot_reason)
{
    AutotuneSession_BootIdentity_t next = {0};
    uint32_t sequence = g_autotune_session_boot_identity.commit_seq;

    if ((sequence & 1U) != 0U) {
        sequence++;
    }
    next.commit_seq = sequence;
    next.magic = AUTOTUNE_SESSION_BOOT_MAGIC;
    next.version = AUTOTUNE_SESSION_VERSION;
    next.size = (uint16_t)sizeof(next);
    next.profile = (uint32_t)g_profile;
    next.firmware_build_id = g_firmware_build_id;
    next.boot_generation = g_boot_generation;
    next.boot_reason = boot_reason;
    next.boot_state = state;
    next.mailbox_version = AUTOTUNE_SESSION_MAILBOX_VERSION;
    next.boot_count = g_boot_count;
    next.crc32 = AutotuneSession_Crc32(((const uint8_t *)&next) + 4U, 36U);

    g_autotune_session_boot_identity.commit_seq = sequence + 1U;
    AutotuneSession_Barrier();
    g_autotune_session_boot_identity.magic = next.magic;
    g_autotune_session_boot_identity.version = next.version;
    g_autotune_session_boot_identity.size = next.size;
    g_autotune_session_boot_identity.profile = next.profile;
    g_autotune_session_boot_identity.firmware_build_id = next.firmware_build_id;
    g_autotune_session_boot_identity.boot_generation = next.boot_generation;
    g_autotune_session_boot_identity.boot_reason = next.boot_reason;
    g_autotune_session_boot_identity.boot_state = next.boot_state;
    g_autotune_session_boot_identity.mailbox_version = next.mailbox_version;
    g_autotune_session_boot_identity.boot_count = next.boot_count;
    g_autotune_session_boot_identity.crc32 = next.crc32;
    AutotuneSession_Barrier();
    g_autotune_session_boot_identity.commit_seq = sequence + 2U;
    AutotuneSession_Barrier();
}

static void StatusCommit(uint32_t state, uint32_t last_reject_reason)
{
    AutotuneSession_Status_t next = {0};
    uint32_t sequence = g_autotune_session_status.commit_seq;

    if ((sequence & 1U) != 0U) {
        sequence++;
    }
    next.commit_seq = sequence;
    next.magic = AUTOTUNE_SESSION_STATUS_MAGIC;
    next.version = AUTOTUNE_SESSION_VERSION;
    next.size = (uint16_t)sizeof(next);
    next.accepted_request_id = g_last_request_id;
    next.accepted_session_id = g_session_id;
    next.active_experiment_id = g_experiment_id;
    next.state = state;
    next.boot_generation = g_boot_generation;
    next.session_generation = g_session_generation;
    next.sample_generation = g_sample_generation;
    next.start_sample_generation =
        g_autotune_session_status.start_sample_generation;
    next.sample_count = g_sample_count;
    next.expected_sample_count = AUTOTUNE_SESSION_SMOKE_SAMPLE_COUNT;
    next.last_rejected_request_id =
        g_autotune_session_status.last_rejected_request_id;
    next.last_reject_reason = last_reject_reason;
    next.result_ack_request_id = g_result_ack_request_id;
    next.status_crc32 = AutotuneSession_Crc32(((const uint8_t *)&next) + 4U, 60U);

    g_autotune_session_status.commit_seq = sequence + 1U;
    AutotuneSession_Barrier();
    g_autotune_session_status.magic = next.magic;
    g_autotune_session_status.version = next.version;
    g_autotune_session_status.size = next.size;
    g_autotune_session_status.accepted_request_id = next.accepted_request_id;
    g_autotune_session_status.accepted_session_id = next.accepted_session_id;
    g_autotune_session_status.active_experiment_id = next.active_experiment_id;
    g_autotune_session_status.state = next.state;
    g_autotune_session_status.boot_generation = next.boot_generation;
    g_autotune_session_status.session_generation = next.session_generation;
    g_autotune_session_status.sample_generation = next.sample_generation;
    g_autotune_session_status.start_sample_generation =
        next.start_sample_generation;
    g_autotune_session_status.sample_count = next.sample_count;
    g_autotune_session_status.expected_sample_count = next.expected_sample_count;
    g_autotune_session_status.last_rejected_request_id =
        next.last_rejected_request_id;
    g_autotune_session_status.last_reject_reason = next.last_reject_reason;
    g_autotune_session_status.result_ack_request_id = next.result_ack_request_id;
    g_autotune_session_status.status_crc32 = next.status_crc32;
    AutotuneSession_Barrier();
    g_autotune_session_status.commit_seq = sequence + 2U;
    AutotuneSession_Barrier();
}

static uint8_t RequestValid(const AutotuneSession_Request_t *request)
{
    return (request->magic == AUTOTUNE_SESSION_REQUEST_MAGIC) &&
           (request->version == AUTOTUNE_SESSION_VERSION) &&
           (request->size == sizeof(*request)) &&
           (request->request_id != 0U) &&
           (request->crc32 ==
            AutotuneSession_Crc32(((const uint8_t *)request) + 4U, 40U));
}

static uint8_t BootRequestValid(const AutotuneSession_BootRequest_t *request)
{
    return (request->magic == AUTOTUNE_SESSION_BOOT_MAGIC) &&
           (request->version == AUTOTUNE_SESSION_VERSION) &&
           (request->size == sizeof(*request)) &&
           (request->request_id != 0U) &&
           (request->crc32 ==
            AutotuneSession_Crc32(((const uint8_t *)request) + 4U, 40U));
}

static uint8_t TakeRequest(AutotuneSession_Request_t *request)
{
    AutotuneSession_Request_t candidate = g_autotune_session_request;

    if (RequestValid(&candidate) == 0U) {
        return 0U;
    }
    g_autotune_session_request.magic = 0U;
    AutotuneSession_Barrier();
    *request = candidate;
    return 1U;
}

static uint8_t RequestIsNew(uint32_t request_id)
{
    if (request_id == 0U) {
        return 0U;
    }
    if ((g_last_request_id != 0U) &&
        !AutotuneSession_GenerationIsForward(request_id, g_last_request_id)) {
        return 0U;
    }
    if ((g_last_request_id == 0U) &&
        (g_last_consumed_boot_request_id != 0U) &&
        !AutotuneSession_GenerationIsForward(request_id,
                                              g_last_consumed_boot_request_id)) {
        return 0U;
    }
    return 1U;
}

static void ClearResult(void)
{
    memset((void *)&g_autotune_session_result, 0,
           sizeof(g_autotune_session_result));
}

static void CommitResult(uint32_t final_state, uint32_t abort_reason)
{
    AutotuneSession_Result_t final = {0};
    uint32_t start = g_autotune_session_status.start_sample_generation;

    final.magic = AUTOTUNE_SESSION_RESULT_FINAL_MAGIC;
    final.version = AUTOTUNE_SESSION_VERSION;
    final.size = (uint16_t)sizeof(final);
    final.session_id = g_session_id;
    final.experiment_id = g_experiment_id;
    final.boot_generation = g_boot_generation;
    final.session_generation = g_session_generation;
    final.start_sample_generation = start;
    final.first_sample_generation = (g_sample_count == 0U) ?
                                    start : g_autotune_session_samples[0].generation;
    final.last_sample_generation = (g_sample_count == 0U) ?
                                   start : g_autotune_session_samples[g_sample_count - 1U].generation;
    final.sample_count = g_sample_count;
    final.ring_start_index = 0U;
    final.ring_count = g_sample_count;
    final.final_state = final_state;
    final.abort_reason = abort_reason;
    final.result_flags = 1U;
    final.ring_crc32 = AutotuneSession_Crc32(
        (const void *)g_autotune_session_samples,
        g_sample_count * sizeof(AutotuneSession_Sample_t));
    final.result_crc32 = AutotuneSession_Crc32(&final, 60U);

    /* Commit-last: magic remains zero while every other byte is published. */
    memset((void *)&g_autotune_session_result, 0,
           sizeof(g_autotune_session_result));
    g_autotune_session_result.version = final.version;
    g_autotune_session_result.size = final.size;
    g_autotune_session_result.session_id = final.session_id;
    g_autotune_session_result.experiment_id = final.experiment_id;
    g_autotune_session_result.boot_generation = final.boot_generation;
    g_autotune_session_result.session_generation = final.session_generation;
    g_autotune_session_result.start_sample_generation = final.start_sample_generation;
    g_autotune_session_result.first_sample_generation = final.first_sample_generation;
    g_autotune_session_result.last_sample_generation = final.last_sample_generation;
    g_autotune_session_result.sample_count = final.sample_count;
    g_autotune_session_result.ring_start_index = final.ring_start_index;
    g_autotune_session_result.ring_count = final.ring_count;
    g_autotune_session_result.final_state = final.final_state;
    g_autotune_session_result.abort_reason = final.abort_reason;
    g_autotune_session_result.result_flags = final.result_flags;
    g_autotune_session_result.ring_crc32 = final.ring_crc32;
    g_autotune_session_result.result_crc32 = final.result_crc32;
    AutotuneSession_Barrier();
    g_autotune_session_result.magic = AUTOTUNE_SESSION_RESULT_FINAL_MAGIC;
    AutotuneSession_Barrier();
}

static void RejectRequest(const AutotuneSession_Request_t *request,
                          uint32_t reason)
{
    g_autotune_session_status.last_rejected_request_id = request->request_id;
    StatusCommit(g_autotune_session_status.state, reason);
}

uint8_t AutotuneSession_AppendSample(uint32_t now_ms, int32_t target,
                                     int32_t actual, int32_t pwm,
                                     uint32_t flags)
{
    AutotuneSession_Sample_t sample = {0};
    uint32_t index = g_sample_count % AUTOTUNE_SESSION_SAMPLE_CAPACITY;

    if ((g_autotune_session_status.state !=
         AUTOTUNE_SESSION_STATE_RUNNING) ||
        (g_sample_count >= AUTOTUNE_SESSION_SAMPLE_CAPACITY)) {
        return 0U;
    }

    g_sample_generation++;
    if (g_sample_generation == 0U) {
        g_sample_generation = 1U;
    }
    sample.generation = g_sample_generation;
    sample.timestamp_ms = now_ms;
    sample.session_id = g_session_id;
    sample.experiment_id = g_experiment_id;
    sample.sample_index = g_sample_count;
    sample.target = target;
    sample.actual = actual;
    sample.pwm = pwm;
    sample.state = AUTOTUNE_SESSION_STATE_RUNNING;
    sample.flags = flags;
    g_autotune_session_samples[index] = sample;
    g_sample_count++;
    StatusCommit(AUTOTUNE_SESSION_STATE_RUNNING,
                 AUTOTUNE_SESSION_ABORT_NONE);
    return 1U;
}

void AutotuneSession_Complete(void)
{
    if ((g_autotune_session_status.state !=
         AUTOTUNE_SESSION_STATE_RUNNING) ||
        (g_sample_count != AUTOTUNE_SESSION_SMOKE_SAMPLE_COUNT)) {
        return;
    }
    CommitResult(AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED,
                 AUTOTUNE_SESSION_ABORT_NONE);
    StatusCommit(AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED,
                 AUTOTUNE_SESSION_ABORT_NONE);
    BootIdentityCommit(AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED,
                       AUTOTUNE_SESSION_BOOT_REASON_SOFTWARE_RUN);
    RetainedCommit();
}

static void HandleRequest(const AutotuneSession_Request_t *request)
{
    uint32_t state = g_autotune_session_status.state;

    if (RequestIsNew(request->request_id) == 0U) {
        RejectRequest(request, AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
        return;
    }
    if ((request->boot_generation_hint != 0U) &&
        (request->boot_generation_hint != g_boot_generation)) {
        RejectRequest(request, AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
        return;
    }

    if (request->request_type == AUTOTUNE_SESSION_REQUEST_CREATE_SESSION) {
        if ((state != AUTOTUNE_SESSION_STATE_READY) &&
            (state != AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED) &&
            (state != AUTOTUNE_SESSION_STATE_ABORT_LATCHED)) {
            RejectRequest(request, AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
            return;
        }
        g_last_request_id = request->request_id;
        g_session_id = request->session_id;
        g_experiment_id = 0U;
        g_session_generation++;
        if (g_session_generation == 0U) {
            g_session_generation = 1U;
        }
        g_sample_count = 0U;
        g_autotune_session_status.start_sample_generation = g_sample_generation;
        g_result_ack_request_id = 0U;
        ClearResult();
        StatusCommit(AUTOTUNE_SESSION_STATE_ARMED, AUTOTUNE_SESSION_ABORT_NONE);
        BootIdentityCommit(AUTOTUNE_SESSION_STATE_ARMED,
                           AUTOTUNE_SESSION_BOOT_REASON_SOFTWARE_RUN);
        RetainedCommit();
        return;
    }

    if (request->request_type == AUTOTUNE_SESSION_REQUEST_START_EXPERIMENT) {
        if ((state != AUTOTUNE_SESSION_STATE_ARMED) ||
            (request->session_id != g_session_id) ||
            (request->payload[0] != g_sample_generation)) {
            RejectRequest(request, AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
            return;
        }
        g_last_request_id = request->request_id;
        g_experiment_id = request->experiment_id;
        g_autotune_session_status.start_sample_generation = g_sample_generation;
        g_sample_count = 0U;
        /* RUNNING is committed before the first sample generation changes. */
        StatusCommit(AUTOTUNE_SESSION_STATE_RUNNING,
                     AUTOTUNE_SESSION_ABORT_NONE);
        BootIdentityCommit(AUTOTUNE_SESSION_STATE_RUNNING,
                           AUTOTUNE_SESSION_BOOT_REASON_SOFTWARE_RUN);
        RetainedCommit();
        return;
    }

    if (request->request_type == AUTOTUNE_SESSION_REQUEST_ACK_RESULT) {
        if (((state != AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED) &&
             (state != AUTOTUNE_SESSION_STATE_ABORT_LATCHED)) ||
            (request->session_id != g_session_id) ||
            (request->experiment_id != g_experiment_id)) {
            RejectRequest(request, AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
            return;
        }
        g_last_request_id = request->request_id;
        g_result_ack_request_id = request->request_id;
        ClearResult();
        StatusCommit(AUTOTUNE_SESSION_STATE_READY,
                     AUTOTUNE_SESSION_ABORT_NONE);
        BootIdentityCommit(AUTOTUNE_SESSION_STATE_READY,
                           AUTOTUNE_SESSION_BOOT_REASON_SOFTWARE_RUN);
        RetainedCommit();
        return;
    }

    if (request->request_type == AUTOTUNE_SESSION_REQUEST_STOP) {
        if ((state != AUTOTUNE_SESSION_STATE_ARMED) &&
            (state != AUTOTUNE_SESSION_STATE_RUNNING)) {
            RejectRequest(request, AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
            return;
        }
        g_last_request_id = request->request_id;
        CommitResult(AUTOTUNE_SESSION_STATE_ABORT_LATCHED,
                     AUTOTUNE_SESSION_ABORT_HOST_STOP);
        StatusCommit(AUTOTUNE_SESSION_STATE_ABORT_LATCHED,
                     AUTOTUNE_SESSION_ABORT_HOST_STOP);
        BootIdentityCommit(AUTOTUNE_SESSION_STATE_ABORT_LATCHED,
                           AUTOTUNE_SESSION_BOOT_REASON_SOFTWARE_RUN);
        RetainedCommit();
        return;
    }

    RejectRequest(request, AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
}

void AutotuneSession_Init(uint32_t firmware_build_id,
                          AutotuneSession_Profile_t profile,
                          AutotuneSession_BootReason_t boot_reason)
{
    AutotuneSession_Retained_t retained = g_autotune_session_retained;
    AutotuneSession_BootRequest_t boot_request =
        g_autotune_session_boot_request;

    g_firmware_build_id = firmware_build_id;
    g_profile = profile;
    if ((boot_reason == AUTOTUNE_SESSION_BOOT_REASON_POWER_ON) ||
        (RetainedValid(&retained) == 0U)) {
        g_boot_generation = 1U;
        g_boot_count = 1U;
    } else {
        g_boot_generation = retained.boot_generation + 1U;
        if (g_boot_generation == 0U) {
            g_boot_generation = 1U;
        }
        g_boot_count = retained.boot_count + 1U;
        if (g_boot_count == 0U) {
            g_boot_count = 1U;
        }
    }
    g_last_consumed_boot_request_id =
        RetainedValid(&retained) ? retained.last_consumed_request_id : 0U;
    if (BootRequestValid(&boot_request) != 0U) {
        g_last_consumed_boot_request_id = boot_request.request_id;
    }
    g_last_request_id = 0U;
    g_session_generation = 0U;
    g_session_id = 0U;
    g_experiment_id = 0U;
    g_sample_generation = 0U;
    g_sample_count = 0U;
    g_result_ack_request_id = 0U;
    memset((void *)&g_autotune_session_boot_identity, 0,
           sizeof(g_autotune_session_boot_identity));
    memset((void *)&g_autotune_session_status, 0,
           sizeof(g_autotune_session_status));
    memset((void *)&g_autotune_session_result, 0,
           sizeof(g_autotune_session_result));
    memset((void *)g_autotune_session_samples, 0,
           sizeof(g_autotune_session_samples));
    memset((void *)&g_autotune_session_request, 0,
           sizeof(g_autotune_session_request));
    memset((void *)&g_autotune_session_boot_request, 0,
           sizeof(g_autotune_session_boot_request));

    BootIdentityCommit(AUTOTUNE_SESSION_STATE_INIT, boot_reason);
    if ((g_firmware_build_id == 0U) || (g_profile == 0U)) {
        StatusCommit(AUTOTUNE_SESSION_STATE_FAULT,
                     AUTOTUNE_SESSION_ABORT_INVALID_REQUEST);
        BootIdentityCommit(AUTOTUNE_SESSION_STATE_FAULT, boot_reason);
        return;
    }
    StatusCommit(AUTOTUNE_SESSION_STATE_READY,
                 AUTOTUNE_SESSION_ABORT_NONE);
    BootIdentityCommit(AUTOTUNE_SESSION_STATE_READY, boot_reason);
    RetainedCommit();
}

void AutotuneSession_Process(uint32_t now_ms)
{
    AutotuneSession_Request_t request;
    uint32_t state = g_autotune_session_status.state;

    if (TakeRequest(&request) != 0U) {
        HandleRequest(&request);
        state = g_autotune_session_status.state;
    }
    (void)state;
    (void)now_ms;
}
