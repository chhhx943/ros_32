#ifndef BSP_AUTOTUNE_SAFE_SESSION_H
#define BSP_AUTOTUNE_SAFE_SESSION_H

#include <stddef.h>
#include <stdint.h>

#define AUTOTUNE_SESSION_BOOT_MAGIC 0x41544249UL
#define AUTOTUNE_SESSION_REQUEST_MAGIC 0x53525154UL
#define AUTOTUNE_SESSION_STATUS_MAGIC 0x53544154UL
#define AUTOTUNE_SESSION_RESULT_FINAL_MAGIC 0x52534C54UL
#define AUTOTUNE_SESSION_SAMPLE_MAGIC 0x53504C45UL

#define AUTOTUNE_SESSION_VERSION 1U
#define AUTOTUNE_SESSION_MAILBOX_VERSION 1U
#define AUTOTUNE_SESSION_SAMPLE_CAPACITY 64U
#define AUTOTUNE_SESSION_SMOKE_SAMPLE_COUNT 32U
#define AUTOTUNE_SESSION_SMOKE_SAMPLE_PERIOD_MS 10U

typedef enum {
    AUTOTUNE_SESSION_STATE_EARLY = 0U,
    AUTOTUNE_SESSION_STATE_INIT = 1U,
    AUTOTUNE_SESSION_STATE_READY = 2U,
    AUTOTUNE_SESSION_STATE_ARMED = 3U,
    AUTOTUNE_SESSION_STATE_RUNNING = 4U,
    AUTOTUNE_SESSION_STATE_COMPLETE_LATCHED = 5U,
    AUTOTUNE_SESSION_STATE_ABORT_LATCHED = 6U,
    AUTOTUNE_SESSION_STATE_FAULT = 7U
} AutotuneSession_State_t;

typedef enum {
    AUTOTUNE_SESSION_REQUEST_NONE = 0U,
    AUTOTUNE_SESSION_REQUEST_CREATE_SESSION = 1U,
    AUTOTUNE_SESSION_REQUEST_START_EXPERIMENT = 2U,
    AUTOTUNE_SESSION_REQUEST_ACK_RESULT = 3U,
    AUTOTUNE_SESSION_REQUEST_STOP = 4U,
    AUTOTUNE_SESSION_REQUEST_RECOVER = 5U
} AutotuneSession_RequestType_t;

typedef enum {
    AUTOTUNE_SESSION_BOOT_REASON_UNKNOWN = 0U,
    AUTOTUNE_SESSION_BOOT_REASON_SOFTWARE_RUN = 1U,
    AUTOTUNE_SESSION_BOOT_REASON_POWER_ON = 2U,
    AUTOTUNE_SESSION_BOOT_REASON_WATCHDOG = 3U
} AutotuneSession_BootReason_t;

typedef enum {
    AUTOTUNE_SESSION_ABORT_NONE = 0U,
    AUTOTUNE_SESSION_ABORT_INVALID_REQUEST = 1U,
    AUTOTUNE_SESSION_ABORT_HOST_STOP = 2U,
    AUTOTUNE_SESSION_ABORT_TIMEOUT = 3U
} AutotuneSession_AbortReason_t;

typedef enum {
    AUTOTUNE_SESSION_PROFILE_LOCAL_SMOKE = 1U,
    AUTOTUNE_SESSION_PROFILE_AUTOTUNE_SAFE = 2U
} AutotuneSession_Profile_t;

typedef struct {
    uint32_t magic;
    uint32_t boot_generation;
    uint32_t boot_count;
    uint32_t last_consumed_request_id;
    uint32_t crc32;
} AutotuneSession_Retained_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t request_id;
    uint32_t request_type;
    uint32_t session_id;
    uint32_t experiment_id;
    uint32_t payload[3];
    uint32_t boot_generation_hint;
    uint32_t reserved;
    uint32_t crc32;
} AutotuneSession_BootRequest_t;

typedef struct {
    uint32_t commit_seq;
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t profile;
    uint32_t firmware_build_id;
    uint32_t boot_generation;
    uint32_t boot_reason;
    uint32_t boot_state;
    uint32_t mailbox_version;
    uint32_t boot_count;
    uint32_t crc32;
} AutotuneSession_BootIdentity_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t request_id;
    uint32_t request_type;
    uint32_t session_id;
    uint32_t experiment_id;
    uint32_t payload[3];
    uint32_t boot_generation_hint;
    uint32_t reserved;
    uint32_t crc32;
} AutotuneSession_Request_t;

typedef struct {
    uint32_t commit_seq;
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t accepted_request_id;
    uint32_t accepted_session_id;
    uint32_t active_experiment_id;
    uint32_t state;
    uint32_t boot_generation;
    uint32_t session_generation;
    uint32_t sample_generation;
    uint32_t start_sample_generation;
    uint32_t sample_count;
    uint32_t expected_sample_count;
    uint32_t last_rejected_request_id;
    uint32_t last_reject_reason;
    uint32_t result_ack_request_id;
    uint32_t status_crc32;
} AutotuneSession_Status_t;

typedef struct {
    uint32_t generation;
    uint32_t timestamp_ms;
    uint32_t session_id;
    uint32_t experiment_id;
    uint32_t sample_index;
    int32_t target;
    int32_t actual;
    int32_t pwm;
    uint32_t state;
    uint32_t flags;
} AutotuneSession_Sample_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t session_id;
    uint32_t experiment_id;
    uint32_t boot_generation;
    uint32_t session_generation;
    uint32_t start_sample_generation;
    uint32_t first_sample_generation;
    uint32_t last_sample_generation;
    uint32_t sample_count;
    uint32_t ring_start_index;
    uint32_t ring_count;
    uint32_t final_state;
    uint32_t abort_reason;
    uint32_t result_flags;
    uint32_t ring_crc32;
    uint32_t result_crc32;
} AutotuneSession_Result_t;

_Static_assert(sizeof(AutotuneSession_Retained_t) == 20U,
               "retained session layout changed");
_Static_assert(sizeof(AutotuneSession_BootRequest_t) == 48U,
               "boot request layout changed");
_Static_assert(sizeof(AutotuneSession_BootIdentity_t) == 44U,
               "boot identity layout changed");
_Static_assert(offsetof(AutotuneSession_BootIdentity_t, commit_seq) == 0U,
               "boot identity sequence offset changed");
_Static_assert(offsetof(AutotuneSession_BootIdentity_t, crc32) == 40U,
               "boot identity CRC offset changed");
_Static_assert(sizeof(AutotuneSession_Request_t) == 48U,
               "runtime request layout changed");
_Static_assert(sizeof(AutotuneSession_Status_t) == 68U,
               "status layout changed");
_Static_assert(offsetof(AutotuneSession_Status_t, commit_seq) == 0U,
               "status sequence offset changed");
_Static_assert(offsetof(AutotuneSession_Status_t, status_crc32) == 64U,
               "status CRC offset changed");
_Static_assert(sizeof(AutotuneSession_Sample_t) == 40U,
               "sample layout changed");
_Static_assert(sizeof(AutotuneSession_Result_t) == 68U,
               "result layout changed");
_Static_assert(offsetof(AutotuneSession_Result_t, result_crc32) == 64U,
               "result CRC offset changed");

#ifdef __cplusplus
extern "C" {
#endif

extern volatile AutotuneSession_Retained_t g_autotune_session_retained;
extern volatile AutotuneSession_BootRequest_t g_autotune_session_boot_request;
extern volatile AutotuneSession_BootIdentity_t g_autotune_session_boot_identity;
extern volatile AutotuneSession_Request_t g_autotune_session_request;
extern volatile AutotuneSession_Status_t g_autotune_session_status;
extern volatile AutotuneSession_Result_t g_autotune_session_result;
extern volatile AutotuneSession_Sample_t
    g_autotune_session_samples[AUTOTUNE_SESSION_SAMPLE_CAPACITY];

uint32_t AutotuneSession_Crc32(const void *data, uint32_t size);
uint32_t AutotuneSession_GenerationDelta(uint32_t newer, uint32_t older);
uint8_t AutotuneSession_GenerationIsForward(uint32_t newer, uint32_t older);

void AutotuneSession_Init(uint32_t firmware_build_id,
                          AutotuneSession_Profile_t profile,
                          AutotuneSession_BootReason_t boot_reason);
void AutotuneSession_Process(uint32_t now_ms);
uint8_t AutotuneSession_AppendSample(uint32_t now_ms, int32_t target,
                                     int32_t actual, int32_t pwm,
                                     uint32_t flags);
void AutotuneSession_Complete(void);

#ifdef __cplusplus
}
#endif

#endif
