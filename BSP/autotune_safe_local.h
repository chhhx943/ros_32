#ifndef BSP_AUTOTUNE_SAFE_LOCAL_H
#define BSP_AUTOTUNE_SAFE_LOCAL_H

#include <stdint.h>

#include "autotune_safe.h"

#define AUTOTUNE_SAFE_LOCAL_REQUEST_MAGIC 0x41544C52UL
#define AUTOTUNE_SAFE_LOCAL_RESULT_MAGIC  0x4C52534CUL
#define AUTOTUNE_SAFE_LOCAL_COMMAND_NONE  0U
#define AUTOTUNE_SAFE_LOCAL_COMMAND_START 1U
#define AUTOTUNE_SAFE_LOCAL_COMMAND_STOP  2U
#define AUTOTUNE_SAFE_LOCAL_COMMAND_RECOVER 3U
#define AUTOTUNE_SAFE_LOCAL_BOOT_MAGIC     0x41544254UL
#define AUTOTUNE_SAFE_LOCAL_SAMPLE_CAPACITY 128U
#define AUTOTUNE_SAFE_LOCAL_POINT_COUNT 4U
#define AUTOTUNE_SAFE_LOCAL_CANDIDATE_CAPACITY 6U

typedef enum {
    AUTOTUNE_SAFE_LOCAL_IDLE = 0U,
    AUTOTUNE_SAFE_LOCAL_WAIT_STILL = 1U,
    AUTOTUNE_SAFE_LOCAL_LOAD_CANDIDATE = 2U,
    AUTOTUNE_SAFE_LOCAL_RAMP_UP = 3U,
    AUTOTUNE_SAFE_LOCAL_HOLD = 4U,
    AUTOTUNE_SAFE_LOCAL_RAMP_DOWN = 5U,
    AUTOTUNE_SAFE_LOCAL_EVALUATE = 6U,
    AUTOTUNE_SAFE_LOCAL_PASS = 7U,
    AUTOTUNE_SAFE_LOCAL_ABORT = 8U,
    AUTOTUNE_SAFE_LOCAL_COOLING = 9U,
    AUTOTUNE_SAFE_LOCAL_COMPLETE = 10U,
    AUTOTUNE_SAFE_LOCAL_OPEN_LOOP_SCAN = 11U
} AutotuneSafe_LocalState_t;

typedef struct {
    uint32_t timestamp_ms;
    uint8_t wheel;
    uint8_t candidate_id;
    uint16_t reserved;
    float kp;
    float ki;
    float kd;
    int16_t target_mmps;
    int16_t actual_mmps;
    int32_t encoder_delta;
    uint32_t dt_ms;
    int32_t mcu_speed_mmps;
    int32_t recomputed_speed_mmps;
    int64_t counts_per_wheel_rev;
    int64_t circumference_mm_x1000;
    int16_t pwm;
    int16_t pid_output;
    uint8_t state;
    uint8_t stall;
    uint8_t saturation;
    uint8_t overspeed;
    uint8_t oscillation;
    uint8_t abort_reason;
} AutotuneSafe_LocalSample_t;

typedef struct {
    uint8_t valid;
    uint8_t wheel;
    uint8_t candidate_id;
    uint8_t passed;
    float kp;
    float ki;
    float kd;
    uint32_t sample_count;
    int32_t max_pwm;
    int32_t max_speed;
    int64_t error_abs_sum;
    int64_t error_sq_sum;
    int32_t steady_error_abs_sum;
    uint32_t steady_count;
    int32_t peak_to_peak;
    uint8_t abort_reason;
} AutotuneSafe_LocalSummary_t;

/* Written by a host programmer transaction before a tool-issued run/reset.
   The section is deliberately outside .bss so a debugger reset cannot erase
   the request before the application consumes it. It is accepted only once
   and is cleared before any experiment can run. */
typedef struct {
    uint32_t magic;
    uint32_t command;
    uint32_t sequence;
    uint32_t wheel;
} AutotuneSafe_LocalBootRequest_t;

typedef struct {
    volatile uint32_t request_magic;
    volatile uint32_t request_command;
    volatile uint32_t request_sequence;
    volatile uint8_t request_wheel;
    volatile uint8_t state;
    volatile uint8_t active_wheel;
    volatile uint8_t candidate_id;
    volatile uint8_t point_index;
    volatile uint8_t candidate_count;
    volatile uint8_t reserved0[3];
    volatile uint32_t status_magic;
    volatile uint32_t sample_write_index;
    volatile uint32_t total_sample_count;
    volatile uint32_t completed_candidate_count;
    volatile uint16_t open_loop_start_pwm;
    volatile uint16_t open_loop_max_pwm;
    volatile uint8_t open_loop_response;
    volatile uint8_t open_loop_completed;
    volatile uint8_t reserved1[2];
    volatile AutotuneSafe_LocalSummary_t left_summary;
    volatile AutotuneSafe_LocalSummary_t right_summary;
    volatile AutotuneSafe_LocalSummary_t candidate_summaries[2][AUTOTUNE_SAFE_LOCAL_CANDIDATE_CAPACITY];
    volatile AutotuneSafe_LocalSample_t samples[AUTOTUNE_SAFE_LOCAL_SAMPLE_CAPACITY];
} AutotuneSafe_LocalControl_t;

#ifdef __cplusplus
extern "C" {
#endif

extern volatile AutotuneSafe_LocalControl_t g_autotune_safe_local_control;
#ifdef AUTOTUNE_SAFE_PROFILE
extern volatile AutotuneSafe_LocalBootRequest_t g_autotune_safe_local_boot_request;
#endif

void AutotuneSafe_LocalExperiment_Init(void);
void AutotuneSafe_LocalExperiment_Process(uint32_t now_ms);
uint8_t AutotuneSafe_LocalExperiment_IsEngaged(void);

#ifdef __cplusplus
}
#endif

#endif
