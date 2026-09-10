#ifndef __ACKERMANN_BENCH_H
#define __ACKERMANN_BENCH_H

#ifdef ACKERMANN_BENCH_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ACKERMANN_BENCH_RESULT_MAGIC        0xACCEB001UL
#define ACKERMANN_BENCH_PHASE_COUNT         5U
#define ACKERMANN_BENCH_GROUP_PERIOD_MS     20U
#define ACKERMANN_BENCH_HOLD_MS             1000U
#define ACKERMANN_BENCH_BODY_SPEED_MMPS    200
#define ACKERMANN_BENCH_TURN_STEERING_MRAD 200
#define ACKERMANN_BENCH_TURN_LEFT_MMPS      183
#define ACKERMANN_BENCH_TURN_RIGHT_MMPS     217
#define ACKERMANN_BENCH_TX_WAIT_MS          5U

typedef enum {
    ACKERMANN_BENCH_PASS = 0,
    ACKERMANN_BENCH_ERR_DEINIT = 1,
    ACKERMANN_BENCH_ERR_INIT = 2,
    ACKERMANN_BENCH_PHASE_MISMATCH = 3,
    ACKERMANN_BENCH_TX_FAILURE = 4,
    ACKERMANN_BENCH_ESTOP = 5
} Ackermann_BenchStatus_t;

typedef struct {
    uint32_t magic;
    uint32_t passed;
    uint32_t status;
    uint32_t phase_reached;
    uint32_t groups_sent;
    uint32_t tx_failures;
    int16_t steering_mrad[ACKERMANN_BENCH_PHASE_COUNT];
    int16_t left_target_mmps[ACKERMANN_BENCH_PHASE_COUNT];
    int16_t right_target_mmps[ACKERMANN_BENCH_PHASE_COUNT];
    int16_t actual_left_mmps[ACKERMANN_BENCH_PHASE_COUNT];
    int16_t actual_right_mmps[ACKERMANN_BENCH_PHASE_COUNT];
    uint16_t applied_seq_after[ACKERMANN_BENCH_PHASE_COUNT];
    uint16_t fault_after[ACKERMANN_BENCH_PHASE_COUNT];
    uint32_t left_ccr_after[ACKERMANN_BENCH_PHASE_COUNT];
    uint32_t right_ccr_after[ACKERMANN_BENCH_PHASE_COUNT];
    int32_t left_encoder_delta_after[ACKERMANN_BENCH_PHASE_COUNT];
    int32_t right_encoder_delta_after[ACKERMANN_BENCH_PHASE_COUNT];
    uint8_t tb6612_pins_after[ACKERMANN_BENCH_PHASE_COUNT];
} Ackermann_BenchResult_t;

extern Ackermann_BenchResult_t g_ackermann_bench_result;

void Ackermann_Bench_Run(void);

#ifdef __cplusplus
}
#endif

#endif
