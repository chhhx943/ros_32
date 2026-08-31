#ifndef __STEERING_BENCH_H
#define __STEERING_BENCH_H

#ifdef STEERING_BENCH_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define STEERING_BENCH_RESULT_MAGIC       0x57EEB001UL
#define STEERING_BENCH_PHASE_COUNT         5U
#define STEERING_BENCH_GROUP_PERIOD_MS    20U
#define STEERING_BENCH_HOLD_MS           1000U
#define STEERING_BENCH_TARGET_MRAD         87
#define STEERING_BENCH_CENTER_PULSE_US    1500U
#define STEERING_BENCH_POSITIVE_PULSE_US  1572U
#define STEERING_BENCH_NEGATIVE_PULSE_US  1428U
#define STEERING_BENCH_TX_WAIT_MS         5U

typedef enum {
    STEERING_BENCH_PASS = 0,
    STEERING_BENCH_ERR_DEINIT = 1,
    STEERING_BENCH_ERR_INIT = 2,
    STEERING_BENCH_PHASE_MISMATCH = 3,
    STEERING_BENCH_TX_FAILURE = 4
} Steering_BenchStatus_t;

typedef struct {
    uint32_t magic;
    uint32_t passed;
    uint32_t status;
    uint32_t phase_reached;
    uint32_t groups_sent;
    uint32_t tx_failures;
    int16_t steering_mrad[STEERING_BENCH_PHASE_COUNT];
    uint16_t pulse_us[STEERING_BENCH_PHASE_COUNT];
    uint16_t applied_seq_after[STEERING_BENCH_PHASE_COUNT];
    uint16_t fault_after[STEERING_BENCH_PHASE_COUNT];
} Steering_BenchResult_t;

extern Steering_BenchResult_t g_steering_bench_result;

void Steering_Bench_Run(void);

#ifdef __cplusplus
}
#endif

#endif
