#ifndef __H4_CHARACTERIZATION_H
#define __H4_CHARACTERIZATION_H

#ifdef H4_CHARACTERIZATION_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define H4_CHARACTERIZATION_RESULT_MAGIC 0x48340001UL
#define H4_CHARACTERIZATION_PHASE_COUNT  8U
#define H4_CHARACTERIZATION_BODY_SPEED_MMPS 200

typedef enum {
    H4_PHASE_OPEN_LEFT = 0,
    H4_PHASE_OPEN_RIGHT = 1,
    H4_PHASE_OPEN_BOTH = 2,
    H4_PHASE_CLOSED_BOTH = 3,
    H4_PHASE_CLOSED_SMALL_LEFT = 4,
    H4_PHASE_CLOSED_SMALL_RIGHT = 5,
    H4_PHASE_ACKERMANN_LEFT_INNER = 6,
    H4_PHASE_ACKERMANN_RIGHT_INNER = 7
} H4_CharacterizationPhase_t;

typedef struct {
    uint32_t magic;
    uint32_t passed;
    uint32_t status;
    uint32_t phase_reached;
    uint32_t groups_sent;
    uint32_t tx_failures;
    uint32_t timestamp_ms[H4_CHARACTERIZATION_PHASE_COUNT];
    uint16_t command_seq[H4_CHARACTERIZATION_PHASE_COUNT];
    int16_t target_left_mmps[H4_CHARACTERIZATION_PHASE_COUNT];
    int16_t target_right_mmps[H4_CHARACTERIZATION_PHASE_COUNT];
    int16_t actual_left_mmps[H4_CHARACTERIZATION_PHASE_COUNT];
    int16_t actual_right_mmps[H4_CHARACTERIZATION_PHASE_COUNT];
    int32_t encoder_delta_left[H4_CHARACTERIZATION_PHASE_COUNT];
    int32_t encoder_delta_right[H4_CHARACTERIZATION_PHASE_COUNT];
    int16_t pwm_left[H4_CHARACTERIZATION_PHASE_COUNT];
    int16_t pwm_right[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_left_p[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_left_i[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_left_d[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_left_output[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_right_p[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_right_i[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_right_d[H4_CHARACTERIZATION_PHASE_COUNT];
    float pid_right_output[H4_CHARACTERIZATION_PHASE_COUNT];
    uint16_t tim3_ccr_left[H4_CHARACTERIZATION_PHASE_COUNT];
    uint16_t tim3_ccr_right[H4_CHARACTERIZATION_PHASE_COUNT];
    uint8_t tb6612_pins[H4_CHARACTERIZATION_PHASE_COUNT];
    uint8_t safety_state[H4_CHARACTERIZATION_PHASE_COUNT];
    uint16_t fault_code[H4_CHARACTERIZATION_PHASE_COUNT];
} H4_CharacterizationResult_t;

extern H4_CharacterizationResult_t g_h4_characterization_result;

void H4_Characterization_Run(void);

#ifdef __cplusplus
}
#endif

#endif
