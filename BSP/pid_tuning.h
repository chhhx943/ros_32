#ifndef __PID_TUNING_H
#define __PID_TUNING_H

#include <stdint.h>

#define PID_TUNING_PROTOCOL_VERSION 1U
#define PID_TUNING_ID_CMD_GAINS      0x123U
#define PID_TUNING_ID_CMD_D          0x124U
#define PID_TUNING_ID_FB_GAINS       0x187U
#define PID_TUNING_AXIS_LEFT         0x01U
#define PID_TUNING_AXIS_RIGHT        0x02U
#define PID_TUNING_AXIS_BOTH         0x03U
#define PID_TUNING_COMMIT_COOKIE     0xC35AU
#define PID_TUNING_MAX_GAIN          16.0f

typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Tuning_Gains_t;

typedef enum {
    PID_TUNING_STATUS_NONE = 0U,
    PID_TUNING_STATUS_ACCEPTED = 1U,
    PID_TUNING_STATUS_REJECTED_FORMAT = 2U,
    PID_TUNING_STATUS_REJECTED_UNSAFE = 3U,
    PID_TUNING_STATUS_REJECTED_TIMEOUT = 4U
} PID_Tuning_Status_t;

typedef struct {
    uint16_t transaction_seq;
    uint8_t status;
    uint8_t axis_mask;
} PID_Tuning_Feedback_t;

void PID_Tuning_Init(void);
void PID_Tuning_OnFrame(uint16_t std_id,
                        uint8_t dlc,
                        uint8_t ide,
                        uint8_t rtr,
                        const uint8_t data[8],
                        uint32_t now_ms,
                        uint8_t command_fresh,
                        uint8_t command_is_stop,
                        uint8_t command_is_zero,
                        uint8_t estop_active,
                        uint8_t fault_active);
void PID_Tuning_GetGains(PID_Tuning_Gains_t *left, PID_Tuning_Gains_t *right);
uint8_t PID_Tuning_GetFeedback(PID_Tuning_Feedback_t *feedback);
void PID_Tuning_BuildFeedbackFrame(uint8_t data[8]);

#endif
