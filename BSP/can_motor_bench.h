#ifndef __CAN_MOTOR_BENCH_H
#define __CAN_MOTOR_BENCH_H

#ifdef CAN_MOTOR_BENCH_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#include "can.h"
#include "tim.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Protocol V1 is frozen in docs/CAN_PROTOCOL.md. */
#define CAN_MOTOR_BENCH_PROTOCOL_VERSION    0x01U

#define CAN_MOTOR_BENCH_RESULT_MAGIC        0xCA2B0B01UL
#define CAN_MOTOR_BENCH_PHASE_COUNT         11U
#define CAN_MOTOR_BENCH_GROUP_PERIOD_MS     20U
#define CAN_MOTOR_BENCH_TARGET_MMPS         600

#define CAN_MOTOR_BENCH_DRIVE_MS            2000U
#define CAN_MOTOR_BENCH_STOP_MS             500U
#define CAN_MOTOR_BENCH_REVERSE_MS          2000U
#define CAN_MOTOR_BENCH_WATCHDOG_MS         300U
#define CAN_MOTOR_BENCH_RECOVERY_DRIVE_MS   200U
#define CAN_MOTOR_BENCH_PHYSICAL_WAIT_MS    20000U
#define CAN_MOTOR_BENCH_PHYSICAL_RELEASE_MS 10000U

/* Loopback self-injection shares the 3 TX mailboxes with the production
 * feedback pump, which burst-fills them right before each command group.
 * A frame waits at most this long for a free mailbox (one feedback burst
 * needs ~0.7 ms at 1 Mbit/s, so 5 ms is a generous bound). */
#define CAN_MOTOR_BENCH_TX_WAIT_MS          5U

typedef enum {
    CAN_MOTOR_BENCH_PASS = 0,
    CAN_MOTOR_BENCH_ERR_DEINIT = 1,
    CAN_MOTOR_BENCH_ERR_INIT = 2,
    CAN_MOTOR_BENCH_PHASE_MISMATCH = 3
} CAN_Motor_BenchStatus_t;

typedef struct {
    uint32_t magic;
    uint32_t passed;
    uint32_t status;
    uint32_t phase_reached;
    uint32_t groups_sent;
    uint32_t tx_failures;
    uint16_t applied_seq_after[CAN_MOTOR_BENCH_PHASE_COUNT];
    uint16_t fault_after[CAN_MOTOR_BENCH_PHASE_COUNT];
    uint32_t left_ccr_after[CAN_MOTOR_BENCH_PHASE_COUNT];
    uint32_t right_ccr_after[CAN_MOTOR_BENCH_PHASE_COUNT];
    int32_t left_encoder_delta_after[CAN_MOTOR_BENCH_PHASE_COUNT];
    int32_t right_encoder_delta_after[CAN_MOTOR_BENCH_PHASE_COUNT];
    uint8_t tb6612_pins_after[CAN_MOTOR_BENCH_PHASE_COUNT];
    uint32_t calibration_state;
    uint32_t calibration_stage;
    uint32_t calibration_exit_reason;
    int32_t calibration_left_delta;
    int32_t calibration_right_delta;
} CAN_Motor_BenchResult_t;

extern CAN_Motor_BenchResult_t g_can_motor_bench_result;

void CAN_Motor_Bench_Run(void);
void CAN_Motor_Bench_RunCalibration(void);

void CAN_Motor_Bench_BuildSteeringFrame(uint16_t command_seq,
                                        uint8_t mode_flags,
                                        int16_t steering_mrad,
                                        uint8_t data[8]);
void CAN_Motor_Bench_BuildWheelsFrame(uint16_t command_seq,
                                      uint8_t mode_flags,
                                      int16_t left_velocity_mmps,
                                      int16_t right_velocity_mmps,
                                      uint8_t data[8]);
void CAN_Motor_Bench_BuildCalibrationFrame(uint16_t service_seq,
                                           uint8_t opcode,
                                           uint8_t options,
                                           uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif
