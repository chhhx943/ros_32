#include "ackermann_bench.h"

#ifndef ACKERMANN_BENCH_HOST_TEST

#include "bsp_bxcan.h"
#include "bsp_motor.h"
#include "can.h"
#include "can_motor_bench.h"
#include "chassis_control.h"
#include "encoder.h"
#include "physical_estop.h"

#define ACKERMANN_BENCH_PIN_LEFT_IN1   0x01U
#define ACKERMANN_BENCH_PIN_LEFT_IN2   0x02U
#define ACKERMANN_BENCH_PIN_RIGHT_IN1  0x04U
#define ACKERMANN_BENCH_PIN_RIGHT_IN2  0x08U

Ackermann_BenchResult_t g_ackermann_bench_result;

static uint16_t g_ackermann_bench_command_seq;
static uint32_t g_ackermann_bench_checks_failed;

static void Ackermann_Bench_ResetResult(void)
{
    uint32_t i;

    g_ackermann_bench_result.magic = ACKERMANN_BENCH_RESULT_MAGIC;
    g_ackermann_bench_result.passed = 0U;
    g_ackermann_bench_result.status = (uint32_t)ACKERMANN_BENCH_PASS;
    g_ackermann_bench_result.phase_reached = 0U;
    g_ackermann_bench_result.groups_sent = 0U;
    g_ackermann_bench_result.tx_failures = 0U;
    for (i = 0U; i < ACKERMANN_BENCH_PHASE_COUNT; ++i) {
        g_ackermann_bench_result.steering_mrad[i] = 0;
        g_ackermann_bench_result.left_target_mmps[i] = 0;
        g_ackermann_bench_result.right_target_mmps[i] = 0;
        g_ackermann_bench_result.actual_left_mmps[i] = 0;
        g_ackermann_bench_result.actual_right_mmps[i] = 0;
        g_ackermann_bench_result.applied_seq_after[i] = 0U;
        g_ackermann_bench_result.fault_after[i] = 0U;
        g_ackermann_bench_result.left_ccr_after[i] = 0U;
        g_ackermann_bench_result.right_ccr_after[i] = 0U;
        g_ackermann_bench_result.left_encoder_delta_after[i] = 0;
        g_ackermann_bench_result.right_encoder_delta_after[i] = 0;
        g_ackermann_bench_result.tb6612_pins_after[i] = 0U;
    }
}

static uint8_t Ackermann_Bench_SendFrame(uint16_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox = 0U;
    uint32_t wait_start = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if ((uint32_t)(HAL_GetTick() - wait_start) > ACKERMANN_BENCH_TX_WAIT_MS) {
            g_ackermann_bench_result.tx_failures++;
            return 0U;
        }
    }

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &mailbox) != HAL_OK) {
        g_ackermann_bench_result.tx_failures++;
        return 0U;
    }
    return 1U;
}

static uint8_t Ackermann_Bench_SendCommandGroup(int16_t steering_mrad,
                                                int16_t left_mmps,
                                                int16_t right_mmps)
{
    uint8_t steering[8];
    uint8_t wheels[8];
    uint16_t sequence = g_ackermann_bench_command_seq++;

    CAN_Motor_Bench_BuildSteeringFrame(
        sequence, BSP_BXCAN_MODE_VELOCITY, steering_mrad, steering);
    CAN_Motor_Bench_BuildWheelsFrame(
        sequence, BSP_BXCAN_MODE_VELOCITY, left_mmps, right_mmps, wheels);
    if (Ackermann_Bench_SendFrame(BSP_BXCAN_ID_CMD_STEERING, steering) == 0U) {
        return 0U;
    }
    if (Ackermann_Bench_SendFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, wheels) == 0U) {
        return 0U;
    }
    g_ackermann_bench_result.groups_sent++;
    return 1U;
}

static uint8_t Ackermann_Bench_ReadPins(void)
{
    uint8_t pins = 0U;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
        pins |= ACKERMANN_BENCH_PIN_LEFT_IN1;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) {
        pins |= ACKERMANN_BENCH_PIN_LEFT_IN2;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) {
        pins |= ACKERMANN_BENCH_PIN_RIGHT_IN1;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) {
        pins |= ACKERMANN_BENCH_PIN_RIGHT_IN2;
    }
    return pins;
}

static uint8_t Ackermann_Bench_Init(void)
{
    Ackermann_Bench_ResetResult();
    g_ackermann_bench_command_seq = 1U;
    g_ackermann_bench_checks_failed = 0U;

    if (HAL_CAN_DeInit(&hcan1) != HAL_OK) {
        g_ackermann_bench_result.status = (uint32_t)ACKERMANN_BENCH_ERR_DEINIT;
        return 0U;
    }
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        g_ackermann_bench_result.status = (uint32_t)ACKERMANN_BENCH_ERR_INIT;
        return 0U;
    }
    Chassis_ControlInit();
    if (Physical_EStop_IsAsserted() != 0U) {
        g_ackermann_bench_result.status = (uint32_t)ACKERMANN_BENCH_ESTOP;
        return 0U;
    }
    return 1U;
}

static void Ackermann_Bench_RunPhase(uint32_t phase,
                                    int16_t steering_mrad,
                                    int16_t left_target,
                                    int16_t right_target)
{
    EncoderSample_t left_start;
    EncoderSample_t right_start;
    EncoderSample_t left_end;
    EncoderSample_t right_end;
    uint32_t hold_start;
    uint8_t expected_pins = ACKERMANN_BENCH_PIN_LEFT_IN1 |
                            ACKERMANN_BENCH_PIN_RIGHT_IN2;
    uint8_t expect_drive = (left_target != 0) || (right_target != 0);

    left_start = Encoder_Sample(1U, ACKERMANN_BENCH_GROUP_PERIOD_MS);
    right_start = Encoder_Sample(2U, ACKERMANN_BENCH_GROUP_PERIOD_MS);
    if (Ackermann_Bench_SendCommandGroup(steering_mrad, left_target, right_target) == 0U) {
        g_ackermann_bench_checks_failed++;
        return;
    }

    HAL_Delay(ACKERMANN_BENCH_GROUP_PERIOD_MS);
    Chassis_ControlProcess(HAL_GetTick());
    hold_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - hold_start) < ACKERMANN_BENCH_HOLD_MS) {
        HAL_Delay(ACKERMANN_BENCH_GROUP_PERIOD_MS);
        if (Physical_EStop_IsAsserted() != 0U) {
            g_ackermann_bench_result.status = (uint32_t)ACKERMANN_BENCH_ESTOP;
            Motor_EmergencyBrakeAll();
            g_ackermann_bench_checks_failed++;
            return;
        }
        if (Ackermann_Bench_SendCommandGroup(steering_mrad, left_target, right_target) == 0U) {
            g_ackermann_bench_checks_failed++;
            return;
        }
        Chassis_ControlProcess(HAL_GetTick());
    }

    Chassis_ControlProcess(HAL_GetTick());
    left_end = Encoder_Sample(1U, ACKERMANN_BENCH_GROUP_PERIOD_MS);
    right_end = Encoder_Sample(2U, ACKERMANN_BENCH_GROUP_PERIOD_MS);
    g_ackermann_bench_result.steering_mrad[phase] = steering_mrad;
    g_ackermann_bench_result.left_target_mmps[phase] = left_target;
    g_ackermann_bench_result.right_target_mmps[phase] = right_target;
    g_ackermann_bench_result.actual_left_mmps[phase] = (int16_t)left_end.velocity_mmps;
    g_ackermann_bench_result.actual_right_mmps[phase] = (int16_t)right_end.velocity_mmps;
    g_ackermann_bench_result.applied_seq_after[phase] = BSP_BXCAN_GetAppliedCommandSeq();
    g_ackermann_bench_result.fault_after[phase] = BSP_BXCAN_GetFault();
    g_ackermann_bench_result.left_ccr_after[phase] = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    g_ackermann_bench_result.right_ccr_after[phase] = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    g_ackermann_bench_result.left_encoder_delta_after[phase] =
        (int32_t)(left_end.accumulated_counts - left_start.accumulated_counts);
    g_ackermann_bench_result.right_encoder_delta_after[phase] =
        (int32_t)(right_end.accumulated_counts - right_start.accumulated_counts);
    g_ackermann_bench_result.tb6612_pins_after[phase] = Ackermann_Bench_ReadPins();
    g_ackermann_bench_result.phase_reached = phase + 1U;

    if (g_ackermann_bench_result.fault_after[phase] != BSP_BXCAN_FAULT_NONE) {
        g_ackermann_bench_checks_failed++;
    }
    if (expect_drive == 0U) {
        if ((g_ackermann_bench_result.left_ccr_after[phase] != 0U) ||
            (g_ackermann_bench_result.right_ccr_after[phase] != 0U) ||
            (g_ackermann_bench_result.tb6612_pins_after[phase] != 0U)) {
            g_ackermann_bench_checks_failed++;
        }
    } else {
        if ((g_ackermann_bench_result.left_ccr_after[phase] == 0U) ||
            (g_ackermann_bench_result.right_ccr_after[phase] == 0U) ||
            (g_ackermann_bench_result.tb6612_pins_after[phase] != expected_pins) ||
            (g_ackermann_bench_result.left_encoder_delta_after[phase] <= 0) ||
            (g_ackermann_bench_result.right_encoder_delta_after[phase] <= 0)) {
            g_ackermann_bench_checks_failed++;
        }
    }
    if ((steering_mrad > 0) &&
        (g_ackermann_bench_result.left_encoder_delta_after[phase] >=
         g_ackermann_bench_result.right_encoder_delta_after[phase])) {
        g_ackermann_bench_checks_failed++;
    }
    if ((steering_mrad < 0) &&
        (g_ackermann_bench_result.right_encoder_delta_after[phase] >=
         g_ackermann_bench_result.left_encoder_delta_after[phase])) {
        g_ackermann_bench_checks_failed++;
    }
}

void Ackermann_Bench_Run(void)
{
    static const int16_t steering[ACKERMANN_BENCH_PHASE_COUNT] = {
        0, 0, ACKERMANN_BENCH_TURN_STEERING_MRAD,
        -ACKERMANN_BENCH_TURN_STEERING_MRAD, 0
    };
    static const int16_t left_target[ACKERMANN_BENCH_PHASE_COUNT] = {
        0, ACKERMANN_BENCH_BODY_SPEED_MMPS, ACKERMANN_BENCH_TURN_LEFT_MMPS,
        ACKERMANN_BENCH_TURN_RIGHT_MMPS, 0
    };
    static const int16_t right_target[ACKERMANN_BENCH_PHASE_COUNT] = {
        0, ACKERMANN_BENCH_BODY_SPEED_MMPS, ACKERMANN_BENCH_TURN_RIGHT_MMPS,
        ACKERMANN_BENCH_TURN_LEFT_MMPS, 0
    };
    uint32_t phase;

    if (Ackermann_Bench_Init() == 0U) {
        Motor_EmergencyBrakeAll();
        while (1) {
            Chassis_ControlProcess(HAL_GetTick());
        }
    }

    for (phase = 0U; phase < ACKERMANN_BENCH_PHASE_COUNT; ++phase) {
        Ackermann_Bench_RunPhase(phase, steering[phase], left_target[phase], right_target[phase]);
    }

    (void)Ackermann_Bench_SendCommandGroup(0, 0, 0);
    Chassis_ControlProcess(HAL_GetTick());
    if (g_ackermann_bench_result.status == (uint32_t)ACKERMANN_BENCH_ESTOP) {
        Motor_EmergencyBrakeAll();
    } else if (g_ackermann_bench_result.tx_failures != 0U) {
        g_ackermann_bench_result.status = (uint32_t)ACKERMANN_BENCH_TX_FAILURE;
    } else if (g_ackermann_bench_checks_failed != 0U) {
        g_ackermann_bench_result.status = (uint32_t)ACKERMANN_BENCH_PHASE_MISMATCH;
    } else {
        g_ackermann_bench_result.passed = 1U;
    }

    while (1) {
        Chassis_ControlProcess(HAL_GetTick());
    }
}

#endif
