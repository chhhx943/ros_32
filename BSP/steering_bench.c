#include "steering_bench.h"

#ifndef STEERING_BENCH_HOST_TEST

#include "can.h"
#include "can_motor_bench.h"
#include "chassis_control.h"
#include "bsp_bxcan.h"
#include "servo.h"

Steering_BenchResult_t g_steering_bench_result;

static uint16_t g_steering_bench_command_seq;
static uint32_t g_steering_bench_checks_failed;

static void Steering_Bench_ResetResult(void)
{
    uint32_t i;

    g_steering_bench_result.magic = STEERING_BENCH_RESULT_MAGIC;
    g_steering_bench_result.passed = 0U;
    g_steering_bench_result.status = (uint32_t)STEERING_BENCH_PASS;
    g_steering_bench_result.phase_reached = 0U;
    g_steering_bench_result.groups_sent = 0U;
    g_steering_bench_result.tx_failures = 0U;
    for (i = 0U; i < STEERING_BENCH_PHASE_COUNT; ++i) {
        g_steering_bench_result.steering_mrad[i] = 0;
        g_steering_bench_result.pulse_us[i] = 0U;
        g_steering_bench_result.applied_seq_after[i] = 0U;
        g_steering_bench_result.fault_after[i] = 0U;
    }
}

static uint8_t Steering_Bench_SendFrame(uint16_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox = 0U;
    uint32_t wait_start = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if ((uint32_t)(HAL_GetTick() - wait_start) > STEERING_BENCH_TX_WAIT_MS) {
            g_steering_bench_result.tx_failures++;
            return 0U;
        }
    }

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &mailbox) != HAL_OK) {
        g_steering_bench_result.tx_failures++;
        return 0U;
    }
    return 1U;
}

static uint8_t Steering_Bench_SendCommandGroup(int16_t steering_mrad)
{
    uint8_t steering[8];
    uint8_t wheels[8];
    uint16_t sequence = g_steering_bench_command_seq;

    CAN_Motor_Bench_BuildSteeringFrame(
        sequence, BSP_BXCAN_MODE_VELOCITY, steering_mrad, steering);
    CAN_Motor_Bench_BuildWheelsFrame(
        sequence, BSP_BXCAN_MODE_VELOCITY, 0, 0, wheels);
    g_steering_bench_command_seq++;

    if (Steering_Bench_SendFrame(BSP_BXCAN_ID_CMD_STEERING, steering) == 0U) {
        return 0U;
    }
    if (Steering_Bench_SendFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, wheels) == 0U) {
        return 0U;
    }
    g_steering_bench_result.groups_sent++;
    return 1U;
}

static uint16_t Steering_Bench_ExpectedPulse(int16_t steering_mrad)
{
    int32_t pulse = (int32_t)STEERING_BENCH_CENTER_PULSE_US +
                    ((int32_t)steering_mrad * 500) / 600;
    return (uint16_t)pulse;
}

static void Steering_Bench_RunPhase(uint32_t phase, int16_t steering_mrad)
{
    uint32_t hold_start;

    if (Steering_Bench_SendCommandGroup(steering_mrad) == 0U) {
        g_steering_bench_checks_failed++;
        return;
    }

    /* Allow the complete CAN group to be consumed and one 10 ms control
       period to run before sampling the PWM for this phase. */
    HAL_Delay(STEERING_BENCH_GROUP_PERIOD_MS);
    Chassis_ControlProcess(HAL_GetTick());
    g_steering_bench_result.steering_mrad[phase] = steering_mrad;
    g_steering_bench_result.pulse_us[phase] = Servo_GetPulseUs();
    g_steering_bench_result.applied_seq_after[phase] = BSP_BXCAN_GetAppliedCommandSeq();
    g_steering_bench_result.fault_after[phase] = BSP_BXCAN_GetFault();
    if ((g_steering_bench_result.pulse_us[phase] !=
         Steering_Bench_ExpectedPulse(steering_mrad)) ||
        (g_steering_bench_result.fault_after[phase] != BSP_BXCAN_FAULT_NONE)) {
        g_steering_bench_checks_failed++;
    }
    g_steering_bench_result.phase_reached = phase + 1U;

    /* Refresh both frames throughout the hold so the 100 ms command watchdog
       never neutralizes the steering while the operator observes the phase. */
    hold_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - hold_start) < STEERING_BENCH_HOLD_MS) {
        HAL_Delay(STEERING_BENCH_GROUP_PERIOD_MS);
        if (Steering_Bench_SendCommandGroup(steering_mrad) == 0U) {
            g_steering_bench_checks_failed++;
            return;
        }
        Chassis_ControlProcess(HAL_GetTick());
    }
}

static uint8_t Steering_Bench_Init(void)
{
    Steering_Bench_ResetResult();
    g_steering_bench_command_seq = 1U;
    g_steering_bench_checks_failed = 0U;

    if (HAL_CAN_DeInit(&hcan1) != HAL_OK) {
        g_steering_bench_result.status = (uint32_t)STEERING_BENCH_ERR_DEINIT;
        return 0U;
    }
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        g_steering_bench_result.status = (uint32_t)STEERING_BENCH_ERR_INIT;
        return 0U;
    }

    Chassis_ControlInit();
    return 1U;
}

void Steering_Bench_Run(void)
{
    static const int16_t phases[STEERING_BENCH_PHASE_COUNT] = {
        0,
        STEERING_BENCH_TARGET_MRAD,
        0,
        -STEERING_BENCH_TARGET_MRAD,
        0,
    };
    uint32_t phase;

    if (Steering_Bench_Init() == 0U) {
        while (1) {
            Chassis_ControlProcess(HAL_GetTick());
        }
    }

    for (phase = 0U; phase < STEERING_BENCH_PHASE_COUNT; ++phase) {
        Steering_Bench_RunPhase(phase, phases[phase]);
    }
    Chassis_ControlProcess(HAL_GetTick());

    if (g_steering_bench_result.tx_failures != 0U) {
        g_steering_bench_result.status = (uint32_t)STEERING_BENCH_TX_FAILURE;
    } else if (g_steering_bench_checks_failed != 0U) {
        g_steering_bench_result.status = (uint32_t)STEERING_BENCH_PHASE_MISMATCH;
    } else {
        g_steering_bench_result.passed = 1U;
    }

    while (1) {
        Chassis_ControlProcess(HAL_GetTick());
    }
}

#endif
