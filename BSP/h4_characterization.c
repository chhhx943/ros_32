#include "h4_characterization.h"

#ifndef H4_CHARACTERIZATION_HOST_TEST

#include "bsp_bxcan.h"
#include "bsp_motor.h"
#include "can.h"
#include "can_motor_bench.h"
#include "chassis_control.h"
#include "encoder.h"
#include "physical_estop.h"
#include "safety_manager.h"
#include "tim.h"
#include "watchdog.h"

#define H4_HOLD_MS             1200U
#define H4_SAMPLE_PERIOD_MS    20U
#ifndef H4_OPEN_LOOP_PWM
#define H4_OPEN_LOOP_PWM       500
#endif
#define H4_TX_WAIT_MS          5U

H4_CharacterizationResult_t g_h4_characterization_result;

static uint16_t g_h4_command_seq;
static uint32_t g_h4_failures;
static uint32_t g_h4_tx_failures;

static uint8_t H4_ReadTb6612Pins(void)
{
    uint8_t pins = 0U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) pins |= 0x01U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) pins |= 0x02U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) pins |= 0x04U;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) pins |= 0x08U;
    return pins;
}

static void H4_ResetResult(void)
{
    g_h4_characterization_result = (H4_CharacterizationResult_t){0};
    g_h4_characterization_result.magic = H4_CHARACTERIZATION_RESULT_MAGIC;
    g_h4_command_seq = 1U;
    g_h4_failures = 0U;
    g_h4_tx_failures = 0U;
}

static uint8_t H4_SendFrame(uint16_t id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox = 0U;
    uint32_t start = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if ((uint32_t)(HAL_GetTick() - start) > H4_TX_WAIT_MS) {
            g_h4_tx_failures++;
            return 0U;
        }
    }
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox) != HAL_OK) {
        g_h4_tx_failures++;
        return 0U;
    }
    return 1U;
}

static uint8_t H4_SendGroup(int16_t steering_mrad,
                            int16_t left_mmps,
                            int16_t right_mmps,
                            uint16_t *sequence_out)
{
    uint8_t steering[8];
    uint8_t wheels[8];
    uint16_t sequence = g_h4_command_seq++;

    uint8_t mode = ((left_mmps == 0) && (right_mmps == 0) && (steering_mrad == 0))
                       ? BSP_BXCAN_MODE_STOP : BSP_BXCAN_MODE_VELOCITY;

    CAN_Motor_Bench_BuildSteeringFrame(sequence, mode,
                                       steering_mrad, steering);
    CAN_Motor_Bench_BuildWheelsFrame(sequence, mode,
                                     left_mmps, right_mmps, wheels);
    if ((H4_SendFrame(BSP_BXCAN_ID_CMD_STEERING, steering) == 0U) ||
        (H4_SendFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, wheels) == 0U)) {
        return 0U;
    }
    g_h4_characterization_result.groups_sent++;
    if (sequence_out != 0) {
        *sequence_out = sequence;
    }
    return 1U;
}

static uint8_t H4_EstopOrFault(void)
{
    if (Physical_EStop_IsAsserted() != 0U) {
        Motor_EmergencyBrakeAll();
        g_h4_failures++;
        return 1U;
    }
    if (BSP_BXCAN_GetFault() != BSP_BXCAN_FAULT_NONE) {
        Motor_CoastAll();
        g_h4_failures++;
        return 1U;
    }
    return 0U;
}

static void H4_Record(uint32_t phase,
                      uint16_t sequence,
                      int16_t target_left,
                      int16_t target_right,
                      int32_t left_delta,
                      int32_t right_delta,
                      const Chassis_ControlTelemetry_t *telemetry)
{
    H4_CharacterizationResult_t *result = &g_h4_characterization_result;

    result->timestamp_ms[phase] = HAL_GetTick();
    result->command_seq[phase] = sequence;
    result->target_left_mmps[phase] = target_left;
    result->target_right_mmps[phase] = target_right;
    result->encoder_delta_left[phase] = left_delta;
    result->encoder_delta_right[phase] = right_delta;
    result->pwm_left[phase] = 0;
    result->pwm_right[phase] = 0;
    if (telemetry != 0) {
        result->actual_left_mmps[phase] = telemetry->actual_left_mmps;
        result->actual_right_mmps[phase] = telemetry->actual_right_mmps;
        result->pwm_left[phase] = telemetry->pwm_left;
        result->pwm_right[phase] = telemetry->pwm_right;
        result->pid_left_p[phase] = telemetry->pid_left_p;
        result->pid_left_i[phase] = telemetry->pid_left_i;
        result->pid_left_d[phase] = telemetry->pid_left_d;
        result->pid_left_output[phase] = telemetry->pid_left_output;
        result->pid_right_p[phase] = telemetry->pid_right_p;
        result->pid_right_i[phase] = telemetry->pid_right_i;
        result->pid_right_d[phase] = telemetry->pid_right_d;
        result->pid_right_output[phase] = telemetry->pid_right_output;
    }
    result->tim3_ccr_left[phase] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    result->tim3_ccr_right[phase] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    result->tb6612_pins[phase] = H4_ReadTb6612Pins();
    result->safety_state[phase] = (uint8_t)Safety_Manager_GetState();
    result->fault_code[phase] = Safety_Manager_GetFault();
    result->phase_reached = phase + 1U;
}

static void H4_RunOpenLoopPhase(uint32_t phase, int16_t left_target, int16_t right_target)
{
    EncoderSample_t left_end = {0};
    EncoderSample_t right_end = {0};
    EncoderSample_t left_steady = {0};
    EncoderSample_t right_steady = {0};
    Chassis_ControlTelemetry_t telemetry = {0};
    uint32_t start;

    Motor_CoastAll();
    HAL_Delay(100U);
    Encoder_Reset();
    (void)Encoder_Sample(1U, H4_SAMPLE_PERIOD_MS);
    (void)Encoder_Sample(2U, H4_SAMPLE_PERIOD_MS);
    if (left_target != 0) {
        Motor_Drive(1U, (left_target > 0) ? H4_OPEN_LOOP_PWM : -H4_OPEN_LOOP_PWM);
    }
    if (right_target != 0) {
        Motor_Drive(2U, (right_target > 0) ? H4_OPEN_LOOP_PWM : -H4_OPEN_LOOP_PWM);
    }
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < H4_HOLD_MS) {
        HAL_Delay(H4_SAMPLE_PERIOD_MS);
        Safety_Manager_Process(HAL_GetTick());
        if (H4_EstopOrFault() != 0U) {
            return;
        }
        left_end = Encoder_Sample(1U, H4_SAMPLE_PERIOD_MS);
        right_end = Encoder_Sample(2U, H4_SAMPLE_PERIOD_MS);
        BSP_Watchdog_Feed();
    }
    left_steady = left_end;
    right_steady = right_end;
    (void)Encoder_Sample(1U, H4_SAMPLE_PERIOD_MS);
    (void)Encoder_Sample(2U, H4_SAMPLE_PERIOD_MS);
    telemetry.timestamp_ms = HAL_GetTick();
    telemetry.target_left_mmps = left_target;
    telemetry.target_right_mmps = right_target;
    telemetry.actual_left_mmps = (int16_t)left_steady.velocity_mmps;
    telemetry.actual_right_mmps = (int16_t)right_steady.velocity_mmps;
    telemetry.encoder_delta_left = left_steady.delta_counts;
    telemetry.encoder_delta_right = right_steady.delta_counts;
    telemetry.pwm_left = (left_target == 0) ? 0 : ((left_target > 0) ? H4_OPEN_LOOP_PWM : -H4_OPEN_LOOP_PWM);
    telemetry.pwm_right = (right_target == 0) ? 0 : ((right_target > 0) ? H4_OPEN_LOOP_PWM : -H4_OPEN_LOOP_PWM);
    telemetry.safety_state = (uint8_t)Safety_Manager_GetState();
    telemetry.fault_code = Safety_Manager_GetFault();
    H4_Record(phase, 0U, left_target, right_target,
              left_steady.delta_counts,
              right_steady.delta_counts,
              &telemetry);
    Motor_CoastAll();
}

static void H4_RunClosedLoopPhase(uint32_t phase,
                                  int16_t steering_mrad,
                                  int16_t left_target,
                                  int16_t right_target)
{
    Chassis_ControlTelemetry_t telemetry;
    uint16_t last_sequence = 0U;
    uint32_t start;

    Motor_CoastAll();
    (void)H4_SendGroup(0, 0, 0, 0);
    Chassis_ControlProcess(HAL_GetTick());
    for (uint32_t idle = 0U; idle < 100U; idle += H4_SAMPLE_PERIOD_MS) {
        HAL_Delay(H4_SAMPLE_PERIOD_MS);
        Chassis_ControlProcess(HAL_GetTick());
        BSP_Watchdog_Feed();
    }
    Encoder_Reset();
    (void)Encoder_Sample(1U, H4_SAMPLE_PERIOD_MS);
    (void)Encoder_Sample(2U, H4_SAMPLE_PERIOD_MS);
    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < H4_HOLD_MS) {
        if (H4_SendGroup(steering_mrad, left_target, right_target, &last_sequence) == 0U) {
            g_h4_failures++;
            return;
        }
        HAL_Delay(H4_SAMPLE_PERIOD_MS);
        Chassis_ControlProcess(HAL_GetTick());
        if (H4_EstopOrFault() != 0U) {
            return;
        }
        BSP_Watchdog_Feed();
    }
    Chassis_Control_GetTelemetry(&telemetry);
    H4_Record(phase, last_sequence, left_target, right_target,
              telemetry.encoder_delta_left,
              telemetry.encoder_delta_right,
              &telemetry);
    (void)H4_SendGroup(0, 0, 0, &last_sequence);
    Chassis_ControlProcess(HAL_GetTick());
}

static uint8_t H4_Init(void)
{
    H4_ResetResult();
    if (HAL_CAN_DeInit(&hcan1) != HAL_OK) {
        return 0U;
    }
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        return 0U;
    }
    Chassis_ControlInit();
    return (Physical_EStop_IsAsserted() == 0U) ? 1U : 0U;
}

void H4_Characterization_Run(void)
{
    if (H4_Init() == 0U) {
        Motor_EmergencyBrakeAll();
        g_h4_characterization_result.status = 1U;
        return;
    }

    H4_RunOpenLoopPhase(H4_PHASE_OPEN_LEFT, H4_CHARACTERIZATION_BODY_SPEED_MMPS, 0);
    H4_RunOpenLoopPhase(H4_PHASE_OPEN_RIGHT, 0, H4_CHARACTERIZATION_BODY_SPEED_MMPS);
    H4_RunOpenLoopPhase(H4_PHASE_OPEN_BOTH, H4_CHARACTERIZATION_BODY_SPEED_MMPS,
                        H4_CHARACTERIZATION_BODY_SPEED_MMPS);
    H4_RunClosedLoopPhase(H4_PHASE_CLOSED_BOTH, 0, 200, 200);
    H4_RunClosedLoopPhase(H4_PHASE_CLOSED_SMALL_LEFT, 0, 180, 220);
    H4_RunClosedLoopPhase(H4_PHASE_CLOSED_SMALL_RIGHT, 0, 220, 180);
    H4_RunClosedLoopPhase(H4_PHASE_ACKERMANN_LEFT_INNER, 200, 150, 250);
    H4_RunClosedLoopPhase(H4_PHASE_ACKERMANN_RIGHT_INNER, -200, 250, 150);

    Motor_CoastAll();
    g_h4_characterization_result.tx_failures = g_h4_tx_failures;
    g_h4_characterization_result.status = (g_h4_failures == 0U) ? 0U : 2U;
    g_h4_characterization_result.passed =
        (g_h4_failures == 0U) && (g_h4_tx_failures == 0U) &&
        (g_h4_characterization_result.phase_reached == H4_CHARACTERIZATION_PHASE_COUNT);
}

#endif
