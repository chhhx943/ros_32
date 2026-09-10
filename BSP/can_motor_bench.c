#include "can_motor_bench.h"

/* TB6612 pin encoding used by tb6612_pins_after[]: bit0=PB12 bit1=PB13 bit2=PB14 bit3=PB15. */
#define CAN_MOTOR_BENCH_PIN_LEFT_IN1   0x01U
#define CAN_MOTOR_BENCH_PIN_LEFT_IN2   0x02U
#define CAN_MOTOR_BENCH_PIN_RIGHT_IN1  0x04U
#define CAN_MOTOR_BENCH_PIN_RIGHT_IN2  0x08U
#define CAN_MOTOR_BENCH_PIN_BRAKE_ALL  0x0FU

static void CAN_Motor_Bench_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CAN_Motor_Bench_WriteI16LE(uint8_t *data, int16_t value)
{
    CAN_Motor_Bench_WriteU16LE(data, (uint16_t)value);
}

void CAN_Motor_Bench_BuildSteeringFrame(uint16_t command_seq,
                                        uint8_t mode_flags,
                                        int16_t steering_mrad,
                                        uint8_t data[8])
{
    data[0] = CAN_MOTOR_BENCH_PROTOCOL_VERSION;
    CAN_Motor_Bench_WriteU16LE(&data[1], command_seq);
    data[3] = mode_flags;
    CAN_Motor_Bench_WriteI16LE(&data[4], steering_mrad);
    data[6] = 0U;
    data[7] = 0U;
}

void CAN_Motor_Bench_BuildWheelsFrame(uint16_t command_seq,
                                      uint8_t mode_flags,
                                      int16_t left_velocity_mmps,
                                      int16_t right_velocity_mmps,
                                      uint8_t data[8])
{
    data[0] = CAN_MOTOR_BENCH_PROTOCOL_VERSION;
    CAN_Motor_Bench_WriteU16LE(&data[1], command_seq);
    data[3] = mode_flags;
    CAN_Motor_Bench_WriteI16LE(&data[4], left_velocity_mmps);
    CAN_Motor_Bench_WriteI16LE(&data[6], right_velocity_mmps);
}

void CAN_Motor_Bench_BuildCalibrationFrame(uint16_t service_seq,
                                           uint8_t opcode,
                                           uint8_t options,
                                           uint8_t data[8])
{
    data[0] = CAN_MOTOR_BENCH_PROTOCOL_VERSION;
    CAN_Motor_Bench_WriteU16LE(&data[1], service_seq);
    data[3] = opcode;
    data[4] = options;
    CAN_Motor_Bench_WriteU16LE(&data[5], 0xC35AU);
    data[7] = 0U;
}

#ifndef CAN_MOTOR_BENCH_HOST_TEST

#include "bsp_bxcan.h"
#include "chassis_control.h"
#include "bsp_motor.h"
#include "encoder.h"
#include "physical_estop.h"
#include "watchdog.h"
#include "wheel_calibration.h"
#include "wheel_calibration_service.h"

CAN_Motor_BenchResult_t g_can_motor_bench_result;

static uint16_t g_bench_command_seq;
static uint32_t g_bench_checks_failed;
static int64_t g_bench_left_phase_start_counts;
static int64_t g_bench_right_phase_start_counts;
static uint8_t g_left_forward_capture_active;
static uint8_t g_left_forward_capture_finalized;
static uint32_t g_left_forward_raw_start;
static uint32_t g_left_forward_sample_sequence_start;
static uint32_t g_left_forward_untrusted_samples_start;

static uint8_t CAN_Motor_Bench_Init(void);

static void CAN_Motor_Bench_ResetResult(void)
{
    uint32_t i;

    g_can_motor_bench_result.magic = CAN_MOTOR_BENCH_RESULT_MAGIC;
    g_can_motor_bench_result.passed = 0U;
    g_can_motor_bench_result.status = (uint32_t)CAN_MOTOR_BENCH_PASS;
    g_can_motor_bench_result.phase_reached = 0U;
    g_can_motor_bench_result.groups_sent = 0U;
    g_can_motor_bench_result.tx_failures = 0U;
    g_can_motor_bench_result.calibration_state = WHEEL_CALIBRATION_TX_NONE;
    g_can_motor_bench_result.calibration_stage = WHEEL_CALIBRATION_STAGE_NONE;
    g_can_motor_bench_result.calibration_exit_reason = WHEEL_CALIBRATION_EXIT_NONE;
    g_can_motor_bench_result.calibration_left_delta = 0;
    g_can_motor_bench_result.calibration_right_delta = 0;
    g_can_motor_bench_result.calibration_left_forward_command_pwm = 0;
    g_can_motor_bench_result.calibration_left_forward_pwm_permille = 0U;
    g_can_motor_bench_result.calibration_left_forward_ccr1 = 0U;
    g_can_motor_bench_result.calibration_left_forward_tim1_raw_start = 0U;
    g_can_motor_bench_result.calibration_left_forward_tim1_raw_end = 0U;
    g_can_motor_bench_result.calibration_left_forward_tim1_raw_delta = 0;
    g_can_motor_bench_result.calibration_left_forward_tb6612_pins = 0U;
    g_can_motor_bench_result.calibration_left_forward_captured = 0U;
    g_can_motor_bench_result.calibration_left_forward_encoder_accum_start = 0;
    g_can_motor_bench_result.calibration_left_forward_encoder_accum_end = 0;
    g_can_motor_bench_result.calibration_left_forward_encoder_accum_delta = 0;
    g_can_motor_bench_result.calibration_left_forward_last_sample_raw_delta = 0;
    g_can_motor_bench_result.calibration_left_forward_last_sample_applied_delta = 0;
    g_can_motor_bench_result.calibration_left_forward_sample_count = 0U;
    g_can_motor_bench_result.calibration_left_forward_untrusted_samples = 0U;
    g_can_motor_bench_result.calibration_left_forward_last_dt_ms = 0U;
    g_can_motor_bench_result.calibration_left_forward_last_sample_trusted = 0U;
    g_can_motor_bench_result.calibration_left_forward_service_start_counts = 0;
    g_can_motor_bench_result.calibration_left_forward_service_delta_counts = 0;
    g_can_motor_bench_result.calibration_left_forward_service_pwm_permille = 0U;
    g_can_motor_bench_result.calibration_left_forward_service_sample_trusted = 0U;
    g_can_motor_bench_result.calibration_left_forward_service_response_detected = 0U;

    for (i = 0U; i < CAN_MOTOR_BENCH_PHASE_COUNT; ++i) {
        g_can_motor_bench_result.applied_seq_after[i] = 0U;
        g_can_motor_bench_result.fault_after[i] = 0U;
        g_can_motor_bench_result.left_ccr_after[i] = 0U;
        g_can_motor_bench_result.right_ccr_after[i] = 0U;
        g_can_motor_bench_result.left_encoder_delta_after[i] = 0;
        g_can_motor_bench_result.right_encoder_delta_after[i] = 0;
        g_can_motor_bench_result.tb6612_pins_after[i] = 0U;
    }
}

static uint8_t CAN_Motor_Bench_SendFrame(uint16_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox = 0U;
    uint32_t wait_start = HAL_GetTick();

    /* The production feedback pump burst-fills all three TX mailboxes right
     * before each command group (loopback self-injection shares them), so
     * wait for a free mailbox like a node waiting for bus arbitration. */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if ((HAL_GetTick() - wait_start) > CAN_MOTOR_BENCH_TX_WAIT_MS) {
            g_can_motor_bench_result.tx_failures++;
            return 0U;
        }
    }

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &mailbox) != HAL_OK) {
        g_can_motor_bench_result.tx_failures++;
        return 0U;
    }

    return 1U;
}

static void CAN_Motor_Bench_SendCommandGroup(uint8_t mode_flags,
                                             int16_t left_velocity_mmps,
                                             int16_t right_velocity_mmps)
{
    uint8_t steering[8];
    uint8_t wheels[8];

    CAN_Motor_Bench_BuildSteeringFrame(g_bench_command_seq, mode_flags, 0, steering);
    CAN_Motor_Bench_BuildWheelsFrame(g_bench_command_seq,
                                     mode_flags,
                                     left_velocity_mmps,
                                     right_velocity_mmps,
                                     wheels);
    g_bench_command_seq++;

    if (CAN_Motor_Bench_SendFrame(BSP_BXCAN_ID_CMD_STEERING, steering) == 0U) {
        return;
    }
    if (CAN_Motor_Bench_SendFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, wheels) == 0U) {
        return;
    }

    g_can_motor_bench_result.groups_sent++;
}

static uint8_t CAN_Motor_Bench_ReadTb6612Pins(void)
{
    uint8_t pins = 0U;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
        pins |= CAN_MOTOR_BENCH_PIN_LEFT_IN1;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) {
        pins |= CAN_MOTOR_BENCH_PIN_LEFT_IN2;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) {
        pins |= CAN_MOTOR_BENCH_PIN_RIGHT_IN1;
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) {
        pins |= CAN_MOTOR_BENCH_PIN_RIGHT_IN2;
    }

    return pins;
}

static int32_t CAN_Motor_Bench_TimerDelta(const TIM_HandleTypeDef *timer,
                                          uint32_t start,
                                          uint32_t end)
{
    uint64_t modulo = (uint64_t)timer->Init.Period + 1ULL;
    int64_t delta = (int64_t)end - (int64_t)start;
    int64_t half = (int64_t)(modulo / 2ULL);

    if (delta > half) {
        delta -= (int64_t)modulo;
    } else if (delta < -half) {
        delta += (int64_t)modulo;
    }

    return (int32_t)delta;
}

/* Snapshot only: this never changes the calibration state machine, thresholds,
 * safety action, or motor command.  It runs immediately before the normal
 * chassis process so the pins/CCR still describe the active LEFT_FORWARD
 * output, even if that process terminates the stage and coasts the motor. */
static void CAN_Motor_Bench_CaptureLeftForwardSnapshot(void)
{
    EncoderDiagnostics_t diagnostics;
    Wheel_Calibration_Recommendation_t recommendation;
    uint32_t ccr1;
    uint32_t period;

    if (Wheel_Calibration_Service_GetStage() != WHEEL_CALIBRATION_STAGE_LEFT_FORWARD) {
        return;
    }

    if (g_left_forward_capture_active == 0U) {
        g_left_forward_capture_active = 1U;
        g_left_forward_raw_start = __HAL_TIM_GET_COUNTER(&htim1);
        Encoder_GetDiagnostics(1U, &diagnostics);
        g_left_forward_sample_sequence_start = diagnostics.sample_sequence;
        g_left_forward_untrusted_samples_start = diagnostics.untrusted_samples;
        g_can_motor_bench_result.calibration_left_forward_tim1_raw_start =
            g_left_forward_raw_start;
        g_can_motor_bench_result.calibration_left_forward_encoder_accum_start =
            (int32_t)diagnostics.accumulated_counts;
        g_can_motor_bench_result.calibration_left_forward_captured = 1U;
    }

    ccr1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    period = htim3.Init.Period;
    g_can_motor_bench_result.calibration_left_forward_ccr1 = ccr1;
    if (period != 0U) {
        uint64_t permille = ((uint64_t)ccr1 * 1000ULL) / (uint64_t)period;
        if (permille > 1000ULL) {
            permille = 1000ULL;
        }
        g_can_motor_bench_result.calibration_left_forward_pwm_permille =
            (uint16_t)permille;
    }

    if (Wheel_Calibration_Service_GetRecommendation(&recommendation) != 0U) {
        g_can_motor_bench_result.calibration_left_forward_command_pwm =
            recommendation.left_pwm;
    }
    g_can_motor_bench_result.calibration_left_forward_tb6612_pins =
        (uint8_t)(CAN_Motor_Bench_ReadTb6612Pins() &
                  (CAN_MOTOR_BENCH_PIN_LEFT_IN1 | CAN_MOTOR_BENCH_PIN_LEFT_IN2));
}

static void CAN_Motor_Bench_FinalizeLeftForwardSnapshot(void)
{
    EncoderDiagnostics_t diagnostics;

    if ((g_left_forward_capture_active == 0U) ||
        (g_left_forward_capture_finalized != 0U) ||
        (Wheel_Calibration_Service_GetStage() == WHEEL_CALIBRATION_STAGE_LEFT_FORWARD)) {
        return;
    }

    g_can_motor_bench_result.calibration_left_forward_tim1_raw_end =
        __HAL_TIM_GET_COUNTER(&htim1);
    g_can_motor_bench_result.calibration_left_forward_tim1_raw_delta =
        CAN_Motor_Bench_TimerDelta(
            &htim1,
            g_can_motor_bench_result.calibration_left_forward_tim1_raw_start,
            g_can_motor_bench_result.calibration_left_forward_tim1_raw_end);
    Encoder_GetDiagnostics(1U, &diagnostics);
    g_can_motor_bench_result.calibration_left_forward_encoder_accum_end =
        (int32_t)diagnostics.accumulated_counts;
    g_can_motor_bench_result.calibration_left_forward_encoder_accum_delta =
        g_can_motor_bench_result.calibration_left_forward_encoder_accum_end -
        g_can_motor_bench_result.calibration_left_forward_encoder_accum_start;
    g_can_motor_bench_result.calibration_left_forward_last_sample_raw_delta =
        diagnostics.raw_delta_counts;
    g_can_motor_bench_result.calibration_left_forward_last_sample_applied_delta =
        diagnostics.applied_delta_counts;
    g_can_motor_bench_result.calibration_left_forward_sample_count =
        diagnostics.sample_sequence - g_left_forward_sample_sequence_start;
    g_can_motor_bench_result.calibration_left_forward_untrusted_samples =
        diagnostics.untrusted_samples - g_left_forward_untrusted_samples_start;
    g_can_motor_bench_result.calibration_left_forward_last_dt_ms = diagnostics.dt_ms;
    g_can_motor_bench_result.calibration_left_forward_last_sample_trusted = diagnostics.trusted;
    g_left_forward_capture_finalized = 1U;
}

static uint8_t CAN_Motor_Bench_ExpectedLeftPins(int16_t velocity_mmps)
{
    if (velocity_mmps > 0) {
        return CAN_MOTOR_BENCH_PIN_LEFT_IN1;
    }
    if (velocity_mmps < 0) {
        return CAN_MOTOR_BENCH_PIN_LEFT_IN2;
    }
    return 0U;
}

static uint8_t CAN_Motor_Bench_ExpectedRightPins(int16_t velocity_mmps)
{
    if (velocity_mmps > 0) {
        return CAN_MOTOR_BENCH_PIN_RIGHT_IN2;
    }
    if (velocity_mmps < 0) {
        return CAN_MOTOR_BENCH_PIN_RIGHT_IN1;
    }
    return 0U;
}

static void CAN_Motor_Bench_SampleAndCheck(uint32_t phase,
                                           uint8_t expect_drive,
                                           int8_t expected_encoder_sign,
                                           uint8_t exp_pins,
                                           uint16_t exp_fault,
                                           uint16_t exp_applied_seq)
{
    uint32_t left_ccr;
    uint32_t right_ccr;
    uint8_t pins;
    EncoderSample_t left_sample;
    EncoderSample_t right_sample;

    if (phase >= CAN_MOTOR_BENCH_PHASE_COUNT) {
        g_bench_checks_failed++;
        return;
    }

    left_sample = Encoder_Sample(1U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
    right_sample = Encoder_Sample(2U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
    left_ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
    right_ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
    pins = CAN_Motor_Bench_ReadTb6612Pins();

    g_can_motor_bench_result.left_ccr_after[phase] = left_ccr;
    g_can_motor_bench_result.right_ccr_after[phase] = right_ccr;
    g_can_motor_bench_result.tb6612_pins_after[phase] = pins;
    g_can_motor_bench_result.fault_after[phase] = BSP_BXCAN_GetFault();
    g_can_motor_bench_result.applied_seq_after[phase] = BSP_BXCAN_GetAppliedCommandSeq();
    g_can_motor_bench_result.left_encoder_delta_after[phase] =
        (int32_t)(left_sample.accumulated_counts - g_bench_left_phase_start_counts);
    g_can_motor_bench_result.right_encoder_delta_after[phase] =
        (int32_t)(right_sample.accumulated_counts - g_bench_right_phase_start_counts);

    if (((expect_drive != 0U) && ((left_ccr == 0U) || (right_ccr == 0U))) ||
        ((expect_drive == 0U) && ((left_ccr != 0U) || (right_ccr != 0U))) ||
        (pins != exp_pins) ||
        (g_can_motor_bench_result.fault_after[phase] != exp_fault) ||
        (g_can_motor_bench_result.applied_seq_after[phase] != exp_applied_seq)) {
        g_bench_checks_failed++;
    }

    if (expected_encoder_sign > 0 &&
        ((g_can_motor_bench_result.left_encoder_delta_after[phase] <= 0) ||
         (g_can_motor_bench_result.right_encoder_delta_after[phase] <= 0))) {
        g_bench_checks_failed++;
    }
    if (expected_encoder_sign < 0 &&
        ((g_can_motor_bench_result.left_encoder_delta_after[phase] >= 0) ||
         (g_can_motor_bench_result.right_encoder_delta_after[phase] >= 0))) {
        g_bench_checks_failed++;
    }
}

static void CAN_Motor_Bench_StreamPhase(uint32_t duration_ms,
                                        uint8_t mode_flags,
                                        int16_t left_velocity_mmps,
                                        int16_t right_velocity_mmps)
{
    uint32_t start = HAL_GetTick();
    uint32_t phase = g_can_motor_bench_result.phase_reached;
    uint8_t exp_pins = 0U;
    uint8_t expect_drive = 0U;
    int8_t expected_encoder_sign = 0;

    {
        EncoderSample_t left_sample = Encoder_Sample(1U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
        EncoderSample_t right_sample = Encoder_Sample(2U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
        g_bench_left_phase_start_counts = left_sample.accumulated_counts;
        g_bench_right_phase_start_counts = right_sample.accumulated_counts;
    }

    if ((mode_flags & BSP_BXCAN_MODE_MASK) == BSP_BXCAN_MODE_VELOCITY) {
        expect_drive = 1U;
        exp_pins = CAN_Motor_Bench_ExpectedLeftPins(left_velocity_mmps) |
                   CAN_Motor_Bench_ExpectedRightPins(right_velocity_mmps);
        if ((left_velocity_mmps > 0) && (right_velocity_mmps > 0)) {
            expected_encoder_sign = 1;
        } else if ((left_velocity_mmps < 0) && (right_velocity_mmps < 0)) {
            expected_encoder_sign = -1;
        }
    }

    while ((uint32_t)(HAL_GetTick() - start) < duration_ms) {
        CAN_Motor_Bench_SendCommandGroup(mode_flags, left_velocity_mmps, right_velocity_mmps);
        HAL_Delay(CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
        Chassis_ControlProcess(HAL_GetTick());
    }
    Chassis_ControlProcess(HAL_GetTick());

    CAN_Motor_Bench_SampleAndCheck(phase,
                                   expect_drive,
                                   expected_encoder_sign,
                                   exp_pins,
                                   BSP_BXCAN_FAULT_NONE,
                                   (uint16_t)(g_bench_command_seq - 1U));
    g_can_motor_bench_result.phase_reached = phase + 1U;
}

static void CAN_Motor_Bench_WatchdogPhase(void)
{
    uint32_t start = HAL_GetTick();
    uint32_t phase = g_can_motor_bench_result.phase_reached;
    uint16_t seq_before = BSP_BXCAN_GetAppliedCommandSeq();

    while ((uint32_t)(HAL_GetTick() - start) < CAN_MOTOR_BENCH_WATCHDOG_MS) {
        Chassis_ControlProcess(HAL_GetTick());
    }

    CAN_Motor_Bench_SampleAndCheck(phase,
                                   0U,
                                   0,
                                   0U,
                                   BSP_BXCAN_FAULT_COMMAND_TIMEOUT,
                                   seq_before);
    g_can_motor_bench_result.phase_reached = phase + 1U;
}

static void CAN_Motor_Bench_EstopPhase(void)
{
    uint8_t wheels[8];
    uint32_t phase = g_can_motor_bench_result.phase_reached;
    uint16_t seq_before = BSP_BXCAN_GetAppliedCommandSeq();

    CAN_Motor_Bench_BuildWheelsFrame(g_bench_command_seq, BSP_BXCAN_FLAG_ESTOP, 0, 0, wheels);
    g_bench_command_seq++;
    (void)CAN_Motor_Bench_SendFrame(BSP_BXCAN_ID_CMD_REAR_WHEELS, wheels);

    HAL_Delay(2U);
    Chassis_ControlProcess(HAL_GetTick());

    CAN_Motor_Bench_SampleAndCheck(phase,
                                   0U,
                                   0,
                                   CAN_MOTOR_BENCH_PIN_BRAKE_ALL,
                                   BSP_BXCAN_FAULT_ESTOP_ACTIVE,
                                   seq_before);
    g_can_motor_bench_result.phase_reached = phase + 1U;
}

static void CAN_Motor_Bench_ResetEstopPhase(void)
{
    uint32_t phase = g_can_motor_bench_result.phase_reached;

    CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_RESET_FAULT,
                                     0,
                                     0);
    HAL_Delay(2U);
    Chassis_ControlProcess(HAL_GetTick());

    CAN_Motor_Bench_SampleAndCheck(phase,
                                   0U,
                                   0,
                                   0U,
                                   BSP_BXCAN_FAULT_NONE,
                                   (uint16_t)(g_bench_command_seq - 1U));
    g_can_motor_bench_result.phase_reached = phase + 1U;
}

static void CAN_Motor_Bench_PostResetDrivePhase(void)
{
    CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_RECOVERY_DRIVE_MS,
                                 BSP_BXCAN_MODE_VELOCITY,
                                 CAN_MOTOR_BENCH_TARGET_MMPS,
                                 CAN_MOTOR_BENCH_TARGET_MMPS);
}

static void CAN_Motor_Bench_PhysicalEstopPhase(void)
{
    uint32_t phase;
    uint32_t start;
    uint16_t expected_drive_seq;

    /* The input must be released before this phase; otherwise no automatic
     * motion is started while the operator is already holding the E-stop. */
    if (Physical_EStop_IsAsserted() != 0U) {
        g_bench_checks_failed++;
        return;
    }

    phase = g_can_motor_bench_result.phase_reached;
    CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_VELOCITY,
                                     CAN_MOTOR_BENCH_TARGET_MMPS,
                                     CAN_MOTOR_BENCH_TARGET_MMPS);
    expected_drive_seq = (uint16_t)(g_bench_command_seq - 1U);
    start = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start) < CAN_MOTOR_BENCH_PHYSICAL_WAIT_MS) {
        HAL_Delay(CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
        Chassis_ControlProcess(HAL_GetTick());
        if (Physical_EStop_IsAsserted() != 0U) {
            break;
        }
        CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_VELOCITY,
                                         CAN_MOTOR_BENCH_TARGET_MMPS,
                                         CAN_MOTOR_BENCH_TARGET_MMPS);
    }

    if (Physical_EStop_IsAsserted() == 0U) {
        g_bench_checks_failed++;
        CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_STOP, 0, 0);
        Chassis_ControlProcess(HAL_GetTick());
        return;
    }

    Chassis_ControlProcess(HAL_GetTick());
    CAN_Motor_Bench_SampleAndCheck(phase,
                                   0U,
                                   0,
                                   CAN_MOTOR_BENCH_PIN_BRAKE_ALL,
                                   BSP_BXCAN_FAULT_ESTOP_ACTIVE,
                                   expected_drive_seq);
    g_can_motor_bench_result.phase_reached = phase + 1U;

    start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < CAN_MOTOR_BENCH_PHYSICAL_RELEASE_MS) {
        Chassis_ControlProcess(HAL_GetTick());
        if (Physical_EStop_IsAsserted() == 0U) {
            break;
        }
        HAL_Delay(CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
    }

    if (Physical_EStop_IsAsserted() != 0U) {
        g_bench_checks_failed++;
        return;
    }

    phase = g_can_motor_bench_result.phase_reached;
    CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_VELOCITY,
                                     CAN_MOTOR_BENCH_TARGET_MMPS,
                                     CAN_MOTOR_BENCH_TARGET_MMPS);
    expected_drive_seq = (uint16_t)(g_bench_command_seq - 1U);
    HAL_Delay(2U);
    Chassis_ControlProcess(HAL_GetTick());
    CAN_Motor_Bench_SampleAndCheck(phase,
                                   0U,
                                   0,
                                   CAN_MOTOR_BENCH_PIN_BRAKE_ALL,
                                   BSP_BXCAN_FAULT_ESTOP_ACTIVE,
                                   expected_drive_seq);
    g_can_motor_bench_result.phase_reached = phase + 1U;

    phase = g_can_motor_bench_result.phase_reached;
    CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_STOP | BSP_BXCAN_FLAG_RESET_FAULT,
                                     0,
                                     0);
    HAL_Delay(2U);
    Chassis_ControlProcess(HAL_GetTick());
    CAN_Motor_Bench_SampleAndCheck(phase,
                                   0U,
                                   0,
                                   0U,
                                   BSP_BXCAN_FAULT_NONE,
                                   (uint16_t)(g_bench_command_seq - 1U));
    g_can_motor_bench_result.phase_reached = phase + 1U;
}

void CAN_Motor_Bench_RunCalibration(void)
{
    uint8_t calibration[8];
    uint32_t start;
    uint32_t last_stop = 0U;
    EncoderSample_t left_start;
    EncoderSample_t right_start;

    g_left_forward_capture_active = 0U;
    g_left_forward_capture_finalized = 0U;
    g_left_forward_raw_start = 0U;
    g_left_forward_sample_sequence_start = 0U;
    g_left_forward_untrusted_samples_start = 0U;

    if (CAN_Motor_Bench_Init() == 0U) {
        Motor_CoastAll();
        while (1) {
            BSP_Watchdog_Feed();
            Chassis_ControlProcess(HAL_GetTick());
        }
    }

    if (Physical_EStop_IsAsserted() != 0U) {
        g_can_motor_bench_result.status = (uint32_t)CAN_MOTOR_BENCH_PHASE_MISMATCH;
        Motor_EmergencyBrakeAll();
        while (1) {
            Chassis_ControlProcess(HAL_GetTick());
        }
    }

    left_start = Encoder_Sample(1U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
    right_start = Encoder_Sample(2U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS);
    CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_STOP, 0, 0);
    HAL_Delay(2U);
    Chassis_ControlProcess(HAL_GetTick());

    CAN_Motor_Bench_BuildCalibrationFrame(1U,
                                           WHEEL_CALIBRATION_OPCODE_START,
                                           WHEEL_CALIBRATION_OPTION_MASK,
                                           calibration);
    (void)CAN_Motor_Bench_SendFrame(BSP_BXCAN_ID_CMD_CALIBRATION, calibration);
    start = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start) < (WHEEL_CALIBRATION_TOTAL_TIMEOUT_MS + 2000U)) {
        uint32_t now = HAL_GetTick();

        if ((last_stop == 0U) || ((uint32_t)(now - last_stop) >= CAN_MOTOR_BENCH_GROUP_PERIOD_MS)) {
            CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_STOP, 0, 0);
            last_stop = now;
        }
        CAN_Motor_Bench_CaptureLeftForwardSnapshot();
        Chassis_ControlProcess(now);
        CAN_Motor_Bench_FinalizeLeftForwardSnapshot();
        BSP_Watchdog_Feed();
        if ((Wheel_Calibration_Service_IsActive() == 0U) &&
            (Wheel_Calibration_Service_GetState() != WHEEL_CALIBRATION_TX_NONE)) {
            break;
        }
        HAL_Delay(10U);
    }

    Chassis_ControlProcess(HAL_GetTick());
    CAN_Motor_Bench_FinalizeLeftForwardSnapshot();
    {
        Wheel_Calibration_StageDiagnostics_t service_diagnostics;

        Wheel_Calibration_Service_GetStageDiagnostics(&service_diagnostics);
        g_can_motor_bench_result.calibration_left_forward_service_start_counts =
            (int32_t)service_diagnostics.left_start_counts;
        g_can_motor_bench_result.calibration_left_forward_service_delta_counts =
            (int32_t)service_diagnostics.left_delta_counts;
        g_can_motor_bench_result.calibration_left_forward_service_pwm_permille =
            service_diagnostics.stage_pwm_permille;
        g_can_motor_bench_result.calibration_left_forward_service_sample_trusted =
            service_diagnostics.left_sample_trusted;
        g_can_motor_bench_result.calibration_left_forward_service_response_detected =
            service_diagnostics.response_detected;
    }
    g_can_motor_bench_result.calibration_state = Wheel_Calibration_Service_GetState();
    g_can_motor_bench_result.calibration_stage = Wheel_Calibration_Service_GetStage();
    g_can_motor_bench_result.calibration_exit_reason = Wheel_Calibration_Service_GetExitReason();
    {
        Wheel_Calibration_TerminalSample_t terminal_sample;

        Wheel_Calibration_Service_GetTerminalSample(&terminal_sample);
        if (terminal_sample.valid != 0U) {
            /* Use the sample from the service's decision cycle.  A fresh
               Encoder_Sample() here would observe a later control period and
               make the reported phase delta disagree with the decision. */
            g_can_motor_bench_result.calibration_left_delta =
                (int32_t)(terminal_sample.left_sample.accumulated_counts -
                          left_start.accumulated_counts);
        } else {
            g_can_motor_bench_result.calibration_left_delta =
                (int32_t)(Encoder_Sample(1U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS).accumulated_counts -
                          left_start.accumulated_counts);
        }
    }
    g_can_motor_bench_result.calibration_right_delta =
        (int32_t)(Encoder_Sample(2U, CAN_MOTOR_BENCH_GROUP_PERIOD_MS).accumulated_counts -
                  right_start.accumulated_counts);

    CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_STOP, 0, 0);
    Chassis_ControlProcess(HAL_GetTick());

    if ((g_can_motor_bench_result.calibration_state == WHEEL_CALIBRATION_TX_SUCCEEDED) &&
        (g_can_motor_bench_result.calibration_exit_reason == WHEEL_CALIBRATION_EXIT_SUCCESS) &&
        (Wheel_Calibration_IsValid() != 0U) &&
        (g_can_motor_bench_result.tx_failures == 0U)) {
        g_can_motor_bench_result.passed = 1U;
    } else {
        g_can_motor_bench_result.status = (uint32_t)CAN_MOTOR_BENCH_PHASE_MISMATCH;
    }

    while (1) {
        Chassis_ControlProcess(HAL_GetTick());
        BSP_Watchdog_Feed();
    }
}

static uint8_t CAN_Motor_Bench_Init(void)
{
    CAN_Motor_Bench_ResetResult();
    g_bench_command_seq = 1U;
    g_bench_checks_failed = 0U;

    if (HAL_CAN_DeInit(&hcan1) != HAL_OK) {
        g_can_motor_bench_result.status = (uint32_t)CAN_MOTOR_BENCH_ERR_DEINIT;
        return 0U;
    }

    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        g_can_motor_bench_result.status = (uint32_t)CAN_MOTOR_BENCH_ERR_INIT;
        return 0U;
    }

    /* The bench image is an opt-in test harness.  Do not let an IWDG reset
       cause left by an earlier watchdog probe reject its synthetic STOP
       precondition; normal firmware keeps the reset cause latched. */
    __HAL_RCC_CLEAR_RESET_FLAGS();
    Chassis_ControlInit();
    return 1U;
}

void CAN_Motor_Bench_Run(void)
{
    if (CAN_Motor_Bench_Init() == 0U) {
        Motor_CoastAll();
        while (1) {
            /* safe idle: motors coasted, result records the init failure */
        }
    }

    CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_DRIVE_MS, BSP_BXCAN_MODE_VELOCITY, CAN_MOTOR_BENCH_TARGET_MMPS, CAN_MOTOR_BENCH_TARGET_MMPS);
    CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_STOP_MS, BSP_BXCAN_MODE_STOP, 0, 0);
    CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_REVERSE_MS, BSP_BXCAN_MODE_VELOCITY, -CAN_MOTOR_BENCH_TARGET_MMPS, -CAN_MOTOR_BENCH_TARGET_MMPS);
    CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_STOP_MS, BSP_BXCAN_MODE_STOP, 0, 0);
    CAN_Motor_Bench_WatchdogPhase();
    CAN_Motor_Bench_EstopPhase();
    CAN_Motor_Bench_ResetEstopPhase();
    CAN_Motor_Bench_PostResetDrivePhase();
    CAN_Motor_Bench_SendCommandGroup(BSP_BXCAN_MODE_STOP, 0, 0);
    Chassis_ControlProcess(HAL_GetTick());
    CAN_Motor_Bench_PhysicalEstopPhase();

    if (g_bench_checks_failed != 0U) {
        g_can_motor_bench_result.status = (uint32_t)CAN_MOTOR_BENCH_PHASE_MISMATCH;
    }

    if ((g_bench_checks_failed == 0U) &&
        (g_can_motor_bench_result.status == (uint32_t)CAN_MOTOR_BENCH_PASS)) {
        g_can_motor_bench_result.passed = 1U;
    }

    while (1) {
        Chassis_ControlProcess(HAL_GetTick());
    }
}

#endif /* CAN_MOTOR_BENCH_HOST_TEST */
