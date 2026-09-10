#ifndef BSP_AUTOTUNE_SAFE_H
#define BSP_AUTOTUNE_SAFE_H

#include <stdint.h>

#define AUTOTUNE_SAFE_PROFILE_ID       0xA1U
#define AUTOTUNE_SAFE_ID_FB_IDENTITY   0x189U
#define AUTOTUNE_SAFE_ID_FB_STATE      0x18AU
#define AUTOTUNE_SAFE_ID_FB_LIMITS     0x18BU
#define AUTOTUNE_SAFE_ID_FB_WHEEL      0x18CU
#define AUTOTUNE_SAFE_ID_FB_PID_PI     0x18DU
#define AUTOTUNE_SAFE_ID_FB_PID_DO     0x18EU
#define AUTOTUNE_SAFE_ID_FB_SAFETY     0x18FU
#define AUTOTUNE_SAFE_ID_FB_COUNTERS   0x190U

typedef struct {
    float kp;
    float ki;
    float kd;
} AutotuneSafe_Gains_t;

typedef enum {
    AUTOTUNE_SAFE_LEVEL_L0_LOCKED = 0U,
    AUTOTUNE_SAFE_LEVEL_L1_INITIAL = 1U,
    AUTOTUNE_SAFE_LEVEL_L2 = 2U,
    AUTOTUNE_SAFE_LEVEL_L3 = 3U,
    AUTOTUNE_SAFE_LEVEL_L4 = 4U
} AutotuneSafe_Level_t;

typedef enum {
    AUTOTUNE_SAFE_STATE_LOCKED = 0U,
    AUTOTUNE_SAFE_STATE_READY = 1U,
    AUTOTUNE_SAFE_STATE_PREFLIGHT = 2U,
    AUTOTUNE_SAFE_STATE_RAMP = 3U,
    AUTOTUNE_SAFE_STATE_RUNNING = 4U,
    AUTOTUNE_SAFE_STATE_STOPPING = 5U,
    AUTOTUNE_SAFE_STATE_COMPLETE = 6U,
    AUTOTUNE_SAFE_STATE_ABORT = 7U,
    AUTOTUNE_SAFE_STATE_COOLING = 8U
} AutotuneSafe_State_t;

typedef enum {
    AUTOTUNE_SAFE_GAIN_ACCEPTED = 0U,
    AUTOTUNE_SAFE_GAIN_REJECTED_ABSOLUTE = 1U,
    AUTOTUNE_SAFE_GAIN_REJECTED_STEP = 2U,
    AUTOTUNE_SAFE_GAIN_REJECTED_FORMAT = 3U
} AutotuneSafe_GainResult_t;

#define AUTOTUNE_SAFE_LOCAL_PHASE_P_ONLY 1U
#define AUTOTUNE_SAFE_LOCAL_PHASE_PI     2U
#define AUTOTUNE_SAFE_LOCAL_PHASE_D      3U

typedef enum {
    AUTOTUNE_SAFE_ABORT_NONE = 0U,
    AUTOTUNE_SAFE_ABORT_STALL = 1U,
    AUTOTUNE_SAFE_ABORT_OVERSPEED = 2U,
    AUTOTUNE_SAFE_ABORT_OSCILLATION = 3U,
    AUTOTUNE_SAFE_ABORT_SATURATION = 4U,
    AUTOTUNE_SAFE_ABORT_SAFETY = 5U,
    AUTOTUNE_SAFE_ABORT_WATCHDOG = 6U,
    AUTOTUNE_SAFE_ABORT_ENCODER = 7U,
    AUTOTUNE_SAFE_ABORT_THERMAL = 8U,
    AUTOTUNE_SAFE_ABORT_NO_MOTION = 9U
} AutotuneSafe_AbortReason_t;

#define AUTOTUNE_SAFE_OPCODE_START           1U
#define AUTOTUNE_SAFE_OPCODE_STOP            2U
#define AUTOTUNE_SAFE_OPCODE_HEARTBEAT       3U
#define AUTOTUNE_SAFE_OPCODE_QUERY           4U
#define AUTOTUNE_SAFE_OPCODE_REQUEST_PROMOTE 5U

typedef struct {
    uint8_t state;
    uint8_t level;
    uint8_t bootstrap_active;
    uint8_t promotion_eligible;
    uint16_t target_limit_mmps;
    uint16_t pwm_limit;
    uint8_t abort_reason;
    uint8_t abort_flag;
    uint8_t stall;
    uint8_t saturation;
    uint8_t overspeed;
    uint8_t oscillation;
    uint8_t speed_limit_hit;
    uint16_t stall_time_ms;
    uint16_t saturation_time_ms;
    uint16_t oscillation_count;
    uint16_t last_valid_command_age_ms;
    uint32_t session_id;
    uint32_t experiment_id;
    int16_t requested_target;
    int16_t effective_target;
    int16_t actual_left;
    int16_t actual_right;
    int16_t pwm_left;
    int16_t pwm_right;
    float left_pid_p;
    float left_pid_i;
    float left_pid_d;
    float left_pid_output;
    float right_pid_p;
    float right_pid_i;
    float right_pid_d;
    float right_pid_output;
    uint8_t safety_state;
    uint16_t fault_code;
    AutotuneSafe_Gains_t best_known_safe;
} AutotuneSafe_Status_t;

#ifdef __cplusplus
extern "C" {
#endif

void AutotuneSafe_Init(void);
void AutotuneSafe_Process(uint32_t now_ms);
uint8_t AutotuneSafe_ConfirmPreflight(uint8_t safety_ok,
                                      uint8_t estop_clear,
                                      uint8_t still,
                                      uint8_t firmware_ok);
void AutotuneSafe_GetStatus(AutotuneSafe_Status_t *status);
void AutotuneSafe_GetRecoveryGains(AutotuneSafe_Gains_t *gains);
AutotuneSafe_GainResult_t AutotuneSafe_ValidateCandidate(
    const AutotuneSafe_Gains_t *candidate);
AutotuneSafe_GainResult_t AutotuneSafe_ValidateLocalCandidate(
    const AutotuneSafe_Gains_t *candidate,
    const AutotuneSafe_Gains_t *baseline,
    uint8_t phase);
void AutotuneSafe_SetCandidate(const AutotuneSafe_Gains_t *candidate);
void AutotuneSafe_RecordCompletePass(void);
void AutotuneSafe_RecordLocalCandidatePass(
    const AutotuneSafe_Gains_t *candidate);
uint8_t AutotuneSafe_RequestPromote(void);
void AutotuneSafe_RecordAbort(AutotuneSafe_AbortReason_t reason);
void AutotuneSafe_RequestStop(void);
uint8_t AutotuneSafe_BeginExperiment(uint32_t session_id,
                                     uint32_t experiment_id,
                                     int16_t requested_target_mmps);
int16_t AutotuneSafe_GetEffectiveTarget(void);
uint8_t AutotuneSafe_Heartbeat(uint32_t session_id,
                               uint32_t experiment_id,
                               uint16_t sequence,
                               uint32_t now_ms);
void AutotuneSafe_RecordSample(uint32_t now_ms,
                               int16_t target_mmps,
                               int16_t actual_mmps,
                               int16_t pwm,
                               uint8_t encoder_valid,
                               uint8_t safety_ok,
                               uint8_t estop_clear);
void AutotuneSafe_RecordStillness(uint32_t now_ms,
                                  int16_t actual_left_mmps,
                                  int16_t actual_right_mmps);
void AutotuneSafe_RecordWheelTelemetry(uint8_t axis,
                                       int16_t actual_mmps,
                                       int16_t pwm,
                                       float pid_p,
                                       float pid_i,
                                       float pid_d,
                                       float pid_output);
void AutotuneSafe_SetSafetyTelemetry(uint8_t safety_state,
                                     uint16_t fault_code);
void AutotuneSafe_SetSafetyInput(uint8_t safety_ok,
                                 uint8_t estop_clear);
uint8_t AutotuneSafe_BuildTelemetryFrame(uint16_t std_id,
                                         uint8_t snapshot_seq,
                                         uint8_t axis,
                                         uint8_t data[8]);
void AutotuneSafe_OnCanFrame(uint8_t dlc,
                             uint8_t ide,
                             uint8_t rtr,
                             const uint8_t data[8],
                             uint32_t now_ms);

#ifdef AUTOTUNE_SAFE_PROFILE
int16_t AutotuneSafe_ActuatorGate(uint8_t motor_num, int16_t requested_pwm);
#else
static inline int16_t AutotuneSafe_ActuatorGate(uint8_t motor_num,
                                                int16_t requested_pwm)
{
    (void)motor_num;
    return requested_pwm;
}
#endif

#ifdef __cplusplus
}
#endif

#endif
