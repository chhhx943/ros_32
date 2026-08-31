#include "PID.h"

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float PID_RawOutput(const PID_t *p, float integral, float derivative)
{
    return (p->Kp * p->Error0) + (p->Ki * integral) + (p->Kd * derivative);
}

void PID_Reset(PID_t *p)
{
    if (p == 0) {
        return;
    }

    p->Out = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->ErrorInt = 0.0f;
}

void PID_UpdateDt(PID_t *p, float dt_s)
{
    float previous_error;
    float previous_integral;
    float candidate_integral;
    float derivative = 0.0f;
    float current_raw;
    float current_clamped;
    float candidate_raw;
    float candidate_clamped;
    uint8_t accept_integral = 0U;

    if (p == 0) {
        return;
    }

    previous_error = p->Error0;
    previous_integral = p->ErrorInt;

    p->Error1 = previous_error;
    p->Error0 = p->Target - p->Actual;

    if (dt_s > 0.0f) {
        derivative = (p->Error0 - p->Error1) / dt_s;
    }

    if ((p->Ki == 0.0f) || (dt_s <= 0.0f)) {
        p->ErrorInt = 0.0f;
        p->Out = PID_Clamp(PID_RawOutput(p, p->ErrorInt, derivative), p->OutMin, p->OutMax);
        return;
    }

    candidate_integral = previous_integral + (p->Error0 * dt_s);

    current_raw = PID_RawOutput(p, previous_integral, derivative);
    current_clamped = PID_Clamp(current_raw, p->OutMin, p->OutMax);
    candidate_raw = PID_RawOutput(p, candidate_integral, derivative);
    candidate_clamped = PID_Clamp(candidate_raw, p->OutMin, p->OutMax);

    if (current_raw == current_clamped) {
        accept_integral = 1U;
    } else if ((current_clamped >= p->OutMax) && (p->Error0 < 0.0f)) {
        accept_integral = 1U;
    } else if ((current_clamped <= p->OutMin) && (p->Error0 > 0.0f)) {
        accept_integral = 1U;
    }

    if (accept_integral != 0U) {
        p->ErrorInt = candidate_integral;
        p->Out = candidate_clamped;
    } else {
        p->ErrorInt = previous_integral;
        p->Out = current_clamped;
    }
}

void PID_Update(PID_t *p)
{
    PID_UpdateDt(p, 1.0f);
}
