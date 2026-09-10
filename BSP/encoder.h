#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef ENCODER_HOST_TEST
#include <stdint.h>
#else
#include "main.h"
#endif

typedef struct {
    int32_t delta_counts;
    int64_t accumulated_counts;
    int32_t velocity_mmps;
    uint8_t trusted;
} EncoderSample_t;

typedef struct {
    int32_t raw_delta_counts;
    uint32_t dt_ms;
    int32_t mcu_speed_mmps;
    int32_t recomputed_speed_mmps;
    int64_t counts_per_wheel_rev;
    int64_t circumference_mm_x1000;
} EncoderSpeedEvidence_t;

/* Read-only evidence for diagnosing a live sample.  Getting this structure
 * never samples the timer and therefore cannot disturb the encoder state. */
typedef struct {
    int32_t raw_delta_counts;
    int32_t applied_delta_counts;
    uint32_t dt_ms;
    int64_t accumulated_counts;
    uint8_t trusted;
    uint32_t sample_sequence;
    uint32_t trusted_samples;
    uint32_t untrusted_samples;
} EncoderDiagnostics_t;

void Encoder_Init(void);
void Encoder_Reset(void);
EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms);
int32_t Encoder_RecomputeSpeedMmps(int32_t delta_counts, uint32_t dt_ms);
void Encoder_GetSpeedEvidence(const EncoderSample_t *sample,
                              uint32_t dt_ms,
                              EncoderSpeedEvidence_t *evidence);
void Encoder_GetDiagnostics(uint8_t num, EncoderDiagnostics_t *out);

float Encoder_Get(uint8_t num);

#endif
