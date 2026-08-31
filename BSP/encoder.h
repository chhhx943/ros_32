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

void Encoder_Init(void);
void Encoder_Reset(void);
EncoderSample_t Encoder_Sample(uint8_t num, uint32_t dt_ms);

float Encoder_Get(uint8_t num);

#endif
