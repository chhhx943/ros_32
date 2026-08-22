#include "encoder.h"
#include "tim.h"
#include "math.h"
#include "stdio.h"

#define ENCODER_PPR     500*4      // 每圈脉冲数
#define WHEEL_RADIUS    0.0337f   // 轮半径 (m)
#define SAMPLE_PERIOD   0.02f     // 采样间隔 (s)

float Encoder_Get(uint8_t num)
{
    float velocity = 0;
    static int32_t last_count1 = 0;
    static int32_t last_count2 = 0;
    int32_t current_count, delta;

    if (num == 1)
    {
        current_count = __HAL_TIM_GET_COUNTER(&htim3);
        delta = current_count - last_count1;

        // 处理溢出（16 位编码器）
        if (delta > 32767) delta -= 65536;
        else if (delta < -32768) delta += 65536;

        last_count1 = current_count;

        // 转换为线速度 (m/s)
        velocity = (float)delta * (2 * 3.14 * WHEEL_RADIUS) / (ENCODER_PPR * SAMPLE_PERIOD*28);
    }
    else if (num == 2)
    {
        current_count = __HAL_TIM_GET_COUNTER(&htim2);  // 
        delta = current_count - last_count2;

        if (delta > 32767) delta -= 65536;
        else if (delta < -32768) delta += 65536;

        last_count2 = current_count;

        velocity = (float)delta * (2 * 3.14 * WHEEL_RADIUS) / (ENCODER_PPR * SAMPLE_PERIOD*28);
    }
//    printf("ΔN=%d, v=%.3f m/s\r\n", delta, velocity);
    return velocity;  // 返回 m/s
}
