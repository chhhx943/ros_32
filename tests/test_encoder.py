import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class EncoderHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_encoder.c")
            exe = os.path.join(tmp, "test_encoder.exe")
            with open(os.path.join(tmp, "main.h"), "w", encoding="utf-8") as f:
                f.write("#ifndef TEST_MAIN_H\n#define TEST_MAIN_H\n#include <stdint.h>\n#endif\n")
            with open(os.path.join(tmp, "tim.h"), "w", encoding="utf-8") as f:
                f.write(
                    "#ifndef TEST_TIM_H\n"
                    "#define TEST_TIM_H\n"
                    "extern TIM_HandleTypeDef htim1;\n"
                    "extern TIM_HandleTypeDef htim2;\n"
                    "#endif\n"
                )
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DENCODER_HOST_TEST",
                    "-I",
                    tmp,
                    "-I",
                    ROOT,
                    "-I",
                    os.path.join(ROOT, "BSP"),
                    src,
                    "-o",
                    exe,
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([exe], check=True, cwd=ROOT)

    def test_starts_tim1_and_tim2_encoder_channels(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>

                #define TIM_CHANNEL_ALL 0xFFFFFFFFU

                typedef struct {
                    struct {
                        uint32_t Period;
                    } Init;
                    uint32_t counter;
                    uint8_t started;
                    uint32_t started_channel;
                } TIM_HandleTypeDef;

                TIM_HandleTypeDef htim1 = {{65535U}, 0U, 0U, 0U};
                TIM_HandleTypeDef htim2 = {{0xFFFFFFFFU}, 0U, 0U, 0U};

                uint32_t fake_get_counter(TIM_HandleTypeDef *htim) { return htim->counter; }
                int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel)
                {
                    htim->started = 1U;
                    htim->started_channel = channel;
                    return 0;
                }

                #define __HAL_TIM_GET_COUNTER(htim) fake_get_counter((htim))

                #include "BSP/encoder.c"

                int main(void)
                {
                    Encoder_Init();

                    if (htim1.started != 1U || htim1.started_channel != TIM_CHANNEL_ALL) abort();
                    if (htim2.started != 1U || htim2.started_channel != TIM_CHANNEL_ALL) abort();
                    return 0;
                }
                """
            )
        )

    def test_samples_tim1_left_tim2_right_velocity_and_accumulation(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>

                #define TIM_CHANNEL_ALL 0xFFFFFFFFU

                typedef struct {
                    struct {
                        uint32_t Period;
                    } Init;
                    uint32_t counter;
                } TIM_HandleTypeDef;

                TIM_HandleTypeDef htim1 = {{65535U}, 1000U};
                TIM_HandleTypeDef htim2 = {{0xFFFFFFFFU}, 2000U};

                uint32_t fake_get_counter(TIM_HandleTypeDef *htim) { return htim->counter; }
                int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel)
                {
                    (void)htim;
                    (void)channel;
                    return 0;
                }

                #define __HAL_TIM_GET_COUNTER(htim) fake_get_counter((htim))

                #include "BSP/encoder.c"

                int main(void)
                {
                    EncoderSample_t left;
                    EncoderSample_t right;

                    Encoder_Reset();
                    htim1.counter = 1560U;
                    htim2.counter = 1720U;

                    left = Encoder_Sample(1U, 10U);
                    right = Encoder_Sample(2U, 10U);

                    if (left.trusted != 1U || left.delta_counts != 560) return 1;
                    if (left.accumulated_counts != 560) return 2;
                    /* 33.25 mm radius -> approximately 208.9 mm circumference. */
                    if (left.velocity_mmps < 208 || left.velocity_mmps > 210) return 3;

                    /* TIM2 raw direction is inverted on the vehicle right wheel;
                     * the public sample must use the vehicle-forward sign. */
                    if (right.trusted != 1U || right.delta_counts != 280) return 4;
                    if (right.accumulated_counts != 280) return 5;
                    if (right.velocity_mmps < 104 || right.velocity_mmps > 105) return 6;
                    return 0;
                }
                """
            )
        )

    def test_rejects_unphysical_delta_without_updating_accumulation(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>

                #define TIM_CHANNEL_ALL 0xFFFFFFFFU

                typedef struct {
                    struct {
                        uint32_t Period;
                    } Init;
                    uint32_t counter;
                } TIM_HandleTypeDef;

                TIM_HandleTypeDef htim1 = {{65535U}, 1000U};
                TIM_HandleTypeDef htim2 = {{0xFFFFFFFFU}, 0U};

                uint32_t fake_get_counter(TIM_HandleTypeDef *htim) { return htim->counter; }
                int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t channel)
                {
                    (void)htim;
                    (void)channel;
                    return 0;
                }

                #define __HAL_TIM_GET_COUNTER(htim) fake_get_counter((htim))

                #include "BSP/encoder.c"

                int main(void)
                {
                    EncoderSample_t sample;

                    Encoder_Reset();
                    htim1.counter = 1560U;
                    sample = Encoder_Sample(1U, 10U);
                    if (sample.trusted != 1U || sample.accumulated_counts != 560) abort();

                    htim1.counter = 31560U;
                    sample = Encoder_Sample(1U, 10U);
                    if (sample.trusted != 0U) abort();
                    if (sample.accumulated_counts != 560) abort();
                    return 0;
                }
                """
            )
        )

    def test_generated_timer_config_supports_full_quadrature_counters(self):
        with open(os.path.join(ROOT, "Core", "Src", "tim.c"), encoding="utf-8") as f:
            source = f.read()
        with open(os.path.join(ROOT, "ros.ioc"), encoding="utf-8") as f:
            ioc = f.read()

        tim1 = source.split("htim1.Instance = TIM1;", 1)[1].split(
            "/* USER CODE BEGIN TIM1_Init 2 */", 1
        )[0]
        tim2 = source.split("htim2.Instance = TIM2;", 1)[1].split(
            "/* USER CODE BEGIN TIM2_Init 2 */", 1
        )[0]

        self.assertRegex(tim1, r"htim1\.Init\.Prescaler\s*=\s*0\s*;")
        self.assertRegex(tim1, r"htim1\.Init\.Period\s*=\s*65535\s*;")
        self.assertRegex(tim1, r"sConfig\.EncoderMode\s*=\s*TIM_ENCODERMODE_TI12\s*;")

        self.assertRegex(tim2, r"htim2\.Init\.Prescaler\s*=\s*0\s*;")
        self.assertRegex(tim2, r"htim2\.Init\.Period\s*=\s*0xFFFFFFFFU\s*;")
        self.assertRegex(tim2, r"sConfig\.EncoderMode\s*=\s*TIM_ENCODERMODE_TI12\s*;")

        self.assertRegex(ioc, r"(?m)^TIM1\.Prescaler=0$")
        self.assertRegex(ioc, r"(?m)^TIM1\.Period=65535$")
        self.assertRegex(ioc, r"(?m)^TIM1\.EncoderMode=TIM_ENCODERMODE_TI12$")
        self.assertRegex(ioc, r"(?m)^TIM2\.Prescaler=0$")
        self.assertRegex(ioc, r"(?m)^TIM2\.Period=0xFFFFFFFFU$")
        self.assertRegex(ioc, r"(?m)^TIM2\.EncoderMode=TIM_ENCODERMODE_TI12$")


if __name__ == "__main__":
    unittest.main()
