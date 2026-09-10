import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class MotorHostTest(unittest.TestCase):
    def compile_and_run(self, body):
        with tempfile.TemporaryDirectory() as tmp:
            with open(os.path.join(tmp, "tim.h"), "w", encoding="utf-8") as handle:
                handle.write(
                    textwrap.dedent(
                        r"""
                        #ifndef TEST_TIM_H
                        #define TEST_TIM_H
                        #include <stdint.h>

                        typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
                        typedef struct { uint32_t marker; } GPIO_TypeDef;
                        extern GPIO_TypeDef gpio_b;
                        #define GPIOB (&gpio_b)
                        #define GPIO_PIN_12 (1U << 12)
                        #define GPIO_PIN_13 (1U << 13)
                        #define GPIO_PIN_14 (1U << 14)
                        #define GPIO_PIN_15 (1U << 15)

                        typedef struct {
                            struct { uint32_t Period; } Init;
                        } TIM_HandleTypeDef;
                        extern TIM_HandleTypeDef htim3;

                        #define TIM_CHANNEL_1 1U
                        #define TIM_CHANNEL_2 2U
                        uint32_t HAL_GetTick(void);
                        int HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel);
                        void HAL_GPIO_WritePin(GPIO_TypeDef *port,
                                               uint16_t pin,
                                               GPIO_PinState state);
                        void fake_set_compare(TIM_HandleTypeDef *htim,
                                              uint32_t channel,
                                              uint32_t value);
                        #define __HAL_TIM_SET_COMPARE(htim, channel, value) \
                            fake_set_compare((htim), (channel), (value))
                        #endif
                        """
                    )
                )

            source = textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>
                #include "tim.h"

                GPIO_TypeDef gpio_b = {0U};
                TIM_HandleTypeDef htim3 = {{999U}};
                static uint32_t fake_now_ms;
                static GPIO_PinState left_in1;
                static GPIO_PinState left_in2;
                static uint32_t left_compare;

                uint32_t HAL_GetTick(void) { return fake_now_ms; }
                int HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t channel)
                {
                    (void)htim;
                    (void)channel;
                    return 0;
                }
                void HAL_GPIO_WritePin(GPIO_TypeDef *port,
                                       uint16_t pin,
                                       GPIO_PinState state)
                {
                    if (port != GPIOB) return;
                    if (pin == GPIO_PIN_12) left_in1 = state;
                    if (pin == GPIO_PIN_13) left_in2 = state;
                }
                void fake_set_compare(TIM_HandleTypeDef *htim,
                                      uint32_t channel,
                                      uint32_t value)
                {
                    (void)htim;
                    if (channel == TIM_CHANNEL_1) left_compare = value;
                }

                #include "BSP/bsp_motor.c"

                """ + body + r"""
                """
            )
            source_path = os.path.join(tmp, "test_bsp_motor.c")
            exe_path = os.path.join(tmp, "test_bsp_motor.exe")
            with open(source_path, "w", encoding="utf-8") as handle:
                handle.write(source)

            subprocess.run(
                [
                    "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-I", tmp, "-I", ROOT, "-I", os.path.join(ROOT, "BSP"),
                    source_path, "-o", exe_path,
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([exe_path], check=True, cwd=ROOT)

    def test_left_forward_then_reverse_applies_tb6612_reverse_after_deadtime(self):
        self.compile_and_run(
            r"""
            int main(void)
            {
                Motor_Init();
                fake_now_ms = 10U;
                Motor_Drive(1U, 200);
                if (left_in1 != GPIO_PIN_SET || left_in2 != GPIO_PIN_RESET) return 1;
                if (left_compare == 0U) return 2;

                fake_now_ms = 20U;
                Motor_Drive(1U, -200);
                if (left_in1 != GPIO_PIN_RESET || left_in2 != GPIO_PIN_RESET) return 3;
                if (left_compare != 0U) return 4;

                Motor_Process(21U);
                if (left_in1 != GPIO_PIN_RESET || left_in2 != GPIO_PIN_RESET) return 5;
                Motor_Process(22U);
                if (left_in1 != GPIO_PIN_RESET || left_in2 != GPIO_PIN_SET) return 6;
                if (left_compare == 0U) return 7;

                /* A repeated reverse command must stay applied, not requeue. */
                fake_now_ms = 23U;
                Motor_Drive(1U, -200);
                if (left_in1 != GPIO_PIN_RESET || left_in2 != GPIO_PIN_SET) return 8;
                return 0;
            }
            """
        )


if __name__ == "__main__":
    unittest.main()
