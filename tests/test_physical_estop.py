import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class PhysicalEstopHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_physical_estop.c")
            exe = os.path.join(tmp, "test_physical_estop.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DPHYSICAL_ESTOP_HOST_TEST",
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

    def test_generated_firmware_configures_pe1_as_active_low_exti(self):
        with open(os.path.join(ROOT, "Core", "Src", "gpio.c"), encoding="utf-8") as f:
            gpio = f.read()
        with open(os.path.join(ROOT, "Core", "Src", "stm32f4xx_it.c"), encoding="utf-8") as f:
            interrupts = f.read()

        self.assertIn("GPIO_PIN_1", gpio)
        self.assertIn("GPIO_MODE_IT_FALLING", gpio)
        self.assertIn("GPIO_PULLUP", gpio)
        self.assertIn("HAL_NVIC_EnableIRQ(EXTI1_IRQn)", gpio)
        self.assertIn("void EXTI1_IRQHandler(void)", interrupts)
        self.assertIn("HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1)", interrupts)
        self.assertIn("Physical_EStop_OnExti", interrupts)

    def test_active_low_exti_latches_event_until_consumed_and_release_is_not_reset(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>

                uint8_t g_physical_estop_pin_level = 1U;

                #include "BSP/physical_estop.c"

                int main(void)
                {
                    Physical_EStop_Init();
                    if (Physical_EStop_IsAsserted() != 0U) return 1;

                    g_physical_estop_pin_level = 0U;
                    Physical_EStop_Process();
                    if (Physical_EStop_IsAsserted() != 1U) return 2;
                    if (Physical_EStop_ConsumeAssertEvent() != 1U) return 3;
                    if (Physical_EStop_ConsumeAssertEvent() != 0U) return 4;

                    g_physical_estop_pin_level = 1U;
                    Physical_EStop_Process();
                    if (Physical_EStop_IsAsserted() != 0U) return 5;

                    Physical_EStop_OnExti(PHYSICAL_ESTOP_PIN_NUMBER);
                    if (Physical_EStop_IsAsserted() != 1U) return 6;
                    if (Physical_EStop_ConsumeAssertEvent() != 1U) return 7;
                    return 0;
                }
                """
            )
        )


if __name__ == "__main__":
    unittest.main()
