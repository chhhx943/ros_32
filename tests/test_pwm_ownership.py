import os
import re
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as f:
        return f.read()


class PwmOwnershipTest(unittest.TestCase):
    REMOVED_PWM_FILES = (
        "BSP/bsp_pwm.c",
        "BSP/bsp_pwm.h",
        "BSP/bsp_pwm_driver.c",
        "BSP/bsp_pwm_driver.h",
        "BSP/pwm_app.h",
    )

    def test_generic_pwm_subsystem_is_removed(self):
        for rel_path in self.REMOVED_PWM_FILES:
            self.assertFalse(os.path.exists(os.path.join(ROOT, rel_path)), rel_path)

    def test_motor_driver_owns_fixed_tim3_channels(self):
        source = read_rel("BSP/bsp_motor.c")

        self.assertIn('#include "tim.h"', source)
        self.assertIn("TIM_CHANNEL_1", source)
        self.assertIn("TIM_CHANNEL_2", source)
        self.assertIn("HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)", source)
        self.assertIn("HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2)", source)
        self.assertIn("__HAL_TIM_SET_COMPARE(&htim3", source)

    def test_active_sources_and_projects_do_not_reference_pwm_layer(self):
        for rel_path in (
            "BSP/bsp_motor.c",
            "BSP/bsp_motor.h",
            "Core/Src/main.c",
            "CMakeLists.txt",
            "MDK-ARM/ros.uvprojx",
        ):
            text = read_rel(rel_path)
            self.assertNotRegex(text, r"bsp_pwm|pwm_app|PWM_Handle_t|BSP_PWM_")


if __name__ == "__main__":
    unittest.main()
