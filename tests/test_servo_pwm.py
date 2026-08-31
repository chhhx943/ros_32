import os
import re
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as handle:
        return handle.read()


class ServoPwmConfigurationTest(unittest.TestCase):
    def test_servo_uses_tim4_ch1_pb6_at_50_hz_and_one_us_ticks(self):
        tim = read_rel("Core/Src/tim.c")
        tim_header = read_rel("Core/Inc/tim.h")
        ioc = read_rel("ros.ioc")

        self.assertIn("TIM_HandleTypeDef htim4;", tim)
        self.assertRegex(tim, r"void MX_TIM4_Init\(void\)")
        self.assertRegex(tim, r"htim4\.Init\.Prescaler\s*=\s*84-1\s*;")
        self.assertRegex(tim, r"htim4\.Init\.Period\s*=\s*20000-1\s*;")
        self.assertIn("TIM_CHANNEL_1", tim.split("void MX_TIM4_Init", 1)[1])
        self.assertIn("extern TIM_HandleTypeDef htim4;", tim_header)
        self.assertIn("void MX_TIM4_Init(void);", tim_header)
        self.assertIn("__HAL_RCC_TIM4_CLK_ENABLE();", tim)
        self.assertIn("GPIO_PIN_6", tim)
        self.assertIn("GPIO_AF2_TIM4", tim)
        tim3 = tim.split("void MX_TIM3_Init", 1)[1].split("void MX_TIM4_Init", 1)[0]
        tim3_post_init = tim.split("if(timHandle->Instance==TIM3)", 1)[1].split(
            "else if(timHandle->Instance==TIM4)", 1
        )[0]
        self.assertNotIn("TIM_CHANNEL_3", tim3)
        self.assertNotIn("GPIO_PIN_0", tim3_post_init)

        self.assertIn("Mcu.IP7=TIM4", ioc)
        self.assertIn("Mcu.IP8=TIM6", ioc)
        self.assertIn("PB6.Signal=S_TIM4_CH1", ioc)
        self.assertNotIn("PB0.Signal=S_TIM3_CH3", ioc)
        self.assertIn("TIM4.Period=20000-1", ioc)
        self.assertIn("TIM4.Prescaler=84-1", ioc)


class ServoBspConfigurationTest(unittest.TestCase):
    def test_servo_bsp_exposes_safe_neutral_and_angle_mapping(self):
        header = read_rel("BSP/servo.h")
        source = read_rel("BSP/servo.c")

        self.assertIn("void Servo_Init(void);", header)
        self.assertIn("void Servo_SetNeutral(void);", header)
        self.assertIn("void Servo_SetAngleMrad(int16_t angle_mrad);", header)
        self.assertIn("HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1)", source)
        self.assertIn("SERVO_CENTER_PULSE_US", source)
        self.assertIn("Servo_SetNeutral();", source)


if __name__ == "__main__":
    unittest.main()
