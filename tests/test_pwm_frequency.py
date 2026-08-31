import os
import re
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as f:
        return f.read()


class PwmFrequencyTest(unittest.TestCase):
    def test_tim3_source_is_configured_for_20khz(self):
        source = read_rel("Core/Src/tim.c")
        tim3 = source.split("htim3.Instance = TIM3;", 1)[1].split(
            "/* USER CODE BEGIN TIM3_Init 2 */", 1
        )[0]

        self.assertRegex(tim3, r"htim3\.Init\.Prescaler\s*=\s*0\s*;")
        self.assertRegex(tim3, r"htim3\.Init\.Period\s*=\s*4200-1\s*;")

    def test_ioc_source_is_configured_for_20khz(self):
        ioc = read_rel("ros.ioc")
        self.assertRegex(ioc, r"(?m)^TIM3\.Prescaler=0$")
        self.assertRegex(ioc, r"(?m)^TIM3\.Period=4200-1$")


if __name__ == "__main__":
    unittest.main()
