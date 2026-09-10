import inspect
import pathlib
import re
import unittest

from tools.pid.transport import CAN_BITRATE, CAN_V1_BITRATE, PythonCanTransport


ROOT = pathlib.Path(__file__).resolve().parents[1]


def _value(text: str, pattern: str) -> int:
    match = re.search(pattern, text)
    if match is None:
        raise AssertionError("missing pattern: " + pattern)
    return int(match.group(1))


class CanBitrateTest(unittest.TestCase):
    def test_host_defaults_are_frozen_to_500_kbit(self):
        self.assertEqual(CAN_BITRATE, 500_000)
        self.assertEqual(CAN_V1_BITRATE, 500_000)
        self.assertIs(CAN_V1_BITRATE, CAN_BITRATE)
        self.assertEqual(
            inspect.signature(PythonCanTransport).parameters["bitrate"].default,
            CAN_V1_BITRATE,
        )
        automated = (ROOT / "tools" / "pid" / "automated_runner.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("from tools.pid.transport import CAN_V1_BITRATE", automated)
        self.assertIn("default=CAN_V1_BITRATE", automated)

    def test_autotune_cli_passes_the_canonical_bitrate(self):
        autotune = (ROOT / "tools" / "pid" / "autotune.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("from tools.pid.transport import CAN_V1_BITRATE", autotune)
        self.assertIn("--bitrate", autotune)
        self.assertIn("default=CAN_V1_BITRATE", autotune)
        self.assertIn("bitrate=args.bitrate", autotune)

    def test_stm32_timing_matches_actual_42_mhz_can_clock(self):
        ioc = (ROOT / "ros.ioc").read_text(encoding="utf-8")
        can_c = (ROOT / "Core" / "Src" / "can.c").read_text(encoding="utf-8")

        sysclk = _value(ioc, r"(?m)^RCC\.SYSCLKFreq_VALUE=(\d+)$")
        apb1_divisor = _value(ioc, r"(?m)^RCC\.APB1CLKDivider=RCC_HCLK_DIV(\d+)$")
        can_clock = sysclk // apb1_divisor
        self.assertEqual(can_clock, 42_000_000)
        self.assertEqual(
            _value(ioc, r"(?m)^RCC\.APB1Freq_Value=(\d+)$"), can_clock
        )

        prescaler = _value(can_c, r"hcan1\.Init\.Prescaler\s*=\s*(\d+)")
        bs1 = _value(can_c, r"hcan1\.Init\.TimeSeg1\s*=\s*CAN_BS1_(\d+)TQ")
        bs2 = _value(can_c, r"hcan1\.Init\.TimeSeg2\s*=\s*CAN_BS2_(\d+)TQ")
        bitrate = can_clock // (prescaler * (1 + bs1 + bs2))
        self.assertEqual((prescaler, bs1, bs2), (7, 8, 3))
        self.assertEqual(bitrate, CAN_V1_BITRATE)
        bxcan_header = (ROOT / "BSP" / "bsp_bxcan.h").read_text(encoding="utf-8")
        self.assertIn("BSP_BXCAN_V1_BITRATE                   500000U", bxcan_header)

    def test_ioc_and_protocol_freeze_rate_without_changing_safety_timing(self):
        ioc = (ROOT / "ros.ioc").read_text(encoding="utf-8")
        protocol = (ROOT / "docs" / "CAN_PROTOCOL.md").read_text(encoding="utf-8")
        self.assertIn("CAN1.CalculateBaudRate=500000", ioc)
        self.assertIn("500 kbit/s", protocol)
        self.assertNotIn("1 Mbit/s", protocol)
        self.assertIn("10 ms", protocol)
        self.assertIn("100 ms", protocol)

    def test_relevant_documents_and_bench_comment_use_500_kbit(self):
        paths = (
            ROOT / "docs" / "CHASSIS_CONTROL_DESIGN.md",
            ROOT / "docs" / "H4_HARDWARE_CHARACTERIZATION.md",
            ROOT / "docs" / "AUTOTUNE_SAFE_FIRST_L1.md",
            ROOT / "BSP" / "can_motor_bench.h",
            ROOT / "docs" / "superpowers" / "specs" /
            "2026-08-22-stm32-bottom-controller-design.md",
        )
        text = "\n".join(path.read_text(encoding="utf-8") for path in paths)
        self.assertNotRegex(text, r"1\s*Mbit/s|1,000,000\s*bit/s")


if __name__ == "__main__":
    unittest.main()
