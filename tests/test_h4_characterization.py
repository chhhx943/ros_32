import json
import os
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as handle:
        return handle.read()


class H4CharacterizationStructureTest(unittest.TestCase):
    def test_h4_is_opt_in_and_exports_required_record_fields(self):
        cmake = read_rel("CMakeLists.txt")
        presets = json.loads(read_rel("CMakePresets.json"))
        configure = {item["name"]: item for item in presets["configurePresets"]}
        header = read_rel("BSP/h4_characterization.h")
        source = read_rel("BSP/h4_characterization.c")

        self.assertRegex(cmake, r'option\(H4_CHARACTERIZATION_TEST\s+"[^"]+"\s+OFF\)')
        self.assertRegex(cmake, r'H4_OPEN_LOOP_PWM.*CACHE STRING')
        self.assertIn("H4Characterization", configure)
        for field in (
            "timestamp_ms", "command_seq", "target_left_mmps", "actual_left_mmps",
            "encoder_delta_left", "pwm_left", "pid_left_p", "pid_left_i",
            "pid_left_d", "pid_left_output", "tim3_ccr_left", "tim3_ccr_right",
            "tb6612_pins", "safety_state", "fault_code",
        ):
            self.assertIn(field, header)
        self.assertIn("H4_RunOpenLoopPhase", source)
        self.assertIn("H4_RunClosedLoopPhase", source)
        self.assertIn("H4_PHASE_ACKERMANN_LEFT_INNER", source)
        self.assertIn("#ifndef H4_OPEN_LOOP_PWM", source)


if __name__ == "__main__":
    unittest.main()
