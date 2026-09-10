import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class AutotuneSafeCanRoutingTest(unittest.TestCase):
    def read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8")

    def test_mcu_can_router_consumes_autotune_control_only_in_safe_profile(self):
        source = self.read("BSP/bsp_bxcan.c")
        self.assertIn("AUTOTUNE_SAFE_ID_CMD_CONTROL", source)
        self.assertIn("AutotuneSafe_OnCanFrame", source)
        guarded = source.split("AUTOTUNE_SAFE_ID_CMD_CONTROL", 1)[0]
        self.assertIn("#ifdef AUTOTUNE_SAFE_PROFILE", guarded[-500:])

    def test_normal_build_does_not_compile_autotune_can_actuator_entry(self):
        cmake = self.read("CMakeLists.txt")
        main = self.read("Core/Src/main.c")
        self.assertIn("if(AUTOTUNE_SAFE_PROFILE)", cmake)
        self.assertIn("#ifdef AUTOTUNE_SAFE_PROFILE", main)
        self.assertIn("AUTOTUNE_SAFE_PROFILE", self.read("BSP/autotune_safe.c"))


if __name__ == "__main__":
    unittest.main()
