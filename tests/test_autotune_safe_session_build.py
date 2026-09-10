import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class AutotuneSafeSessionBuildTest(unittest.TestCase):
    def read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8")

    def test_smoke_source_graph_excludes_actuator_and_bus_modules(self):
        cmake = self.read("CMakeLists.txt")
        smoke_block = cmake.split("if(LOCAL_SESSION_SMOKE)", 1)[1].split(
            "else()", 1
        )[0]
        self.assertIn("BSP/autotune_safe_session.c", smoke_block)
        for name in (
            "bsp_motor.c", "chassis_control.c", "PID.c", "encoder.c",
            "servo.c", "wheel_calibration.c", "safety_manager.c",
            "bsp_bxcan.c", "can_motor_bench.c", "h4_characterization.c",
        ):
            self.assertNotIn(name, smoke_block)

    def test_smoke_generated_sources_do_not_select_normal_main_or_peripherals(self):
        cmake = self.read("cmake/stm32cubemx/CMakeLists.txt")
        smoke_block = cmake.split("if(LOCAL_SESSION_SMOKE)", 1)[1].split(
            "else()", 1
        )[0]
        self.assertIn("main_local_session_smoke.c", smoke_block)
        self.assertNotIn("Core/Src/main.c", smoke_block)
        for name in ("Core/Src/can.c", "Core/Src/tim.c", "hal_can.c", "hal_tim.c"):
            self.assertNotIn(name, smoke_block)

    def test_smoke_main_and_contract_have_no_actuator_api_reference(self):
        source = self.read("Core/Src/main_local_session_smoke.c")
        irq = self.read("Core/Src/stm32f4xx_it_local_session_smoke.c")
        msp = self.read("Core/Src/stm32f4xx_hal_msp_local_session_smoke.c")
        contract = self.read("BSP/autotune_safe_session.c")
        smoke = self.read("BSP/autotune_safe_local_smoke.c")
        for name in (
            "Motor_Drive", "Motor_Brake", "Motor_Coast", "Servo_Set",
            "Encoder_Sample", "PID_Update", "Wheel_Calibration",
            "HAL_CAN", "Chassis_Control",
        ):
            self.assertNotIn(name, source)
            self.assertNotIn(name, irq)
            self.assertNotIn(name, msp)
            self.assertNotIn(name, contract)
            self.assertNotIn(name, smoke)

    def test_linker_declares_noinit_as_noload(self):
        linker = self.read("STM32F407XX_FLASH.ld")
        self.assertIn(".noinit (NOLOAD)", linker)
        self.assertIn("*(.noinit)", linker)


if __name__ == "__main__":
    unittest.main()
