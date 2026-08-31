import json
import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as f:
        return f.read()


class CanMotorBenchStructureTest(unittest.TestCase):
    def test_bench_option_is_default_off_and_wired_in_cmake(self):
        cmake = read_rel("CMakeLists.txt")

        self.assertRegex(
            cmake,
            r'option\(CAN_MOTOR_BENCH_TEST\s+"[^"]+"\s+OFF\)',
        )
        self.assertIn("BSP/can_motor_bench.c", cmake)
        self.assertIn("target_compile_definitions", cmake)
        self.assertIn("if(CAN_MOTOR_BENCH_TEST)", cmake)

    def test_benchcan_preset_enables_bench_and_debug_disables_it(self):
        presets = json.loads(read_rel("CMakePresets.json"))
        configure = {p["name"]: p for p in presets["configurePresets"]}
        build = {p["name"]: p for p in presets["buildPresets"]}

        self.assertIn("BenchCan", configure)
        self.assertEqual(
            configure["BenchCan"]["cacheVariables"].get("CAN_MOTOR_BENCH_TEST"),
            "ON",
        )
        self.assertEqual(
            configure["Debug"]["cacheVariables"].get("CAN_MOTOR_BENCH_TEST"),
            "OFF",
        )
        self.assertIn("BenchCan", build)
        self.assertEqual(build["BenchCan"]["configurePreset"], "BenchCan")

    def test_main_runs_can_bench_only_under_macro(self):
        main = read_rel("Core/Src/main.c")

        self.assertIn("#elif defined(CAN_MOTOR_BENCH_TEST)", main)
        self.assertIn('#include "can_motor_bench.h"', main)
        self.assertIn("CAN_Motor_Bench_Run();", main)
        self.assertIn("#else", main)
        self.assertIn("Chassis_ControlInit();", main)
        self.assertIn("Chassis_ControlProcess(HAL_GetTick());", main)

    def test_bench_module_exposes_result_and_uses_protocol(self):
        self.assertTrue(
            os.path.exists(os.path.join(ROOT, "BSP", "can_motor_bench.h"))
        )
        self.assertTrue(
            os.path.exists(os.path.join(ROOT, "BSP", "can_motor_bench.c"))
        )

        header = read_rel("BSP/can_motor_bench.h")
        source = read_rel("BSP/can_motor_bench.c")

        self.assertIn("CAN_MOTOR_BENCH_RESULT_MAGIC", header)
        self.assertIn("CAN_MOTOR_BENCH_PHASE_COUNT", header)
        self.assertIn("CAN_MOTOR_BENCH_PHASE_COUNT         11U", header)
        self.assertIn("CAN_Motor_BenchResult_t", header)
        self.assertIn("g_can_motor_bench_result", header)
        self.assertIn("CAN_Motor_Bench_Run", header)
        self.assertIn("CAN_Motor_Bench_BuildSteeringFrame", header)
        self.assertIn("CAN_Motor_Bench_BuildWheelsFrame", header)
        self.assertIn("CAN_Motor_Bench_BuildCalibrationFrame", header)

        self.assertIn("CAN_MODE_LOOPBACK", source)
        self.assertIn("BSP_BXCAN_ID_CMD_STEERING", source)
        self.assertIn("BSP_BXCAN_ID_CMD_REAR_WHEELS", source)
        self.assertIn("BSP_BXCAN_MODE_VELOCITY", source)
        self.assertIn("BSP_BXCAN_MODE_STOP", source)
        self.assertIn("BSP_BXCAN_FLAG_ESTOP", source)
        self.assertIn("BSP_BXCAN_FLAG_RESET_FAULT", source)
        self.assertIn("Physical_EStop_IsAsserted", source)
        self.assertIn("CAN_Motor_Bench_PhysicalEstopPhase", source)
        self.assertIn("Chassis_ControlInit", source)
        self.assertIn("Chassis_ControlProcess", source)
        self.assertIn("BSP_BXCAN_GetAppliedCommandSeq", source)

    def test_send_frame_waits_bounded_for_free_tx_mailbox(self):
        """The production feedback pump burst-fills all three TX mailboxes right
        before each command group (loopback self-injection shares the mailboxes
        with the pump), so SendFrame must wait for a free mailbox with a bounded
        timeout before enqueueing, mirroring bus arbitration on a real bus."""
        header = read_rel("BSP/can_motor_bench.h")
        source = read_rel("BSP/can_motor_bench.c")
        send_frame = source.split("static uint8_t CAN_Motor_Bench_SendFrame", 1)[1]
        add_position = send_frame.index("HAL_CAN_AddTxMessage")

        self.assertIn("CAN_MOTOR_BENCH_TX_WAIT_MS", header)

        wait_block = send_frame.split("HAL_CAN_AddTxMessage", 1)[0]
        self.assertIn("HAL_CAN_GetTxMailboxesFreeLevel", wait_block)
        self.assertLess(
            wait_block.index("HAL_CAN_GetTxMailboxesFreeLevel"),
            add_position,
        )
        self.assertIn("CAN_MOTOR_BENCH_TX_WAIT_MS", wait_block)
        self.assertIn("tx_failures", wait_block)

    def test_bench_sequence_is_safe_and_ordered(self):
        source = read_rel("BSP/can_motor_bench.c")
        run_body = source.split("void CAN_Motor_Bench_Run(void)", 1)[1]

        self.assertIn("CAN_Motor_Bench_Init()", run_body)

        ordered_steps = (
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_DRIVE_MS, BSP_BXCAN_MODE_VELOCITY, CAN_MOTOR_BENCH_TARGET_MMPS, CAN_MOTOR_BENCH_TARGET_MMPS);",
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_STOP_MS, BSP_BXCAN_MODE_STOP, 0, 0);",
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_REVERSE_MS, BSP_BXCAN_MODE_VELOCITY, -CAN_MOTOR_BENCH_TARGET_MMPS, -CAN_MOTOR_BENCH_TARGET_MMPS);",
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_STOP_MS, BSP_BXCAN_MODE_STOP, 0, 0);",
            "CAN_Motor_Bench_WatchdogPhase();",
            "CAN_Motor_Bench_EstopPhase();",
            "CAN_Motor_Bench_ResetEstopPhase();",
            "CAN_Motor_Bench_PostResetDrivePhase();",
            "CAN_Motor_Bench_PhysicalEstopPhase();",
        )
        positions = []
        search_from = 0
        for step in ordered_steps:
            position = run_body.index(step, search_from)
            positions.append(position)
            search_from = position + len(step)
        self.assertEqual(positions, sorted(positions))

        init_position = run_body.index("CAN_Motor_Bench_Init()")
        self.assertLess(init_position, positions[0])

    def test_bench_exercises_closed_loop_motion_and_records_encoder_response(self):
        header = read_rel("BSP/can_motor_bench.h")
        source = read_rel("BSP/can_motor_bench.c")
        run_body = source.split("void CAN_Motor_Bench_Run(void)", 1)[1]

        self.assertIn("CAN_MOTOR_BENCH_TARGET_MMPS", header)
        self.assertIn("left_encoder_delta_after", header)
        self.assertIn("right_encoder_delta_after", header)
        self.assertIn("Encoder_Sample", source)

        ordered_steps = (
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_DRIVE_MS, BSP_BXCAN_MODE_VELOCITY, CAN_MOTOR_BENCH_TARGET_MMPS, CAN_MOTOR_BENCH_TARGET_MMPS);",
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_STOP_MS, BSP_BXCAN_MODE_STOP, 0, 0);",
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_REVERSE_MS, BSP_BXCAN_MODE_VELOCITY, -CAN_MOTOR_BENCH_TARGET_MMPS, -CAN_MOTOR_BENCH_TARGET_MMPS);",
            "CAN_Motor_Bench_StreamPhase(CAN_MOTOR_BENCH_STOP_MS, BSP_BXCAN_MODE_STOP, 0, 0);",
            "CAN_Motor_Bench_WatchdogPhase();",
            "CAN_Motor_Bench_EstopPhase();",
            "CAN_Motor_Bench_ResetEstopPhase();",
            "CAN_Motor_Bench_PostResetDrivePhase();",
            "CAN_Motor_Bench_PhysicalEstopPhase();",
        )
        positions = []
        search_from = 0
        for step in ordered_steps:
            position = run_body.index(step, search_from)
            positions.append(position)
            search_from = position + len(step)
        self.assertEqual(positions, sorted(positions))

        self.assertIn("left_sample.accumulated_counts", source)
        self.assertIn("right_sample.accumulated_counts", source)


class CanMotorBenchFrameBuilderTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_can_motor_bench.c")
            exe = os.path.join(tmp, "test_can_motor_bench.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DCAN_MOTOR_BENCH_HOST_TEST",
                    "-I",
                    ROOT,
                    src,
                    "-o",
                    exe,
                ],
                check=True,
                cwd=ROOT,
            )
            subprocess.run([exe], check=True, cwd=ROOT)

    def test_command_frames_match_protocol_vectors(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdint.h>
                #include <stdlib.h>

                #include "BSP/can_motor_bench.h"
                #include "BSP/can_motor_bench.c"

                static void expect_bytes(const uint8_t *actual, const uint8_t *expected)
                {
                    int i;
                    for (i = 0; i < 8; ++i) {
                        if (actual[i] != expected[i]) {
                            abort();
                        }
                    }
                }

                int main(void) {
                    uint8_t steering[8];
                    uint8_t wheels[8];
                    uint8_t estop[8];
                    uint8_t calibration[8];

                    /* CMD_STEERING: version, seq LE, VELOCITY, steering 0, reserved zero. */
                    CAN_Motor_Bench_BuildSteeringFrame(0x1234, 0x01, 0, steering);
                    {
                        const uint8_t expected[8] = {0x01, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00, 0x00};
                        expect_bytes(steering, expected);
                    }

                    /* CMD_STEERING: negative steering -1000 mrad as i16 LE, STOP mode. */
                    CAN_Motor_Bench_BuildSteeringFrame(0x0001, 0x00, -1000, steering);
                    {
                        const uint8_t expected[8] = {0x01, 0x01, 0x00, 0x00, 0x18, 0xFC, 0x00, 0x00};
                        expect_bytes(steering, expected);
                    }

                    /* CMD_REAR_WHEELS: +1500 and -3000 mm/s as i16 LE, VELOCITY mode. */
                    CAN_Motor_Bench_BuildWheelsFrame(0x1234, 0x01, 1500, -3000, wheels);
                    {
                        const uint8_t expected[8] = {0x01, 0x34, 0x12, 0x01, 0xDC, 0x05, 0x48, 0xF4};
                        expect_bytes(wheels, expected);
                    }

                    /* Single-frame E-stop bypass on 0x121: FLAG_ESTOP, zero targets. */
                    CAN_Motor_Bench_BuildWheelsFrame(0x0042, 0x04, 0, 0, estop);
                    {
                        const uint8_t expected[8] = {0x01, 0x42, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
                        expect_bytes(estop, expected);
                    }

                    CAN_Motor_Bench_BuildCalibrationFrame(0x1234, 0x01, 0x03, calibration);
                    {
                        const uint8_t expected[8] = {0x01, 0x34, 0x12, 0x01, 0x03, 0x5A, 0xC3, 0x00};
                        expect_bytes(calibration, expected);
                    }

                    return 0;
                }
                """
            )
        )


if __name__ == "__main__":
    unittest.main()
