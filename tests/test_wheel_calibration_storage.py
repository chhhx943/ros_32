import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


COMMON = textwrap.dedent(
    r"""
    #include <stdint.h>
    #include <stdlib.h>

    #include "BSP/wheel_calibration.h"
    #include "BSP/wheel_calibration_storage.h"
    #include "BSP/wheel_calibration_storage.c"
    #include "BSP/wheel_calibration.c"

    static CalibrationData_t make_data(uint16_t version, uint16_t left_pwm)
    {
        CalibrationData_t data = {1U, 1, -1, left_pwm, 120U, version, 0U};
        return data;
    }
    """
)


class WheelCalibrationStorageHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_wheel_calibration_storage.c")
            exe = os.path.join(tmp, "test_wheel_calibration_storage.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-DWHEEL_CALIBRATION_HOST_TEST",
                    "-DWHEEL_CALIBRATION_STORAGE_HOST_TEST",
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

    def test_save_load_and_pending_commit_survive_reinitialization(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                CalibrationData_t expected = make_data(7U, 180U);
                const CalibrationData_t *active;

                Wheel_Calibration_Storage_ResetForTest();
                Wheel_Calibration_Init();
                if (Wheel_Calibration_IsValid() != 0U) return 1;
                if (Wheel_Calibration_SetPending(&expected) == 0U) return 2;
                if (Wheel_Calibration_CommitPending() == 0U) return 3;

                Wheel_Calibration_Init();
                if (Wheel_Calibration_IsValid() == 0U) return 4;
                active = Wheel_Calibration_GetActive();
                if (active->version != expected.version ||
                    active->left_min_start_pwm_permille != expected.left_min_start_pwm_permille ||
                    active->right_min_start_pwm_permille != expected.right_min_start_pwm_permille) return 5;
                return 0;
            }
            """
        )

    def test_invalid_latest_slot_falls_back_to_previous_valid_slot(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                CalibrationData_t first = make_data(1U, 100U);
                CalibrationData_t second = make_data(2U, 220U);
                CalibrationData_t loaded = {0};

                Wheel_Calibration_Storage_ResetForTest();
                if (Wheel_Calibration_Storage_Save(&first) == 0U) return 1;
                if (Wheel_Calibration_Storage_Save(&second) == 0U) return 2;
                Wheel_Calibration_Storage_CorruptSlotForTest(1U);
                if (Wheel_Calibration_Storage_Load(&loaded) == 0U) return 3;
                if (loaded.version != first.version ||
                    loaded.left_min_start_pwm_permille != first.left_min_start_pwm_permille) return 4;
                return 0;
            }
            """
        )

    def test_uncommitted_record_is_ignored(self):
        self.compile_and_run(
            COMMON
            + r"""
            int main(void)
            {
                CalibrationData_t data = make_data(3U, 140U);
                CalibrationData_t loaded = {0};

                Wheel_Calibration_Storage_ResetForTest();
                if (Wheel_Calibration_Storage_Save(&data) == 0U) return 1;
                Wheel_Calibration_Storage_DropCommitForTest(0U);
                if (Wheel_Calibration_Storage_Load(&loaded) != 0U) return 2;
                return 0;
            }
            """
        )


if __name__ == "__main__":
    unittest.main()
