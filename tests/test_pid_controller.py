import os
import subprocess
import tempfile
import textwrap
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class PidControllerHostTest(unittest.TestCase):
    def compile_and_run(self, source):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "test_pid_controller.c")
            exe = os.path.join(tmp, "test_pid_controller.exe")
            with open(src, "w", encoding="utf-8") as f:
                f.write(source)

            subprocess.run(
                [
                    "gcc",
                    "-std=c99",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
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

    def test_reset_clears_state(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdlib.h>
                #include "BSP/PID.c"

                int main(void)
                {
                    PID_t pid = {0};
                    pid.Out = 123.0f;
                    pid.Error0 = 4.0f;
                    pid.Error1 = 3.0f;
                    pid.ErrorInt = 5.0f;

                    PID_Reset(&pid);

                    if (pid.Out != 0.0f) abort();
                    if (pid.Error0 != 0.0f) abort();
                    if (pid.Error1 != 0.0f) abort();
                    if (pid.ErrorInt != 0.0f) abort();
                    return 0;
                }
                """
            )
        )

    def test_update_dt_uses_seconds_for_integral_and_clamps_output(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdlib.h>
                #include "BSP/PID.c"

                int main(void)
                {
                    PID_t pid = {0};
                    pid.Kp = 1.0f;
                    pid.Ki = 2.0f;
                    pid.Kd = 0.0f;
                    pid.OutMin = -1000.0f;
                    pid.OutMax = 1000.0f;
                    pid.Target = 100.0f;
                    pid.Actual = 40.0f;

                    PID_UpdateDt(&pid, 0.5f);

                    if (pid.ErrorInt < 29.9f || pid.ErrorInt > 30.1f) abort();
                    if (pid.Out < 119.9f || pid.Out > 120.1f) abort();

                    pid.Target = 10000.0f;
                    pid.Actual = 0.0f;
                    PID_UpdateDt(&pid, 0.5f);
                    if (pid.Out != 1000.0f) abort();
                    return 0;
                }
                """
            )
        )

    def test_conditional_integration_does_not_wind_deeper_into_saturation(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdlib.h>
                #include "BSP/PID.c"

                int main(void)
                {
                    PID_t pid = {0};
                    float saturated_integral;

                    pid.Kp = 0.0f;
                    pid.Ki = 10.0f;
                    pid.Kd = 0.0f;
                    pid.OutMin = -100.0f;
                    pid.OutMax = 100.0f;
                    pid.Target = 20.0f;
                    pid.Actual = 0.0f;

                    PID_UpdateDt(&pid, 1.0f);
                    saturated_integral = pid.ErrorInt;

                    PID_UpdateDt(&pid, 1.0f);
                    if (pid.ErrorInt != saturated_integral) abort();

                    pid.Target = -20.0f;
                    PID_UpdateDt(&pid, 1.0f);
                    if (pid.ErrorInt >= saturated_integral) abort();
                    return 0;
                }
                """
            )
        )

    def test_legacy_update_delegates_with_one_second_dt(self):
        self.compile_and_run(
            textwrap.dedent(
                r"""
                #include <stdlib.h>
                #include "BSP/PID.c"

                int main(void)
                {
                    PID_t pid = {0};
                    pid.Kp = 0.0f;
                    pid.Ki = 1.0f;
                    pid.Kd = 0.0f;
                    pid.OutMin = -1000.0f;
                    pid.OutMax = 1000.0f;
                    pid.Target = 5.0f;
                    pid.Actual = 0.0f;

                    PID_Update(&pid);

                    if (pid.ErrorInt != 5.0f) abort();
                    if (pid.Out != 5.0f) abort();
                    return 0;
                }
                """
            )
        )


if __name__ == "__main__":
    unittest.main()
