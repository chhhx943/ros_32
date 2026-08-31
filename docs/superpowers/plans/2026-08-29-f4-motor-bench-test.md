# F4 Motor Bench Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a compile-time guarded firmware test that drives motor 1, stops it, then drives motor 2, while leaving the normal control firmware unchanged by default.

**Architecture:** Keep the test in the existing `main.c` startup path behind `MOTOR_BENCH_TEST`. Reuse `Motor_Init`, `Motor_Drive`, and `Motor_Coast`; do not add a second motor/PWM implementation. CMake defaults the option off and enables it only for the temporary bench image.

**Tech Stack:** STM32F4 HAL/CubeMX, CMake/Ninja, Python `unittest`, STM32CubeProgrammer CLI.

---

### Task 1: Add the failing bench-test structure test

**Files:**
- Create: `tests/test_motor_bench_test.py`

- [ ] **Step 1: Write the failing test**

Create a host-side structural test that requires the CMake option to default to off and requires `main.c` to contain this ordered sequence:

```python
import os
import re
import unittest


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def read_rel(path):
    with open(os.path.join(ROOT, path), encoding="utf-8", errors="ignore") as f:
        return f.read()


class MotorBenchTestStructure(unittest.TestCase):
    def test_bench_option_is_default_off_and_wired_in_cmake(self):
        cmake = read_rel("CMakeLists.txt")
        self.assertRegex(
            cmake,
            r'option\(MOTOR_BENCH_TEST\s+"[^"]+"\s+OFF\)',
        )
        self.assertIn("MOTOR_BENCH_TEST", cmake)

    def test_main_runs_motors_sequentially_then_coasts(self):
        main = read_rel("Core/Src/main.c")
        self.assertIn("#ifdef MOTOR_BENCH_TEST", main)
        self.assertIn("Motor_Init();", main)
        self.assertIn("Motor_CoastAll();", main)

        ordered_steps = (
            "Motor_Drive(1U, 200);",
            "HAL_Delay(3000);",
            "Motor_Coast(1U);",
            "HAL_Delay(1000);",
            "Motor_Drive(2U, 200);",
            "HAL_Delay(3000);",
            "Motor_Coast(2U);",
        )
        positions = [main.index(step) for step in ordered_steps]
        self.assertEqual(positions, sorted(positions))
        self.assertRegex(main, r"Motor_Coast\(2U\);\s*while \(1\)")
        self.assertIn("Chassis_ControlInit();", main)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run it to verify it fails**

Run:

```powershell
python -m unittest discover -s tests -p test_motor_bench_test.py -v
```

Expected: FAIL because `MOTOR_BENCH_TEST` is not yet present in `CMakeLists.txt` or `main.c`.

### Task 2: Implement the guarded firmware sequence

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `Core/Src/main.c`

- [ ] **Step 1: Add the default-off CMake option**

Add:

```cmake
option(MOTOR_BENCH_TEST "Run the one-shot sequential motor bench test." OFF)
```

and add this conditional definition alongside the existing loopback option:

```cmake
if(MOTOR_BENCH_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        MOTOR_BENCH_TEST
    )
endif()
```

- [ ] **Step 2: Add the one-shot sequence before the normal control path**

In `Core/Src/main.c`, place this branch before the existing loopback branch:

```c
#ifdef MOTOR_BENCH_TEST
  Motor_Init();
  Motor_CoastAll();
  Motor_Drive(1U, 200);
  HAL_Delay(3000);
  Motor_Coast(1U);
  HAL_Delay(1000);
  Motor_Drive(2U, 200);
  HAL_Delay(3000);
  Motor_Coast(2U);
  while (1)
  {
    Motor_CoastAll();
  }
#elif defined(BSP_BXCAN_RUN_LOOPBACK_SELF_TEST)
```

Keep the existing loopback `#else` chain and normal `Chassis_ControlInit()` path intact. The normal main-loop processing remains excluded only by `BSP_BXCAN_RUN_LOOPBACK_SELF_TEST`; the bench branch never reaches that loop because it stays in its safe infinite loop.

- [ ] **Step 3: Run the focused test to verify it passes**

Run:

```powershell
python -m unittest discover -s tests -p test_motor_bench_test.py -v
```

Expected: 2 tests pass.

### Task 3: Build, flash, observe, and restore

**Files:**
- Build outputs: `build/Bench/ros.elf`, `build/Debug/ros.elf`

- [ ] **Step 1: Configure and build the bench image**

Run from the repository root:

```powershell
cmake -S . -B build/Bench -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMOTOR_BENCH_TEST=ON
cmake --build build/Bench --target ros
```

Expected: configure succeeds and the `ros` target links with exit code 0.

- [ ] **Step 2: Flash the bench image**

Run:

```powershell
D:\stm32cubeclt\STM32CubeCLT_1.19.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe -c port=SWD freq=4000 -w build/Bench/ros.elf -v -rst
```

Expected: ST-LINK connects, the image downloads, verification succeeds, and the target resets. Observe that motor 1 runs first for 3 seconds, both outputs are quiet for 1 second, and motor 2 then runs for 3 seconds.

- [ ] **Step 3: Restore the ordinary firmware**

Configure and build with the option explicitly off:

```powershell
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMOTOR_BENCH_TEST=OFF -DBSP_BXCAN_RUN_LOOPBACK_SELF_TEST=OFF
cmake --build build/Debug --target ros
```

Flash the restored image with the same STM32CubeProgrammer command, replacing `build/Bench/ros.elf` with `build/Debug/ros.elf`.

- [ ] **Step 4: Run the full host suite and inspect the diff**

Run:

```powershell
python -m unittest discover -s tests -v
git diff --check
git status --short
```

Expected: all host tests pass, `git diff --check` is clean, and only the intended test/build-source changes are present in addition to the pre-existing dirty worktree.

