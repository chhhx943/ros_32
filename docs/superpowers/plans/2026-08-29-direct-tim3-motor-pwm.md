# Direct TIM3 Motor PWM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the generic PWM subsystem and make `bsp_motor.c` directly drive TIM3 channels 1 and 2 without changing the public motor API.

**Architecture:** CubeMX continues to initialize TIM3 and its GPIOs. The motor BSP owns the fixed motor-to-channel mapping, starts both PWM channels during `Motor_Init()`, and writes compare registers directly; no PWM handles or software state remain.

**Tech Stack:** STM32F407 HAL, C11, TIM3 PWM, Python `unittest`, CMake/Ninja, Keil project metadata

---

## File Structure

- Modify `tests/test_pwm_ownership.py`: replace old ownership expectations with direct-TIM3 and removal expectations.
- Modify `BSP/bsp_motor.c`: own TIM3 CH1/CH2 mapping, start, clamp, and compare writes.
- Modify `BSP/bsp_motor.h`: remove the PWM-layer include dependency while preserving public motor functions.
- Modify `Core/Src/main.c`: remove PWM handles, includes, and direct PWM calls.
- Modify `CMakeLists.txt`: stop compiling the removed PWM sources.
- Modify `MDK-ARM/ros.uvprojx`: remove the removed PWM files from the Keil project.
- Delete `BSP/bsp_pwm.c`, `BSP/bsp_pwm.h`, `BSP/bsp_pwm_driver.c`, `BSP/bsp_pwm_driver.h`, and `BSP/pwm_app.h`.

### Task 1: Define the simplified PWM ownership contract

**Files:**
- Modify: `tests/test_pwm_ownership.py`

- [ ] **Step 1: Replace the old wrapper-ownership tests with failing removal and direct-control tests**

```python
class PwmOwnershipTest(unittest.TestCase):
    REMOVED_PWM_FILES = (
        "BSP/bsp_pwm.c",
        "BSP/bsp_pwm.h",
        "BSP/bsp_pwm_driver.c",
        "BSP/bsp_pwm_driver.h",
        "BSP/pwm_app.h",
    )

    def test_generic_pwm_subsystem_is_removed(self):
        for rel_path in self.REMOVED_PWM_FILES:
            self.assertFalse(os.path.exists(os.path.join(ROOT, rel_path)), rel_path)

    def test_motor_driver_owns_fixed_tim3_channels(self):
        source = read_rel("BSP/bsp_motor.c")
        self.assertIn('#include "tim.h"', source)
        self.assertIn("TIM_CHANNEL_1", source)
        self.assertIn("TIM_CHANNEL_2", source)
        self.assertIn("HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)", source)
        self.assertIn("HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2)", source)
        self.assertIn("__HAL_TIM_SET_COMPARE(&htim3", source)

    def test_active_sources_and_projects_do_not_reference_pwm_layer(self):
        for rel_path in (
            "BSP/bsp_motor.c",
            "BSP/bsp_motor.h",
            "Core/Src/main.c",
            "CMakeLists.txt",
            "MDK-ARM/ros.uvprojx",
        ):
            text = read_rel(rel_path)
            self.assertNotRegex(text, r"bsp_pwm|pwm_app|PWM_Handle_t|BSP_PWM_")
```

- [ ] **Step 2: Run the focused test and verify it fails for the existing subsystem**

Run: `python -m unittest discover -s tests -p test_pwm_ownership.py -v`

Expected: FAIL because the five PWM files still exist and active sources still reference the old API.

- [ ] **Step 3: Commit the red test**

```powershell
git add -- tests/test_pwm_ownership.py
git commit -m "test: define direct TIM3 PWM ownership"
```

### Task 2: Move PWM hardware control into the motor BSP

**Files:**
- Modify: `BSP/bsp_motor.c`
- Modify: `BSP/bsp_motor.h`

- [ ] **Step 1: Remove handle ownership and add a fixed motor-to-channel mapping**

At the top of `BSP/bsp_motor.c`, replace the PWM handle declarations and `Motor_GetPwm()` with:

```c
#include "bsp_motor.h"
#include "tim.h"

static int Motor_IsValid(uint8_t num)
{
    return (num == 1U) || (num == 2U);
}

static uint32_t Motor_GetPwmChannel(uint8_t num)
{
    return (num == 1U) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}
```

- [ ] **Step 2: Add the only required PWM operation**

Add this private helper after `Motor_AbsClampPwm()`:

```c
static void Motor_SetDuty(uint8_t num, uint16_t duty)
{
    if (!Motor_IsValid(num)) {
        return;
    }
    if (duty > 1000U) {
        duty = 1000U;
    }

    __HAL_TIM_SET_COMPARE(&htim3, Motor_GetPwmChannel(num),
                          (htim3.Init.Period + 1U) * duty / 1000U);
}
```

- [ ] **Step 3: Start TIM3 CH1/CH2 directly and route all motor duty writes through the helper**

Implement initialization as:

```c
void Motor_Init(void)
{
    Motor_SetDuty(1U, 0U);
    Motor_SetDuty(2U, 0U);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    Motor_CoastAll();
}
```

Add `Motor_IsValid()` before the channel helper; it returns true only for motor
numbers 1 and 2. In `Motor_Drive`, reject `!Motor_IsValid(num)`, set duty to
zero before changing direction, then apply `Motor_AbsClampPwm(pwm)`. In
`Motor_Coast` and `Motor_Brake`, reject invalid motor numbers and call
`Motor_SetDuty(num, 0U)` before changing GPIO state. This validity helper is
required because `TIM_CHANNEL_1` is numerically zero and cannot serve as an
invalid-channel sentinel.

- [ ] **Step 4: Remove the PWM include from the public motor header**

Keep `<stdint.h>` available for both host and firmware builds and remove `#include "bsp_pwm.h"`:

```c
#include <stdint.h>
```

- [ ] **Step 5: Run the focused test and observe only subsystem-removal assertions still failing**

Run: `python -m unittest discover -s tests -p test_pwm_ownership.py -v`

Expected: the direct TIM3 assertions pass; removal/reference assertions still fail until Task 3.

- [ ] **Step 6: Commit the direct motor control**

```powershell
git add -- BSP/bsp_motor.c BSP/bsp_motor.h
git commit -m "refactor: drive motor PWM directly from TIM3"
```

### Task 3: Delete the generic PWM subsystem and all integration residue

**Files:**
- Modify: `Core/Src/main.c`
- Modify: `CMakeLists.txt`
- Modify: `MDK-ARM/ros.uvprojx`
- Delete: `BSP/bsp_pwm.c`
- Delete: `BSP/bsp_pwm.h`
- Delete: `BSP/bsp_pwm_driver.c`
- Delete: `BSP/bsp_pwm_driver.h`
- Delete: `BSP/pwm_app.h`

- [ ] **Step 1: Remove main-level PWM ownership and bypass calls**

Delete `#include "bsp_pwm.h"`, both global `PWM_Handle_t` declarations, and these two main-loop calls:

```c
BSP_PWM_SetDuty(pwm1,500);
BSP_PWM_SetDuty(pwm2,500);
```

- [ ] **Step 2: Remove PWM sources from CMake**

Delete these entries from `target_sources`:

```cmake
BSP/bsp_pwm.c
BSP/bsp_pwm_driver.c
```

- [ ] **Step 3: Remove all four PWM source/header `<File>` blocks from the Keil BSP group**

Remove the blocks whose `FileName` values are `bsp_pwm.c`, `bsp_pwm.h`, `bsp_pwm_driver.c`, and `bsp_pwm_driver.h` while leaving adjacent BSP entries unchanged.

- [ ] **Step 4: Delete the obsolete PWM files**

Delete exactly the five files listed in this task. Do not remove CubeMX TIM3 initialization from `Core/Src/tim.c` or `ros.ioc`.

- [ ] **Step 5: Run the focused test and verify it passes**

Run: `python -m unittest discover -s tests -p test_pwm_ownership.py -v`

Expected: PASS, 3 tests.

- [ ] **Step 6: Search for stale active references**

Run:

```powershell
rg -n --glob '!build/**' --glob '!MDK-ARM/ros/**' --glob '!docs/**' "bsp_pwm|pwm_app|PWM_Handle_t|BSP_PWM_|PWM_Register|PWM_SetDuty" BSP Core CMakeLists.txt MDK-ARM/ros.uvprojx
```

Expected: no matches.

- [ ] **Step 7: Commit subsystem removal**

```powershell
git add -- Core/Src/main.c CMakeLists.txt MDK-ARM/ros.uvprojx BSP/bsp_pwm.c BSP/bsp_pwm.h BSP/bsp_pwm_driver.c BSP/bsp_pwm_driver.h BSP/pwm_app.h
git commit -m "refactor: remove generic PWM subsystem"
```

### Task 4: Verify behavior and build integrity

**Files:**
- Test: `tests/test_pwm_ownership.py`
- Test: `tests/test_motor_bench_test.py`
- Test: `tests/test_chassis_control.py`
- Build inputs: `CMakeLists.txt`, `CMakePresets.json`

- [ ] **Step 1: Run the complete host test suite**

Run: `python -m unittest discover -s tests -v`

Expected: all discovered tests pass with zero errors and zero failures.

- [ ] **Step 2: Configure the embedded Release build**

Run: `cmake --preset Release`

Expected: configuration completes successfully and generates `build/Release`.

- [ ] **Step 3: Build the embedded firmware**

Run: `cmake --build --preset Release`

Expected: build exits with code 0 and produces `build/Release/ros.elf`.

- [ ] **Step 4: Check the complete diff for whitespace errors and unintended PWM changes**

Run: `git diff --check`

Expected: no whitespace errors. Confirm TIM3 remains configured for CH1/CH2 and no unrelated user changes were reverted.

- [ ] **Step 5: Commit any final test-only correction if required**

If verification required a test correction, stage only that test and commit it with:

```powershell
git add -- tests/test_pwm_ownership.py
git commit -m "test: verify direct TIM3 motor PWM"
```
