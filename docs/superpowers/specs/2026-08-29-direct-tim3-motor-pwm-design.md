# Direct TIM3 Motor PWM Simplification

## Goal

Remove the generic PWM subsystem and let the TB6612 motor BSP control the two
used TIM3 PWM channels directly. Preserve the existing motor-facing behavior
while eliminating unused registration, handle state, ramping, status codes, and
duplicate abstraction layers.

## Scope

- Motor 1 PWM is fixed to `TIM3_CH1`.
- Motor 2 PWM is fixed to `TIM3_CH2`.
- Duty commands retain the existing `0..1000` scale.
- The existing `Motor_Init`, `Motor_Drive`, `Motor_Coast`, `Motor_Brake`,
  `Motor_CoastAll`, `Motor_EmergencyBrakeAll`, and compatibility
  `Motor_SetPWM` APIs remain available.
- TIM3 peripheral and GPIO initialization remain owned by CubeMX-generated
  timer code.
- Steering PWM and other future PWM consumers are outside this change.

## Design

`bsp_motor.c` will include `tim.h` and contain one private mapping helper from
motor number to `TIM_CHANNEL_1` or `TIM_CHANNEL_2`. A second private helper will
clamp the duty to `0..1000` and write the corresponding compare register with
`__HAL_TIM_SET_COMPARE(&htim3, channel, ...)`.

`Motor_Init()` will set both compare registers to zero, start PWM on both TIM3
channels, and put both TB6612 outputs into coast mode. Direction changes will
continue to set PWM to zero before changing the TB6612 direction pins, then
apply the requested magnitude. Coast and brake will both force PWM to zero
before setting their respective TB6612 input states.

No PWM state is duplicated in software. The TIM3 compare registers are the
single source of truth for output duty.

## Removed Components

The following generic PWM components will be deleted:

- `BSP/bsp_pwm.c`
- `BSP/bsp_pwm.h`
- `BSP/bsp_pwm_driver.c`
- `BSP/bsp_pwm_driver.h`
- `BSP/pwm_app.h`

Their entries will also be removed from CMake and the Keil project. Global
`PWM_Handle_t` instances and PWM includes will be removed from `main.c` and
`bsp_motor.h`. The direct, incorrectly typed `BSP_PWM_SetDuty` calls in the main
loop will be removed so all motor output remains owned by `bsp_motor.c`.

## Error Handling

An unsupported motor number remains a no-op, matching the current public
behavior. Duty magnitude is saturated at 1000. HAL PWM start return values are
not promoted into a new abstraction because the existing motor API returns
`void`; startup failures remain observable through normal board bring-up and
debugging rather than through unused PWM status plumbing.

## Verification

Tests will assert that:

- no source or project file references the removed PWM abstraction;
- motor 1 and motor 2 map only to TIM3 CH1 and CH2;
- motor initialization starts both channels;
- duty conversion uses the configured TIM3 period and clamps at 1000;
- direction changes, coast, and brake zero PWM before changing TB6612 state.

The complete host test suite and an embedded firmware build will be run after
the refactor.
