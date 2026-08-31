# PID autotune findings

- PID execution is in `BSP/chassis_control.c`, every elapsed control period of at least 10 ms, with independent `PID_t` instances for motors 1 and 2.
- Current runtime defaults are hard-coded in `Chassis_ControlConfigurePid`: `Kp=0.20`, `Ki=0.60`, `Kd=0.0`, output range `-1000..1000`.
- Existing encoder samples provide `velocity_mmps` and `trusted`; existing CAN feedback provides wheel velocity/position and diagnostics, but no current or voltage.
- Existing CAN command IDs are `0x120..0x122`; feedback IDs are `0x180..0x186`. Calibration already uses `0x122/0x185` and cannot be overloaded casually.
- Existing safety manager and CAN timeout/ESTOP paths must remain authoritative. The tuning runner must send zero STOP groups and restore safe gains after every trial and on every abort.
- No host tools directory or CAN Python dependency exists. The host implementation must be dependency-light and simulator-first.
