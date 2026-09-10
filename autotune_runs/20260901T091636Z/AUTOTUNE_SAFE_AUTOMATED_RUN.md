# AUTOTUNE_SAFE_AUTOMATED_RUN

状态：**SELF_LOOP_PASS**

本报告由自动执行器生成。MCU AutotuneSafe 仍是最终 actuator gate；host/AI 不能关闭、放宽或直接设置 level。

## Preflight

```json
{
  "programmer_available": true,
  "target_connected": true,
  "can_available": false,
  "can_detail": "socketcan/can0: [WinError 10047] 使用了与请求的协议不兼容的地址。",
  "independent_driver_interlock": false,
  "profile_id": null,
  "level": null,
  "pwm_limit": null
}
```

## 硬策略

- L1-only probe: `0 → 25 → 50 → 75 → 100 mm/s → 0`。
- PWM hard limit: `≤150‰`；target 只允许正方向；不请求 H5 或 L2+。
- 无独立 driver-enable 互锁时保持 low-energy-only，不扩大 envelope。
- 未确认 STOP + stillness，不恢复 Debug。

## Actions

- software validation: PASS
- ST-LINK probe: PASS
- CAN probe: FAIL: socketcan/can0: [WinError 10047] 使用了与请求的协议不兼容的地址。
- self-loop: no Flash, no CAN, no physical actuator
- self-loop: bounded P-only → PI → optional D search completed
- self-loop: simulated target ramp/PWM limit/slew and STOP completed
