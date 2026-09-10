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

## Software evidence

```json
{
  "python_regression": {
    "command": [
      "D:\\anaconda\\python.exe",
      "-m",
      "unittest",
      "discover",
      "-s",
      "tests",
      "-q"
    ],
    "returncode": 0,
    "stdout": "",
    "stderr": "----------------------------------------------------------------------\nRan 174 tests in 52.935s\n\nOK\n"
  },
  "debug_build": {
    "command": [
      "cmake",
      "--build",
      "--preset",
      "Debug"
    ],
    "returncode": 0,
    "stdout": "ninja: no work to do.\n",
    "stderr": ""
  },
  "autotune_safe_build": {
    "command": [
      "cmake",
      "--build",
      "--preset",
      "AutotuneSafe"
    ],
    "returncode": 0,
    "stdout": "ninja: no work to do.\n",
    "stderr": ""
  },
  "passed": true,
  "images": {
    "build\\Debug\\ros.elf": {
      "bytes": 1284248,
      "sha256": "47134248b0063284d46d8465767a2a683f3aabf1de91e366c801c3b025721d9b"
    },
    "build\\AutotuneSafe\\ros.elf": {
      "bytes": 1300904,
      "sha256": "d7813d2db3e35a8384ba75adf20d555cd72270c30f4099104604a9a07828d702"
    },
    "build\\AutotuneSafe\\ros_autotune_safe.bin": {
      "bytes": 48040,
      "sha256": "5d63f373ba55c9767130c85aa568de05eb4471fe0382c377f1e6d52c12ff4589"
    }
  }
}
```
