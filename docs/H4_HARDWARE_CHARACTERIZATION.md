# H4 Hardware Characterization Run Sheet

状态：待人工确认轮子架空、机械固定、物理 E-stop 可触达后执行。此记录不包含 H5 地面测试。

CAN V1 bench transport, when used, is fixed at 500 kbit/s (Classic CAN 2.0A,
11-bit standard ID, DLC 8); this does not alter the H4 command or safety timing.

## 边界复核

| 边界 | 软件证据 | 真机动作/结果 |
|---|---|---|
| `command_seq` 回绕 | Host test covers `0xFFFF -> 0x0000` as fresh and duplicate `0` as stale | 已完成代码/回归复核 |
| Sector6/7 掉电原子性 | CRC + commit-last + inactive-slot write; corruption/drop host tests pass | 待做受控掉电验证 |
| IWDG | 2 s register-level IWDG; feed only after scheduler work | 调度卡死已触发复位；SAFE_STOP 对照 4 s 未复位 |
| PE1/CAN E-stop | EXTI/IRQ latch, next scheduler arbitration, PWM/方向脚 safety action | 功能路径已人工确认；待做示波器/逻辑分析仪延迟测量 |

## 测试顺序

1. 单轮开环：L、R 分别 200 mm/s 等效 PWM，记录启动 PWM、稳态速度和编码器增量。
2. 双轮同速开环：`200/200`。
3. 双轮闭环同速：`200/200`。
4. 小差速闭环：`180/220`、`220/180`。
5. Ackermann 差速：`150/250`、`250/150`，使用主机计算的 steering/目标对。

每个稳态窗口记录：

`timestamp, command_seq, target_left, target_right, actual_left, actual_right, encoder_delta_left, encoder_delta_right, pwm_left, pwm_right, pid_left_P, pid_left_I, pid_left_D, pid_left_output, pid_right_P, pid_right_I, pid_right_D, pid_right_output, safety_state, fault_code`

## 判定规则

- PWM 正确、速度错误：检查电机、机械负载、编码器尺度/轮胎等效周长。
- PWM 已错误：检查 PID 或左右通道映射。
- actual 正常、encoder 异常：检查编码器极性/计数链。
- 仅瞬态错误：检查 PID；开环异常时禁止先调 PID 或 Ackermann 数学。
- 必须稳定满足 `target_left < target_right => steady_actual_left < steady_actual_right`，反向亦然，且安全硬件验收通过后，才可申请 H5。

## 实测记录

舵机只有 PWM 开环位置控制；以下“闭环”仅指后轮速度环，Ackermann 阶段记录的是转角命令，不包含舵机位置反馈。

2026-09-01 台架运行（轮子抬起、自动 H4 镜像，修复 TIM3 初始化回归后重复运行，运行器结果 `passed=1/status=0`，`tx_failures=0`，8/8 阶段完成）：

| 阶段 | timestamp ms | seq | target L/R | actual L/R | encoder Δ L/R | PWM L/R | PID P/I/D→out (L/R) | safety/fault |
|---|---:|---:|---:|---:|---:|---:|---|---|
| open L | 1319 | 0 | 200/0 | 16/0 | 87/0 | 500/0 | 0/0/0→0 ; 0/0/0→0 | STANDBY/0 |
| open R | 2638 | 0 | 0/200 | 0/14 | 0/80 | 0/500 | 0/0/0→0 ; 0/0/0→0 | STANDBY/0 |
| open both | 3957 | 0 | 200/200 | 16/14 | 86/80 | 500/500 | 0/0/0→0 ; 0/0/0→0 | STANDBY/0 |
| closed 200/200 | 5280 | 59 | 200/200 | 4/4 | 28/25 | 183/183 | 39.2/144.23/0→183.43 ; 39.2/144.19/0→183.39 | DRIVE/0 |
| closed 180/220 | 6603 | 119 | 180/220 | 4/5 | 25/29 | 165/201 | 35.2/129.97/0→165.17 ; 43/159.01/0→202.01 | DRIVE/0 |
| closed 220/180 | 7926 | 179 | 220/180 | 5/4 | 31/24 | 202/165 | 43/158.80/0→201.80 ; 35.2/130.22/0→165.42 | DRIVE/0 |
| Ackermann + | 9249 | 239 | 150/250 | 3/6 | 21/34 | 138/230 | 29.4/108.39/0→137.79 ; 49/180.61/0→229.61 | DRIVE/0 |
| Ackermann - | 10572 | 299 | 250/150 | 6/3 | 37/18 | 229/138 | 48.8/180.37/0→229.17 ; 29.4/108.60/0→138.00 | DRIVE/0 |

The first H4 run had zero CCR because `HAL_TIM_Base_Init(&htim3)` had been removed while deleting the TIM3 Base IRQ; the generated PWM MSP path therefore never enabled the TIM3 clock. Restoring Base initialization (without any Base IRQ/start) restored CCR output and encoder response. The repeated run is now physically active, but speed is only a few mm/s under this bench load and the single-sample `180/220` result is `4/5`, so the strict stable differential gate is not yet accepted. Keep the current PWM/PID mapping and measure wheel circumference, minimum-start PWM, and repeated steady windows before H5. 

## 重复稳态窗口与开环扫描

两次独立 H4 稳态记录均包含相同的差速窗口。第一次记录为 `180/220 -> 4/4`、`220/180 -> 5/4`；第二次为 `180/220 -> 4/5`、`220/180 -> 5/4`。因此方向趋势大体正确，但 `180/220` 的一个窗口未满足严格小于关系，且速度量化在几 mm/s，不能宣称稳定通过。Ackermann 两个方向分别记录为 `3/6` 与 `6/3`，同样需要更高有效速度和重复窗口。

H4 开环 PWM 扫描（轮子架空、同一台架条件，记录窗口为启动后的短稳态观察）：

| PWM 命令 | CCR（L/R，正向） | actual L/R（mm/s） | encoder Δ L/R | 结论 |
|---:|---:|---:|---:|---|
| 25 | 105/105 | 0/0 | 0/0 | 未启动 |
| 40 | 168/168 | 0/0 | 0/0 | 未启动 |
| 50 | 210/210 | 1/0 | 6/0 | 不对称/噪声 |
| 60 | 252/252 | 1/1 | 8/6 | 首次观察到双轮响应 |
| 75 | 315/315 | 1/1 | 10/9 | 较可重复的低响应 |
| 100 | 420/420 | 2/2 | 14/13 | 更稳健的台架起转点 |

这不是最终标定提交：最小启动 PWM 仍需按左右轮分别重复扫描并在实际负载下复核；当前 `PPR=500`、四倍频、减速比 `28.0:1` 为配置值，轮胎等效周长尚未完成实测闭环确认。现有软件值为半径 `33.25 mm`、周长约 `208.9 mm`，与 Ackermann 设计文档中的 `32.5 mm` 半径存在待收敛差异。

## 看门狗探针结果

`IWDG_STALL_TEST=ON` 实机运行后读到 `g_iwdg_stall_result=0x49574447`，`RCC_CSR.IWDGRSTF=1`，证明调度卡死约 2 s 后确实由 IWDG 复位。`SAFE_STOP_WATCHDOG_TEST=ON` 运行 4 s 读到 `g_safe_stop_watchdog_result=0x53414645`、安全状态 `SAFE_STOP`、故障 `0x0004 COMMAND_TIMEOUT`，RCC 无 IWDG 复位标志，证明正常 SAFE_STOP 不误触发复位。

Sector6/7 断电原子性仍只有主机损坏/未提交记录测试和 Flash 回读证据；需要在真实供电控制下对擦除、数据写入、CRC、commit marker 各阶段断电，并确认重启始终回退到上一有效槽。PE1 到 PWM/方向脚的确切响应延迟也未用示波器或逻辑分析仪测得；目前仅有代码路径和人工按下后停止的功能证据。
