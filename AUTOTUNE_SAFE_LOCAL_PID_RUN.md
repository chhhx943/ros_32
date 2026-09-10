# AUTOTUNE_SAFE_LOCAL_PID_RUN

## Open-loop breakaway scan update (2026-09-02)

- Added the missing MCU-local pre-stage before PID candidates: `OPEN_LOOP_SCAN -> LOAD_CANDIDATE`.
- Fixed firmware scan points are `40/60/75/100‰`, one positive direction, 120 ms per point, with a 5 mm/s low target. Scan rows use `candidate_id=0xFF` and do not affect PID scoring.
- The first measured response records `open_loop_start_pwm`; any PID startup bias remains subject to the L1 PWM cap, slew limiter, and final actuator gate. No response aborts with `AUTOTUNE_SAFE_ABORT_NO_MOTION`; no PID can pass or become best-known-safe.
- Verification after this change: `190/190` Python tests, Debug build, and AutotuneSafe build passed.
- A new hardware flash was verified, but the connected ST-LINK reset path did not re-run application initialization/consume the retained boot request. No valid new PID session result was accepted; ordinary Debug was restored and verified.
- This is consistent with earlier evidence that approximately `75/100‰` could drive the lifted wheel: the previous PID candidates only reached `17..40‰`, below that breakaway region.

状态：**REAL_L1_RUN_COMPLETE_NO_VALID_PID_RESTORE_PENDING**

本阶段严格限定为 `AUTOTUNE_SAFE_PROFILE`、后轮架空、L1、单方向正转、目标不超过 `100 mm/s`、MCU PWM 不超过 `150‰`。没有外部 CAN 时，实验请求可通过 ST-LINK 写入 MCU 本地 RAM；CAN 不是本地执行器的依赖。当前尚未启动真实电机，因此以下“真实”字段不能填入仿真值。

## 已完成的软件安全边界

- 本地执行状态机为 `LOAD_CANDIDATE → WAIT_STILL → RAMP_UP → HOLD → RAMP_DOWN → WAIT_STILL → EVALUATE → PASS/ABORT → COOLING`。
- 每个 candidate/测试点清 PID 动态状态；另一只轮保持 `PWM=0`；目标只能 `0 → 正速度 → 0`。
- MCU 本地执行 stall、saturation、overspeed、oscillation、session watchdog、stillness、运行时间/高 PWM 冷却代理、Safety/E-stop；最终输出仍经过 `Motor_Drive()` 的 AutotuneSafe gate。
- abort 顺序为：停止继续增益/输出、停止命令、清 PID、COAST、等待轮速归零、恢复该轮 best-known-safe、记录 abort、进入 cooling。Safety abort 不进入 best 集合，也不允许本轮继续搜索。
- 本地 RAM ring buffer 记录 timestamp、wheel、candidate、PID、target/actual、encoder delta、dt、MCU/offline speed、PWM、状态和 abort flags；不使用高频阻塞日志。
- L1 candidate 顺序遵守 `P-only → PI → optional D`，没有 Ziegler–Nichols 持续振荡实验。当前固定候选为：

| candidate | phase | Kp | Ki | Kd |
|---|---:|---:|---:|---:|
| 0 | P-only | 0.17 | 0.00 | 0.00 |
| 1 | P-only | 0.19 | 0.00 | 0.00 |
| 2 | P-only | 0.21 | 0.00 | 0.00 |
| 3 | PI | 0.19 | 0.45 | 0.00 |
| 4 | PI | 0.19 | 0.60 | 0.00 |
| 5 | optional D | 0.19 | 0.60 | 0.02 |

仿真得到的 `Kp=0.25, Ki=0.60, Kd=0` 仍只作为参考 seed，没有被认定为真实最优参数。

## 编码器测速真实性

软件链路已用相同 MCU 转换函数验证：`raw delta → dt_ms → counts/rev → circumference → mm/s`，并同时记录 MCU 速度与离线重算速度。当前工程配置为 `500 PPR/channel × quadrature 4 × gearbox 28 = 56000 counts/wheel revolution`；本地记录还保存实际 `dt`、原始计数差和 circumference。

结论：**软件换算链一致；真实轮上量纲尚未实测确认**。本次尝试在 GDB 会话失联后被中止，未取得有效电机/编码器时序。若真实首轮出现几十 `mm/s` 目标却只有此前 `3–6 mm/s` 量级的反馈，必须立即 abort，先查 dt、计数倍率、极性、定点和 telemetry scale，不得提高 Kp 掩盖问题。

## 真实 characterization 与 PID 结果

| 项目 | 左轮 | 右轮 |
|---|---|---|
| PWM→speed 实测 | PENDING_REAL_RUN | PENDING_REAL_RUN |
| 起转区间 | PENDING_REAL_RUN | PENDING_REAL_RUN |
| `0→25→0` | PENDING_REAL_RUN | PENDING_REAL_RUN |
| `0→50→0` | PENDING_REAL_RUN | PENDING_REAL_RUN |
| `0→75→0` | PENDING_REAL_RUN | PENDING_REAL_RUN |
| `0→100→0` | PENDING_REAL_RUN | PENDING_REAL_RUN |
| abort 记录 | PENDING_REAL_RUN | PENDING_REAL_RUN |
| 最终 PID | PENDING_REAL_RUN | PENDING_REAL_RUN |
| deadzone/feedforward | PENDING_REAL_RUN | PENDING_REAL_RUN |

旧的 `25/40/50/60/75/100‰` 起转观察只作为待验证假设，不在代码中硬编码为事实。

真实通过条件仍需分别得到左右轮 best PID，并完成 `50/50`、`75/75`、`100/100` 双轮验证及 `60/80`、`80/60`、`70/100`、`100/70` 小差速验证。低于目标的轮速关系必须与目标关系一致。H4 超出 L1 的部分保持：`H4_FULL_PENDING_HIGHER_ENVELOPE`。

## 运行入口

默认只打印请求计划，不写 MCU：

```powershell
python tools/pid/local_swd_runner.py --request start --wheel 1
```

只有完成后轮架空、E-stop 可立即触达、L1、短时、人工监督等物理 preflight 后，才可显式使用 `--execute`。工具只写 `request_command/request_sequence/request_wheel/request_magic`，不提供 level、PWM limit、速度上限或 Safety threshold 写入口。

停止请求：

```powershell
python tools/pid/local_swd_runner.py --request stop
```

请求写入后可追加 `--read-status` 读取 MCU RAM mailbox；完整 summary/样本需通过 SWD 读取并等待 `state=COMPLETE` 或 abort 后 stillness/cooling。工具写入完成会 resume MCU；host 不得依据自身超时继续发下一 candidate。

## 当前软件证据

- Python regression：当前实现变更后的最终结果见下方收尾验证。
- GDB/MI、本地 boot handoff、请求保留、heartbeat 序号与 STOP keepalive 定向测试均已通过。
- Debug build：PASS；普通 Debug 未链接 `g_autotune_safe_local_control` 本地执行入口。
- AutotuneSafe build：PASS；仅该 profile 链接本地执行器。
- 曾刷写 AutotuneSafe 并发出一次左轮本地 START 请求，但 GDB server 会话随后失联；mailbox 未消费，不能证明电机曾运行，因此该轮实验作废。未进入右轮。
- 本次硬件 retry 已实际完成 AutotuneSafe L1 左轮自回环；收尾时必须刷回普通 Debug 并复位。
- 尚无有效真实 PWM→speed/温升/电流数据。

因此本报告不能给出左/右最终 PID，也不能声称电机、TB6612 或堵转电流已经通过真实验证。没有电流/温度传感器时，软件运行时间/高 PWM 冷却只是保守 proxy，不等于真实电流保护。

## 后续监控修复与当前硬件状态（2026-09-01）

- 已定位并修复 SWD 监控器的两个问题：GDB/MI 现在在同一会话启用 `mi-async`，通过 `-exec-interrupt --all` 有界轮询；输出读取器按字符识别无换行的 `(gdb)` prompt。超时会在同一会话写 STOP。
- 只读复现证明：普通 CubeProgrammer RAM 重连会触发复位，可能在初始化前读到全零；单会话 GDB 能读到正常的 `status_magic`、PC 和 Safety 状态。
- 修复后已执行左轮 START `2026090211`；它完成了完整候选流程但无有效运动响应，未产生 PID。当前仍是 AutotuneSafe L1/空闲/零输出，收尾应刷回普通 Debug 并复位确认。

监控命令需要先启动 ST-LINK GDB server（端口 `61234`），然后仅允许 L1 左/右单轮命令，例如：

```powershell
python tools/pid/local_swd_runner.py --request start --wheel 1 --sequence 123 --execute --monitor-seconds 130 --poll-ms 1000
```

该命令不接受 level、PWM limit 或 Safety threshold 参数；GDB 连接失败、状态读失败或监控超时均不得接受为 PID 结果。

## L1 self-loop hardware retry (2026-09-02)

- `CubeProgrammer -run` was tested with a non-motion RAM marker and confirmed to reset/clear the ordinary `.bss` mailbox. Added a profile-only `.noinit` boot handoff (`g_autotune_safe_local_boot_request`) so the request survives that reset and is consumed once by MCU startup; the host cannot write level, PWM, or safety limits.
- Fixed the first local heartbeat sequence collision. The initial point heartbeat and first control heartbeat now use a monotonic MCU-local sequence; a repeated sequence remains a watchdog abort.
- Fixed zero-command freshness during cooling/complete states. Local STOP keepalive is sent as an explicit STOP command, preventing an old zero command from becoming a CAN timeout while no wheel is allowed to drive. Any abort now ends the local session after its cooling window.
- Flash/verify succeeded with AutotuneSafe image, then left-wheel-only START sequence `2026090211` was issued through ST-LINK. The MCU completed all 6 fixed candidates and 625 local samples, then returned to `READY`, L1 (`100 mm/s`, `150‰`), `STANDBY`, `COAST`; safety fault was zero.
- Candidate summaries: all six were `valid=1`, `passed=0`, `abort_reason=0`; max PWM was 17/19/21/30/34/34‰; every sample reported actual speed 0 and peak-to-peak 0. No candidate is eligible as `best-known-safe`, and no real PID is produced.
- This is a valid safe negative result: the wheels did not provide usable motion/encoder response at the tested output. Do not increase PWM or Kp to force motion. Recheck motor enable/power, encoder wiring/polarity, and the conservative open-loop start characterization under a separately reviewed firmware change.
- No right-wheel run, level promotion, H4/H5, Ackermann change, or AI-led aggressive search was performed. The board must be restored to ordinary Debug before handoff.
- Final handoff action completed: ordinary `Debug` image was flashed and verified at `0x08000000`; normal Debug ELF contains no `g_autotune_safe_local_control` symbol, so its AutotuneSafe local actuator entry is not present.

## Second L1 retry (2026-09-02)

- Repeated the same left-wheel-only L1 session as `2026090212`, with no PWM or level increase. The MCU completed all six fixed candidates and 627 samples, then returned to `READY`, `STANDBY`, `COAST`, with fault code `0`.
- All six summaries again reported `passed=0`, `max_speed=0`, `peak_to_peak=0`, and actual speed `0`; maximum PWM remained only `17/19/21/30/34/40‰`. No PID was accepted or emitted.
- Ordinary Debug was flashed and verified again after this retry. The board is not left in AutotuneSafe mode.
