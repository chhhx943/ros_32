# AUTOTUNE_SAFE_SWD_SESSION_VALIDATION

日期：2026-09-03  
范围：AutotuneSafe SWD / ST-LINK boot、session、capture、restore 链路。  
当前结论：`ACTUATOR_ADMISSION = BLOCKED`。

本阶段不解释电机不动、编码器异常或 PID 失败。任何缺失 boot、attach、session、generation、CRC、sample 或 restore 证据的结果都归类为 `ORCHESTRATION_FAILURE`。

## 1. 固定流程

```text
build LocalSessionSmoke
 -> CubeProgrammer flash + verify + -rst -run
 -> 等待 CubeProgrammer 退出
 -> ST-LINK GDB server --swd --attach（不 reset target）
 -> GDB/MI attach
 -> 轮询 boot identity READY
 -> CREATE_SESSION / MCU ACK / ARMED
 -> 记录 generation_before_start
 -> START_EXPERIMENT / MCU ACK / RUNNING
 -> synthetic samples
 -> COMPLETE_LATCHED
 -> 一次性读取 status + result + ring
 -> CRC / magic / identity / generation / count 校验
 -> ACK_RESULT
 -> Debug flash + verify + -rst -run
```

runner 不把固定 `sleep()` 当作 MCU 已启动证据；READY、ARMED、RUNNING、终态和 ACK 均通过稳定 mailbox snapshot 轮询确认。

## 2. 规格修正与实现

- 终态为 `COMPLETE_LATCHED` / `ABORT_LATCHED`，持续到匹配 `ACK_RESULT` 或下一次合法 `CREATE_SESSION`；immutable result 同期保持不变。
- result 不使用含义不清的 `experiment_start_generation` / `experiment_finish_generation`。实验执行由 `experiment_id`、`start_sample_generation`、`first_sample_generation`、`last_sample_generation`、`sample_count` 和完整 ring 证明。
- MCU 在接收 START 时先写 `start_sample_generation = current sample_generation`，再提交 `RUNNING`；第一条 sample 随后才递增 sample generation。host 只要求 start generation 等于 START 前 ARMED snapshot，不要求 RUNNING 读取时 current generation 未变。
- boot identity 与 runtime status 使用 odd/even seqlock：odd 表示写入中，even 表示提交完成。host 采用 `seq1 -> snapshot -> seq2`，并要求 sequence 偶数、相等、header 正确、CRC 正确；瞬时失败 retry，超 deadline 才为 `DATA_INTEGRITY_FAIL`。
- result 使用 commit-last：按最终 magic 计算最终 CRC，RAM result.magic 先保持 0，写其它字段和 CRC，barrier 后最后写 `FINAL_MAGIC`。host 看到 magic=0 永不接受。
- `boot_generation` 只区分当前 retained RAM 生命周期；POR/SRAM 丢失时允许从 1 重建。`session_generation` 只在本次 boot 内单调；`sample_generation` 按当前 boot/session contract 单调。比较统一使用 uint32 wrap-aware delta。
- 联合身份是 `build_id + boot_generation + session_generation + session_id + experiment_id`；任一 stale/mismatch 都拒绝。

固定 layout 位于 [`BSP/autotune_safe_session.h`](BSP/autotune_safe_session.h)：boot identity 44 B、status 68 B、sample 40 B、result 68 B；`.noinit` retained/boot request 位于 linker 的 `NOLOAD` 区。

## 3. CubeProgrammer / ST-LINK 丢失点与修复

已复现并修复的链路问题：

1. 仅使用 CubeProgrammer `-run` 会恢复旧 target 执行状态，不能证明刚刷入的 image 已从 reset vector 启动。runner 改为显式 `-rst -run`，并在 CubeProgrammer 进程退出后才 attach。
2. 初版用 dummy TCP 连接测试 GDB server readiness；ST-LINK server 是单 client，该连接会消费真正的 GDB attach。现改为读取 server 的 `Waiting for debugger connection...` 输出，不再连接测试端口。
3. pipe 模式 GDB 的 `(gdb)` prompt 可能无换行。现用字符级 reader、MI async、显式 interrupt/continue。
4. LocalSessionSmoke 清理继承 NVIC 状态后曾忘记重新 enable global IRQ，SysTick 停止，导致只有一条 sample。已加入 `__enable_irq()`，并使用 smoke 专用 IRQ/MSP 文件。
5. attach server 退出后，立即使用 CubeProgrammer restore 偶发 `DEV_USB_COMM_ERR`。现保留每次命令证据，并对 Debug restore 做最多三次有界重试；这属于 ST-LINK lease teardown，不是 MCU 启动等待。

当前安装工具证据：CubeProgrammer `2.20.0`、GDB `14.2.90`、ST-LINK GDB server `7.11.0`；server help 明确包含 `--attach`。实际 no-reset argv 不含 reset、halt、erase 或 download 选项。

## 4. TDD / software evidence

已通过的 RED/GREEN cases：

- delayed COMPLETE 仍可取得终态；
- ABORT 终态不丢失；
- START 后第一条 sample 先产生仍正确握手；
- torn status 不形成合法 snapshot；
- transient CRC failure retry；
- magic-last 半写 result 永不接受；
- uint32 generation wrap；
- POR generation=1 不接受旧 artifact；
- build、boot、session、experiment 任一 identity mismatch 均拒绝。

当前 focused 结果：session contract `10/10`、runtime C harness `2/2`、runner orchestration `11/11`、build isolation `4/4`、legacy SWD `8/8`、stress lifecycle `1/1`（50 cycles）；最新全回归 `217/217 OK`（56.294 s）。runner orchestration 新增的超时注入用例确认 `timed_out`、timeout、stdout/stderr 均可落盘。

LocalSessionSmoke 构建确认：`.noinit` 为 RAM `NOBITS/NOLOAD`；Smoke ELF 不包含 Motor_Drive、Motor_Brake、Motor_Coast、Servo_Set、Encoder_Sample、PID_Update、Wheel_Calibration、Chassis_Control、HAL_CAN 等 actuator/control 路径。

## 5. 实际 session 证据

一笔完整真实 transaction 已通过：

| 字段 | 证据 |
|---|---|
| boot | `boot_generation=23`，READY，`mailbox_version=1`，boot identity CRC 正确 |
| session | `session_generation=1`，MCU 确认 ARMED |
| experiment | MCU 确认 RUNNING，`start_sample_generation=0` |
| samples | `sample_count=32`，generation `1..32` |
| terminal | `COMPLETE_LATCHED`，commit sequence 稳定 |
| result | final magic、result CRC、ring CRC 正确 |
| handoff | ACK 后 READY，Debug restore 成功 |

逐 session artifact：[`autotune_runs/20260902T083629Z_session_smoke_001_a317d668/`](autotune_runs/20260902T083629Z_session_smoke_001_a317d668/)。

此前另有一次未落盘 artifact 的现场批次在控制台显示 `10/10`，但它不能替代逐 session 审计证据。加入 artifact 保存后的第一轮在 session 2 的 Debug restore 遇到三次 `DEV_USB_COMM_ERR`；随后新批次在初始 flash 处同样遇到 `DEV_USB_COMM_ERR`。

本次按要求重新执行逐轮 artifact：session 1～9 均完成 flash/verify/run/attach/READY/ARMED/RUNNING/32 samples/COMPLETE_LATCHED/full readback/ACK/Debug restore，并分别保存 artifact；session 10 的实验与结果读取也完成，但 Debug restore 三次失败。因此严格结果为 `9/10`，不是 `10/10`。本轮目录为 `autotune_runs/20260902T093115Z_session_smoke_001_c62c2dd4` 至 `autotune_runs/20260902T093229Z_session_smoke_010_220449b4`。

随后按本轮提示词执行硬件 stress，目标 50 次，并在首个失败处停止：session 1～5 完整成功并保存 artifact；session 6 已完成 boot/attach/READY/ARMED/RUNNING/32 samples/COMPLETE_LATCHED/full readback/ACK，随后显式 detach 和 GDB server teardown 也有日志，但三次新的 ST-LINK reopen barrier 均返回 `DEV_USB_COMM_ERR`，最终归类为 `RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT`。因此硬件 stress 严格结果为 `5/50`，不是通过；没有重刷或重跑 session 以掩盖该 cleanup/ownership 失败。session 6 artifact 为 `autotune_runs/20260902T095211Z_session_smoke_006_4dd7edd4/`，其中 `restore_debug.json`、GDB/GDB server/CubeProgrammer 日志和 command timeline 可重建失败边界。

2026-09-03 按首个失败即停止规则重新执行：先完成 1 次独立 transaction 并通过，artifact 为 `autotune_runs/20260903T071454Z_session_smoke_001_91f5dab8/`；随后正式 10 次批次中 session 1～3 完整通过并保存 artifact，session 4 完成 flash/verify/run/attach/READY/ARMED/RUNNING/32 samples/COMPLETE_LATCHED/full readback/ACK，但 Debug restore 的 3 次 ST-LINK reopen 均返回 `DEV_USB_COMM_ERR`，artifact 为 `autotune_runs/20260903T071552Z_session_smoke_004_0721fea4/`。失败时无 GDB/server 残留进程、61234 端口已释放；随后只读 dry probe 恢复成功，故当前证据仍定位为间歇性 ST-LINK USB/ownership reopen 故障。严格批次结果为 `3/10`，不是 `10/10`；未继续重跑以掩盖首个失败。

为满足 fail-closed 清理边界，批次失败后另行执行普通 Debug image 恢复：首次直接 flash/verify/run 加 3 次有界重试均返回 `DEV_USB_COMM_ERR`，因此不能证明 Debug image 已写入并运行。随后只读 dry probe 仍成功枚举 ST-LINK、工具版本与 no-reset 能力，且没有残留 GDB/server 进程；这不改变板上最终 image/state 为未知的结论，需外部处理 ST-LINK/USB/目标供电或复位状态后再做一次独立 Debug restore proof。

当前只读 dry probe 能成功，但实际 CubeProgrammer connect/flash/verify/run 仍返回 `DEV_USB_COMM_ERR`。这说明当前阻断点位于 ST-LINK USB/target connection state，尚未能重新完成 Debug restore，也尚未能重新完成可审计的 10/10。

最新失败批次未能证明 Debug restore 成功；在 ST-LINK 恢复连接并重新执行成功的 Debug flash/verify/run 之前，板上当前 image/state 必须视为未知，不得启动任何 actuator。

## 6. Stress smoke

软件 stress 已完成 50 次连续生命周期，覆盖：重复/旧 CREATE、重复 START、START-after-terminal、延迟读取 latched result、ACK replay、唯一 session/experiment、result CRC/identity 不污染。

硬件 stress 已实际启动但在第 6 次首个失败处停止：`5/50`，失败码为 `RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT`。软件 stress 仍为 50 cycles PASS；硬件 50/50 未通过，不能进入 actuator admission。待 ST-LINK ownership/reopen 问题修复并重新满足逐轮 10/10 后，仍需重跑并落盘 delayed START、old request id、duplicate CREATE/START、ACK replay、read-after-latch、half-read then reattach、bad CRC request 及各状态非法 request。

## 7. 40/60/75/100‰与 PID 准入

本阶段没有有效 actuator-only 结果，因此：

| PWM | CCR | CNT | speed | 判定 |
|---:|---:|---:|---:|---|
| 40‰ | null | null | null | 未执行 |
| 60‰ | null | null | null | 未执行 |
| 75‰ | null | null | null | 未执行 |
| 100‰ | null | null | null | 未执行 |

禁止输出 `NO_MOTION`、`PID_FAIL`、`ENCODER_FAIL`。此前仿真 `Kp=0.25, Ki=0.60` 仍仅是参考 seed，不是真实结果。

只有重新取得硬件 session orchestration 连续稳定、10/10 每轮完整 artifact、stress 硬件用例通过，并再证明开环 encoder response、speed conversion、STOP 和 Safety 后，才恢复 P-only -> PI -> optional D。

## 8. 当前验收结论

```text
BOOT/SESSION CONTRACT: software PASS
NO-RESET TOOL CAPABILITY: proven by help/argv
LIVE SINGLE SESSION: PASS (artifact saved)
SOFTWARE STRESS: PASS (50 cycles)
AUDITABLE HARDWARE 10/10: NOT ACCEPTED (2026-09-03 batch stopped at 3/10; session 4 Debug restore DEV_USB_COMM_ERR)
HARDWARE STRESS: 5/50 then RESTORE_DEBUG_STLINK_RELEASE_TIMEOUT
ACTUATOR-ONLY 40/60/75/100‰: BLOCKED
PID / H4 / H5 / motor-encoder diagnosis: BLOCKED
```
