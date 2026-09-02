# AutotuneSafe SWD Boot/Session Handshake Design

状态：设计基线，供下一阶段 TDD 实现使用
日期：2026-09-02
适用 profile：LOCAL_SESSION_SMOKE、AUTOTUNE_SAFE_PROFILE
当前阶段：暂停 PID 搜索、真实 PWM 扫描和电机/编码器故障判断

## 1. 目标与硬边界

本设计只解决一次实验是否真实启动、真实执行、真实产生本轮数据，以及 host 是否完整读回本轮数据。它不调 PID、不扩大 L1 envelope、不修改 Ackermann 数学，也不把最近的 actual=0 解释为电机、编码器或 PID 故障。

唯一有效流程：

~~~text
build Smoke
  -> flash + verify
  -> run
  -> MCU boot identity READY
  -> non-reset SWD attach
  -> host session request
  -> MCU ARMED
  -> host START experiment
  -> MCU RUNNING
  -> sample generation 增长
  -> MCU COMPLETE/ABORT
  -> 一次性读取 summary + ring
  -> CRC/session/experiment/generation 校验
  -> 保存结果
  -> flash + verify + run ordinary Debug
~~~

任何 boot、attach、generation、session、CRC、sample 或 restore 证据不足，都只能产生 ORCHESTRATION_FAILURE。不得产生或暗示 NO_MOTION、ENCODER_FAIL、PID_FAIL。

L1 仍由 MCU 负责：目标速度不超过 100 mm/s，PWM 不超过 150‰。host 没有写入 envelope 的接口。

## 2. 已知问题

现有工程已经有 profile-only 的 local executor、.noinit boot request、GDB/MI 字符级 prompt 处理、AutotuneSafe/Debug 双构建，以及 local mailbox、summary 和 ring。

本阶段必须解决的差距：

1. CubeProgrammer -run、外部 GDB server 和 runner 当前分裂，runner 不能证明每个边界的顺序与 no-reset attach。
2. 没有独立的 boot magic/version/profile/build/boot reason/state/generation/CRC identity。
3. 写 request 不等于 MCU 已接受 session；缺少 READY -> ARMED -> RUNNING 的 MCU ACK。
4. sample_write_index/total_sample_count 不能证明数据属于本轮 boot/session/experiment。
5. monitor 主要做末尾 header 读取，没有完整 immutable result、ring CRC 和 generation 校验。
6. 当前 local executor 包含 motor、encoder、PID 和 Safety 依赖，不满足 smoke 的编译期 actuator isolation。

因此历史 75/100‰ 证据可以保留，但最近 session 的零值不能被解释为无运动。

## 3. 总体架构

内存分成三个职责明确的区域：

~~~text
.noinit retained area
  retained boot counter + one-shot boot request

runtime mailbox
  boot identity + host request + MCU status

result area
  immutable result summary + fixed ring buffer
~~~

新增不依赖电机的 session contract 模块：

- BSP/autotune_safe_session.h
- BSP/autotune_safe_session.c

它负责 retained boot generation、boot identity、request CRC/consume-once、session state machine、session/experiment/sample generation、result commit、ring 写入和 CRC。

新增 smoke 实现：

- BSP/autotune_safe_local_smoke.c
- Core/Src/main_local_session_smoke.c

它只调用 session contract 和 synthetic sample writer。普通 AutotuneSafe local executor 继续服务未来 actuator 阶段，但不参与本阶段 smoke。

local_swd_runner.py 是唯一 host orchestration 入口，内部拆分为：

- BuildArtifacts：build、ELF/BIN、build id、git commit、binary hash；
- CubeProgrammer：identify、program、verify、run、退出等待；
- GdbServerProcess：启动、验证、关闭 GDB server；
- GdbMiSession：no-reset attach、内存读写、continue、interrupt；
- BootPoller：identity/READY 轮询；
- SessionOrchestrator：session/experiment handshake；
- ResultReader/Validator：一次性完整读取和验证；
- DebugRestorer：所有结局都恢复 ordinary Debug。

## 4. Build profile 与 actuator isolation

| Profile | 用途 | actuator |
|---|---|---|
| Debug | 普通车辆控制、最终恢复 | 不含 local Autotune executor |
| AutotuneSafe | 后续 L1 lifted-wheel actuator | MCU-owned gate，PWM <=150‰ |
| LocalSessionSmoke | 当前 SWD orchestration 验证 | 无 actuator |

LocalSessionSmoke 定义 AUTOTUNE_SAFE_PROFILE 和 LOCAL_SESSION_SMOKE，但使用独立 main 和 source list：

- 使用 main_local_session_smoke.c；
- 加入 autotune_safe_session.c、autotune_safe_local_smoke.c；
- 不加入 autotune_safe_local.c；
- 不加入 bsp_motor.c、chassis_control.c、PID.c、encoder.c、servo.c、wheel_calibration*.c、safety_manager.c；
- 不定义任何 bench 或 H4/H5 宏。

Smoke 允许链接必要的 startup、clock、HAL、GPIO 基础源，但不调用 actuator 初始化、PWM compare、CAN command 或 timer drive path。

构建后必须检查 nm 符号。Smoke ELF 不得出现：

~~~text
Motor_Drive
Motor_Brake
Motor_Coast
Servo_Set
Encoder_Sample
PID_Update
Wheel_Calibration
~~~

host test 同时检查 CMake source list，不能依赖 linker garbage collection 偶然去除 actuator object。普通 Debug ELF 也必须没有 local executor mailbox/execution symbol。

## 5. Boot identity

### 5.1 Runtime identity layout

Boot identity 固定为 40 bytes，little-endian、4-byte aligned：

| Offset | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 4 | magic | 0x42544944 |
| 4 | 2 | version | 1 |
| 6 | 2 | size | 40 |
| 8 | 4 | firmware_profile | Smoke 0xA2，AutotuneSafe 0xA1 |
| 12 | 4 | firmware_build_id | 本次 ELF 派生的 32-bit id |
| 16 | 4 | boot_generation | 本次 MCU boot 代数 |
| 20 | 4 | boot_reason | POR、software reset、watchdog 等 |
| 24 | 4 | boot_state | EARLY、INIT、READY、FAULT |
| 28 | 4 | mailbox_version | 1 |
| 32 | 4 | boot_count | retained boot 次数 |
| 36 | 4 | crc32 | offset 0..35 的 CRC |

boot state：

~~~text
BOOT_STATE_EARLY = 0
BOOT_STATE_INIT  = 1
BOOT_STATE_READY = 2
BOOT_STATE_FAULT = 3
~~~

初始化顺序：

1. Reset handler 进入 C runtime；
2. 校验 retained boot record；
3. 写 EARLY；
4. HAL_Init、clock 和必要 BSP 初始化；
5. 清零 runtime mailbox/result；
6. 校验 profile/build；
7. 写 INIT；
8. 全部成功后写 READY；
9. 失败写 FAULT，并保持 actuator 禁止。

READY 是唯一允许 host 创建 session 的 boot state。固定 sleep 永远不是 boot 证据。

### 5.2 Retained boot record

.noinit 只保留：

~~~c
typedef struct {
    uint32_t magic;
    uint32_t boot_generation;
    uint32_t boot_count;
    uint32_t last_consumed_request_id;
    uint32_t crc32;
} AutotuneRetained_t;
~~~

magic 或 CRC 无效时从 generation=1、count=1 开始；有效时两者各递增一次，零值跳过。host 只能读取，不能写 generation/count。

.noinit 不存 runtime state、PWM、PID、encoder、sample 或 result。runtime 对象每次 boot 显式清零。

## 6. .noinit boot request

### 6.1 Layout

boot request 固定为 48 bytes：

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | magic 0x42525154 |
| 4 | 2 | version 1 |
| 6 | 2 | size 48 |
| 8 | 4 | request_id |
| 12 | 4 | request_type |
| 16 | 4 | session_id |
| 20 | 4 | experiment_id |
| 24 | 12 | payload[3] |
| 36 | 4 | boot_generation_hint |
| 40 | 4 | reserved=0 |
| 44 | 4 | crc32，offset 0..43 |

request type 仅允许 NONE、BOOT_PROFILE、STOP、RECOVER。session 创建与 START 发生在 no-reset attach 后，通过 runtime request 完成，不用 reset 触发。

### 6.2 Write/consume protocol

host 按此顺序写：

1. 先清 magic；
2. 写其它字段；
3. 写 CRC；
4. 最后写 magic；
5. 执行 CubeProgrammer -run；
6. 等待 programmer 完全退出，再 attach。

MCU boot 只接受 magic、version、size、reserved、CRC 和 type 全部合法的 request。复制到局部变量后立即清空 .noinit request，更新 retained 的 last_consumed_request_id，并只执行非 actuator boot action。

重复 request_id、CRC 错误、旧 boot_generation_hint 和非法 type 都不能递增 session generation，也不能触发 actuator。

### 6.3 Linker/startup proof

linker 保持：

~~~ld
.noinit (NOLOAD) : ALIGN(4)
{
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
} >RAM
~~~

验证必须包括：

- objdump/readelf 显示 .noinit 为 NOBITS/NOLOAD 且位于 RAM；
- startup zero/copy loop 只覆盖 .data/.bss，不覆盖 .noinit；
- nm 显示 retained/request 在 .noinit；
- C host test 覆盖 CRC 与 consume-once；
- -run 前后 identity 有新 generation，request 已被清零。

## 7. Runtime mailbox 与状态机

session state 固定为：

~~~text
SESSION_BOOT      = 0
SESSION_READY     = 1
SESSION_ARMED    = 2
SESSION_RUNNING  = 3
SESSION_COMPLETE = 4
SESSION_ABORT    = 5
~~~

唯一允许的迁移：

~~~text
BOOT -> READY
READY -> ARMED -> RUNNING -> COMPLETE -> READY
READY/ARMED/RUNNING -> ABORT -> READY
~~~

state 只能由 MCU 写。host 只能写独立 request block。

runtime request 为独立 48-byte block，包含：

~~~text
magic 0x53525154
version/size
request_id
request_type = CREATE_SESSION | START_EXPERIMENT | STOP | RECOVER
session_id
experiment_id
payload
crc32
~~~

MCU status 至少包含：

~~~text
accepted_request_id
accepted_session_id
active_experiment_id
state
session_generation
sample_generation
start_sample_generation
finish_sample_generation
sample_count
expected_sample_count
last_rejected_request_id
last_reject_reason
status_crc32
~~~

host 不能把旧 accepted_session 或 request 被清零当作 ACK。只有 status CRC 正确且 ACK 字段、generation、state 同时匹配才算 ACK。

## 8. Session handshake

### 8.1 Create session

host 生成不可复用的 request_id 和 session_id。写 CREATE_SESSION 后轮询，直到：

~~~text
state == ARMED
accepted_request_id == requested_request_id
accepted_session_id == requested_session_id
session_generation > generation_before_request
boot_generation == this_boot_generation
status_crc32 valid
~~~

记录 armed_session_generation。

### 8.2 Start experiment

在 ARMED 写匹配 session 的 START_EXPERIMENT。MCU：

1. 记录 active_experiment_id；
2. 记录 start_sample_generation；
3. 清 ring/result counters；
4. 写 RUNNING；
5. 产生第一条 synthetic sample 或等待下一周期。

host 必须读回：

~~~text
state == RUNNING
accepted_session_id == requested_session_id
active_experiment_id == requested_experiment_id
session_generation == armed_session_generation
start_sample_generation == sample_generation_at_start
status_crc32 valid
~~~

全部成立才记录 EXPERIMENT_STARTED。写 request、request 被清零或看到非零 sample 都不能替代该确认。

### 8.3 Reject rules

CRC、magic、version、size、reserved、session、experiment、request_id 或当前 state 不合法时：

- state 不改变；
- session generation 不递增；
- sample generation 不递增；
- 写入 reject reason；
- host 只得到 SESSION_REQUEST_REJECTED 或对应 handshake failure。

## 9. Generation

### boot_generation

每次真实 MCU boot 递增，由 retained record 维护。session 创建和 debugger attach 不应改变它。attach 前后各读一次；变化即 ATTACH_RESET_DETECTED，停止本轮。

### session_generation

每次 MCU 接受新的、合法、未消费的 CREATE_SESSION 递增。host 不能写入它。

### sample_generation

每产生一条有效 sample 递增。每条 sample 携带当前 boot/session/experiment identity。Smoke 固定 32 条，必须满足：

~~~text
last_sample_generation - first_sample_generation + 1 == 32
sample_count == 32
~~~

RUNNING 后 deadline 内不增长，分类为 SAMPLE_PROGRESS_TIMEOUT，不是 motor failure。generation 使用无符号单调差值，禁止把正常回绕误判为回退。

## 10. Ring buffer 与 result

### 10.1 Sample layout

Smoke 和未来真实实验共用固定 40-byte entry：

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | sample_generation |
| 4 | 4 | timestamp_ms |
| 8 | 4 | session_id |
| 12 | 4 | experiment_id |
| 16 | 4 | sample_index |
| 20 | 4 | target |
| 24 | 4 | actual |
| 28 | 4 | pwm |
| 32 | 4 | state |
| 36 | 4 | flags |

Smoke capacity=64，固定产生 32 条，不覆盖 ring。真实 actuator 以后再单独扩展 TIM3/encoder/PID telemetry；本阶段不加入这些字段。

### 10.2 Immutable result layout

result 固定分配 72 bytes：

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | magic 0x52534C54 |
| 4 | 2 | version 1 |
| 6 | 2 | size 72 |
| 8 | 4 | session_id |
| 12 | 4 | experiment_id |
| 16 | 4 | boot_generation |
| 20 | 4 | session_generation |
| 24 | 4 | first_sample_generation |
| 28 | 4 | last_sample_generation |
| 32 | 4 | experiment_start_generation |
| 36 | 4 | experiment_finish_generation |
| 40 | 4 | sample_count |
| 44 | 4 | ring_start_index |
| 48 | 4 | ring_count |
| 52 | 4 | final_state |
| 56 | 4 | abort_reason |
| 60 | 4 | result_flags |
| 64 | 4 | ring_crc32 |
| 68 | 4 | result_crc32 |

offset 64 放 ring_crc32，offset 68 放 result_crc32。result CRC 排除 offset 68 自身；ring CRC 覆盖按 ring_start_index/ring_count 选出的完整 rows。实现必须使用该 72-byte layout，不能沿用旧的 64-byte 假设。

MCU 先写 result fields、ring CRC，再写 result CRC，最后写 commit magic/valid marker。下一次合法 CREATE_SESSION 前，COMPLETE result 不得修改。

### 10.3 COMPLETE proof

host 必须同时确认：

~~~text
result magic/version/size/CRC valid
result boot_generation == this_boot_generation
result session_id == requested_session_id
result experiment_id == requested_experiment_id
result session_generation == armed_session_generation
result final_state == COMPLETE
result sample_count == expected_sample_count
result ring_count == sample_count
last_sample_generation >= first_sample_generation
last_sample_generation > start_sample_generation
experiment_finish_generation > experiment_start_generation
every row session/experiment matches
sample generations strictly increase
ring CRC valid
~~~

任意失败均为 INCOMPLETE_EXPERIMENT_EVIDENCE 或数据完整性错误，禁止解释 actual。

## 11. CRC

全部 CRC 使用仓库已有 CRC-32/IEEE reflected 算法：

~~~text
initial 0xFFFFFFFF
polynomial 0xEDB88320
final xor 0xFFFFFFFF
~~~

分别计算 retained record、.noinit request、boot identity、runtime request、runtime status、ring block 和 immutable result。CRC 字段排除或按零处理。

host 先让 target 进入可读的 halted state，再一次性读取 status、result、ring；不把不同时间的字段拼成一个 snapshot。不能在不 reset 的情况下完成读取时，报告 DATA_INTEGRITY_FAIL/ATTACH_FAIL。

## 12. CubeProgrammer、GDB server 与 attach

### 12.1 Flash

runner 顺序固定为：

~~~text
identify -> program Smoke -> verify -> run -> wait process exit -> close programmer
~~~

verify 失败不允许 run、attach 或写 request。不得修改 option bytes、RDP、boot config 或其它永久设置。

### 12.2 Non-reset attach

CubeProgrammer -run 完全退出后，runner 才启动 GDB server 和 GDB/MI。每次运行记录：

- server executable 和完整参数；
- 是否 connect-under-reset；
- 是否包含 reset/halt-reset/download；
- server stdout/stderr；
- GDB attach response；
- attach 前后的 boot identity。

主路径禁止 monitor reset、monitor reset halt、connect-under-reset、重新 download、第二个 CubeProgrammer RAM read 替代 GDB session，以及 attach 失败后复用旧 mailbox。

实际安装的 ST-LINK GDB server 必须通过其 help/启动输出证明 no-reset 能力。若当前版本无法证明 non-reset attach，runner 返回 ATTACH_FAIL，停止实验，并在报告中明确工具能力不足。

attach 可以短暂 halt 读取 identity，但不能 halt+reset。Smoke 可用 interrupt/read/continue 条件轮询；未来真实 actuator 默认不在 10 ms 控制窗口周期性 halt，需使用运行态 memory read 或低侵入 MCU telemetry，并在真实 PWM 前单独验证。

### 12.3 Boot polling

attach 后使用条件轮询：

~~~text
read identity
verify magic/version/size/profile/build/CRC
if READY: continue
if FAULT: BOOT_HANDSHAKE_FAIL
if deadline: BOOT_HANDSHAKE_TIMEOUT
otherwise condition wait
~~~

READY 前禁止写 session 或 actuator request。固定 sleep 只能作为轮询间隔，不能作为成功判据。

## 13. Runner 方法与错误

local_swd_runner.py 的公共生命周期方法：

~~~text
build
flash
verify
run
attach_non_reset
wait_boot_ready
create_session
wait_armed
start_experiment
wait_running
poll_progress
wait_complete_or_abort
read_result_once
validate_result
save_artifact
restore_debug
~~~

每个方法返回命令、时间、return code、stdout/stderr 摘要和读取到的 generation。状态只由条件轮询推进。

固定错误 code：

~~~text
BUILD_FAIL
FLASH_FAIL
VERIFY_FAIL
RUN_FAIL
ATTACH_FAIL
ATTACH_RESET_DETECTED
BOOT_MAGIC_INVALID
BOOT_VERSION_MISMATCH
BOOT_PROFILE_MISMATCH
BOOT_BUILD_ID_MISMATCH
BOOT_CRC_FAIL
BOOT_HANDSHAKE_TIMEOUT
SESSION_REQUEST_REJECTED
SESSION_CRC_FAIL
SESSION_ID_MISMATCH
ARM_TIMEOUT
START_HANDSHAKE_FAIL
EXPERIMENT_ID_MISMATCH
SAMPLE_PROGRESS_TIMEOUT
SAMPLE_GENERATION_INVALID
UNEXPECTED_MCU_RESET
RESULT_CRC_FAIL
RING_CRC_FAIL
RESULT_GENERATION_MISMATCH
RESULT_SESSION_MISMATCH
RESULT_EXPERIMENT_MISMATCH
INCOMPLETE_EXPERIMENT_EVIDENCE
DATA_INTEGRITY_FAIL
RESTORE_DEBUG_FAIL
~~~

这些 code 全部属于 ORCHESTRATION_FAILURE，不参与 motor/PID diagnosis。

## 14. LOCAL_SESSION_SMOKE

smoke 固定行为：

1. boot identity 进入 READY；
2. READY 接受新的 CREATE_SESSION；
3. MCU 写 accepted session、递增 session generation、进入 ARMED；
4. ARMED 接受同 session 的 START；
5. MCU 写 active experiment/start generation，进入 RUNNING；
6. 每个周期写一条 synthetic sample；
7. actual=target，pwm=0，flags=SYNTHETIC；
8. 固定生成 32 条；
9. 计算 ring/result CRC；
10. 写 COMPLETE，回到 READY，保留 result 至下一合法 session。

smoke 不调用 Motor_Drive、Motor_Brake、Motor_Coast、Servo output、calibration、PID、encoder、CAN command、Ackermann 或 PWM compare。pwm=0 只是 synthetic 字段，不是实测 actuator telemetry。

## 15. Artifact 与 Debug restore

每次运行写入唯一目录：

~~~text
autotune_runs/YYYYMMDDTHHMMSSZ/session_smoke_NNN/
~~~

至少包含：

- run.json；
- boot_identity.json；
- session_status.json；
- result.json；
- ring.json；
- commands.json；
- validation.json；
- AUTOTUNE_SAFE_SWD_SESSION_VALIDATION.md。

artifact 必须保存 expected/actual profile、build id、ELF/BIN、binary SHA-256、git commit、target identity、GDB server args、connect-under-reset evidence、三个 generation、request/session/experiment id、sample count、result/ring CRC、final state 和 Debug restore 状态。无证据字段写 null，不能用零值冒充。

无论 PASS、FAIL、超时、异常退出或人工中止，cleanup 都必须：

1. 如仍 RUNNING，通过同一 SWD 会话请求 MCU STOP；
2. 等待非 RUNNING 和 target=0/PWM=0 evidence；
3. 关闭 GDB/MI 与 GDB server；
4. flash ordinary Debug；
5. verify Debug；
6. -run；
7. 保存 restore command evidence；
8. 检查 Debug ELF 不含 local executor symbol。

STOP 无法确认时仍要尝试 Debug restore，但报告显著标记 RESTORE_DEBUG_FAIL 或“安全状态未被 SWD 证实”。

## 16. TDD 测试矩阵

生产代码前先写 failing tests，并观察预期失败。

MCU/C host tests：

- noinit request CRC 接受/拒绝；
- request consume-once 和 duplicate reject；
- boot identity version/size/profile/build/CRC；
- retained boot generation 单调递增；
- session generation 只由合法 CREATE_SESSION 递增；
- READY/ARMED/RUNNING 迁移；
- stale session/experiment reject；
- sample generation 单调；
- 32 sample 后才 COMPLETE；
- result commit-last、ring CRC、result CRC；
- generation/session/experiment mismatch reject；
- incomplete COMPLETE reject；
- smoke source/link 不含 actuator symbols；
- Debug 不含 local executor。

Runner tests：

- build/flash/verify/run 顺序和失败短路；
- CubeProgrammer 退出后才 attach；
- reset/connect-under-reset/download 触发 ATTACH_FAIL；
- attach 前后 generation 变化触发 ATTACH_RESET_DETECTED；
- boot mismatch、ARM timeout、START mismatch、progress timeout 分类；
- full result/ring 一次性读取；
- CRC/generation/session/experiment validator；
- 任意异常触发 Debug restore；
- restore failure 显著标记；
- stale mailbox 不污染 artifact。

## 17. Smoke 10/10 与 stress 验收

每次 10 次真实经历：

~~~text
flash/verify/run
-> non-reset attach
-> READY
-> ARMED
-> RUNNING
-> 32 synthetic samples
-> COMPLETE
-> full readback
-> Debug restore
~~~

每次必须满足：

- boot handshake 正确；
- attach 未 reset；
- session/experiment 唯一且匹配；
- session generation 正确；
- sample generation 严格增长；
- sample count=32；
- ring/result CRC 正确；
- COMPLETE evidence 完整；
- 无旧 session/mailbox 污染；
- Debug restore 成功。

必须 10/10 PASS。只要一次失败，停止真实 actuator。

随后执行不驱动 actuator 的 stress：快速连续 session、session 后延迟 START、旧 request_id、重复 CREATE、重复 START、COMPLETE 后重复读、半读后重新 attach、错误 CRC request，以及各状态下的非法 request。旧消息不得创建新实验、改变 generation 或清除已完成 result。

## 18. 后续 actuator-only 准入

仅在 10/10 和 stress 通过，并且实际工具版本有 non-reset attach 证据后，才允许刷 AutotuneSafe 运动 profile，执行：

~~~text
40‰ -> 0
60‰ -> 0
75‰ -> 0
100‰ -> 0
~~~

保持后轮架空、单方向、短时、L1、PWM<=150‰、E-stop 可达，完全绕过 PID。真实 sample 另加 requested/effective/final PWM、TIM3 CCR/CEN/CCER、direction、TIM1/TIM2 CNT、delta、speed、safety/autotune state 和 abort reason。

只有 orchestration evidence 完整时才允许使用：

| Evidence | 允许结论 |
|---|---|
| requested PWM 非零、effective PWM 为零 | AutotuneSafe/state gate 路径问题 |
| effective PWM 非零、CCR 为零 | Motor/TIM 软件路径问题 |
| CCR 非零、CNT 不变 | actuator/encoder physical path unresolved，不能直接判 encoder |
| CNT 变化但 delta 为零 | encoder delta software bug |
| delta 非零但 speed 为零 | speed conversion bug |
| internal speed 非零但 stored telemetry 为零 | SWD/mailbox/logging bug |

## 19. PID 恢复条件

全部满足以下条件才恢复：

1. smoke 10/10；
2. stress smoke 通过；
3. attach/reset 生命周期可信；
4. session/experiment/generation/CRC 稳定；
5. 真实 75/100‰ 得到可信 encoder response；
6. CCR、CNT、delta、speed conversion 链正确；
7. STOP 正常；
8. Safety 未被绕过。

恢复顺序固定为 P-only -> PI -> optional D。仿真 Kp=0.25、Ki=0.60 只作为参考 seed，不是本阶段真实结果。

## 20. 实现顺序

1. 写 layout、CRC、generation、session validator 的 failing tests；
2. 实现 session contract 和 retained/noinit lifecycle；
3. 实现 LocalSessionSmoke 独立 target 与 symbol isolation；
4. 重构 local_swd_runner 的 build/flash/verify/run/attach/poll/capture/restore；
5. focused tests、完整 Python regression；
6. Debug 和 LocalSessionSmoke 构建，检查 section/symbol；
7. 真实 ST-LINK dry probe，确认 no-reset attach 能力；
8. 自动 10 次 smoke；
9. stress smoke；
10. 生成 AUTOTUNE_SAFE_SWD_SESSION_VALIDATION.md；
11. 只有验收通过后另行进入 actuator-only 计划。

本设计阶段不执行真实 PWM、不调用 PID 搜索、不判断电机/编码器故障。
