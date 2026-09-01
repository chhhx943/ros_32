# STM32F407 AutotuneSafe PID 调参系统设计

日期：2026-09-01  
状态：已冻结，按此文档实施

## 目标与边界

AutotuneSafe 是一个仅用于架空后轮、人工监督的 PID 调参固件 profile。它允许 host/AI 提交候选参数和测试请求，但 MCU 始终拥有最终 actuator authority。没有真实电流/温度传感器时，系统只提供保守的时间/负载代理，不能声称具备真实过流保护能力。

本次不修改 Ackermann 数学，不进入 H5 地面测试，不把 AutotuneSafe 能力通过运行时 CAN flag 暴露给普通 Debug/Release。普通固件仍可保留通用 PID 数据结构和只读诊断，但不能被任何 CAN 帧切换到 autotune actuator 模式。

## 权限边界与最终输出闸门

AI 只分析结构化实验结果、提出下一组 PID 和搜索方向。host runner 只请求写参、开始/停止测试、申请等级提升、采集 telemetry 和计算 score。MCU 负责参数合法性、会话 freshness、等级、目标/PWM envelope、ramp/slew、实时 abort、E-stop、watchdog、恢复和冷却。

`AUTOTUNE_SAFE_PROFILE` 下，`BSP/autotune_safe.c/.h` 是调参策略与安全证据的唯一实现；`BSP/bsp_motor.c` 的 `Motor_Drive()` 是最终 actuator gate。所有普通 velocity、Ackermann、runtime PID tuning、calibration/static steering 等可能产生驱动输出的调用都必须经过此 gate。gate 在最终写 TB6612 PWM 前再次检查：profile 授权、会话状态、E-stop/Safety 状态、当前 level 的 PWM envelope、target/ramp/slew 结果和 abort 锁存状态。

`Motor_Coast()`、`Motor_Brake()`、`Motor_CoastAll()`、`Motor_EmergencyBrakeAll()` 是安全输出通道，任何 Safety/E-stop/abort 可以立即调用，不受 PWM slew 延迟。abort 先禁止新的 drive，再执行更高优先级安全动作；旧积分和旧输出不能在等待轮速归零期间继续驱动。

普通 Debug/Release 不定义 `AUTOTUNE_SAFE_PROFILE`，不链接 autotune actuator 授权路径，且其 CAN 接收路径不能切换 profile。静态测试检查宏、CMake source/profile 和 `Motor_Drive` 路径；Debug build 只验证普通控制功能，不验证或启用自动调参。

## MCU 状态、等级和 envelope

状态：

`LOCKED -> READY -> PREFLIGHT -> RAMP -> RUNNING -> STOPPING -> COMPLETE`；任何异常进入 `ABORT`，需要静止/冷却时进入 `COOLING`，严重 abort 保持锁定或降级。

等级由 MCU 编译期配置和本地历史决定，host 只能发送 `REQUEST_PROMOTE`，不能写 level 或 limits：

- `L0_LOCKED`：禁止驱动，只允许查询、参数检查和 recovery。
- `L1_INITIAL`：请求目标绝对值不超过 100 mm/s，PWM hard limit 默认 150‰；必要时人工修改编译期安全配置到最高 200‰。只允许 `0 -> 正速度 -> 0`。
- `L2+`：不预设 250/350/500‰ 等已确认值。每一级的 target/PWM envelope 必须在 MCU 编译期配置，并在前一级完成多轮真机安全验证后由人工确认；AI/host 永远不能动态写入。

升级必须同时满足：连续编译期配置的 N 轮完整 PASS；无 stall、overspeed、oscillation abort、saturation timeout、Safety/CAN fault；每轮正常 STOP 且实际轮速低于 stillness threshold 并持续确认；CAN/session heartbeat 和反馈完整。严重 abort 后禁止升级，必要时自动降级并进入 `COOLING`/`LOCKED`。

## PID 与输出保护

PID 候选先检查 Q8.8 定点的原始范围、解码溢出、非法符号和 NaN/Inf，再检查每项 absolute min/max。bootstrap 阶段使用人工冻结的极保守 seed PID 或已验证当前 PID 作为 safe baseline，并使用独立 absolute step limit；bootstrap 产生第一组完整实测安全结果后，才启用相对 `best_known_safe` 的单次 25% step limit。

连续失败不能把搜索区域推离 baseline：任何 Safety abort 不更新 `best_known_safe`，并让当前等级保持/降级；候选仍需以当前 safe baseline 为参照，不能以失败候选为新中心。

每次加载候选或 recovery 参数都清除 integral、previous error 和 derivative history。输出链严格为：

`requested target -> MCU target envelope -> target ramp -> PID -> PWM hard limit -> PWM slew -> final Motor_Drive gate`

target ramp 和 PWM slew 是两层独立保护；E-stop/Safety/abort 使用 COAST/BRAKE 立即停车，不等待 slew。

## 本地实时保护和 abort

stall、overspeed、oscillation、saturation 使用 MCU 本地 10 ms 控制数据，不依赖 telemetry 回传或 host 日志线程。初始阶段禁止直接反转；只有实际速度低于 stillness threshold 并持续 hold 后，才允许下一正向实验，TB6612 既有 COAST dead-time 保留。

abort 优先级从高到低为：E-stop/Safety fault、watchdog/freshness、encoder invalid、overspeed、stall、oscillation、saturation timeout、thermal/runtime proxy、协议/参数错误。动作固定为：

`禁止继续增加输出 -> actuator safety action -> 清除/冻结 PID 状态 -> 等待实际轮速归零 -> 恢复 best_known_safe -> 记录 abort -> LOCK/COOLING`

Autotune 状态机不能改变 PE1/CAN E-stop 语义；PE1 本地状态和现有 safety_manager 是唯一 E-stop 依据，host preflight 只是附加检查。

无电流/温度传感器时，MCU 本地累计单轮连续驱动、高 PWM 时间、session 运行时间和累计负载；高负载实验之间强制静止/冷却。session 重启、host 重连和 AI 重启不能清除已生效的冷却计时/累计限制。

## CAN 扩展与 snapshot

保留现有 `0x123/0x124` PID transaction 和现有实时反馈布局，在未占用 ID 上增加 autotune control/status/metrics 帧。Classic CAN 8-byte 限制下拆成带同一 `snapshot_seq`、`session_id`、`experiment_id` 的多帧快照；缺帧或 sequence 不一致时 host 不得评分。

Control 至少支持：写候选 PID、START、STOP、heartbeat、查询 status、`REQUEST_PROMOTE`。每个 session 有唯一 `session_id`，每个实验有唯一 `experiment_id`。旧 seq、重复帧、乱序帧、上一轮延迟帧不能刷新 watchdog 或污染当前 session。

Telemetry 覆盖 profile_id、session/experiment、state/level、requested/effective target、PWM limit、actual L/R、PWM L/R、PID P/I/D/output、stall、saturation、overspeed、oscillation、abort_reason、speed_limit_hit、safety_state、fault_code、last_valid_command_age。现有五帧实时反馈不重排，新增诊断帧按 snapshot sequence 关联。

## Host runner 与搜索

runner 每轮执行：确认 AutotuneSafe profile -> status/E-stop/stillness/preflight -> 加载 candidate -> MCU accepted -> `0 -> ramp -> target -> ramp -> 0` -> host 二次 abort 监测 -> STOP -> MCU 确认实际归零 -> 指标/历史/JSON/CSV。MCU abort 优先于 host 判断；host watchdog 只负责主动 STOP，不能替代 MCU session watchdog。

搜索固定为 `P-only -> PI -> 必要时小 D -> local refinement`。禁止经典 Ziegler-Nichols 持续振荡找 Ku，禁止纯随机暴力搜索和直接反转。Safety abort 获得极大 penalty，永远不能进入 best-known-safe 或 leaderboard。

## 验证和真机边界

软件阶段先完成 host simulator、MCU host-test、协议、每种 abort、host-loss/watchdog、bypass、Debug isolation、全 Python regression、Debug build 和 AutotuneSafe build。软件阶段不自动启动电机。

软件完成后才允许人工刷入 AutotuneSafe：后轮架空、E-stop 可触达、L1、目标不超过 100 mm/s、PWM 不超过 150‰（除非人工确认编译期配置为 200‰）、短时、人工监督。调参结束必须刷回普通 Debug。软件测试通过不等于 envelope 已经安全，也不授权跳级或进入 H5。
