# CAN 电机台架测试设计（内部回环自注入）

## 目标

在直驱 PWM 台架试验（2026-08-29）验证 TB6612 链路之后，验证更高一层的完整软件链路：
CAN 命令帧 → `bsp_bxcan` V1 协议解析 → `chassis_control` 桥接 → TB6612 方向/PWM 输出 → 电机。

试验由 `CAN_MOTOR_BENCH_TEST` 编译开关控制，默认关闭。命令来源为 F4 自身：CAN1 置于
内部回环模式（`CAN_MODE_LOOPBACK`），固件按 V1 协议构造 `0x120`+`0x121` 命令组并流式发送，
帧经硬件回环进入 RX FIFO0，由生产接收路径（RX 中断 → `BSP_BXCAN_OnRxFrame`）解析。
本试验不验证物理总线收发器与外部节点，不验证编码器与 PID 闭环。

## 机制

- 回环切换复用板上已验证模式：`HAL_CAN_DeInit(&hcan1)` → `hcan1.Init.Mode = CAN_MODE_LOOPBACK`
  → `HAL_CAN_Init(&hcan1)`，随后调用 `Chassis_ControlInit()` 原样复用生产初始化
  （电机初始化、IDLIST 过滤器、CAN 启动、RX FIFO0 中断）。
- 过滤器只接收 `0x120`/`0x121`，因此固件每 20 ms 发出的反馈帧 `0x180..0x184` 被硬件
  过滤器挡在 RX 之外，不会干扰命令解析。
- 命令看门狗为 100 ms：驱动相位必须以 20 ms 周期流式发送完整命令组（两帧背靠背，
  间隔远小于 10 ms 组窗口），`command_seq` 逐组递增，与协议规定的宿主行为一致。
- 相位结束时直接采样 TB6612 物理输出作为端到端证据：TIM3 CCR1/CCR2（PWM 占空比）
  与 PB12..PB15（TB6612 方向引脚），连同故障码与 `applied_command_seq` 记入 RAM 结果
  结构体 `g_can_motor_bench_result`，供调试器/STM32CubeProgrammer 读取。
- **TX 邮箱竞争（2026-08-30 板上首跑发现）**：回环自注入与生产反馈泵共用仅有的 3 个
  TX 邮箱。台架循环为「发命令组 → 延时 20 ms → Process（反馈泵一次灌 3 帧占满邮箱）」，
  循环回边后下一组命令在几微秒内到达，此时反馈帧仍在发送（每帧约 130 µs），
  `HAL_CAN_AddTxMessage` 因邮箱全满而失败——首跑 550 组中约 95% 被跳过（tx_failures=522），
  偶发成功的组间隔远超 100 ms 看门狗，电机驱动被清零，采样全部失败。修复：
  `CAN_Motor_Bench_SendFrame` 入队前有界等待空邮箱（`CAN_MOTOR_BENCH_TX_WAIT_MS`=5 ms，
  覆盖一次 5 帧反馈突发约 0.7 ms），模拟真实总线上的仲裁等待；超时仍计入 `tx_failures`。
- **调试读取注意**：STM32CubeProgrammer CLI 默认连接会把目标按在复位态并在会话结束
  释放时使板子重启重跑试验（表现为电机反复动作）。读取 RAM 结果或寄存器必须加
  `mode=Hotplug`（已验证不干扰目标运行）。

## 相位序列（总时长约 11.5 s）

| # | 动作 | 期望采样 |
|---|------|----------|
| 1 | 流式 VELOCITY left=+1500, right=0，3000 ms | CCR1=2100，PB12=1/PB13=0；右轮 COAST |
| 2 | 流式 STOP（全零），1000 ms | 双轮 COAST（CCR=0，引脚全 0） |
| 3 | 流式 VELOCITY left=0, right=+1500，3000 ms | CCR2=2100，PB14=0/PB15=1；左轮 COAST |
| 4 | 流式 STOP，1000 ms | 双轮 COAST |
| 5 | 流式 VELOCITY left=-1500, right=-1500，2000 ms（反转覆盖方向路径） | 双 CCR=2100，PB12=0/PB13=1，PB14=1/PB15=0 |
| 6 | 流式 STOP，1000 ms | 双轮 COAST |
| 7 | 看门狗观察：停发任何帧 300 ms（期间持续调用 `Chassis_ControlProcess`） | 双轮 COAST，fault=0x0004（命令超时） |
| 8 | 单帧 ESTOP（`0x121`，`BSP_BXCAN_FLAG_ESTOP`） | 双轮 BRAKE（引脚全 1，CCR=0），fault=0x0001 锁存，`applied_command_seq` 与相位 7 相同（协议规定 ESTOP 不更新 seq） |

速度到占空比映射沿用 `chassis_control` 开环规则：1500 mm/s → 500/1000 → CCR=(4200)·500/1000=2100。
期望 CCR 由 `(htim3.Init.Period + 1) * duty / 1000` 运行时计算，不硬编码。

## 验收与安全

- 验收条件：复位后人工观察到左轮正转 3 s → 停 1 s → 右轮正转 3 s → 停 1 s →
  双轮反转 2 s → 停 1 s → 急停刹车；RAM 结果 `magic=0xCA2B0B01, passed=1, status=0`，
  且 `fault_after[6]=0x0004`、`fault_after[7]=0x0001`。
- 测试前车轮架空、电机电源限流；测试期间不连接负载或人为阻挡转轴。
- 初始化任一步失败立即 `Motor_CoastAll()` 并进入安全空转，结果结构体记录失败码。
- 序列结束后保持安全空转（ESTOP 锁存，双轮刹车），固件测试完成后重新构建并烧录
  默认关闭 `CAN_MOTOR_BENCH_TEST` 的正常固件。
