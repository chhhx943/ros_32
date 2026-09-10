# STM32 底层控制器设计

日期：2026-08-22  
目标平台：STM32F407VET6  
适用协议：CAN Protocol V1  
状态：设计已确认，待实施计划

## 1. 目标与范围

本设计定义 R3X 阿克曼小车 STM32 底层控制器。ROS2 主控通过 CAN 下发等效前轮转角与左右后轮线速度；STM32 负责协议校验、转向舵机映射、左右后轮闭环、编码器采样、安全停车、故障管理与周期反馈。

本阶段只定义架构和行为，不改动固件实现。现有 `PID.c`、`encoder.c`、`bsp_motor.c` 和 CAN 协议代码均作为已有资产复用和演进，不另建功能重复的平行实现。

## 2. 已确认的硬件与总线配置

| 功能 | 资源 | 约定 |
|---|---|---|
| CAN | CAN1，PB8/PB9 remap（PB8 RX，PB9 TX） | Classic CAN 2.0A，500 kbit/s，11-bit ID |
| 左轮编码器 | TIM1，PE9/PE11 | 正交编码器；实际方向由标定配置确定 |
| 右轮编码器 | TIM2，PA0/PA1 | 正交编码器；实际方向由标定配置确定 |
| 左右电机 PWM | TIM3_CH1/CH2，PA6/PA7 | 20 kHz，占空比内部统一为 0..1000 |
| TB6612 方向 | PB12-PB15 | 沿用现有接线；不增加 MCU 控制的 STBY |
| 转向舵机 | TIM4_CH1，PB6 | 标准舵机 PWM，50 Hz，比较值单位为 us |
| 物理急停 | PE1/EXTI1 | 上拉、低有效、常闭回路、断线即急停 |
| 调度基准 | TIM6 | 1 kHz，只生成事件，不在 ISR 中运行控制算法 |

TB6612 模块应将 STBY 硬件拉高，必须在台架阶段验证。TB6612FNG 支持最高 100 kHz PWM，20 kHz 载波在其能力范围内。电机持续电流和堵转电流必须按实际电机与模块散热条件验证，不以模块商品页峰值宣传作为依据。

CAN 总线采用短线连接，总线两端各放置 120 ohm 终端，固定速率为 500 kbit/s。ROS/SocketCAN 必须配置为同一波特率。联调记录应包含线长、终端、供电、CAN error counter 和 bus-off 行为。

## 3. 总体架构

采用裸机分层与固定节拍调度，不引入 FreeRTOS。

```text
CAN/EXTI/TIM6 ISR
        |
        v
事件、CAN RX 队列、急停闩锁
        |
        v
Chassis_Process() 主循环调度
        |
        +-- CAN 传输与 V1 协议组装
        +-- Safety Manager
        +-- 10 ms 编码器采样与双轮 PID
        +-- 转向目标映射
        +-- 20 ms 反馈快照与 CAN TX
        +-- IWDG 健康门控
        |
        v
encoder.c / PID.c / bsp_motor.c / PWM / servo BSP
```

分层原则：

- ISR 只完成必要的寄存器/HAL 操作、收帧入队、事件计数和急停闩锁。
- `safety_manager` 是普通执行器输出的唯一裁决者。
- PE1 或合法 CAN 急停帧可走紧急旁路，立即令双轮短刹，不等待 10 ms 控制事件。
- CAN 协议层不直接写电机或舵机。
- BSP 不保存业务状态，不自行决定车辆是否允许运动。
- 同一硬件资源只有一个模块拥有写权限。

## 4. 调度与并发

TIM6 每 1 ms 触发一次。ISR 使用饱和事件计数器生成任务：

- 每 10 ms：一个轮速控制事件，100 Hz。
- 每 20 ms：一个反馈事件，50 Hz。
- 每 1 ms：更新时间基准和轻量健康计数，不做浮点运算。

主循环固定优先级：

1. 急停和锁存故障事件。
2. CAN RX 队列及命令组装。
3. 安全状态机转换。
4. 10 ms 编码器采样和双轮闭环。
5. 20 ms 反馈快照与非阻塞发送。
6. CAN 错误处理与 IWDG 健康检查。

控制事件不能使用单个布尔标志，以免主循环延迟时静默丢周期。若积压超过一个控制周期，进入 `SAFE_STOP`，清除旧事件，不连续补算多个过期 PID 周期。反馈事件允许丢弃旧周期，只发送最新快照。

CAN RX 使用固定长度环形队列，初始容量 8 帧。完整命令采用短临界区或双缓冲原子提交。主循环不得使用 `HAL_Delay()`，不得阻塞等待 CAN 邮箱。

所有时间判断使用固定宽度无符号差值，支持 `HAL_GetTick()`、`uint16` 序列号和 `uint32 device_time_ms` 自然回绕。

## 5. CAN 命令处理

协议保持 V1 契约：`0x120` 转向和 `0x121` 后轮命令组成一个原子控制组。仅在以下条件全部满足时提交：

- 标准数据帧、DLC=8、版本=1。
- 两帧序号和 `mode_flags` 相同。
- reserved 字段和 reserved bit 为零。
- 转角和轮速在集中配置的物理范围内。
- 两帧到达间隔不超过 10 ms。
- 模式为 STOP 或 VELOCITY；V1 默认拒绝 MAINTENANCE 和 RESERVED。

重复 `command_seq` 不刷新 100 ms 命令看门狗，也不重复应用。不同序号视为新鲜命令，包括 `0xFFFF -> 0x0000`。

任一格式合法的命令帧带 `estop_active` 时立即触发急停，不等待另一帧。由于单帧不构成完整控制组，此路径不更新 `applied_command_seq`。

## 6. 安全状态机

状态包括：

- `BOOT`：PWM=0，双轮 COAST；读取复位原因，校验标定并检查 PE1。
- `CALIBRATION_REQUIRED`：锁存 `0x000A`，禁止电机和舵机输出，仅允许受控台架标定流程。
- `STOPPED`：双轮 COAST；允许合法 STOP 命令使舵机回中。
- `ACTIVE`：应用 VELOCITY 控制组并运行双轮闭环。
- `SAFE_STOP`：目标归零并 COAST，清 PID 积分。
- `ESTOP`：双轮短刹，锁存 `0x0001`，清 PID 积分。
- `FAULT`：禁止运动，按故障策略 COAST 或 BRAKE。

安全优先级：

```text
物理急停 = CAN 急停
> 锁存设备故障
> safe_stop / 命令超时 / 调度超期
> STOP
> VELOCITY
```

恢复规则：

- `SAFE_STOP` 必须先收到新鲜全零 STOP 组，回到 `STOPPED`，不能直接恢复 ACTIVE。
- `ESTOP` 必须满足 PE1 连续释放 50 ms、CAN 急停位清零、收到新鲜全零 STOP 双帧组并带 `reset_fault_request`，随后只回到 `STOPPED`。
- 设备故障只有在物理原因消失且协议允许清除时，才能通过全零 STOP + reset 回到 `STOPPED`。
- `calibration_invalid` 不能通过普通 CAN reset 伪造清除，必须写入并验证有效台架标定。

物理和 CAN 急停做逻辑 OR。PE1 EXTI 触发时先令 PWM 为零，再设置 TB6612 两路 `IN1=IN2=1` 进入短刹。10 ms 任务继续复核 PE1 电平，避免仅依赖单次边沿中断。

## 7. 后轮闭环与现有 PID 复用

PID 运行在主循环的 10 ms 控制任务中，不在 TIM6 ISR 或 CAN 回调中。左右轮各有一个独立 `PID_t`。

```text
Chassis_ControlStep10ms()
  Encoder_Sample10ms()
  PID_Update(left, actual_left, 0.01 s)
  PID_Update(right, actual_right, 0.01 s)
  SafetyManager_Arbitrate()
  Motor_Apply(left, right)
```

现有 `PID.c` 在原文件上演进：

- 保留 `PID_t`、Kp/Ki/Kd 和现有调用概念。
- `PID_Update` 显式接收实际值与 `dt`，或由固定周期初始化参数固化 `dt=0.01 s`。
- 增加积分限幅、输出限幅、条件积分/抗饱和和 `PID_Reset`。
- STOP、SAFE_STOP、ESTOP、FAULT、方向切换和控制周期丢失时清积分。
- 首版 `Kd=0`，按 PI 使用，避免差分放大编码器速度噪声。

控制输入输出统一：目标和实际轮速为 mm/s，PID 输出为 `-1000..+1000` 的有符号 PWM 指令。

调参顺序为开环测量、死区补偿、Kp、Ki。左右轮独立标定和调参，不默认共用参数。

## 8. 编码器与车辆标定

现有 `encoder.c` 继续作为编码器主模块并修正：

- 左轮读取 TIM1，右轮读取 TIM2；修复当前左轮错误读取 TIM3 的问题。
- 初始化并启动两个编码器定时器。
- 定时器编码器模式不使用计数预分频，ARR 设置为硬件允许的最大计数范围。
- 以回绕安全方式计算 10 ms 原始增量。
- 内部保存 `int64` 累计脉冲，用于换算协议 `int32 mrad` 累计位置并自然截断回绕。
- 对外提供原始增量、累计计数、轮速和有效性。

编码器 PPR、倍频定义、减速比、轮半径、左右计数极性、最大物理增量集中存放在车辆配置中，不再硬编码在 `encoder.c`。

配置启动校验覆盖：非零和合理范围、换算溢出、左右映射、舵机标定单调性、最大轮速与协议 int16 范围。任何关键项缺失或非法均进入 `CALIBRATION_REQUIRED`。

## 9. TB6612 电机驱动与现有模块复用

现有 `bsp_motor.c` 继续作为 TB6612 主模块，在原接口基础上增加显式模式：

```c
Motor_Init();
Motor_Drive(motor, signed_pwm);
Motor_Coast(motor);
Motor_Brake(motor);
Motor_EmergencyBrakeAll();
```

原有 `Motor_SetPWM()` 可保留为兼容入口并转调 `Motor_Drive()`，但普通控制链路使用显式模式。

TB6612 真值语义：

- DRIVE：IN1/IN2 选择方向，PWM 调速。
- COAST：IN1=0、IN2=0，输出高阻。
- BRAKE：IN1=1、IN2=1，短刹。
- 不使用 MCU STBY；确认模块 STBY 已硬件拉高。

停车策略：普通 STOP、safe stop 和命令超时使用 COAST；急停使用 BRAKE。换向必须执行 PWM 清零、COAST 死区、切换方向、恢复 PWM。死区为集中可配置参数并通过示波器验证。

现有 `bsp_pwm_driver.c` 继续拥有 TIM3 比较寄存器。`bsp_pwm.c` 与 `pwm_app.c` 的重复实现需要合并为工程实际采用的一套，但不能在新旧控制链路之间形成两套硬件所有者。

## 10. 转向舵机

标准 PWM 舵机使用 PB6/TIM4_CH1，50 Hz。TIM4 使用 1 MHz 计数基准和 20 ms 周期，比较值直接表示高电平微秒数。

STM32 保存：中心脉宽、左右极限、方向符号、等效前轮角到脉宽的标定点。目标采用分段线性插值和变化率限制。任何插值失败、非单调标定或脉宽越界均拒绝整组命令并进入 `SAFE_STOP`，上报 `0x0003` 或 `0x000A`。

R3X V1 无舵角传感器，因此 STM32 不做外部舵角 PID，不生成虚假的舵角反馈。标准舵机内部闭环负责到达脉宽目标。

## 11. 故障检测与优先级

| Fault | 检测与处理 | 锁存 |
|---|---|---|
| `0x0001 estop_active` | PE1 低或 CAN estop；立即 BRAKE | 是 |
| `0x0002 rear_encoder_fault` | ACTIVE 下目标和 PWM 超阈值但连续无脉冲，或增量/方向持续异常；双轮 COAST | 是 |
| `0x0003 steering_command_rejected` | 转角/插值/脉宽不合法；整组拒绝并 SAFE_STOP | 否 |
| `0x0004 command_timeout` | 100 ms 无新鲜完整命令组；COAST | 否 |
| `0x0005 command_group_incomplete` | 单帧 10 ms 内未配对 | 否 |
| `0x0006 command_group_inconsistent` | 序号、flags、版本、DLC、reserved 或模式非法 | 否 |
| `0x0007 command_range_invalid` | 转向角或轮速越过车辆包络 | 否 |
| `0x0008 mcu_watchdog_reset` | 启动读取 RCC reset flags；清除前禁止 ACTIVE | 是 |
| `0x0009 motor_driver_fault` | 首版不主动产生；TB6612 未接故障/电流/温度诊断 | 是（预留） |
| `0x000A calibration_invalid` | 集中标定缺失或校验失败 | 是 |
| `0xFFFF unknown_device_fault` | HAL 初始化失败或安全不变量损坏 | 是 |

编码器初始诊断阈值作为集中配置：目标绝对值至少 200 mm/s、PWM 至少 200/1000、无脉冲或反向异常持续 300 ms。它们只是台架起点，必须通过实测调整，避免低速启动误报。

CAN V1 同时只能发布一个 fault code，优先级固定：

```text
ESTOP
> calibration / watchdog / encoder / motor / unknown
> command timeout
> group / range / steering errors
> none
```

内部可以保留多个诊断位。状态表示“是否允许输出”，fault 表示“为什么异常”，两者不能混为一个枚举。

## 12. 反馈一致性

每 20 ms 冻结一次不可变反馈快照，包含状态、故障、最近完整应用序号、轮速、累计位置、有效位和设备时间。`0x180..0x184` 使用同一 `feedback_seq` 和同一快照；发送过程中出现的新状态进入下一周期。

反馈语义：

- `applied_command_seq` 只在完整控制组真正提交时更新。
- `command_group_accepted` 描述最近收到的控制组，不是历史成功标志。
- `steering_command_accepted` 描述最近完整组的转向目标校验结果。
- `encoder_ok` 要求标定有效、定时器已启动、最近采样合法且无对应编码器故障。
- `velocity_valid` 要求本周期完成有效采样，不能用旧速度冒充。
- `position_valid` 要求累计计数链路有效。
- `steering_feedback_available` 固定为零。
- `fault_latched` 在任一锁存故障存在时置位。

CAN TX 非阻塞。如果邮箱不足，保留本周期冻结快照继续发送；若到下一反馈周期仍未完成，记录 TX overrun 诊断并丢弃旧反馈组，不能混合新旧 `feedback_seq`。

## 13. 看门狗与 CAN 错误

IWDG 只在以下条件全部满足后喂狗：调度事件按期消费、最近控制步成功、主循环活性正常、无不可恢复的软件不变量错误。IWDG 复位原因在下一次启动锁存为 `0x0008`。

CAN 保持自动 bus-off 管理。软件记录 error warning、error passive、bus-off、RX queue overflow 和 TX overrun。bus-off 或持续通信异常使控制器进入 `SAFE_STOP`；若无法恢复或 HAL 状态异常则进入 `FAULT`。

## 14. 测试策略

### 14.1 PC 单元测试

保留当前 6 个 `bsp_bxcan` 测试，并逐步将纯协议逻辑拆成不依赖 HAL 的可测试单元。补充：

- 非法 DLC、IDE/RTR、版本、reserved、模式和范围。
- 重复序列不刷新超时，序列与时间回绕。
- 10 ms 配对和 100 ms 超时的边界前后。
- 两种到帧顺序、不完整组、新序列淘汰旧组。
- 单帧急停、完整急停组和 `applied_command_seq` 语义。
- 全部安全状态转换、故障优先级和 reset 门槛。
- PID 输出/积分限幅、reset、方向切换和固定 dt。
- 编码器计数回绕、极性、累计位置和无脉冲诊断。
- 五帧反馈同快照与 TX overrun 行为。

### 14.2 板级测试

- Keil 全量构建零错误、零警告。
- 不接电机时用示波器验证 TIM3 20 kHz、TIM4 50 Hz、GPIO 真值和换向死区。
- 分别验证左右编码器映射、方向和回绕。
- 测量 PE1/CAN 急停到 TB6612 BRAKE 的实际延迟。
- 验证命令断流在 `100 ms + 1 个控制周期` 内进入 COAST。
- 验证无标定时不能进入 ACTIVE。

### 14.3 闭环与整车测试

- 单轮离地完成正方向和开环 PWM-轮速曲线。
- 左右轮分别调 Kp、Ki，首版 Kd=0。
- 10 ms 控制步最大执行时间目标小于 2 ms。
- 测试 STOP、前进、倒车、转向、左右不同轮速、CAN 断线、PE1 急停和恢复。
- 长时间运行不得出现 bus-off、控制事件积压或未解释的编码器异常。

## 15. 实施阶段

1. 修正 CubeMX 资源：TIM1/TIM2、TIM3、TIM4、TIM6、PE1，保持 CAN 500 kbit/s。
2. 原地扩展 `encoder.c`、`bsp_motor.c`、`PID.c` 和 PWM 层，先完成板级波形与单元测试。
3. 从现有 `bsp_bxcan` 提取纯协议边界，保持现有测试持续通过。
4. 实现调度、集中配置、安全状态机和反馈快照。
5. 接入双轮闭环和转向标定。
6. 完成 CAN 台架、单轮、双轮和整车验收。

每一步都保持可构建和可测试。旧 API 仅在有调用者时保留兼容包装；新链路验证完成前不删除现有模块。

## 16. 明确不纳入 V1 的内容

- FreeRTOS。
- MCU 侧阿克曼运动学计算。
- 舵角实际反馈或外部舵角 PID。
- 未接传感器支持的电流、温度或 TB6612 故障判断。
- CAN 应用层 CRC、认证或授权。
- 未经单独协议与测试定义的 MAINTENANCE 运动控制。
