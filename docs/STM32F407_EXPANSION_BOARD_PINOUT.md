# STM32F407 扩展板接口与 PCB 引脚表

**适用工程：** `ros` / STM32F407VET6（LQFP100）  
**依据：** `ros.ioc`、当前 Debug 固件、`Core/Src/tim.c`、`BSP` 驱动  
**版本日期：** 2026-09-07

本文是画扩展板 PCB 时的接口基线。表中“当前使用”表示现有固件已经配置并由底盘控制流程使用；“预留”表示 CubeMX 有引脚配置或已有驱动文件，但当前主程序没有启用。

## 1. PCB 必须连接的接口

### 1.1 左后轮电机驱动接口

默认对应 TB6612 的左电机通道。STM32 只输出逻辑/PWM，电机电源 `VM` 不得从 MCU 3.3 V 取电。

| 建议网络名 | STM32 引脚 | 外设功能 | 连接到 | 方向 | 备注 |
|---|---|---|---|---|---|
| `MOTOR_L_PWM` | `PA6` | `TIM3_CH1` | TB6612 `PWMA` | MCU → 驱动器 | PWM，当前周期 4200 个计数，约 20 kHz |
| `MOTOR_L_IN1` | `PB12` | GPIO 输出 | TB6612 `AIN1` | MCU → 驱动器 | 方向/COAST/BRAKE |
| `MOTOR_L_IN2` | `PB13` | GPIO 输出 | TB6612 `AIN2` | MCU → 驱动器 | 方向/COAST/BRAKE |
| `GND` | GND | 电源地 | 驱动器 GND | — | 必须共地 |
| `VM_MOTOR` | 外部电源 | — | TB6612 VM | — | 电机电源，不接 MCU 3.3 V |

当前工程没有分配 TB6612 `STBY/EN` GPIO。若驱动器有该脚，应在扩展板上通过上拉/固定高电平处理，并预留测试焊盘；不要假定 STM32 能软件关闭它。

### 1.2 右后轮电机驱动接口

| 建议网络名 | STM32 引脚 | 外设功能 | 连接到 | 方向 | 备注 |
|---|---|---|---|---|---|
| `MOTOR_R_PWM` | `PA7` | `TIM3_CH2` | TB6612 `PWMB` | MCU → 驱动器 | PWM，约 20 kHz |
| `MOTOR_R_IN1` | `PB14` | GPIO 输出 | TB6612 `BIN1` | MCU → 驱动器 | 方向/COAST/BRAKE |
| `MOTOR_R_IN2` | `PB15` | GPIO 输出 | TB6612 `BIN2` | MCU → 驱动器 | 方向/COAST/BRAKE |
| `GND` | GND | 电源地 | 驱动器 GND | — | 必须共地 |
| `VM_MOTOR` | 外部电源 | — | TB6612 VM | — | 电机电源，不接 MCU 3.3 V |

### 1.3 左轮编码器接口

代码中电机编号 `1` 对应左轮，使用 TIM1 编码器接口。

| 建议网络名 | STM32 引脚 | 外设功能 | 编码器信号 | 方向 |
|---|---|---|---|---|
| `ENC_L_A` | `PE9` | `TIM1_CH1` | A 相 | 编码器 → MCU |
| `ENC_L_B` | `PE11` | `TIM1_CH2` | B 相 | 编码器 → MCU |
| `ENC_VCC` | 3.3 V 或编码器规定电源 | — | 编码器 VCC | 电源 |
| `GND` | GND | — | 编码器 GND | — |

### 1.4 右轮编码器接口

代码中电机编号 `2` 对应右轮，使用 TIM2 编码器接口。

| 建议网络名 | STM32 引脚 | 外设功能 | 编码器信号 | 方向 |
|---|---|---|---|---|
| `ENC_R_A` | `PA0` | `TIM2_CH1` | A 相 | 编码器 → MCU |
| `ENC_R_B` | `PA1` | `TIM2_CH2` | B 相 | 编码器 → MCU |
| `ENC_VCC` | 3.3 V 或编码器规定电源 | — | 编码器 VCC | 电源 |
| `GND` | GND | — | 编码器 GND | — |

编码器输出电平必须满足 STM32 输入电平要求。若编码器输出 5 V，需确认对应 STM32 引脚为 5 V tolerant，或增加电平转换；PCB 不要直接默认所有编码器都能接 5 V。

### 1.5 转向舵机接口

| 建议网络名 | STM32 引脚 | 外设功能 | 连接到 | 方向 |
|---|---|---|---|---|
| `STEER_SERVO_PWM` | `PB6` | `TIM4_CH1` | 舵机信号线 | MCU → 舵机 |
| `SERVO_V+` | 外部 5/6 V | — | 舵机电源 | 电源 |
| `GND` | GND | — | 舵机地 | — |

当前舵机 PWM：周期约 20 ms，软件中心脉宽 1500 us，正常范围约 1000–2000 us。舵机电源必须单独按舵机电流设计，并与 STM32 共地。

### 1.6 CAN 收发器与 CAN 总线接口

当前 CAN1 使用 PB8/PB9 重映射，**不能与 PA11/PA12 或旧 OLED 引脚互换**。

#### MCU 与 TJA1050 数字侧

| 建议网络名 | STM32 引脚 | TJA1050 引脚 | 方向 |
|---|---|---|---|
| `CAN1_TX_PB9` | `PB9` | `TXD` | STM32 → TJA1050 |
| `CAN1_RX_PB8` | `PB8` | `RXD` | TJA1050 → STM32 |
| `CAN_TJA_VCC` | — | `VCC` | 5 V 收发器电源 |
| `GND` | GND | `GND` | 共地 |

#### TJA1050 总线侧

| 建议网络名 | TJA1050 引脚 | 外部接口 |
|---|---|---|
| `CANH` | `CANH` | Raspberry Pi/MCP2515 的 `CANH` |
| `CANL` | `CANL` | Raspberry Pi/MCP2515 的 `CANL` |
| `GND` | GND | Pi 端地线 |

CANH 与 CANH 相连、CANL 与 CANL 相连，**不交叉**。总线两端各放一个 120 Ω 终端电阻，中间节点不要重复加终端。

TJA1050 使用 5 V 供电时，必须在 PCB 设计阶段确认其 TXD 输入高电平阈值与 STM32 3.3 V CAN_TX 的兼容性；若不能保证，优先改用 3.3 V CAN 收发器或增加电平兼容电路。不要只依据“能偶尔出波形”判断逻辑电平兼容。

当前 CAN 参数：Classic CAN、11-bit 标准帧、500 kbit/s、DLC 8。协议命令使用 `0x120/0x121`，反馈使用 `0x180..0x184`。

### 1.7 物理急停接口

| 建议网络名 | STM32 引脚 | 配置 | 外部连接 |
|---|---|---|---|
| `ESTOP_N` | `PE1` | GPIO 输入、内部上拉、下降沿 EXTI | 急停开关/安全回路输出 |
| `GND` | GND | — | 急停触点参考地 |

当前固件逻辑为：正常状态为高电平，急停有效为低电平。急停输入不要直接接 5 V；如外部安全模块输出 5 V，必须加电平转换或确认其为开漏输出并由 MCU 侧上拉。

## 2. 推荐调试和维护接口

### 2.1 SWD 调试接口

建议至少放置 6 针 SWD 座或测试点：

| SWD 信号 | STM32 引脚/电源 |
|---|---|
| `VTREF` | 3.3 V |
| `SWDIO` | `PA13` |
| `SWCLK` | `PA14` |
| `NRST` | MCU `NRST` |
| `GND` | GND |
| 可选 `SWO` | 当前未配置，不必连接 |

`PA13/PA14` 不要作为普通扩展 GPIO 使用，否则会影响 ST-LINK 下载和调试。

### 2.2 USB FS 接口（预留，当前应用未启用）

CubeMX 已保留以下 USB 引脚，但当前 `MX_USB_OTG_FS_USB_Init()` 为空，工程没有启用完整 USB Device/DFU 协议。因此 PCB 可以预留，不应把它当作当前可用通信接口。

| USB 信号 | STM32 引脚 |
|---|---|
| `USB_ID` | `PA10` |
| `USB_DM` | `PA11` |
| `USB_DP` | `PA12` |
| `USB_VBUS` | USB 5 V 检测/供电网络，按 USB 电源方案设计 |
| `GND` | GND |

当前 CAN 已迁移到 PB8/PB9，PA11/PA12 不能再接 CAN 收发器。

## 3. 当前没有配置、不要直接占用的接口

当前 `ros.ioc` 没有配置可直接使用的 USART、SPI、硬件 I2C 或 ADC 接口。虽然工程中保留了旧的 `Hardware/MyI2C.c` 和 OLED 驱动，但它们不是当前底盘控制链路的一部分，而且存在引脚冲突：

| 旧代码信号 | 旧代码使用引脚 | 当前冲突 |
|---|---|---|
| OLED SCL | `PB8` | 当前为 `CAN1_RX` |
| OLED SDA | `PB9` | 当前为 `CAN1_TX` |
| MyI2C SCL | `PA12` | 当前为 USB DP 预留 |
| MyI2C SDA | `PB13` | 当前为左电机 `IN2` |

因此扩展板上不要把 PB8/PB9 做成 OLED/I2C 插座，也不要把 PB13 作为 I2C SDA。若以后新增 IMU、OLED、串口或 SPI，必须先重新做 CubeMX 引脚规划并同步修改固件。

## 4. 时钟和不可外接信号

以下引脚属于时钟/晶振，不作为扩展接口：

| STM32 引脚 | 用途 |
|---|---|
| `PH0/PH1` | HSE 外部 8 MHz 晶振 |
| `PC14/PC15` | LSE 外部 32.768 kHz 晶振 |

TIM6 的 1 kHz 调度和 SysTick 均为内部功能，没有对应外部引脚。

## 5. 建议扩展板连接器分组

PCB 上建议至少分成以下连接器，避免动力线和逻辑线混在一个排针中：

1. `J_CAN_BUS`：`CANH、CANL、GND`，可选 `+5V_TJA`。
2. `J_MOTOR_L`：`PWM、IN1、IN2、VM、GND`。
3. `J_MOTOR_R`：`PWM、IN1、IN2、VM、GND`。
4. `J_ENC_L`：`A、B、VCC、GND`。
5. `J_ENC_R`：`A、B、VCC、GND`。
6. `J_SERVO`：`PWM、SERVO_V+、GND`。
7. `J_ESTOP`：`ESTOP_N、GND`。
8. `J_SWD`：`VTREF、SWDIO、SWCLK、NRST、GND`。
9. `J_USB`：仅在确定启用 USB Device/DFU 后安装。

## 6. PCB 出图前检查清单

- [ ] CAN 使用 `PB9 → TXD`、`RXD → PB8`，CANH/CANL 不交叉。
- [ ] STM32、TJA1050、MCP2515/Raspberry Pi 共地。
- [ ] TJA1050 的 5 V 供电、去耦和 3.3 V 逻辑兼容性已确认。
- [ ] CAN 120 Ω 终端只放在总线两端，并可用跳线断开。
- [ ] 两路电机 PWM/IN 引脚与左右轮编号保持一致。
- [ ] 电机 VM、舵机 V+ 与 MCU 3.3 V 分开供电，并做好电源地回流。
- [ ] 编码器 A/B 没有接反，输出电平与 STM32 兼容。
- [ ] PE1 急停为高正常、低有效，外部没有强推 5 V。
- [ ] PA13/PA14/NRST 保留 SWD 下载通道。
- [ ] PB8/PB9 没有复用为 OLED/I2C。
- [ ] 所有外部接口在原理图中标明 `MCU 3.3 V`、`TJA 5 V`、`MOTOR VM`、`SERVO V+` 的电压域。

