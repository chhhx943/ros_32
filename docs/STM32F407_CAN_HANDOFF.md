# STM32F407 CAN1 交接说明

状态：联调收尾版

适用对象：Raspberry Pi 5 + MCP2515/TJA1050 主机端、STM32F407 底盘控制器端

对应完整规范：[CAN_PROTOCOL.md](CAN_PROTOCOL.md)

## 1. 已验证结论

- STM32F407 CAN1 已工作在 `CAN_MODE_NORMAL`。
- 总线速率为 500 kbit/s，与 Pi 端 MCP2515 的配置匹配。
- Pi 发送 `0x120` 和 `0x121` 后，STM32 应用层已提交完整命令组，`command_seq=1`、`g_applied_command_seq_valid=1`。
- STM32 CAN1 接收中断路径正常；`RF0R=0` 不能作为“未收到”的判断，因为 RX0 中断会立即取走 FIFO。
- 当前联调固件已恢复 20 ms 心跳，Pi 端应持续收到 STM32 的反馈帧。
- 当前联调镜像关闭了 IWDG，仅用于 CAN 联调；正式车辆镜像不得默认关闭看门狗。

## 2. STM32 端硬件连接

### 2.1 MCU 与 TJA1050

| STM32F407 | TJA1050 | 说明 |
|---|---|---|
| `PB9 / CAN1_TX` | `TXD` | MCU 发送到收发器 |
| `PB8 / CAN1_RX` | `RXD` | 收发器接收回 MCU |
| `5 V` | `VCC` | TJA1050 供电 |
| `GND` | `GND` | 必须与 MCU、Pi 共地 |
| TJA1050 `CANH` | Pi 侧 `CANH` | 直连，不交叉 |
| TJA1050 `CANL` | Pi 侧 `CANL` | 直连，不交叉 |

当前模块没有引出 `RS/S` 控制脚，不需要由 STM32 软件控制。TJA1050 的 `TXD/RXD` 是逻辑侧；`CANH/CANL` 是总线差分侧，不能把 PB8/PB9 直接接到 Pi 的 CANH/CANL。

### 2.2 总线要求

- CANH、CANL 必须同名直连，不能 CANH/CANL 交叉。
- STM32、TJA1050、Pi 收发器必须共地。
- 总线物理两端各使用 120 ohm 终端电阻；中间节点不额外并终端。
- TJA1050 当前使用 5 V 供电。
- 逻辑分析仪测逻辑侧时：
  - CH0 接 `PB8/CAN1_RX`；
  - CH1 接 `PB9/CAN1_TX`；
  - CAN 解码速率设置为 500 kbit/s、11-bit standard ID、data frame。
- 心跳关闭时，PB9 长时间保持空闲是正常现象；恢复心跳后 PB9 会出现周期性 TX 波形。

## 3. STM32 CAN1 初始化参数

当前源码位置：`Core/Src/can.c`。

| 参数 | 当前值 |
|---|---:|
| CAN 外设 | `CAN1` |
| GPIO | `PB8=RX`, `PB9=TX`, `GPIO_AF9_CAN1` |
| 工作模式 | `CAN_MODE_NORMAL` |
| APB1/CAN 时钟 | 42 MHz |
| Prescaler | 7 |
| BS1 | 8 TQ |
| BS2 | 3 TQ |
| SJW | 1 TQ |
| 总 TQ | 12 |
| 计算速率 | `42 MHz / (7 × 12) = 500 kbit/s` |
| 采样点 | `(1 + 8) / 12 = 75%` |
| 自动 Bus-Off 管理 | 开启 |
| 自动重发 | 开启 |
| RX FIFO | FIFO0 |
| RX 中断 | `CAN_IT_RX_FIFO0_MSG_PENDING` |

CAN1/CAN2 共享过滤器块由 bank0 接收。当前联调镜像暂时使用 bank0 全接收，Pi 端必须按 CAN ID 过滤未知帧；正式发布前可收窄为协议所需 ID。

## 4. Pi 端 SocketCAN 配置

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 sample-point 0.75 \
    loopback off listen-only off restart-ms 100
sudo ip link set can0 up
ip -details -statistics link show can0
```

预期：

- `bitrate 500000`
- `sample-point 0.750`
- `can state ERROR-ACTIVE`
- 发送后 `error-pass` 不应持续增加

抓包：

```bash
candump -tz -x can0
```

发送一组 STOP 命令：

```bash
cansend can0 120#0101000000000000
cansend can0 121#0101000000000000
```

这两帧的 `command_seq=1`、`mode_flags=0`，表示同一组全零 STOP 命令。两帧应在 10 ms 内发送完成；正常主循环建议每 20 ms 发送一组新序号命令。

## 5. 主命令协议

所有 V1 帧均为 Classic CAN 2.0A、11-bit standard ID、data frame、DLC=8、byte 0 为 `0x01`。多字节整数均为 little-endian。

### 5.1 `0x120 CMD_STEERING`

| 字节 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | protocol_version=`0x01` |
| 1..2 | u16 LE | command_seq |
| 3 | u8 | mode_flags |
| 4..5 | i16 LE | equivalent_steering_mrad |
| 6..7 | u16 LE | 必须为 0 |

角度单位为 mrad；正值左转，负值右转。当前 MCU 接受范围为 `-600..+600 mrad`。

### 5.2 `0x121 CMD_REAR_WHEELS`

| 字节 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | protocol_version=`0x01` |
| 1..2 | u16 LE | 必须与 `0x120` 相同 |
| 3 | u8 | 必须与 `0x120` 相同 |
| 4..5 | i16 LE | rear_left_velocity_mmps |
| 6..7 | i16 LE | rear_right_velocity_mmps |

速度单位为 mm/s；车辆前进方向为正。当前 MCU 接受范围为每个车轮 `-3000..+3000 mm/s`。

### 5.3 `mode_flags`

| 位 | 含义 |
|---:|---|
| bits 1..0 | `0=STOP`, `1=VELOCITY`, `2=MAINTENANCE`, `3=RESERVED` |
| bit 2 | CAN E-stop；置 1 时单帧即可触发急停 |
| bit 3 | SAFE_STOP 请求 |
| bit 4 | RESET_FAULT 请求 |
| bits 7..5 | 必须为 0 |

`0x120` 和 `0x121` 只有在以下条件全部满足时才会原子提交：序号相同、flags 相同、两帧间隔不超过 10 ms、字段合法、序号为新序号。重复序号不会刷新看门狗。

MCU 命令超时为 100 ms；超过该时间没有新的完整命令组时，后轮目标清零并进入 SAFE_STOP。Pi 端应至少每 20 ms 发送一组新序号命令。

## 6. STM32 反馈协议

当前正常心跳周期为 20 ms。每个周期通常发送以下反馈帧：

`0x180, 0x181, 0x182, 0x183, 0x184, 0x186, 0x187, 0x188`。

`0x185` 只在标定服务使用时发送。`0x187` 为 PID 维护扩展，不参与底盘基本控制。Pi 端必须允许未知/可选反馈 ID，不应因缺少 `0x185` 或 `0x187` 判定 CAN 掉线。

### 6.1 `0x180 FB_STATUS`

```text
byte 0     protocol_version = 0x01
byte 1..2  feedback_seq, u16 LE
byte 3     heartbeat_counter
byte 4..5  applied_command_seq, u16 LE
byte 6..7  fault_code, u16 LE
```

`applied_command_seq` 只有在完整的 `0x120+0x121` 命令组提交后才更新。Pi 端应使用 `heartbeat_counter` 或 `feedback_seq` 判断心跳是否前进，而不是只判断 CAN socket 是否打开。

### 6.2 `0x181 FB_HEALTH`

```text
byte 0     protocol_version = 0x01
byte 1..2  feedback_seq, u16 LE
byte 3     status_flags
byte 4..7  device_time_ms, u32 LE
```

`status_flags`：

| 位 | 含义 |
|---:|---|
| 0 | 最近完整命令组已接受 |
| 1 | 左编码器有效 |
| 2 | 右编码器有效 |
| 3 | 最近转向命令已接受 |
| 5 | E-stop active |
| 6 | SAFE_STOP active |
| 7 | 存在锁存故障 |

### 6.3 `0x182 FB_REAR_VELOCITY`

```text
byte 0     protocol_version
byte 1..2  feedback_seq, u16 LE
byte 3     velocity_flags: bit0=left valid, bit1=right valid
byte 4..5  rear_left_velocity_mmps, i16 LE
byte 6..7  rear_right_velocity_mmps, i16 LE
```

### 6.4 `0x183/0x184` 后轮位置

```text
byte 0     protocol_version
byte 1..2  feedback_seq, u16 LE
byte 3     position_valid, 0 or 1
byte 4..7  signed position_mrad, i32 LE
```

`0x183` 是左后轮，`0x184` 是右后轮。

### 6.5 `0x186 FB_DIAGNOSTICS`

```text
byte 0     protocol_version
byte 1..2  feedback_seq, u16 LE
byte 3     diagnostic_flags
byte 4     safety_state
byte 5     safety_action
byte 6     command_age_10ms, 0xFF means invalid/stale
byte 7     can_error_class
```

`can_error_class`：`0=none`、`1=warning/protocol`、`2=error-passive`、`3=bus-off`、`4=TX failure`。

### 6.6 `0x187 FB_PID_GAINS`

```text
byte 0     protocol_version
byte 1..2  transaction_seq, u16 LE
byte 3     status: 0 none, 1 accepted, 2 bad format,
           3 unsafe, 4 timeout
byte 4     axis_mask: bit0 left, bit1 right
byte 5..7  reserved = 0
```

### 6.7 `0x188 FB_CONTROL_OUTPUT`

```text
byte 0     protocol_version
byte 1..2  feedback_seq, u16 LE
byte 3     validity flags: bit0 left, bit1 right
byte 4..5  signed left motor output, i16 LE, -1000..1000
byte 6..7  signed right motor output, i16 LE, -1000..1000
```

## 7. Pi 端健康判定

建议同时使用以下三层判定：

1. **物理/链路层**：`can0` 为 `ERROR-ACTIVE`，`error-pass`、`bus-off` 不增加。
2. **协议层**：收到有效 `0x180`，且 `heartbeat_counter` 或 `feedback_seq` 持续递增。
3. **命令层**：发送同一序号的 `0x120/0x121` 后，`0x180.applied_command_seq` 与发送序号一致，或 `0x181.status_flags bit0` 置位。

`candump` 中看到 Pi 自己的 `TX` 回显不等于 STM32 收到；必须看实际总线状态和 STM32 反馈。CAN ACK 本身不作为应用数据出现在 `candump` 中。

## 8. RF0R 调试方法

STM32 CAN1 的 `RF0R` 地址为 `0x4000640C`。低两位 `FMP0` 表示 FIFO0 中待处理消息数：

```text
FMP0 = RF0R & 0x03
0     FIFO 空
1/2   FIFO 有待处理帧
bit3  FIFO full
bit4  FIFO overrun
```

当前 RX0 中断回调会立即调用 `HAL_CAN_GetRxMessage()`，因此正常运行时读取到 `FMP0=0` 仍可能表示“刚刚收到并已经处理”。更可靠的应用层证据是：

- `0x180.applied_command_seq` 更新；
- `0x181.status_flags bit0` 置位；
- 或通过 SWD 读取 `g_applied_command_seq_valid` 与 `g_applied_command_seq`。

## 9. 故障排查顺序

1. 先确认 CANH/CANL 没有交叉、两端有 120 ohm、共地、TJA1050 为 5 V。
2. 检查 Pi：`bitrate=500000`、`sample-point=0.75`、`listen-only off`、`loopback off`。
3. 逻辑分析仪测 PB8：Pi 发送时应能解出 `0x120/0x121`、CRC 和 ACK。
4. 检查 STM32 `MCR/MSR/BTR/ESR/IER`：Normal、500 kbps、无错误、RX0 中断开启。
5. 检查 Pi 是否进入 `ERROR-PASSIVE`；若是，优先检查物理层和 ACK，不要先修改应用协议。
6. 检查 `0x180` 的 `applied_command_seq`，确认命令是否被协议层接受。

## 10. 当前交接文件与固件

- 协议规范：[CAN_PROTOCOL.md](CAN_PROTOCOL.md)
- 底盘控制设计：[CHASSIS_CONTROL_DESIGN.md](CHASSIS_CONTROL_DESIGN.md)
- 当前心跳联调镜像：[ros_heartbeat.hex](../build/Heartbeat/ros_heartbeat.hex)
- STM32 CAN 初始化：[can.c](../Core/Src/can.c)
- STM32 CAN 协议实现：[bsp_bxcan.c](../BSP/bsp_bxcan.c)

Pi 端交接时，优先实现 `0x120`、`0x121`、`0x180`、`0x181`、`0x182`、`0x186`；标定、PID 调参和 `0x188` 属于维护/诊断扩展，不应阻塞基本底盘通信上线。
