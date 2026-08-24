# RK3588 ELF2 -> STM32F407 USB DFU OTA 设计报告

日期：2026-08-24
目标主机：瑞芯微 RK3588 ELF2（Linux）
目标设备：优信电子 65407 / STM32F407VET6 开发板及其底盘固件
状态：初版完整设计，允许后续按实物资料修订

## 1. 目标

通过 ELF2 作为 USB 主机，从 HTTP(S) 固件服务器获取带版本信息、SHA-256 摘要和 ECDSA P-256 签名的 STM32 底盘固件，在车辆安全停车后远程升级 STM32。

系统必须满足：

- 断电、USB 断开、传输中止或新固件启动失败时，不覆盖最后一个可启动的确认版本；
- 未通过签名、产品、硬件、版本和尺寸校验的镜像不能进入试运行；
- 新版本必须经过看门狗监督的稳定试运行窗口才能确认；
- 正常升级不增加第二套 CAN 车辆控制协议，也不改变 CAN V1 的硬件所有者；
- 生产版本启用 RDP Level 1 和 Bootloader 扇区写保护后，仍可通过自定义 USB DFU 完成升级；
- 升级期间电机、舵机和 CAN 控制必须处于安全状态。

本设计不把 STM32 ROM DFU 作为生产升级器。ROM DFU 仅用于研发、出厂烧录或受控维修恢复。

## 2. 已确认的硬件事实

### 2.1 STM32F407VET6 开发板

优信电子 65407 板的公开资料显示：

- 板载 USB 接口的 `D-`/`D+` 连接 STM32 `PA11`/`PA12`；
- `PB8` 和 `PB9` 引出到扩展排针；
- `BOOT0`、`NRST` 和 `PE1` 均有可接入的引出位置；
- HSE 为 8 MHz，满足 STM32F4 ROM DFU 对外部时钟倍频的基本要求；
- 板载 USB 可能存在额外 D+ 上拉，正式 DFU 前需实测枚举并确认是否需要移除或修改。

### 2.2 资源分配

生产推荐分配：

| 功能 | STM32 资源 | 说明 |
|---|---|---|
| USB DFU | `PA11/PA12` | 保留给 USB OTG FS，D-/D+ |
| 车辆 CAN1 | `PB8/PB9` | CAN1 重映射，RX/TX |
| 左轮编码器 | `TIM1 / PE9, PE11` | 保持现有规划 |
| 右轮编码器 | `TIM2 / PA0, PA1` | 保持现有规划 |
| 后轮 PWM | `TIM3_CH1/CH2 / PA6, PA7` | 保持现有规划 |
| 舵机 PWM | `TIM4_CH1 / PB6` | 50 Hz，后续实现 |
| 电机方向 | `PB12-PB15` | TB6612 两路方向 |
| 物理急停 | `PE1/EXTI1` | 低有效、常闭回路 |
| 强制自定义 Bootloader | ELF2 GPIO -> STM32 普通 GPIO | 两端具体 GPIO 不在本版冻结；不得连接 STM32 `BOOT0` |
| STM32 复位 | `NRST` | 由 ELF2 GPIO 经安全电路控制 |

CAN1 不改为 CAN2。CAN2 的 `PB5/PB6` 会冲突舵机，`PB12/PB13` 会冲突 TB6612；CAN1 重映射到 `PB8/PB9` 能同时释放 `PA11/PA12` 给 USB。

### 2.3 ELF2 主机

ELF2 是 RK3588 Linux 开发板，提供 USB Host、40Pin 树莓派兼容排针和额外 20Pin 扩展接口。升级程序运行在 ELF2 Linux 用户空间：

- HTTPS 下载和服务器证书校验；
- manifest 校验、签名校验和本地缓存；
- USB 设备枚举和自定义 DFU 会话；
- 通过两个预留 GPIO 控制 STM32 的强制 Bootloader 和 NRST；
- 通过 SocketCAN 与 STM32 进行升级前安全停车和升级后健康确认。

ELF2 GPIO 名称必须以 ELF2 的管脚分配表和设备树为准，不能直接假定树莓派 BCM 编号。

`FORCE_BOOTLOADER` 是自定义 Bootloader 在每次 Flash 启动时采样的普通 STM32 GPIO，不是芯片 `BOOT0`。生产时 `BOOT0` 保持为从用户 Flash 启动；若拉高 `BOOT0`，CPU 会绕过自定义 Bootloader进入 ST ROM Bootloader，在 RDP1 下也不能完成本设计的正常 OTA。

## 3. 总体架构

```text
HTTP(S) firmware server
        |
        v
ELF2 ota-agent
  - manifest/cache/download
  - SHA-256 + ECDSA pre-check
  - vehicle safe-stop gate
  - GPIO BOOTLOADER/NRST control
  - custom USB DFU client
  - CAN post-update health check
        |
        | USB D+/D- (PA12/PA11)
        v
STM32 custom Bootloader
  - immutable public key
  - DFU class transport
  - slot validation and Flash writer
  - metadata journal
  - anti-rollback
  - trial boot / rollback
        |
        +--> Application slot A
        +--> Application slot B
              |
              v
        STM32 chassis application
        - CAN V1
        - motor/encoder/PID
        - safety state machine
```

控制平面和维护平面严格分离：

```text
CAN V1       = 车辆运动、反馈和停车事实
USB DFU      = 固件维护数据
ELF2 GPIO    = 进入 Bootloader 的维护控制信号
```

DFU 不复用 `0x120..0x184`，也不新增 CAN 维护运动命令。CAN V1 的唯一硬件所有者仍是 STM32 底层控制器。

## 4. 为什么生产不能使用 ROM DFU

STM32F4 的 RDP Level 1 开启后，主 Flash 在系统存储器 ROM Bootloader 路径下不能被读、写或擦除。若生产仍使用 ROM DFU 写应用槽，就必须保持 RDP0，固化在自定义 Bootloader 内的公钥也不能形成完整的生产信任边界。

因此：

- 研发阶段可使用 BOOT0 进入 ROM DFU，便于首次烧录和恢复；
- 生产阶段使用 Flash 内自定义 USB DFU/IAP Bootloader；
- 自定义 Bootloader 实现 USB DFU Device 类，兼容 `dfu-util` 的下载流程；
- 生产阶段启用 RDP1，并对 Bootloader 扇区启用写保护；
- 不启用 RDP2，因为 RDP2 不可逆且禁用系统存储器 Bootloader，风险不适合当前研发阶段。

## 5. Flash 分区

STM32F407VE 具有 512 KiB 主 Flash。按实际扇区边界划分：

| 区域 | Flash 扇区 | 地址范围 | 大小 | 用途 |
|---|---|---:|---:|---|
| Bootloader | S0-S1 | `0x08000000-0x08007FFF` | 32 KiB | 自定义 DFU、验签、启动选择 |
| Metadata A | S2 | `0x08008000-0x0800BFFF` | 16 KiB | 元数据日志副本 |
| Metadata B | S3 | `0x0800C000-0x0800FFFF` | 16 KiB | 元数据日志副本 |
| Application A | S4-S5 | `0x08010000-0x0803FFFF` | 192 KiB | A 槽 |
| Application B | S6-S7 | `0x08040000-0x0807FFFF` | 256 KiB | B 槽 |

发布镜像统一限制为不超过 192 KiB。B 槽多出的空间不用于发布更大的版本，以保证 A/B 对称可升级。

### 5.1 绝对链接问题

STM32 应用通常按绝对 Flash 地址链接。一个链接到 A 槽的普通 `.bin` 不能仅通过改变向量表就安全运行在 B 槽，绝对地址和常量引用仍然指向 A 槽。因此采用槽位专用构建：

```text
firmware_slot_a.bin  -> linker FLASH ORIGIN = 0x08010000
firmware_slot_b.bin  -> linker FLASH ORIGIN = 0x08040000
```

同一源码版本发布两个镜像产物，并分别生成 descriptor 和签名。Bootloader 根据目标槽验证对应镜像；不得将 A 镜像直接写入 B 槽。

## 6. Metadata 和 A/B 状态

两个 metadata 扇区采用冗余、带 CRC 和 generation 的追加记录。每条记录固定长度，包含：

```text
magic
metadata_format_version
generation
active_slot
confirmed_slot
pending_slot
slot_a_state
slot_b_state
slot_a_security_version
slot_b_security_version
slot_a_image_hash
slot_b_image_hash
slot_a_boot_attempts
slot_b_boot_attempts
last_reset_reason
crc32
```

状态值：

```text
EMPTY       槽无有效镜像
VALID       签名和镜像校验通过，可启动
TESTING     新镜像试运行中
CONFIRMED   已稳定运行并确认
REJECTED    试运行失败或校验失败，不得自动启动
```

更新 metadata 时：

1. 选择 generation 较大的 CRC 正确记录；
2. 在另一副本追加新记录；
3. 写入后读回并校验；
4. 只有新记录有效时才认为状态变化完成；
5. 掉电或擦写中断时保留旧记录。

Bootloader 优先启动尚有剩余试启动次数的 `TESTING` 槽；没有可试运行槽时启动最新有效的 `CONFIRMED` 槽。达到三次失败的 `TESTING` 槽先原子标记为 `REJECTED`，再回滚，不能继续尝试。

## 7. 镜像、Descriptor 和签名

### 7.1 发布文件

服务器可提供：

```text
firmware_slot_a.bin
firmware_slot_b.bin
firmware_slot_a.descriptor
firmware_slot_b.descriptor
manifest.json
manifest.sig
```

JSON 是服务器和人工可读索引；Bootloader 不解析复杂 JSON。发布系统必须直接生成并签名固定长度的二进制 descriptor，ELF2 只能校验和传输，不能把已签名 JSON 转换为另一份未签名头。

### 7.2 固定 descriptor

建议采用 little-endian、固定长度、无指针结构：

```c
struct ota_image_descriptor_v1 {
    uint32_t magic;                 /* 'R3X1' */
    uint16_t format_version;        /* 1 */
    uint16_t header_size;
    uint32_t product_id;
    uint32_t hardware_id;
    uint32_t slot_id;               /* A=0, B=1 */
    uint32_t image_load_address;
    uint32_t image_entry_address;
    uint32_t image_size;
    uint32_t firmware_version;
    uint32_t security_version;
    uint32_t min_bootloader_version;
    uint8_t  image_sha256[32];
    uint8_t  build_id[16];
    uint8_t  reserved[24];
    uint8_t  signature_rs[64];      /* ECDSA P-256 r||s */
};
```

签名覆盖 descriptor 中除 `signature_rs` 外的全部字节。签名哈希流程为：

```text
descriptor_without_signature
  -> SHA-256
  -> ECDSA P-256 verify(public_key, digest, r, s)
```

然后 Bootloader 对 descriptor 指定的镜像范围计算 SHA-256，并与 `image_sha256` 比较。签名验证和镜像摘要验证都通过后，槽才可变为 `VALID`。

### 7.3 版本策略

- `firmware_version` 用于显示和运维；
- `security_version` 用于防降级；
- 普通发布公钥只允许 `security_version` 大于设备当前安全版本的镜像；
- 自动回滚只允许回到此前的 `CONFIRMED` 槽，不视为降级攻击；
- 有意安装更低安全版本必须使用独立维护签名密钥，并且只能通过维护流程；
- Bootloader 版本低于 `min_bootloader_version` 时拒绝安装；
- 版本比较必须使用无符号单调整数，不使用字符串排序。

## 8. 密钥和密码学

- 曲线：NIST P-256；
- 摘要：SHA-256；
- 签名格式：固定 64 字节 `r||s`，避免 DER 长度解析差异；
- 公钥：未压缩 P-256 点 `04 || X || Y`，编译进 Bootloader 只读区域；
- 发布私钥：只存在离线签名机或受控 CI 签名环境，不放在 ELF2；
- ELF2 保存服务器证书/公钥用于 HTTPS，但不拥有固件签名私钥；
- Bootloader 必须拒绝未知产品、硬件、slot、地址、大小和保留字段；
- ECDSA 随机数只在签名端生成，Bootloader 只做验证，不需要保存私钥。

STM32 端可选用适配 STM32F4 的裁剪密码库实现 SHA-256/ECDSA P-256，但必须进行静态链接大小评估，Bootloader 总大小硬上限为 32 KiB。不能为了满足空间而删除签名校验、版本检查或地址范围检查。

## 9. OTA 状态机

### 9.1 ELF2 OTA Agent

```text
IDLE
 -> FETCHING
 -> VERIFIED_HOST
 -> REQUESTING_SAFE_STOP
 -> DFU_ENTERING
 -> DFU_WRITING
 -> DFU_VERIFYING
 -> REBOOT_WAIT
 -> HEALTH_CHECK
 -> COMPLETE
```

任何失败进入 `FAILED`，保留当前运行版本和本地日志，不自动重复写入同一槽超过配置次数。

### 9.2 STM32 Bootloader

```text
BOOT
 -> VALIDATE_METADATA
 -> SELECT_SLOT
 -> DFU_IDLE
 -> RECEIVE_DESCRIPTOR
 -> RECEIVE_IMAGE
 -> VERIFY_IMAGE
 -> MARK_TESTING
 -> JUMP_APPLICATION
```

无有效 `CONFIRMED` 槽，或全部槽验签失败时，Bootloader 停留在 `DFU_IDLE`，不得跳转未知镜像。

### 9.3 应用确认

新槽启动后状态为 `TESTING`：

- IWDG 超时为 2 秒；
- 健康确认窗口为 30 秒；
- 最多 3 次试启动；
- Bootloader 在每次跳入 `TESTING` 槽前先持久化增加启动次数，确保跳转后立即掉电也会被计为一次尝试；
- 只有健康窗口持续满足才调用 Bootloader 共享 API 写入 `CONFIRMED`；
- HardFault、非法状态机、调度积压、关键 CAN/编码器初始化失败或 IWDG 复位均消耗一次试启动机会；
- 三次失败后该槽标记 `REJECTED`，Bootloader 启动上一个 `CONFIRMED` 槽；
- 已经 `CONFIRMED` 的镜像发生一次运行时 IWDG 复位时记录故障，但不自动回滚到更旧版本。

应用确认 API 必须只允许当前运行槽调用，并检查 descriptor、metadata generation 和健康条件，避免旧应用误确认新槽。

IWDG 一旦启动不能由应用关闭。Bootloader 在进行可能较长的 Flash 擦写、SHA-256 和 ECDSA 验证时按自身健康检查喂狗；进入 `TESTING` 槽前重新建立明确的 2 秒监督节奏。应用只能在调度和安全健康门控通过时喂狗，不能由无条件定时中断喂狗。

## 10. 升级入口与安全停车

### 10.1 正常升级

```text
ELF2 下载并验证发布包
 -> 通过 CAN V1 发送全零 STOP 组
 -> 等待 STM32 反馈 SAFE_STOP/STOPPED 和两轮速度有效且接近零
 -> 记录当前活动槽和升级事务 ID
 -> 应用写入一次性 ENTER_DFU metadata 请求
 -> ELF2 置位 FORCE_BOOTLOADER GPIO作为独立兜底
 -> ELF2 拉低 NRST 后释放
 -> STM32 进入自定义 USB DFU
```

CAN V1 不新增维护命令。进入升级前必须满足：

- 后轮目标为零；
- 左右后轮反馈有效并连续确认静止；
- 无物理急停未处理异常；
- 当前槽为 `CONFIRMED`；
- ELF2 已保存可回滚的 metadata 状态。

### 10.2 故障恢复

若应用损坏、无有效确认槽、验签失败或连续试启动耗尽，Bootloader 不进入车辆控制，直接保持 DFU。ELF2 通过 GPIO 进入强制升级流程，成功写入并验签后才允许重新启动应用。

Bootloader 上电第一阶段必须将 PWM 关闭、TB6612 方向脚置于安全电平、舵机输出关闭，并在任何跳转前完成这些动作。不能依赖旧应用在复位前执行停车。

## 11. 自定义 USB DFU 接口

Bootloader 实现 USB DFU Device，建议提供：

- 一个 DFU interface；
- alt setting 0：slot A；
- alt setting 1：slot B；
- alt setting 2：metadata/受控维护区（默认禁用）；
- 下载地址和长度只能落在当前 alt 对应的槽区间；
- 禁止写 Bootloader、其他槽和 option bytes；
- 每个块写入后可读回校验；
- `GETSTATUS` 在擦除、编程和验签期间返回正确状态；
- `ABORT`、USB 断开和超时必须回到安全 DFU 状态；
- 完整镜像和 descriptor 验证通过后才更新 metadata，不能边写边标记可启动。

ELF2 使用 `dfu-util` 或兼容的 libusb DFU 客户端。Bootloader 的 USB VID/PID、DFU functional descriptor、alt 名称和状态码需要在实现阶段冻结并写入 `docs/OTA_PROTOCOL.md`。

## 12. 供电、USB 和 GPIO

STM32 开发板 USB VBUS 与车辆电源不能无条件并联。推荐：

- ELF2 USB Host 的 5 V 由受控限流开关提供；
- STM32 板由车辆电源供电时，USB 仅保留数据，或使用电源隔离；
- USB D+/D- 共地，信号线尽量短；
- 先用示波器确认 USB VBUS、NRST 和 BOOT0 时序，再接电机电源；
- ELF2 GPIO 通过开漏/三极管或电平兼容缓冲控制 `BOOT0`、`NRST`，不直接强推复位线；
- `FORCE_BOOTLOADER` 默认释放态，断电或 ELF2 崩溃不能让车辆进入可运动状态；
- STM32 `BOOT0` 在生产运行中保持低电平；普通 OTA 和强制恢复都不通过拉高 `BOOT0` 实现；
- 自定义 Bootloader 运行时 USB 线不能让 CAN 收发器继续驱动 PA11/PA12，CAN 必须已经迁移到 PB8/PB9。

## 13. 防篡改和 Flash 保护

生产烧录流程：

1. 使用受控工具写入 Bootloader、公钥、初始应用和 metadata；
2. 读回验证 Bootloader 和公钥；
3. 对 Bootloader 扇区启用 WRP 写保护；
4. 设置 RDP Level 1；
5. 记录 option bytes 和设备序列号；
6. 通过 ELF2 执行一次签名升级、断电恢复和回滚演练。

RDP Level 0 仅用于开发。RDP Level 1 到 Level 0 会触发主 Flash mass erase，因此不可作为普通现场恢复方法。RDP Level 2 不采用。

## 14. 服务器和 ELF2 软件设计

### 14.1 服务器

服务器发布每个硬件型号和槽位对应的：

- `manifest.json`；
- slot A/B descriptor；
- slot A/B BIN；
- descriptor 签名；
- 兼容性、发布日期和撤销状态。

服务器必须支持 HTTPS、版本清单原子发布、历史版本保留和撤销标记。撤销标记不能替代设备端签名和防降级检查。

### 14.2 ELF2 ota-agent

模块划分：

```text
manifest_client    HTTPS、证书、重试、缓存
release_verifier   JSON/descriptor 一致性、SHA-256、签名预检
vehicle_gate       CAN STOP、反馈静止、升级互斥
gpio_controller    FORCE_BOOTLOADER、NRST
dfu_client         USB 枚举、DFU 下载、状态和重试
health_monitor     CAN heartbeat、版本、故障和恢复
audit_logger       事务 ID、槽位、版本、摘要、结果和时间线
```

ELF2 预检失败时不得操作 GPIO。DFU 失败时不得清除当前 `CONFIRMED` 状态。升级事务必须可恢复，重启 ota-agent 后从 metadata 和 USB 状态重新判断，不重复盲写。

### 14.3 升级后的固件身份确认

当前 CAN V1 的 `0x180..0x184` 能证明控制器重新运行、反馈新鲜且无故障，但不携带固件版本、security version 或活动槽，不能单独证明目标镜像正在执行。

在 OTA 实施前必须为 `docs/CAN_PROTOCOL.md` 增加一个向后兼容、只读的固件身份反馈扩展，至少发布：

```text
firmware_version
security_version
running_slot
image_state (TESTING or CONFIRMED)
bootloader_api_version
```

该扩展只发布事实，不接受升级、擦写或运动指令，不改变 CAN V1 的控制所有权。旧主机可以忽略新反馈 ID。ELF2 只有同时满足以下条件才报告升级完成：

1. CAN heartbeat 和反馈序列重新新鲜；
2. 固件身份与本次签名 descriptor 一致；
3. 应用报告当前槽为目标槽；
4. 30 秒窗口结束后状态从 `TESTING` 变为 `CONFIRMED`；
5. 没有新的锁存故障或回滚事件。

## 15. 测试和验收

### 15.1 主机和协议测试

- JSON 与固定 descriptor 字段一致性；
- descriptor 签名成功、错误签名、错误公钥、修改任一字段；
- BIN SHA-256 不匹配；
- 产品、硬件、slot、地址、尺寸和 Bootloader 版本不匹配；
- security version 降级拒绝；
- 普通发布密钥不能降级，维护密钥可按策略授权；
- `dfu-util` 断点、重复块、短包、长包、USB 断开和重连；
- 当前 A 写 B、当前 B 写 A 的槽位选择。

### 15.2 STM32 Bootloader 测试

- 两份 metadata 各种掉电点恢复；
- descriptor 写入中断；
- 镜像写入每个 Flash 扇区后的掉电；
- 签名失败不改变活动槽；
- 三次 TESTING 失败后回滚；
- 健康窗口结束前复位不确认；
- 确认后单次 IWDG 只记录运行时故障；
- 无有效槽停留 DFU；
- Bootloader 写保护区域拒绝访问；
- 地址越界不能擦写 Bootloader、metadata 或另一槽。

### 15.3 板级测试

- PB8/PB9 CAN1 重映射与原有 `0x120..0x184` 协议回环；
- PA11/PA12 USB DFU 枚举；
- USB VBUS 与车辆电源无反灌；
- BOOT0/NRST 时序和强制入口；
- Bootloader 启动瞬间 PWM、TB6612 和舵机输出安全；
- 电机不接或架空时完成全套升级，不允许因升级流程产生非零运动命令。

### 15.4 车辆验收

必须先完成：

```text
无电机 -> 架空车轮 -> 低速空旷地面 -> 固定短路线
```

每一步记录 CAN、USB、IWDG、metadata、版本、slot、故障和升级时间线，并配备物理断电和人工急停。

## 16. 实施阶段

### Phase 1：硬件和链接布局

- CubeMX 将 CAN1 重映射到 PB8/PB9；
- 保留 PA11/PA12 USB；
- 增加 TIM4、PE1 和安全输出配置；
- 为 A/B 槽建立两个 linker script；
- 生成两个可启动的 slot-specific ELF/BIN。

### Phase 2：纯 Bootloader 状态和元数据

- 实现 metadata 固定结构、CRC、generation 和双副本；
- 实现槽状态、试启动次数和 reset reason；
- 实现镜像边界和向量表检查；
- 先使用测试公钥和固定测试镜像完成主机无关单元测试。

### Phase 3：密码学和 DFU

- 集成裁剪 SHA-256/ECDSA P-256 验签；
- 实现 USB DFU descriptors、alt setting 和写入范围；
- 实现 descriptor + BIN 的端到端验证；
- 以 `dfu-util` 完成单板下载。

### Phase 4：应用确认和回滚

- 集成 IWDG 2 秒配置；
- 实现 30 秒健康窗口和 3 次试启动；
- 实现 Bootloader 共享 API 和 metadata 确认；
- 注入 HardFault、调度卡死、CAN 失联和看门狗复位验证回滚。

### Phase 5：ELF2 OTA Agent

- 实现 HTTPS manifest 下载和本地缓存；
- 实现车辆安全门控和 CAN STOP；
- 实现 GPIO/NRST 时序；
- 实现 DFU 下载、重试、日志和升级后 CAN 健康确认。

### Phase 6：生产保护和车辆验收

- 验证 WRP、RDP1 和维修恢复流程；
- 完成断电、拔线、回滚、降级拒绝和签名密钥轮换演练；
- 形成带原始日志的升级验收记录。

## 17. 待后续冻结的细节

以下内容有设计方向，但不能在没有实物测量或 ELF2 资料时写死：

- ELF2 `FORCE_BOOTLOADER` 和 `NRST` 的具体 GPIO line，以及 STM32 接收 `FORCE_BOOTLOADER` 的普通 GPIO；
- ELF2 具体 USB Host 口、VBUS 开关和设备树节点；
- STM32 USB VID/PID、DFU interface/alt setting 名称；
- descriptor `product_id`、`hardware_id` 数值；
- Bootloader 确切密码库和 32 KiB 链接大小；
- CAN 停车确认需要的反馈采样次数；
- CAN 固件身份只读扩展的 CAN ID、帧布局和发送周期；
- 服务器 API、认证方式和签名机部署；
- 生产板 USB D+ 额外上拉的处理方式；
- 现场维护密钥的保管和轮换流程。

## 18. 参考资料

- `docs/CAN_PROTOCOL.md`
- `docs/superpowers/specs/2026-08-22-stm32-bottom-controller-design.md`
- ST AN2606：STM32 system memory boot mode
- ST AN4701：STM32F4 RDP protection and in-application programming
- ST RM0090：STM32F4 reference manual
- ST X-CUBE-CRYPTOLIB：SHA-256/ECDSA P-256 implementation options
- 优信电子 65407 STM32F407VET6 开发板公开原理图和引脚资料
