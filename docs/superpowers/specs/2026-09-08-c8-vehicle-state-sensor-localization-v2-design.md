# 《C8 车辆状态、传感器与定位融合架构设计 v2》

日期：2026-09-08
状态：设计评审版
适用范围：R3X Ackermann 移动机器人，Raspberry Pi 5，Ubuntu 24.04 主机，ROS 2 Humble Docker 工作区 `/home/c/robot_ws`

状态标记定义：

- **已实现**：当前 `robot_ws` 已存在对应源码、消息、配置或经过核对的运行接口；不等同于真实硬件生产链已启动或已验收。
- **待实现**：本设计冻结的第一版生产化修改。
- **后续规划**：不进入第一版实施范围，满足明确准入条件后再评审。

## 1. 设计目标

本设计重新审查 C8 的车辆状态、传感器接入与定位融合边界，冻结第一版生产架构为：

```text
map
  |
  |  AMCL / 2D 地图匹配，唯一发布者
  v
odom
  |
  |  robot_localization 局部 EKF，唯一发布者
  v
base_link
```

第一版使用局部 EKF 融合 wheel odom 与 IMU，使用 AMCL 将 2D LiDAR 观测约束到静态地图；不引入第二个 global EKF。

设计目标如下：

1. 保持 `odom -> base_link` 连续、低延迟，局部估计不因 AMCL 重定位而跳变。
2. 由 `map -> odom` 承载全局地图校正，Nav2 通过完整 TF 链获得全局位姿。
3. 消除 wheel odom pose/twist 同源重复融合，避免 EKF 对编码器信息过度自信。
4. 固化 TF 单一发布者规则，禁止设备驱动、第二 EKF 或其他里程计节点抢占主 TF。
5. 将设备健康、局部定位质量、全局地图定位质量和最终运动许可分层表达。
6. 在传感器超时、TF 异常、地图定位失效和 CAN 故障时执行可验证的降级或安全停止。
7. 保留现有 `car_hardware_adapter`、`car_localization`、`car_device_monitor`、`car_localization_monitor`、`car_vehicle_state` 和 Nav2 边界，不建立平行的重复系统。

本设计不修改 STM32 控制、安全状态、CAN 协议或标定门控；不授权烧录、标定或实车运动。

## 2. 当前系统输入输出

### 2.1 当前状态审查

| 模块 | 当前状态 | 已有输入/输出 | v2 结论 |
|---|---|---|---|
| STM32F407 | 已实现 | 编码器采集、底盘控制、CAN 反馈；轮速与累计轮位置进入 Pi | 保持底盘实时控制与安全闭环，不参与 ROS TF 发布 |
| `car_hardware_adapter` | 已实现源码；生产运行配置待实现 | SocketCAN/协议边界，发布 `/car/wheel_odom`、硬件状态与反馈 | 继续作为唯一 CAN owner；不得另起独立 CAN odom 节点 |
| `/car/wheel_odom` | 已实现接口 | `nav_msgs/msg/Odometry`，当前默认帧 `odom`/`base_link` | 作为局部 EKF 的轮式速度观测；其 pose 保留但默认不融合 |
| IM900 驱动 | 已实现且核查时运行 | `imu-dev` 以约 30 Hz、BEST_EFFORT 发布 `/vendor/imu`，帧为 `imu_link` | 设备容器只保留驱动；由主容器适配为 `/car/imu/data` |
| `imu_adapter_node` | 已实现源码 | `/vendor/imu` -> `/car/imu/data`，执行格式和帧校验 | 在 `humble-dev` 运行，作为 EKF 唯一 IMU 输入边界 |
| YDLIDAR 驱动 | 已有 launch 配置；核查时未运行 | 目标原始输出 `/scan` | 由 `lidar-dev` 独占 USB 并只发布扫描数据，不发布主 TF |
| `lidar_adapter_node` | 已实现源码 | 原始扫描 -> 归一化 `/car/scan` | 在 `humble-dev` 运行；AMCL 只订阅 `/car/scan` |
| `car_localization` | 已实现源码 | 封装官方 `robot_localization` EKF，订阅 `/car/wheel_odom`、`/car/imu/data`，发布 `/odom` 与质量信息 | 保留为局部定位唯一承载层；修订输入选择和质量输出 |
| AMCL/Nav2 | mock 导航链已实现 | mock 地图、`/car/scan`、`/car/amcl_pose`、`map -> odom` | TF 所有权正确；生产地图、参数、初始位姿与硬件 launch 待实现 |
| `car_localization_monitor` | 已实现 | 组合 EKF 质量、AMCL、地图身份和 `map -> odom` | 必须拆分 `local_valid` 与 `global_valid`，修正 AMCL/TF 年龄判定 |
| `car_vehicle_state` | 已实现 | 消费传感器健康和定位质量，输出 `/car/vehicle/state` | 必须区分 wheel odom 硬失效、IMU 可降级失效及全局导航失效 |

核查时的真实运行态为：`imu-dev` 正在发布 `/vendor/imu`；`lidar-dev` 与 `humble-dev` 主进程均为交互式 `bash`；`/car/wheel_odom` 接口和实现位于 `humble-dev` 工作区，但核查窗口未发现其活动发布者。该结果只描述核查时刻，不否定接口存在。

### 2.2 第一版主题契约

| 层级 | Topic | 类型/用途 | 所有者 |
|---|---|---|---|
| 原始 IMU | `/vendor/imu` | `sensor_msgs/msg/Imu` | `imu-dev` 中 IM900 驱动 |
| 归一化 IMU | `/car/imu/data` | EKF 输入 | `humble-dev` 中 `imu_adapter_node` |
| 原始雷达 | `/scan` | `sensor_msgs/msg/LaserScan` | `lidar-dev` 中雷达驱动 |
| 归一化雷达 | `/car/scan` | AMCL 与健康监控输入 | `humble-dev` 中 `lidar_adapter_node` |
| 轮式里程计 | `/car/wheel_odom` | `nav_msgs/msg/Odometry` | `car_hardware_adapter` |
| 局部定位 | `/odom` | 连续局部位姿与速度 | `car_localization` facade |
| AMCL 位姿 | `/car/amcl_pose` | 地图坐标系位姿及协方差 | AMCL remap 输出 |
| 传感器健康 | `/car/sensors/health` | wheel/IMU/scan/base controller 分项健康 | `car_device_monitor` |
| 局部定位质量 | `/car/localization/quality` | EKF、输入年龄、协方差与局部 TF | `car_localization` |
| 全局定位状态 | `/car/navigation/localization_state` | AMCL、地图身份、`map -> odom` 与 local/global 有效性 | `car_localization_monitor` |
| 车辆状态 | `/car/vehicle/state` | 最终状态、降级、SafeStop 和运动许可 | `car_vehicle_state` |

设备层 topic 与归一化 topic 不得互相 remap 成同名，避免适配器形成自订阅环路。

## 3. 总体架构图

```text
┌──────────────────────────── 设备与底盘层 ────────────────────────────┐
│ STM32 CAN ──> can0 ──> car_hardware_adapter ──> /car/wheel_odom     │
│ IM900 ──> imu-dev/driver ──> /vendor/imu                            │
│ YDLIDAR ──> lidar-dev/driver ──> /scan                              │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              v
┌────────────────────── humble-dev 归一化与局部层 ─────────────────────┐
│ /vendor/imu ──> imu_adapter ──> /car/imu/data                       │
│ /scan ──> lidar_adapter ──> /car/scan                               │
│                                                                    │
│ /car/wheel_odom ─┐                                                 │
│                   ├─> car_localization / local EKF ──> /odom       │
│ /car/imu/data ───┘                              └─> odom->base_link │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              v
┌────────────────────── humble-dev 地图定位层 ────────────────────────┐
│ map_server ──> map                                                 │
│ /car/scan + map + odom->base_link ──> AMCL                         │
│                                  ├─> /car/amcl_pose                │
│                                  └─> map->odom                     │
│ map->odom->base_link ──> Nav2                                     │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              v
┌────────────────────────── 质量与安全层 ─────────────────────────────┐
│ car_device_monitor ──────────────> /car/sensors/health              │
│ car_localization ────────────────> /car/localization/quality        │
│ car_localization_monitor ────────> /car/navigation/localization_state│
│ car_vehicle_state ───────────────> /car/vehicle/state               │
│ Nav2 adapter / motion gate 仅在相应 profile 的质量门满足时放行       │
└─────────────────────────────────────────────────────────────────────┘
```

模块职责边界冻结如下：

- STM32 负责编码器采集、底盘控制、安全状态和 CAN 反馈，不生成 ROS 位姿或 TF。
- `car_hardware_adapter` 负责 CAN 协议解析、反馈快照聚合、轮速/轮位置单位转换和 `/car/wheel_odom` 发布，不发布 `odom -> base_link`。
- `car_localization` 封装 local EKF，融合 wheel odom 与 IMU，唯一发布 `odom -> base_link`。
- 雷达驱动只访问串口/USB 并发布 `/scan`，不得发布 `odom -> base_link`、`map -> odom` 或静态安装 TF。
- `lidar_adapter_node` 只做扫描消息合法性、frame、时间与输出 topic 归一化，不估计机器人位姿。
- AMCL 使用静态地图、`/car/scan` 和局部 TF，唯一发布 `map -> odom`。
- Nav2 只消费定位结果，不成为 TF 权威来源。
- 质量监控只判定和门控，不修改传感器值，不生成替代位姿。

## 4. TF 设计规范

### 4.1 TF 所有权

| Transform | 类型 | 唯一发布者 | 规则 |
|---|---|---|---|
| `base_link -> imu_link` | 静态 | `robot_state_publisher`；无 URDF 时才使用一个 `static_transform_publisher` | 安装位置和姿态必须来自实测，当前全零配置不得直接作为生产标定 |
| `base_link -> laser_link` | 静态 | `robot_state_publisher`；无 URDF 时才使用一个 `static_transform_publisher` | v2 目标帧名为 `laser_link`，安装外参必须实测 |
| `odom -> base_link` | 动态 | `car_localization` 内 local EKF | 全系统唯一；连续、不得因地图重定位跳变 |
| `map -> odom` | 动态 | AMCL | 第一版全系统唯一；允许地图重定位产生校正 |

当前工作区使用雷达帧 `laser`。v2 将目标帧统一为 `laser_link`，但迁移必须一次性修改雷达驱动 `frame_id`、`lidar_adapter` 期望帧、设备监控、静态 TF、AMCL 输入和测试。迁移完成前继续使用 `laser`；禁止同时发布 `laser` 和 `laser_link` 两套安装 TF 来掩盖配置不一致。

### 4.2 发布规范

- 静态安装关系优先进入机器人 URDF，由一个 `robot_state_publisher` 发布。
- 暂未具备完整 URDF 时，可在 `humble-dev` 使用独立静态 TF 节点；设备容器不得发布安装 TF。
- `base_link` 坐标约定为 x 向前、y 向左、z 向上。IMU 和 LiDAR 数据必须通过静态 TF 正确变换到该约定。
- local EKF 使用 `world_frame=odom`、`odom_frame=odom`、`base_link_frame=base_link`、`publish_tf=true`。
- AMCL 使用 `global_frame_id=map`、`odom_frame_id=odom`、`base_frame_id=base_link`、`tf_broadcast=true`。
- AMCL 未激活或全局定位无效时，`odom -> base_link` 仍可存在；不得伪造 `map -> odom`。
- TF 时间戳必须使用 ROS 时钟；运行时同时用 steady clock 判断接收年龄，防止 ROS 时间跳变掩盖超时。

### 4.3 明确禁止

- 禁止雷达驱动、scan matcher 或 wheel odom 节点发布 `odom -> base_link`。
- 禁止第二个 EKF 重复发布 `odom -> base_link`。
- 禁止 AMCL 与 global EKF 同时发布 `map -> odom`。
- 禁止 `car_hardware_adapter` 与 local EKF 同时发布相同 TF。
- 禁止静态发布 `map -> odom` 或 `odom -> base_link`。
- 禁止通过 TF alias、重复 frame 或零外参临时绕过 frame mismatch。
- 启动审计发现重复发布者、TF 环、父节点不唯一、动态 TF 超时或未来时间戳时，定位质量立即无效，运动门关闭。

## 5. 局部 EKF 设计

### 5.1 EKF 职责

local EKF 负责生成连续局部状态：平面位置、yaw、纵向速度和 yaw rate。其输入仅来自经过校验的 `/car/wheel_odom` 与 `/car/imu/data`，输出经 `car_localization` facade 发布为 `/odom`，并唯一发布 `odom -> base_link`。

第一版保持：

```text
frequency: 50 Hz
two_d_mode: true
world_frame: odom
odom_frame: odom
base_link_frame: base_link
publish_tf: true
sensor_timeout: 0.20 s
```

### 5.2 Wheel odom 输入选择

`/car/wheel_odom` 的 pose 与 twist 均由同一组编码器反馈和同一运动学模型生成。当前 EKF 同时启用 x/y/yaw 与 vx/wz，会将相关信息当作多份独立观测，导致协方差过小、转弯误差被重复强化，并在轮胎打滑时降低滤波器对 IMU 的响应能力。

第一版只融合 wheel odom 的：

- `twist.linear.x`：车辆纵向速度 `vx`；
- `twist.angular.z`：基于 R3X Ackermann 后轴/转向运动学得到的 yaw rate `wz`。

默认不融合：

- pose x、y、yaw；
- lateral velocity `vy`；
- z、roll、pitch、垂直速度和加速度。

推荐配置向量顺序按 `robot_localization` 的 15 状态定义：

```yaml
odom0_config: [false, false, false,
               false, false, false,
               true,  false, false,
               false, false, true,
               false, false, false]
odom0_differential: false
odom0_relative: false
```

wheel odom pose 继续保留在消息中，用于独立诊断、回放、尺度误差评估和与 EKF 输出对比，但不作为第一版 EKF 更新量。这样由 EKF 对 `vx/wz` 进行一次状态积分，不再重复融合编码器自身的积分结果。

### 5.3 IMU 输入选择

第一版基线只融合：

- `angular_velocity.z`：车体 yaw rate。

基线配置：

```yaml
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
imu0_differential: false
imu0_relative: false
```

IMU orientation yaw 不是默认可信量。只有以下条件全部通过后才启用：

1. `orientation_covariance[0] != -1`，四元数归一化且 covariance 有效；
2. `imu_link` 到 `base_link` 的轴向和安装外参完成实测；
3. 静止偏置、短时漂移、重复上电初始 yaw 和电机工作时磁干扰测试通过；
4. orientation yaw 与 gyro z、wheel wz 在直行和圆弧数据中的符号与尺度一致；
5. yaw covariance 来自实测，不使用零值或无依据常数。

启用 orientation yaw 后配置为 yaw + yaw rate，并设置 `imu0_relative=true`，使其约束局部 `odom` 起点后的相对航向，不把磁北或 IMU 内部世界系误当作 `map` 航向：

```yaml
imu0_config: [false, false, false,
              false, false, true,
              false, false, false,
              false, false, true,
              false, false, false]
imu0_relative: true
```

第一版不融合 IMU 线加速度。R3X 当前没有完成振动、重力补偿、温漂和加速度 bias 验收，直接融合会造成静止漂移和速度污染。

### 5.4 同变量融合与协方差

wheel wz 与 IMU gyro z 可同时融合，因为其误差来源不同；前提是 covariance 能反映真实置信度。IMU 负责快速角速度响应，wheel wz 提供无积分漂移的运动学约束。两者持续不一致时不得让 EKF 静默折中，质量监控必须报告 `yaw_rate_innovation_excessive` 并进入降级或失效策略。

当前 `car_hardware_adapter` 对 wheel odom pose/twist 使用固定 `0.01` 对角协方差。该值只能视为开发占位，不能作为生产验收值。待实现要求：

- 分别标定 `vx` 与 `wz` 方差，不共用一个常数；
- 直行、不同半径圆弧、低速、制动和轻微打滑数据分别统计；
- 无转向位置反馈时，对 wheel wz 使用更保守协方差；
- 编码器无效、反馈快照不完整、CAN 超时或安全状态异常时停止向 EKF 转发该观测，不用零值伪装有效数据；
- covariance 不得为 NaN、负数或启用变量上的全零值。

### 5.5 时间、重置和输出

- 输入 stamp 必须非零、单调、不晚于当前 ROS 时间容差，并通过现有 epoch/generation 门控。
- EKF 重置期间停止转发旧观测；重置后必须等待新的 wheel/IMU 样本和连续质量恢复样本。
- `/odom` 只发布通过 facade 质量门的 local EKF 输出。
- local EKF 异常不得由 wheel odom pose 直接旁路生成 `odom -> base_link`。

## 6. 雷达地图定位设计

### 6.1 数据路径

正式链路为：

```text
YDLIDAR USB
  -> lidar-dev / ydlidar_ros2_driver_node
  -> /scan                    frame=laser_link
  -> humble-dev / lidar_adapter_node
  -> /car/scan                normalized LaserScan
  -> AMCL + map_server
  -> /car/amcl_pose
  -> map -> odom
```

AMCL 不直接订阅未校验的 vendor/raw topic。`lidar_adapter_node` 负责拒绝错误 frame、无效时间戳、非有限量程、角度字段不一致和不满足输入契约的扫描。

### 6.2 AMCL 责任与生产配置

AMCL 负责将 LiDAR 扫描匹配到静态地图，并根据局部 `odom -> base_link` 运动增量发布 `map -> odom`。AMCL 不修改 `/odom`，不发布 `odom -> base_link`，不进入 local EKF。

生产参数必须满足：

| 参数 | v2 约束 |
|---|---|
| `global_frame_id` | `map` |
| `odom_frame_id` | `odom` |
| `base_frame_id` | `base_link` |
| `scan_topic` | `/car/scan` |
| `tf_broadcast` | `true`，第一版 AMCL 为 `map -> odom` 唯一发布者 |
| map | 使用真实 R3X 场地地图，登记 `map_id` 与 `map_hash`；mock map 不得进入生产 |
| initial pose | 由明确的初始位姿流程、人工设定或重定位流程提供；不得沿用 mock 的固定零位姿假设 |
| transform tolerance | 初始工程目标 0.20 s，验收不得超过 0.25 s；不得沿用 mock 的 1.0 s 掩盖延迟 |

ROS 2 Humble 当前 AMCL 配置使用 `nav2_amcl::DifferentialMotionModel`。R3X 为 Ackermann 底盘，该模型不是严格的 Ackermann 运动模型，但在 AMCL 只消费已形成的 `odom -> base_link` 增量、车辆不执行原地旋转的前提下，可作为第一版非完整约束近似。该选择必须通过直线、左右圆弧、S 弯和停止重启数据验证；若出现与转弯半径相关的系统性粒子偏置，再进入 Ackermann 专用 motion model 评审。第一版不得假设机器人能够原地旋转。

mock 配置中的 `alpha1..alpha5=0.2`、`update_min_d=0.0`、`update_min_a=0.0` 仅服务确定性测试，不作为生产参数。生产值必须由真实 bag 回放、CPU 负载和地图定位误差共同确定。

### 6.3 全局有效性

全局定位有效必须同时满足：

- local EKF 有效；
- `/car/scan` 在线、频率和 frame 合法；
- 地图 ID/hash 与当前任务一致；
- AMCL lifecycle 为 active；
- AMCL 最近位姿 covariance 在阈值内；车辆运动期间 AMCL pose 年龄必须在阈值内，静止期间 pose 年龄只作为诊断量；
- `map -> odom` 存在、时间新鲜且只有一个发布者；
- ROS time epoch 与 localization epoch 一致。

AMCL 或扫描失效时仅撤销全局定位；局部 `odom -> base_link` 不得被清零、跳变或替换。

## 7. Docker 部署设计

### 7.1 容器职责

| 容器 | 运行内容 | 设备所有权 | 禁止内容 |
|---|---|---|---|
| `humble-dev` | `car_hardware_adapter`、`imu_adapter_node`、`lidar_adapter_node`、`car_device_monitor`、`car_localization`/local EKF、map server、AMCL、`car_localization_monitor`、`car_vehicle_state`、Nav2、diagnostics | 独占 `can0` 的主 ROS 访问链 | 第二个硬件 adapter、第二个 local EKF、重复 TF 发布者 |
| `imu-dev` | IM900 串口驱动，发布 `/vendor/imu` | 独占 IMU 串口 `/dev/ttyAMA0` | adapter、EKF、wheel odom、主 TF、Nav2 |
| `lidar-dev` | YDLIDAR 驱动，发布 `/scan` | 独占雷达 USB/串口 | scan adapter、AMCL、EKF、底盘控制、任何主 TF |
| `humble-camera` | 相机驱动 | 独占相机设备 | C8 定位主链路节点 |

所有容器使用 host network 时必须统一 ROS_DOMAIN_ID、RMW 实现和时间源。当前运行使用 `rmw_fastrtps_cpp`，生产 launch 和服务必须显式保持一致。

### 7.2 设备与启动约束

- 雷达设备必须只有一个运行时 owner。当前 `lidar-dev` 映射为 `/dev/car_lidar`，现有 launch 使用 `/dev/ydlidar`，两者不一致。v2 统一使用稳定 udev 名称 `/dev/car_lidar`，并同步容器映射与驱动参数。
- `humble-dev` 不再打开雷达串口；即使存在历史设备映射，也不得启动第二个驱动。
- 镜像名不能代表角色；部署和诊断以容器名、设备映射和实际进程为准。
- 生产启动顺序为：设备驱动 -> 归一化 adapters/静态 TF -> hardware adapter/local EKF -> map server/AMCL -> localization monitor/vehicle state -> Nav2 lifecycle active。
- Nav2 只有在 local/global 定位状态和 vehicle state 均满足当前 profile 门控后才接受任务。
- 设备容器可独立重启；主容器必须通过 topic 超时进入降级/停止，不依赖容器进程存在来判定数据健康。

当前 `tmini_plus_hardware.launch.py` 只启动传感器和监控，未启动真实 hardware adapter、localization、AMCL 和 Nav2；`mock_c_system.launch.py` 才包含完整组合但使用 mock backend。**待实现**一个生产 C8 bringup/profile，使用真实 SocketCAN 参数、真实几何参数、真实传感器 topics 和本文 TF 所有权。不得直接将 mock YAML 改名后用于实车。

该部署边界带来的工程收益：

- USB/串口故障与主定位进程隔离；
- 设备驱动可独立替换和重启；
- 避免多个节点抢占同一串口或发布重复 topic；
- local/global 定位与设备生命周期解耦；
- 主容器统一管理 TF、Nav2、质量门控和版本化配置，便于部署与回滚。

## 8. 定位质量监控

### 8.1 分层质量接口

质量事实按四层管理：

1. `/car/sensors/health`：原始/归一化传感器在线、有效、频率、年龄、frame 和原因码；
2. `/car/localization/quality`：local EKF 输入、输出、协方差和 `odom -> base_link`；
3. `/car/navigation/localization_state`：地图身份、AMCL、`map -> odom` 以及 local/global 有效性；
4. `/car/vehicle/state`：结合硬件、focus/estop、传感器和定位状态形成最终运动许可。

任一层不得用“节点存在”替代“数据新鲜且有效”。

### 8.2 `/car/localization/quality` v2

现有 `LocalizationQuality.msg` 已实现：`valid`、`score`、`position_covariance`、`heading_covariance`、`age_sec`、`source`、`reason`、`time_epoch`、`localization_epoch`、`recovery_count`。

第一版在消息尾部向后追加以下字段并重编译工作区消费者：

| 字段 | 含义 |
|---|---|
| `wheel_odom_age_sec` | local EKF 最近接受的 wheel odom steady-clock 年龄 |
| `imu_age_sec` | 最近接受的 IMU steady-clock 年龄 |
| `ekf_output_age_sec` | EKF 最近有效输出年龄；现有 `age_sec` 在迁移期保持同值 |
| `local_ekf_state` | `INITIALIZING`、`RUNNING`、`DEGRADED_IMU`、`STALE_WHEEL_ODOM`、`ERROR` |
| `odom_base_tf_available` | `odom -> base_link` 是否存在且 frame 正确 |
| `odom_base_tf_age_sec` | 该动态 TF 的更新时间 |
| `wheel_odom_timeout` | wheel odom 超时事实 |
| `imu_timeout` | IMU 超时事实 |
| `tf_timeout` | local TF 缺失、超时、未来时间或所有权异常事实 |

`position_covariance` 和 `heading_covariance` 继续从 EKF 输出提取，不从 AMCL 覆盖。AMCL 的 covariance、pose 年龄和 `map -> odom` 年龄属于 `/car/navigation/localization_state`。

质量输出频率保持 10 Hz。传感器健康默认阈值继续采用当前工程值：wheel odom 0.25 s、IMU 0.25 s、scan 0.50 s；local EKF 自身 `sensor_timeout` 为 0.20 s。超时判定使用 steady clock，消息 stamp 合法性使用 ROS time。

### 8.3 Local/global 状态拆分

当前 `car_localization_monitor` 将 `local_valid` 与 `global_valid` 赋为同一个 aggregate decision，无法表达“局部仍连续但地图定位失效”。v2 必须拆分：

```text
local_valid = local EKF quality valid

global_valid = local_valid
            && scan healthy
            && AMCL lifecycle active
            && AMCL covariance accepted
            && (robot stationary || AMCL pose fresh)
            && map identity valid
            && map->odom available/fresh
```

AMCL 在机器人静止、未跨过更新阈值时可以不产生新的 pose，故 `amcl_pose_age_sec` 不得作为静止状态下的单一失效条件。监控器应使用 `/odom` twist 或现有 motion state 判定是否处于运动状态；运动期间要求 AMCL pose 推进，静止期间要求 scan、AMCL lifecycle、最近一次 covariance 和 `map -> odom` 保持有效。

当前监控器参数名 `max_amcl_age_sec` 实际被用于 `map -> odom` TF 年龄，语义不一致。v2 分成两个独立阈值：

- `max_amcl_pose_age_sec`：默认 0.50 s；
- `max_map_odom_tf_age_sec`：默认 0.25 s。

`max_amcl_pose_age_sec` 仅在车辆运动时作为硬门。`map -> odom` 允许 AMCL 按 `transform_tolerance` 有界地发布未来时间戳；监控器必须检查带符号时间差，允许范围内的前移，拒绝超过容差的未来时间，不能把任意负年龄直接钳为零并判定新鲜。

`/car/navigation/localization_state` 追加 `map_odom_age_sec`，并保持现有 map ID/hash、epoch 和 reason code。local invalid 必须使 global invalid；global invalid 不反向伪造 local invalid。

## 9. 安全降级策略

### 9.1 故障分类

| 条件 | local EKF | 全局定位/Nav2 | 车辆状态与动作 |
|---|---|---|---|
| IMU 超时，wheel odom 正常 | 继续 wheel-only 预测，状态 `DEGRADED_IMU`，增大 yaw covariance | AMCL 可继续校正；是否允许导航由 profile 决定 | 进入 `DEGRADED`；第一版默认 `motion_allowed=false`，直到降级速度限制链通过验收 |
| IMU orientation 不可信、gyro z 正常 | 禁用 orientation yaw，保留 gyro z + wheel | 可继续 | 标记降级，不触发 TF 切换 |
| wheel odom 超时/无效 | local invalid，停止公开新 `/odom` 和 local TF 更新 | 立即 global invalid，取消 Nav2 任务 | 请求全零速度并进入 `SAFE_STOP` |
| wheel odom 与 IMU 同时超时 | local invalid | global invalid | `SAFE_STOP`，禁止自动恢复运动 |
| scan 超时 | local EKF 不受影响 | global invalid，取消/拒绝地图导航 | 地图导航 profile 进入 `SAFE_STOP`；人工/维护 profile 可保持 local-only 待机 |
| AMCL pose 超时或 covariance 超限 | local EKF 不受影响 | global invalid | 取消/拒绝 Nav2；不得重置 local odom |
| `map -> odom` 缺失/超时 | local EKF 不受影响 | global invalid | 地图导航停止 |
| `odom -> base_link` 缺失/超时 | local invalid | global invalid | 禁止运动并进入 `SAFE_STOP` |
| TF 重复发布、TF 环或父节点冲突 | local/global 均不可信 | Nav2 不得激活 | 启动失败或运行时 `SAFE_STOP` |
| map ID/hash 不匹配 | local EKF 不受影响 | global invalid | 禁止地图导航 |
| CAN Bus-Off、TX failure、底盘 fault | 不再接收可信 wheel 观测 | global invalid | 服从硬件安全链，立即 `SAFE_STOP` |
| ROS 时间回跳/epoch 变化 | 清空旧输入并重建 local EKF generation | AMCL/global 状态失效并增加 localization epoch | 停止任务，重新等待完整恢复门 |

### 9.2 车辆状态策略修订

当前 `car_vehicle_state` 将 IMU 与 wheel odom 同时作为 `hardware_ready` 硬条件，IMU失效直接走 SafeStop。v2 将事实分为：

- **硬件必需**：base controller、wheel odom、急停/focus；
- **局部可降级**：IMU；
- **全局导航必需**：scan、AMCL、地图身份、`map -> odom`；
- **全局硬禁止**：任何 TF 所有权/拓扑异常。

增加 profile 参数：

- `require_global_localization_for_motion`；
- `allow_imu_degraded_motion`；
- `degraded_max_linear_speed_mps`；
- `degraded_max_angular_speed_radps`。

在限速链、Nav2 adapter 和底盘命令门完成端到端测试前，`allow_imu_degraded_motion=false`。因此第一版能正确表达 `DEGRADED_IMU`，但默认仍关闭运动许可；不得仅通过状态名称“DEGRADED”绕过安全门。

### 9.3 恢复策略

- 设备监控至少连续 3 个健康样本后恢复传感器有效；
- local EKF/facade 至少连续 5 个有效输出后恢复 local quality；
- global quality 还必须等待新 AMCL pose、新 `map -> odom`、正确 map identity 和一致 epoch；
- SafeStop 后不复用旧 Nav2 goal 或旧速度命令；必须重新取得 focus/authority；
- 故障恢复不得启动第二个 adapter、第二 EKF 或临时 TF 发布器。

## 10. 后续演进路线

### 10.1 第一阶段：局部融合纠正（待实现）

- 建立真实 SocketCAN/几何参数 production profile 和完整 C8 bringup；
- 修订 local EKF 输入向量为 wheel `vx/wz` + IMU gyro z；
- 将 IMU orientation yaw 置于显式验收开关之后；
- 标定 wheel/IMU covariance，移除固定 `0.01` 的生产依赖；
- 统一雷达帧为 `laser_link`、设备路径为 `/dev/car_lidar`；
- 扩展 `/car/localization/quality` 并拆分 local/global validity；
- 修订 `car_vehicle_state` 的 IMU 降级与 global navigation 门控；
- 增加 TF 单一所有者、输入超时、epoch reset 和故障恢复测试。

### 10.2 第二阶段：只读硬件接入（待实现）

- 使用 `allow_motion=false` 启动真实 CAN、IMU、雷达和 local EKF；
- 记录 `/car/wheel_odom`、`/car/imu/data`、`/car/scan`、`/odom`、TF 与质量 topic；
- 完成 IMU 安装方向、gyro bias、orientation yaw、雷达外参和 wheel covariance 验收；
- 清除并解释 `can0` 现有 error-pass/RX error/drop 计数增长原因后再进入运动测试；
- 验证容器重启、topic 超时和 TF 缺失不会产生运动许可。

### 10.3 第三阶段：AMCL 与 Nav2 生产化（待实现）

- 建立真实地图、`map_id`、`map_hash` 和初始位姿流程；
- 用 R3X 直线、左右圆弧、S 弯、停止和重定位 bag 调整 AMCL motion/noise 参数；
- 验证 `map -> odom` 校正不破坏 `odom -> base_link` 连续性；
- 完成 scan/AMCL/map-TF 故障注入、Nav2 goal 取消和恢复门测试；
- 进行长时间 Nav2 运行，记录定位 covariance、TF 延迟、重定位次数和控制振荡。

### 10.4 第一版不引入 global EKF 的原因

- AMCL 已提供所需全局 `map -> odom` 校正，第二个 EKF 在当前阶段没有独立价值来源；
- AMCL 与 global EKF 极易形成 `map -> odom` 重复发布；
- local odom、AMCL pose 和 IMU orientation covariance 尚未完成真实数据标定，增加滤波层会放大错误置信度；
- 当前 local/global validity 仍未拆分，先增加 global EKF 会使故障来源更难定位；
- 雷达驱动、真实地图、生产 bringup 和长时间 Nav2 稳定性尚未验收；
- R3X Ackermann 运动模型与现有 AMCL differential 近似仍需实测，不应同时引入第二套状态估计变量。

### 10.5 Global EKF 准入条件（后续规划）

只有以下条件全部满足后才重新评审 global EKF：

1. IMU gyro 与可选 orientation yaw 完成温漂、振动、重启和磁干扰验收；
2. wheel odom 的尺度、转弯 yaw rate 和 covariance 完成真实直线/圆弧验证；
3. AMCL 在目标地图上长时间稳定，重定位、遮挡和 scan 恢复行为可重复；
4. `map -> odom -> base_link` 在 Nav2 长时间测试中无重复发布、无时间断裂、无异常跳变；
5. local/global 质量接口、故障注入和车辆状态门控全部通过；
6. 存在明确需求证明 AMCL 直接校正不能满足平滑性或多全局观测融合要求。

若未来引入 global EKF，必须同时执行：

- 设置 AMCL `tf_broadcast=false`；
- global EKF 成为 `map -> odom` 唯一发布者；
- local EKF 仍是 `odom -> base_link` 唯一发布者；
- global EKF 只融合 local `/odom` 与经过质量门的 `/car/amcl_pose`；
- 重新完成 TF 所有权、协方差、一致性和故障恢复评审。

本设计取代 C8 中关于局部/全局定位融合、TF 所有权、相关传感器健康和定位运动门控的旧假设；STM32 CAN 协议与底盘安全设计仍由现有规范约束。
