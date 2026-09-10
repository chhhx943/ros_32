# C8 v2 局部 EKF 设计补充版

## 文档定位

本文件是《C8 车辆状态、传感器与定位融合架构设计 v2》第 5 节“局部 EKF 设计”的工程化补充，不改变既有架构、输入选择或 TF 所有权。

本补充版只覆盖当前 Raspberry Pi 5、ROS 2 Humble、Docker 运行环境下的 local localization bring-up 与静态验证阶段。当前继续保持 `allow_motion=false`，不启动 AMCL、Nav2 或 global EKF，不进行动态运动实验。

## 1. 冻结设计与职责边界

### 1.1 冻结输入

local EKF 的输入保持不变：

| 输入 | 融合变量 | 当前状态 |
|---|---|---|
| `/car/wheel_odom` | `twist.linear.x`（`vx`）、`twist.angular.z`（`wz`） | 已实现并已核查约 50 Hz |
| `/car/imu/data` | `angular_velocity.z` | 已实现并已核查约 30 Hz |

以下变量在当前阶段继续禁止进入 EKF：

- wheel odom pose：`x`、`y`、`yaw`；
- IMU orientation yaw；
- IMU 线加速度；
- `vy`、roll、pitch、z 方向状态。

### 1.2 模块职责

| 模块 | 允许职责 | 禁止职责 |
|---|---|---|
| `car_hardware_adapter` | CAN 解析、反馈聚合、wheel odom 发布 | 发布 `odom -> base_link` TF、旁路生成 `/odom` |
| `imu_adapter_node` | `/vendor/imu` 校验与归一化为 `/car/imu/data` | 发布主 TF、运行 EKF |
| `car_localization` | 承载唯一 local EKF 与 facade，输出 `/odom` 和 `odom -> base_link` | 发布 `map -> odom`、替代车辆安全控制 |
| localization quality/facade | 报告输入、EKF 输出、协方差和 TF 健康 | 直接执行电机停止或替代安全状态机 |
| vehicle safety/state | 根据定位质量和底盘状态决定是否允许运动 | 修改 EKF 输入语义或抢占 TF |

当前只能有一个 local EKF 实例。若 `car_localization` 已封装或启动 `robot_localization`，不得再单独启动第二个 `ekf_node`。

## 2. EKF 频率与传感器频率关系

### 2.1 配置原则

local EKF 保持：

```yaml
frequency: 50.0
sensor_timeout: 0.20
two_d_mode: true
world_frame: odom
odom_frame: odom
base_link_frame: base_link
publish_tf: true
```

`frequency=50 Hz` 是 EKF 状态输出和预测循环的目标频率，不是对所有传感器发布频率的硬性要求。当前 wheel odom 约 50 Hz，IMU 约 30 Hz，二者可以合法地低于或不同于 EKF 频率。

### 2.2 无新 measurement 时的状态更新

`robot_localization` 在每个 EKF 周期根据上一状态、车辆运动模型和过程噪声协方差执行 prediction；当该周期没有新的 wheel 或 IMU measurement 时，不会复制一条旧测量作为新测量，而是将状态预测到当前时间并按过程噪声传播不确定性。

因此：

- 高频 `/odom` 输出表示 EKF 在 50 Hz 提供连续状态接口；
- 不代表 wheel odom 或 IMU 必须都达到 50 Hz；
- 高频输出不等于高频新观测；
- measurement 超过 `sensor_timeout` 后，EKF 不得继续把过期样本当作新观测融合；
- 是否将“仅靠预测维持的输出”视为可用定位，由 localization facade/quality 判定，不由 EKF 单独决定。

`sensor_timeout` 只负责测量时效性，不负责底盘急停、控制权撤销或 CAN 命令发送。

## 3. `odom` 初始化与坐标语义

### 3.1 局部坐标定义

`world_frame=odom` 时，`odom` 表示机器人启动后建立的局部连续坐标系：

- 不代表地图坐标；
- 不代表地理坐标或磁北方向；
- 允许随时间产生累计漂移；
- 不因全局重定位而跳变。

当前 EKF 启动时，`base_link` 默认从本次 local localization generation 的局部原点开始。若没有显式初始状态配置，工程上应将其理解为局部参考起点，而不是车辆在地图中的真实绝对位姿。

wheel odom 消息中的 `header.frame_id=odom` 和 `child_frame_id=base_link` 是消息语义；它们不授予 `car_hardware_adapter` 发布动态 TF 的权限。

### 3.2 TF 层级

当前阶段的目标 TF 关系为：

```text
map
 └── odom                 （当前阶段不启动全局定位时可不存在）
      └── base_link       （local EKF 唯一发布）
           └── imu_link   （静态安装关系）
```

职责固定为：

| TF | 发布者 | 当前阶段要求 |
|---|---|---|
| `odom -> base_link` | local EKF | 必须唯一；静态验证阶段核查 |
| `base_link -> imu_link` | `robot_state_publisher` 或静态 TF 节点 | 必须可查询，参数来源需明确 |
| `map -> odom` | 后续 AMCL/地图匹配模块 | 当前阶段不启动、不验证地图定位 |

local EKF 不负责全局定位，不创建 `map` 坐标，也不发布 `map -> odom`。

## 4. TF 与时间验证要求

### 4.1 必须满足

静态验证阶段必须确认：

1. `odom -> base_link` 只有一个动态发布者，且该发布者是 local EKF；
2. `base_link -> imu_link` 存在且方向正确；
3. `/car/imu/data.header.frame_id` 为 `imu_link`；
4. `/car/wheel_odom.header.frame_id` 为 `odom`，`child_frame_id` 为 `base_link`；
5. 不存在第二个 EKF、wheel odom 节点或设备驱动发布相同动态 TF；
6. 当前未启动全局定位时，不应出现由 local EKF 产生的 `map -> odom`。

### 4.2 时间连续性

应检查 `/car/wheel_odom`、`/car/imu/data` 和 `/odom` 的 timestamp：

- 非零；
- 按消息到达顺序单调不回退；
- 不出现持续的未来时间戳；
- EKF 输出时间与当前 ROS clock 一致；
- TF 时间戳连续，不因单次传感器抖动形成明显跳变。

TF 查询失败、父节点变化、重复 broadcaster 或时间戳异常，均应使 local quality 不通过；不得用 wheel pose 旁路补发 `odom -> base_link`。

## 5. `sensor_timeout` 与安全责任边界

### 5.1 定位估计职责

| 故障 | local EKF 行为 | localization facade/quality 行为 |
|---|---|---|
| IMU timeout | 停止融合过期 IMU measurement；不伪造零角速度 | 标记 IMU 输入超时或 local quality 降级 |
| wheel odom timeout | 停止融合过期 wheel measurement；不伪造零速度 | 标记 wheel 输入失效，local quality 不通过 |
| EKF 输出异常 | 停止接受无效输出 | 标记 EKF/TF 错误，禁止将输出作为有效定位 |

### 5.2 车辆安全职责

车辆安全控制与定位估计分离：

- EKF 不负责急停；
- `sensor_timeout` 不负责发送 CAN safe-stop；
- localization quality 只提供事实和状态；
- vehicle safety/state 根据 wheel、IMU、TF、底盘 fault 和任务 profile 决定是否允许运动；
- wheel timeout 或 local TF 异常时，车辆安全层负责禁止运动；
- IMU timeout 可以由定位层报告降级，但当前安全 profile 仍保持 `allow_motion=false`。

任何输入失效都不得通过发布 `vx=0`、`wz=0` 的伪造观测来掩盖超时。

## 6. Localization bring-up 静态验收标准

### 6.1 Topic

静态验证窗口内应满足：

- `/car/wheel_odom` 频率不低于 40 Hz；
- `/car/imu/data` 达到当前 IMU 配置的预期频率，当前基线约 30 Hz；
- `/odom` 连续稳定输出，目标频率约 50 Hz；
- topic publisher 数量与职责一致，不存在重复 wheel odom 或 `/odom` 发布者。

### 6.2 TF

- `odom -> base_link` 唯一且可连续查询；
- `base_link -> imu_link` 可查询；
- frame ID 与 TF tree 一致；
- 不存在重复 TF broadcaster；
- 当前阶段没有 `map -> odom` 属于正常结果，不得为通过检查而临时启动地图定位节点。

### 6.3 数据有效性

- wheel、IMU、EKF 输出 timestamp 非零且单调；
- 启用变量对应的 covariance 为有限、非负且不能是无依据的全零值；
- 当前 velocity-only 局部 EKF 不融合绝对 pose 和 yaw 观测，`position_covariance` 与 `heading_covariance` 随预测时间增长属于预期现象。两项 covariance 仍必须保持有限、非负，并继续发布到 quality；在该冻结 profile 下不以绝对上限阻断 `/odom`。传感器超时、时间戳、TF 和 covariance 非有限仍属于硬失效条件；
- 静止状态下 `vx` 接近零；
- 静止状态下 `wz` 接近零；
- 静止观察窗口内 pose 不出现快速漂移、跳变或持续异常增长；
- EKF 不报告输入时间、frame、协方差或 TF 配置错误。

当前固定 covariance `0.01` 仅用于开发阶段接口验证，不代表完成 wheel/IMU 标定，也不能据此宣布动态融合验收通过。

### 6.4 安全

静态 EKF 验证必须保持：

```text
allow_motion=false
read_only=true
safe_stop_active=true
tx_count=0
```

EKF 验证过程不得触发任何 CAN 运动控制，不得发送速度、转向或运动目标。CAN 反馈采集和 ROS topic 观察可以继续进行，但不能因启动 EKF 改变底盘安全状态。

## 7. 现场 bring-up 验证命令

以下命令用于现场验收，不改变运行配置：

```bash
# 传感器和 EKF 输出频率
ros2 topic hz /car/wheel_odom
ros2 topic hz /car/imu/data
ros2 topic hz /odom

# 单帧字段、frame_id、时间戳和 covariance
ros2 topic echo /car/wheel_odom --once
ros2 topic echo /car/imu/data --once
ros2 topic echo /odom --once

# publisher 数量和 QoS/类型
ros2 topic info /car/wheel_odom --verbose
ros2 topic info /car/imu/data --verbose
ros2 topic info /odom --verbose

# TF publisher 和 TF tree
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link

# 节点图与重复实例检查
ros2 node list
ros2 node info <local_ekf_node>

# 底盘安全状态，只读检查
ros2 topic echo /car/hardware/status --once
```

时间戳连续性检查使用：

```bash
ros2 topic echo /car/wheel_odom --field header.stamp
ros2 topic echo /car/imu/data --field header.stamp
ros2 topic echo /odom --field header.stamp
```

现场记录应同时保存 topic 频率、单帧内容、TF tree、TF 查询结果、节点列表和底盘安全状态，作为同一轮 bring-up 的完整证据。

## 8. 实现状态与阶段边界

### 已实现

- `/car/wheel_odom` 已由 `car_hardware_adapter` 发布，当前运行频率约 50 Hz；
- `/car/imu/data` 已由主容器 adapter 发布，当前运行频率约 30 Hz；
- 输入选择、frame 约定和 TF 所有权已冻结；
- `car_hardware_adapter` 不发布 `odom -> base_link`；
- 主容器保持 `allow_motion=false`、`read_only=true`。

### 当前验证阶段

- 仅进行 local EKF 的静态 bring-up 验证；
- 只验证预测输出、输入时效性、TF 唯一性、timestamp、covariance 和安全边界；
- 不据此宣称完成动态车辆融合；
- 不启动 AMCL、Nav2 或 global EKF。

### 后续扩展

后续仅在当前静态验收通过、并经过独立评审后，才考虑录包回放或受控运动验证，以及冻结设计中已经列出的输入扩展。任何扩展不得改变本版的 TF 单一发布者规则，也不得在设备容器中增加 EKF、TF 或底盘控制节点。
