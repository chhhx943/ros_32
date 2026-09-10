# C8 定位融合 v2 实施计划

> 本计划把已批准的《C8 车辆状态、传感器与定位融合架构设计 v2》落实为 ROS 2 Humble 生产化改造。实施目标工作区为 Raspberry Pi 5 的 `/home/c/robot_ws`；本文档本身位于当前工程仓库。执行期间不得烧录 STM32、不得绕过 `allow_motion=false`，也不得启动第二个 TF 或 CAN 所有者。

## 概览

**Feature:** C8 R3X IMU + wheel odom 局部 EKF 与 LiDAR AMCL 地图定位

**Goal:** 在 Pi 5 Docker 架构中提供唯一、可监测且可降级的 `map -> odom -> base_link` 定位链：local EKF 只融合 wheel `vx/wz` 与 IMU `gyro z`，AMCL 唯一发布 `map -> odom`，所有运行时健康状态进入可验证的运动门控。

**Architecture:** `imu-dev` 只发布 `/vendor/imu`，`lidar-dev` 独占雷达并只发布 `/scan`；`humble-dev` 中的 adapters 归一化为 `/car/imu/data`、`/car/scan`。`car_hardware_adapter` 发布 `/car/wheel_odom`，`car_localization` 内的 `robot_localization` local EKF 是唯一 `odom -> base_link` 发布者，AMCL 是唯一 `map -> odom` 发布者。`car_localization_monitor` 分别计算 local/global 有效性，`car_vehicle_state` 以此执行 profile 化的安全门控。

**Tech Stack:** ROS 2 Humble、C++、`robot_localization`、Nav2 AMCL、launch_ros、Docker host network、Raspberry Pi 5、SocketCAN、GoogleTest/ament、`colcon test`。

**前置条件:** 在 Pi 上建立隔离 worktree 或独立分支；核对 `/home/c/robot_ws` 当前 HEAD 与容器挂载路径；真实 CAN 与运动均保持禁用。不得覆盖当前 Windows 工作区中用户已暂存的文件。

---

## 1. 建立可重复的 C8 生产 profile 与启动拓扑

**Files:**

- Create: `/home/c/robot_ws/src/car_bringup/config/c8_r3x_localization.yaml`
- Create: `/home/c/robot_ws/src/car_bringup/config/c8_r3x_nav2.yaml`
- Create: `/home/c/robot_ws/src/car_bringup/launch/c8_r3x_bringup.launch.py`
- Modify: `/home/c/robot_ws/src/car_bringup/launch/tmini_plus_hardware.launch.py`
- Modify: `/home/c/robot_ws/src/car_bringup/CMakeLists.txt`
- Create: `/home/c/robot_ws/src/car_bringup/test/test_c8_r3x_bringup_launch.py`

### Step 1: 先写启动拓扑测试

在 `test_c8_r3x_bringup_launch.py` 加载 launch 描述并断言生产 profile 的节点责任：只有一个 `hardware_adapter_node`、一个 `localization_system`、一个 `amcl`；`imu_adapter_node` 和 `lidar_adapter_node` 必须在 `humble-dev` 启动；launch 不得启动任何 LiDAR 串口驱动或第二个 EKF。

测试应检查关键参数，而不是只检查节点名字：

```python
assert params["ekf.world_frame"] == "odom"
assert params["amcl.tf_broadcast"] is True
assert params["amcl.scan_topic"] == "/car/scan"
assert "ydlidar_ros2_driver_node" not in action_executables
```

### Step 2: 运行测试确认其在 profile 缺失时失败

```bash
cd /home/c/robot_ws
colcon test --packages-select car_bringup --ctest-args -R c8_r3x_bringup
colcon test-result --verbose
```

预期：测试因生产 launch/profile 尚不存在或职责断言不满足而失败。

### Step 3: 实现生产 launch 与参数文件

`c8_r3x_bringup.launch.py` 仅在 `humble-dev` 中组合 CAN adapter、IMU/LiDAR adapter、静态安装 TF（过渡期）、device monitor、localization、map server、AMCL、localization monitor、vehicle state 与 Nav2。不要复制 `mock_c_system.launch.py` 的 mock backend、mock map 或固定零初始位姿。

`c8_r3x_localization.yaml` 使用真实 topic/frame 契约，并将以下状态显式参数化：

```yaml
hardware_adapter:
  ros__parameters:
    wheel_odom_topic: /car/wheel_odom
    suppress_wheel_odom_publish: false

localization_system:
  ros__parameters:
    wheel_odom_topic: /car/wheel_odom
    imu_topic: /car/imu/data
    expected_imu_frame: imu_link
    odom_frame: odom
    base_link_frame: base_link
    publish_tf: true

lidar_adapter:
  ros__parameters:
    input_topic: /scan
    output_topic: /car/scan
```

静态 LiDAR frame 已在第 7 项完成原子化迁移，当前统一为 `laser_link`；不得在新 profile 中混用 `laser` 与 `laser_link`。

把 `tmini_plus_hardware.launch.py` 限定为传感器调试入口，增加清晰 launch 参数或注释，禁止它与 C8 production launch 并行启动同一 adapter/静态 TF。

### Step 4: 再次运行该包测试

```bash
cd /home/c/robot_ws
colcon build --packages-select car_bringup --symlink-install
source install/setup.bash
colcon test --packages-select car_bringup --ctest-args -R c8_r3x_bringup
colcon test-result --verbose
```

### Step 5: 提交检查点

```bash
git add src/car_bringup
git commit -m "feat: add C8 R3X production bringup profile"
```

---

## 2. 修订 wheel odom 的生产协方差与 CAN 数据有效性

**Files:**

- Modify: `/home/c/robot_ws/src/car_hardware_adapter/include/car_hardware_adapter/hardware_adapter_node.hpp`
- Modify: `/home/c/robot_ws/src/car_hardware_adapter/src/hardware_adapter_node.cpp`
- Modify: `/home/c/robot_ws/src/car_hardware_adapter/config/hardware_adapter.yaml`
- Create: `/home/c/robot_ws/src/car_bringup/config/c8_r3x_hardware_adapter.yaml`
- Modify: `/home/c/robot_ws/src/car_hardware_adapter/test/test_hardware_adapter_node.cpp`

### Step 1: 添加协方差和失效数据的单元测试

覆盖以下 case：有效完整反馈使用独立的 `vx` 与 `wz` covariance；任何不完整快照、CAN feedback timeout、硬件 fault 或非有限协方差均不发布可被 EKF 接受的 wheel odom 样本；wheel odom 节点不发布 `odom -> base_link` TF。

```cpp
EXPECT_DOUBLE_EQ(msg.twist.covariance[0], vx_variance);
EXPECT_DOUBLE_EQ(msg.twist.covariance[35], wz_variance);
EXPECT_GT(msg.pose.covariance[0], msg.twist.covariance[0]);
EXPECT_FALSE(fixture.hasAcceptedWheelOdomAfterTimeout());
EXPECT_FALSE(fixture.tfBufferHas("odom", "base_link"));
```

### Step 2: 运行失败测试

```bash
cd /home/c/robot_ws
colcon test --packages-select car_hardware_adapter --ctest-args -R hardware_adapter_node
colcon test-result --verbose
```

### Step 3: 实现参数化 covariance 与发布门

将当前统一固定 `0.01` 改为独立参数 `wheel_odom_vx_variance`、`wheel_odom_wz_variance`、`wheel_odom_pose_diagnostic_variance`。仅当反馈快照完整、时间单调、CAN 健康且底盘安全状态允许其作为测量时发布；失效状态发布到既有硬件/诊断接口，不能用 `vx=0` 假装观测成立。

初始 C8 profile 采用保守的显式数值，数值来源必须标注为“未标定的上线前默认值”，并保留 bag 标定后覆盖机制：

```yaml
hardware_adapter:
  ros__parameters:
    backend: socketcan
    can_interface: can0
    wheel_odom_vx_variance: 0.04
    wheel_odom_wz_variance: 0.09
    wheel_odom_pose_diagnostic_variance: 1000000.0
    allow_motion: false
```

这三个数值只作为让 EKF 不会把未标定 encoder 当绝对真值的保守上线默认，不作为最终标定结果。若现有 backend 参数名称不同，必须在实现中保持一个明确的 production profile 映射，不能把 mock `backend: mock` 复制进 C8 profile。

### Step 4: 运行测试与静态检查

```bash
cd /home/c/robot_ws
colcon build --packages-select car_hardware_adapter --symlink-install
source install/setup.bash
colcon test --packages-select car_hardware_adapter
colcon test-result --verbose
```

### Step 5: 提交检查点

```bash
git add src/car_hardware_adapter src/car_bringup/config/c8_r3x_hardware_adapter.yaml
git commit -m "feat: gate wheel odom and parameterize covariance"
```

---

## 3. 将 local EKF 改为 wheel `vx/wz` + IMU gyro-z 基线

**Files:**

- Modify: `/home/c/robot_ws/src/car_localization/src/localization_system.cpp`
- Modify: `/home/c/robot_ws/src/car_localization/include/car_localization/localization_system.hpp`
- Modify: `/home/c/robot_ws/src/car_localization/config/localization.yaml`
- Modify: `/home/c/robot_ws/src/car_localization/test/test_localization_system.cpp`
- Modify: `/home/c/robot_ws/src/car_bringup/config/c8_r3x_localization.yaml`

### Step 1: 编写输入向量与 TF 所有权测试

在 `test_localization_system.cpp` 断言 local EKF 默认接受 wheel 的 index 6 (`vx`) 与 11 (`wz`) 而拒绝 wheel x/y/yaw；默认只接受 IMU index 11 (`angular_velocity.z`)。同时断言 facade 只允许 EKF 输出更新 `/odom` 和 `odom -> base_link`。

```cpp
EXPECT_EQ(ekf.odomConfig(), (std::array<bool, 15>{
  false,false,false, false,false,false, true,false,false,
  false,false,true, false,false,false}));
EXPECT_EQ(ekf.imuConfig(), (std::array<bool, 15>{
  false,false,false, false,false,false, false,false,false,
  false,false,true, false,false,false}));
EXPECT_TRUE(fixture.singleTfPublisher("odom", "base_link", "localization_system"));
```

增加 yaw 开关测试：仅当 `enable_imu_orientation_yaw=true` 时打开 index 5 和 `imu0_relative=true`；没有有效 orientation covariance 时 adapter/facade 必须拒绝该 measurement。

### Step 2: 运行失败测试

```bash
cd /home/c/robot_ws
colcon test --packages-select car_localization --ctest-args -R localization_system
colcon test-result --verbose
```

### Step 3: 实现 EKF 参数构造与质量原因码

把当前 wheel x/y/yaw + vx/wz 与 IMU yaw + wz 的硬编码/配置改为构造明确的 two-dimensional vector。维持 `frequency=50.0`、`sensor_timeout=0.20`、`world_frame=odom`、`publish_tf=true`，并新增显式的 orientation gate：

```yaml
ekf:
  ros__parameters:
    odom0_config: [false, false, false, false, false, false, true, false, false, false, false, true, false, false, false]
    imu0_config:  [false, false, false, false, false, false, false, false, false, false, false, true, false, false, false]
    imu0_relative: false
    enable_imu_orientation_yaw: false
```

`enable_imu_orientation_yaw` 不得只修改 YAML 数组；实现必须同时验证 IMU orientation covariance、四元数有限性和 `base_link <- imu_link` TF。失败时保留 gyro-z 路径并在质量 reason 中报告 `imu_orientation_rejected`。

### Step 4: 运行测试、构建并检查参数

```bash
cd /home/c/robot_ws
colcon build --packages-select car_localization car_bringup --symlink-install
source install/setup.bash
colcon test --packages-select car_localization car_bringup
colcon test-result --verbose
ros2 launch car_bringup c8_r3x_bringup.launch.py allow_motion:=false launch_nav2:=false
```

最后一条仅在隔离测试环境中运行，验证节点参数和 TF 图；不得连接实车并执行运动。

### Step 5: 提交检查点

```bash
git add src/car_localization src/car_bringup/config/c8_r3x_localization.yaml
git commit -m "fix: use velocity-only wheel odom in local EKF"
```

---

## 4. 扩展 local EKF 质量契约

**Files:**

- Modify: `/home/c/robot_ws/src/car_interfaces/msg/LocalizationQuality.msg`
- Modify: `/home/c/robot_ws/src/car_localization/src/localization_system.cpp`
- Modify: `/home/c/robot_ws/src/car_localization/include/car_localization/localization_system.hpp`
- Modify: `/home/c/robot_ws/src/car_localization/test/test_localization_system.cpp`
- Modify: `/home/c/robot_ws/src/car_localization_monitor/src/localization_monitor_node.cpp`
- Modify: `/home/c/robot_ws/src/car_vehicle_state/src/vehicle_state_node.cpp`

### Step 1: 先增加消息与发布行为测试

将新字段追加在 `LocalizationQuality.msg` 尾部，保持已有字段顺序和消费者兼容性。测试应覆盖 running、IMU 超时、wheel 超时、TF 超时和 EKF error 五种状态。

```text
float64 wheel_odom_age_sec
float64 imu_age_sec
float64 ekf_output_age_sec
string local_ekf_state
bool odom_base_tf_available
float64 odom_base_tf_age_sec
bool wheel_odom_timeout
bool imu_timeout
bool tf_timeout
```

```cpp
EXPECT_EQ(quality.local_ekf_state, "DEGRADED_IMU");
EXPECT_TRUE(quality.imu_timeout);
EXPECT_FALSE(quality.wheel_odom_timeout);
EXPECT_TRUE(quality.odom_base_tf_available);
```

### Step 2: 生成接口并运行失败测试

```bash
cd /home/c/robot_ws
colcon build --packages-select car_interfaces --symlink-install
source install/setup.bash
colcon test --packages-select car_localization car_localization_monitor car_vehicle_state
colcon test-result --verbose
```

### Step 3: 实现 steady-clock 年龄与状态机输出

在 `car_localization` 保存“最后一个被 EKF 接受”的 wheel/IMU 收到时间（steady clock），不要用 ROS stamp 的差值替代超时事实。输出 10 Hz 的 quality 时：

- `RUNNING`: wheel、IMU、EKF 与 local TF 均正常；
- `DEGRADED_IMU`: wheel、EKF、local TF 正常，IMU 超时；
- `STALE_WHEEL_ODOM`: wheel 超时，quality `valid=false`；
- `ERROR`: EKF 输出无效、TF 所有权/时间异常或不可恢复配置错误。

迁移期让旧 `age_sec` 等于 `ekf_output_age_sec`。covariance 必须来自 EKF 输出，而不是从 AMCL 或输入 odom 覆盖。

### Step 4: 更新全部消息消费者并运行回归测试

```bash
cd /home/c/robot_ws
colcon build --packages-select car_interfaces car_localization car_localization_monitor car_vehicle_state --symlink-install
source install/setup.bash
colcon test --packages-select car_interfaces car_localization car_localization_monitor car_vehicle_state
colcon test-result --verbose
```

### Step 5: 提交检查点

```bash
git add src/car_interfaces src/car_localization src/car_localization_monitor src/car_vehicle_state
git commit -m "feat: expose local EKF input and TF quality"
```

---

## 5. 拆分 local/global 定位状态并修正 AMCL 新鲜度判断

**Files:**

- Modify: `/home/c/robot_ws/src/car_interfaces/msg/NavLocalizationState.msg`
- Modify: `/home/c/robot_ws/src/car_localization_monitor/include/car_localization_monitor/localization_monitor_runtime_policy.hpp`
- Modify: `/home/c/robot_ws/src/car_localization_monitor/src/localization_monitor_node.cpp`
- Modify: `/home/c/robot_ws/src/car_localization_monitor/config/localization_monitor.yaml`
- Modify: `/home/c/robot_ws/src/car_localization_monitor/test/test_localization_monitor_runtime_policy.cpp`
- Modify: `/home/c/robot_ws/src/car_localization_monitor/test/test_localization_monitor_node.cpp`

### Step 1: 写入 local/global 矩阵测试

测试矩阵必须至少覆盖：local EKF 失效导致 global 失效；scan/AMCL/map-TF 失效只导致 global 失效；静止状态下 old AMCL pose + fresh map-TF 仍 global valid；运动状态下 old AMCL pose 必须 global invalid；小于 transform tolerance 的 future map-TF 可接受；大于 tolerance 的 future map-TF 必须拒绝。

```cpp
EXPECT_TRUE(decide({.local_valid = true, .scan_valid = true,
                    .robot_moving = false, .amcl_pose_age_sec = 3.0,
                    .map_odom_age_sec = 0.1}).global_valid);
EXPECT_FALSE(decide({.local_valid = true, .scan_valid = true,
                     .robot_moving = true, .amcl_pose_age_sec = 0.6,
                     .map_odom_age_sec = 0.1}).global_valid);
EXPECT_FALSE(decideFutureTfAge(-0.30, /* tolerance */ 0.20).valid);
```

### Step 2: 运行失败测试

```bash
cd /home/c/robot_ws
colcon test --packages-select car_localization_monitor --ctest-args -R localization_monitor
colcon test-result --verbose
```

### Step 3: 实现独立 freshness 输入和 signed TF age

消息尾部追加 `float64 map_odom_age_sec`。把误导性的 `max_amcl_age_sec` 拆为：

```yaml
localization_monitor:
  ros__parameters:
    max_amcl_pose_age_sec: 0.50
    max_map_odom_tf_age_sec: 0.25
    max_future_map_odom_tf_sec: 0.20
    moving_linear_speed_mps: 0.02
    moving_angular_speed_radps: 0.05
```

从 `/odom` twist 或已有运动状态计算 `robot_moving`。local validity 只能来源于 `/car/localization/quality`；global validity 按下式独立计算：

```text
global_valid = local_valid && scan_valid && amcl_active && covariance_ok
             && map_identity_ok && map_odom_tf_ok
             && (!robot_moving || amcl_pose_fresh)
```

TF 年龄使用带符号差值。仅允许未来 `map -> odom` stamp 在 `max_future_map_odom_tf_sec` 内；超过该窗口要输出明确 reason，不能钳成零。

### Step 4: 回归测试

```bash
cd /home/c/robot_ws
colcon build --packages-select car_interfaces car_localization_monitor --symlink-install
source install/setup.bash
colcon test --packages-select car_localization_monitor
colcon test-result --verbose
```

### Step 5: 提交检查点

```bash
git add src/car_interfaces src/car_localization_monitor
git commit -m "fix: separate local and global localization validity"
```

---

## 6. 将 IMU 降级、wheel 硬失效和全局导航需求固化到车辆状态

**Files:**

- Modify: `/home/c/robot_ws/src/car_vehicle_state/include/car_vehicle_state/vehicle_state_policy.hpp`
- Modify: `/home/c/robot_ws/src/car_vehicle_state/src/vehicle_state_node.cpp`
- Modify: `/home/c/robot_ws/src/car_vehicle_state/config/vehicle_state.yaml`
- Modify: `/home/c/robot_ws/src/car_vehicle_state/test/test_vehicle_state_policy.cpp`
- Modify: `/home/c/robot_ws/src/car_vehicle_state/test/test_vehicle_state_node.cpp`
- Modify: `/home/c/robot_ws/src/car_nav2_adapter/src/nav2_adapter_node.cpp`
- Modify: `/home/c/robot_ws/src/car_nav2_adapter/test/test_nav2_adapter_node.cpp`

### Step 1: 先写状态和命令门测试

验证 IMU 超时 + wheel 正常会得到 `DEGRADED_IMU` 但第一版 `motion_allowed=false`；wheel 超时或 local TF 异常必为 `SAFE_STOP`；scan 或 global localization 异常在地图导航 profile 中取消/拒绝 Nav2 goal，但不伪造 local invalid。

```cpp
EXPECT_EQ(state.mode, VehicleMode::kDegraded);
EXPECT_FALSE(state.motion_allowed);  // allow_imu_degraded_motion=false
EXPECT_EQ(state.reason, "imu_timeout");

EXPECT_EQ(wheelTimeout.mode, VehicleMode::kSafeStop);
EXPECT_TRUE(nav2Adapter.goalCancelledAfterGlobalInvalid());
```

### Step 2: 运行失败测试

```bash
cd /home/c/robot_ws
colcon test --packages-select car_vehicle_state car_nav2_adapter --ctest-args -R 'vehicle_state|nav2_adapter'
colcon test-result --verbose
```

### Step 3: 实现 profile 化策略

在 `vehicle_state.yaml` 增加并使用：

```yaml
vehicle_state:
  ros__parameters:
    require_global_localization_for_motion: true
    allow_imu_degraded_motion: false
    degraded_max_linear_speed_mps: 0.0
    degraded_max_angular_speed_radps: 0.0
```

优先级固定为：TF 拓扑/本地 TF 异常、wheel/CAN/底盘 fault、focus/estop 异常均立即 `SAFE_STOP`；仅 IMU 超时时为 `DEGRADED_IMU`；scan/AMCL/map-TF 失效在 `require_global_localization_for_motion=true` 的 profile 下关闭运动并取消 Nav2。恢复必须要求 3 个健康传感器样本、5 个 valid local quality 样本，并重新获得任务 authority；不得恢复旧速度命令或旧 Nav2 goal。

### Step 4: 测试与完整依赖包回归

```bash
cd /home/c/robot_ws
colcon build --packages-select car_vehicle_state car_nav2_adapter --symlink-install
source install/setup.bash
colcon test --packages-select car_vehicle_state car_nav2_adapter
colcon test-result --verbose
```

### Step 5: 提交检查点

```bash
git add src/car_vehicle_state src/car_nav2_adapter
git commit -m "feat: gate navigation on C8 localization health"
```

---

## 7. 完成 LiDAR 单设备所有权与 `laser_link` 原子迁移

**Files:**

- Modify: `/home/c/robot_ws/src/car_bringup/launch/tmini_plus_hardware.launch.py`
- Modify: `/home/c/robot_ws/src/car_bringup/launch/c8_r3x_bringup.launch.py`
- Modify: `/home/c/robot_ws/src/car_bringup/config/c8_r3x_localization.yaml`
- Modify: `/home/c/robot_ws/src/car_sensor_adapters/src/lidar_adapter_node.cpp`
- Modify: `/home/c/robot_ws/src/car_sensor_adapters/config/sensor_adapters.yaml`
- Modify: `/home/c/robot_ws/src/car_sensor_adapters/test/test_lidar_adapter_node.cpp`
- Modify: `/home/c/robot_ws/src/car_bringup/test/test_c8_r3x_bringup_launch.py`
- Create: `/home/c/robot_ws/docs/c8-lidar-container-contract.md`

### Step 1: 先写 frame 与设备所有权测试

测试要求 driver 参数使用 `/dev/car_lidar`，只在 `lidar-dev` 启动 LiDAR driver；`humble-dev` 仅有 `lidar_adapter_node`。adapter 必须拒绝非 `laser_link` 的 scan，生产 TF 图中只有 `base_link -> laser_link`，不出现 `base_link -> laser` 别名。

```cpp
EXPECT_EQ(adapter.expectedFrame(), "laser_link");
EXPECT_TRUE(adapter.rejectsScanFrame("laser"));
EXPECT_TRUE(tfGraph.hasSingleEdge("base_link", "laser_link"));
EXPECT_FALSE(tfGraph.hasFrame("laser"));
```

### Step 2: 运行失败测试

```bash
cd /home/c/robot_ws
colcon test --packages-select car_sensor_adapters car_bringup --ctest-args -R 'lidar_adapter|c8_r3x_bringup'
colcon test-result --verbose
```

### Step 3: 实施一次性 frame 迁移

在同一个变更中同步修改 LiDAR driver `frame_id`、adapter expected/output frame、device monitor、AMCL 参数、静态 TF 和测试 fixture 为 `laser_link`。在 `c8-lidar-container-contract.md` 固化容器运行合同：稳定 udev 设备 `/dev/car_lidar` 只能映射给 `lidar-dev`，`humble-dev` 不能保留 LiDAR serial device mapping，也不能启动 `ydlidar_ros2_driver_node`。实际 Docker Compose/systemd 清单由部署仓库维护，本计划不在未核对的仓库中臆造或修改它。

安装外参必须来自实测记录；若尚未标定，启动参数用 `publish_static_laser_tf:=false` 使生产导航无法激活，不能用零外参伪装已标定。标定值进入被版本控制的 YAML/URDF 后才允许该 TF 发布。

### Step 4: 运行单元与 launch 测试

```bash
cd /home/c/robot_ws
colcon build --packages-select car_sensor_adapters car_bringup --symlink-install
source install/setup.bash
colcon test --packages-select car_sensor_adapters car_bringup
colcon test-result --verbose
```

### Step 5: 提交检查点

```bash
git add src/car_sensor_adapters src/car_bringup docs/c8-lidar-container-contract.md
git commit -m "feat: isolate lidar device and normalize laser frame"
```

---

## 8. 配置 AMCL/地图与端到端只读验收

**Files:**

- Modify: `/home/c/robot_ws/src/car_bringup/config/c8_r3x_nav2.yaml`
- Modify: `/home/c/robot_ws/src/car_bringup/launch/c8_r3x_bringup.launch.py`
- Create: `/home/c/robot_ws/src/car_bringup/test/test_c8_amcl_contract.py`
- Create: `/home/c/robot_ws/src/car_bringup/scripts/c8_localization_preflight.sh`
- Create: `/home/c/robot_ws/docs/c8-localization-acceptance.md`

### Step 1: 先写 AMCL 参数与 TF 合同测试

测试验证 `global_frame_id=map`、`odom_frame_id=odom`、`base_frame_id=base_link`、`scan_topic=/car/scan`、`tf_broadcast=true`、`transform_tolerance=0.20`，并检查生产 profile 不引用 mock map/零 initial pose。检查启动描述中无 global EKF。

```python
assert amcl["global_frame_id"] == "map"
assert amcl["odom_frame_id"] == "odom"
assert amcl["base_frame_id"] == "base_link"
assert amcl["tf_broadcast"] is True
assert amcl["transform_tolerance"] == 0.20
assert "mock" not in map_yaml.read_text().lower()
assert "ekf_global" not in launched_nodes
```

### Step 2: 运行失败测试

```bash
cd /home/c/robot_ws
colcon test --packages-select car_bringup --ctest-args -R c8_amcl_contract
colcon test-result --verbose
```

### Step 3: 实现真实地图引用、初始位姿入口和 preflight

`c8_r3x_nav2.yaml` 不写入未经验证的 AMCL alpha 值；将 `alpha*`、update thresholds、地图路径、`map_id`、`map_hash` 和 initial pose 入口显式声明为 deployment 参数。preflight 脚本只读检查：一个 `odom -> base_link` 发布者、一个 `map -> odom` 发布者、没有 TF 环、wheel/IMU/scan ages 在门限内、CAN 无 bus-off、地图 hash 与任务一致、vehicle state 不允许 motion。

```bash
#!/usr/bin/env bash
set -euo pipefail
ros2 topic info /car/wheel_odom --verbose
ros2 topic info /car/imu/data --verbose
ros2 topic info /car/scan --verbose
ros2 run tf2_tools view_frames
ros2 topic echo /car/localization/quality --once
ros2 topic echo /car/navigation/localization_state --once
ros2 topic echo /car/vehicle/state --once
```

验收文档明确将实车试验分为：静止数据采集、手动受控低速直线/左右圆弧/S 弯、AMCL 重定位、scan/IMU/wheel/TF 故障注入、容器单独重启、长时间 Nav2。每一阶段均要求保存 rosbag2 与通过/失败阈值；在 `allow_motion=false` 阶段不得发出任何运动命令。

### Step 4: 全量构建、测试与只读预检

```bash
cd /home/c/robot_ws
colcon build --packages-up-to car_bringup car_vehicle_state car_nav2_adapter --symlink-install
source install/setup.bash
colcon test --packages-select car_bringup car_localization car_localization_monitor car_vehicle_state car_nav2_adapter car_sensor_adapters car_hardware_adapter
colcon test-result --verbose
bash src/car_bringup/scripts/c8_localization_preflight.sh
```

最后一条只能在已启动但 `allow_motion=false` 的受控 Pi 环境执行；如果 LiDAR 外参、真实地图或 CAN 健康尚未达到 preflight 条件，应以失败结果停止在该阶段，不能为了通过检查关闭质量门。

### Step 5: 提交检查点

```bash
git add src/car_bringup docs/c8-localization-acceptance.md
git commit -m "test: add C8 AMCL contract and localization preflight"
```

---

## 全程验证与交付清单

1. 每个 task 先运行对应失败测试，再实现最小修改并运行该包回归；不得跳过红绿循环。
2. 每次修改 `car_interfaces` 消息后，必须重建所有直接消费者：`car_localization`、`car_localization_monitor`、`car_vehicle_state`、`car_nav2_adapter`。
3. 每次启动测试检查 TF 图：`base_link` 只有一个父节点；仅 local EKF 发布 `odom -> base_link`；仅 AMCL 发布 `map -> odom`；设备容器不发布主 TF。
4. 合入前执行 `git diff --check`、全量指定包测试、`colcon test-result --verbose`，并以 `git status --short` 审查改动范围。不要操作 Windows 仓库现有用户暂存内容。
5. 实车验收前必须解释 `can0` 的 RX error/drop 计数增长；如果仍有 bus-off、wheel timeout、TF 冲突或地图身份不匹配，保持 `SAFE_STOP`，不进入运动测试。
6. 第一版禁止引入 global EKF。若未来评审 global EKF，必须先把 AMCL 设为 `tf_broadcast=false`，并重新验证 TF、协方差和故障恢复。

## 完成定义

- C8 production launch 在 `humble-dev` 中只形成一个 local EKF 和一个 AMCL TF 权威发布者。
- `/car/localization/quality` 可准确报告 wheel/IMU/EKF/TF 年龄与 timeout，`/car/navigation/localization_state` 可独立表达 local/global 失效。
- IMU、wheel、scan、AMCL、map-TF 和 TF 拓扑故障均有自动化测试和明确安全动作。
- `lidar-dev` 是 LiDAR USB 唯一运行时 owner；`imu-dev` 是 IMU 串口唯一 owner；`humble-dev` 不抢占设备驱动。
- 全部指定包测试通过，preflight 在无运动许可的运行态通过，且保留可追溯的 rosbag2/验收记录后，才可单独申请受控实车运动测试授权。
