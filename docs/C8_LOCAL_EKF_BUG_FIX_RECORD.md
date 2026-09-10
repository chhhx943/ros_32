# C8 局部 EKF 架构 Bug 处理与推进记录

更新时间：2026-09-09  
项目：R3X Ackermann 移动机器人  
平台：Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Humble / Docker  
代码分支：feature/im900-imu  
局部 EKF 实现基线：b596d74

## 0. 记录范围

本文记录从 C8 车辆状态、传感器与定位融合架构开始，到局部 EKF 实现、职责收敛和静态代码验证完成为止的一个系统性 Bug 及其处理过程。

本文截止于最新局部 EKF 实现，不纳入后续现场主机、电源、USB 设备或掉线问题。后续工作应以本文第 6 节形成的局部 EKF 架构为唯一基线继续推进。

本文不记录动态运动实验结果，也不宣称真实底盘已经完成最终定位验收。

## 1. Bug 定义

### 1.1 Bug 名称

C8 多容器定位链路职责越界与 TF/数据融合所有权不清，导致 wheel odom、IMU、EKF 和后续雷达定位无法形成可审计的唯一数据链。

### 1.2 典型表现

- car_hardware_adapter 的运行容器边界不明确；
- CAN、wheel odom、IMU adapter、EKF 可能被不同容器重复启动；
- wheel odom 可能同时承担原始里程计和 odom TF 发布职责；
- 雷达驱动、local EKF、global EKF、AMCL 的 TF 所有权没有冻结；
- IMU 跨容器 DDS 同时依赖 UDP 和共享内存，可能出现可发现但收不到数据；
- wheel pose 与 twist 的融合边界不清，存在编码器积分结果重复进入 EKF 的风险；
- 传感器超时处理与车辆安全职责混在一起；
- 雷达 frame 在 driver、adapter 和静态 TF 中存在命名不一致风险。

### 1.3 影响

- 可能出现重复节点、重复 publisher 和重复 TF broadcaster；
- EKF 输入的来源、QoS、时间戳和协方差无法确认；
- /odom 可能在传感器异常时被错误放行；
- 后续 AMCL 或 global EKF 接入时可能产生 map -> odom 冲突；
- 无法建立可复现的现场 bring-up 验收流程。

## 2. 根因分析

### 2.1 容器边界未按数据责任划分

设备容器容易被当成完整 ROS 运行容器使用，导致驱动、adapter、底盘控制和定位逻辑同时存在于多个容器。

修正原则：设备容器只拥有设备，主容器拥有 ROS 主链，单个设备只有一个 owner，核心 TF 只有一个 publisher。

### 2.2 TF 所有权没有单一约束

最终冻结的 TF 关系为：

~~~text
map -> odom       后续由 AMCL/地图匹配维护
odom -> base_link local EKF 唯一发布
base_link -> imu_link   静态 TF
base_link -> laser_link 静态 TF
~~~

雷达驱动、硬件 adapter 和第二个 EKF 均不得发布 odom -> base_link。

### 2.3 传感器融合变量没有冻结

wheel odom 同时包含 pose 和 twist。若全部送入 EKF，编码器积分结果可能与 EKF 内部预测重复进入状态估计。

第一阶段冻结为：

~~~text
wheel odom: twist.linear.x, twist.angular.z
IMU:       angular_velocity.z
~~~

不融合 wheel pose、IMU orientation yaw 和线加速度。

### 2.4 跨容器 DDS 合同不明确

IMU driver 位于 imu-dev，IMU adapter 和 EKF 位于 humble-dev。默认 Fast DDS 可能在 UDP 与共享内存之间选择不一致的传输路径。

统一合同为：

~~~text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/home/c/robot_ws/fastdds_udp_only.xml
~~~

发布端和订阅端必须使用相同的 UDP-only profile。

## 3. 调查与定位过程

调查顺序从先调 EKF 改为：

1. 确认容器和进程唯一性；
2. 确认 topic publisher/subscriber 所有权；
3. 确认 TF publisher 所有权；
4. 确认 frame_id、child_frame_id；
5. 确认 timestamp、covariance 和 QoS；
6. 最后检查 EKF 数值和定位质量。

为此增加了 localization system 内部输入链和 endpoint 审计：

~~~text
/car/wheel_odom -> /car/localization/internal/input/wheel_odom
/car/imu/data   -> /car/localization/internal/input/imu
/car/localization/internal/ekf/odometry -> /odom
~~~

输入门控检查 timestamp 非零且单调、frame 正确、数值有限、covariance 有效、静态 TF 存在，并拒绝旧 generation 或过期消息。

## 4. Bug 处理

### 4.1 收敛主容器职责

humble-dev 固定为唯一主 ROS 容器，运行 car_hardware_adapter、car_localization、进程内 EKF、imu_adapter_node、lidar_adapter_node、device monitor 和 localization quality，并负责 can0 的 SocketCAN 访问。

humble-dev 保留 CAP_NET_RAW。

### 4.2 收敛设备容器职责

imu-dev 只运行 im900_serial_node，发布 /vendor/imu，不运行 adapter、EKF、CAN adapter 或主 TF。

lidar-dev 只运行雷达驱动，发布 /scan，不运行 adapter、EKF、AMCL、Nav2 或 odom TF。

humble-camera 只负责相机驱动，不参与底盘、定位和主 TF。

### 4.3 修正 wheel odom 边界

car_hardware_adapter 只负责 CAN 协议解析、编码器反馈、轮速换算和 /car/wheel_odom 发布，不负责 odom -> base_link TF、EKF 或地图定位。

wheel odom frame 冻结为：

~~~text
header.frame_id=odom
child_frame_id=base_link
~~~

### 4.4 实现局部 EKF

当前没有通过 launch 额外启动 standalone ekf_node，而是在 car_localization 进程内创建 robot_localization::RosFilter<robot_localization::Ekf>。

配置冻结为：

~~~text
frequency=50.0
sensor_timeout=0.2
two_d_mode=true
publish_tf=true
predict_to_current_time=true
odom_frame=odom
base_link_frame=base_link
world_frame=odom
~~~

EKF 只负责局部连续状态估计，不负责全局定位和车辆安全动作。

### 4.5 增加 localization facade

facade 负责检查内部 EKF 输出、估计时间、传感器健康状态和静态 TF，维护恢复计数，并在质量满足条件时发布 /odom 和 /car/localization/quality。

当 wheel、IMU、TF 或 EKF 估计无效时，facade 关闭公共 /odom，而不是输出未经验证的状态。

### 4.6 分离定位与安全职责

EKF 遇到 timeout 时停止使用过期观测，facade/device monitor 报告质量下降，vehicle safety 负责禁止运动。不得通过伪造 odom、放宽 timeout 或启动第二条未经验证的链路恢复定位。

### 4.7 统一 frame

统一使用 base_link、imu_link、laser_link、odom、map。雷达 frame 统一为 laser_link。

## 5. 修复后的架构

~~~text
STM32 CAN -> can0 -> car_hardware_adapter -> /car/wheel_odom -> local EKF
IMU driver -> /vendor/imu -> imu_adapter -> /car/imu/data -> local EKF
LiDAR driver -> /scan -> lidar_adapter
~~~

TF 由以下模块分别负责：

- base_link -> imu_link：静态 TF；
- base_link -> laser_link：静态 TF；
- odom -> base_link：local EKF 唯一发布；
- map -> odom：未来 AMCL/地图匹配，当前不启动。

当前阶段不启动 AMCL、Nav2 和 global EKF。

## 6. 代码级验证结果

局部 EKF 和定位 facade 相关验证结果：

~~~text
car_localization C++ test_localization_facade_node: 11/11 passed
C8/D0/D1/D2 Python contract tests: 38 passed
fastdds_udp_only.xml XML parsing: passed
git diff --cached --check: passed
~~~

代码提交基线：

~~~text
b596d74 fix(c8): harden static localization runtime
~~~

以上属于代码、graph contract 和配置验证，不等价于真实动态运动验收。

## 7. 当前结论

本次 Bug 的本质不是单一 EKF 参数错误，而是多容器 ROS 架构的职责、数据入口、TF 所有权和安全边界未冻结。

处理后形成的稳定基线是：

1. humble-dev 是唯一主 ROS 容器；
2. car_hardware_adapter 是唯一 wheel odom 来源；
3. car_localization 是唯一局部 EKF 和 odom TF 管理者；
4. imu-dev、lidar-dev 只提供设备数据；
5. wheel odom 只融合 vx/wz；
6. IMU 只融合 angular_velocity.z；
7. 雷达 frame 统一为 laser_link；
8. /odom 经过 localization facade 质量门控；
9. 第一版不引入 global EKF；
10. AMCL 未来只负责 map -> odom。

## 8. 后续推进基线

后续工作必须从本地 EKF 架构继续，不重新设计输入选择，不新增第二个 EKF。

推进顺序：

1. 验证 wheel odom 的真实 topic、时间戳、covariance 和频率；
2. 验证 IMU 跨容器 DDS、frame 和频率；
3. 验证 local EKF 的 /odom 和 odom -> base_link；
4. 验证 imu_link、laser_link 静态 TF；
5. 验证 lidar driver、/scan 和 laser_link；
6. 局部链路稳定后，单独评审 AMCL 的 map -> odom；
7. 暂不引入 global EKF；
8. 动态运动实验另行审批，并保持安全链路可回退。

