# STM32 CAN 轮式里程计接入设计

日期：2026-09-07

## 1. 目标与范围

在 Pi 的 ROS 2 Docker 工作区中复用现有 `car_hardware_adapter`，把 STM32F407 通过 SocketCAN 提供的左右后轮编码器反馈转换为真实米制距离，并发布 `wheel_odom` 以及 `odom -> base_link`。

本设计只定义反馈解码、单位换算、里程计积分、同步和安全门控，不改变 STM32 的电机控制、安全状态或标定门控，也不因为 odom 接入而绕过 `CALIBRATION_REQUIRED`、编码器检查、故障检查或 `allow_motion`。

交接文档中的实机结论和安全约束作为本设计输入；交接文档的后续阶段计划不视为本次自动执行的烧录或运动授权。

## 2. 已冻结的系统边界

```text
STM32F407 CAN1
    0x181 设备时间/健康状态
    0x182 左右轮速度，mm/s
    0x183 左轮累计角位置，mrad
    0x184 右轮累计角位置，mrad
             |
             v
Pi 主机 can0 / MCP2515
             |
             v
ROS Docker: car_hardware_adapter
    protocol codec -> feedback snapshot -> wheel odom
             |
             +--> /wheel_odom
             +--> nav_msgs/msg/Odometry
             +--> TF: odom -> base_link
```

不新增独立 CAN odom 节点，不新增 CAN ID。原因是现有 `car_hardware_adapter` 已经拥有 SocketCAN、协议编解码、反馈状态和 `wheel_odom` 接口。

## 3. 单位和距离换算

### 3.1 编码器定义

STM32 当前编码器参数为：

```text
encoder PPR       = 500
quadrature factor = 4
gear ratio        = 28:1
counts/rear-wheel-revolution = 500 * 4 * 28 = 56000
```

`0x183/0x184` 并不传原始 count，而是传累计轮子角位置 `position_mrad`。因此 Pi 端必须先做角度到距离的换算，odom 内部和 ROS 消息统一使用 SI 单位：米、米/秒、弧度、弧度/秒。

### 3.2 半径到真实距离

保留轮半径作为距离换算参数。对每个反馈周期：

```text
delta_theta_rad = delta_position_mrad / 1000
delta_distance_m = wheel_radius_m * delta_theta_rad
```

等价地，从原始 count 推导时：

```text
delta_distance_m =
    delta_counts / 56000 * (2 * pi * wheel_radius_m)
```

当前 STM32 默认半径为 `33.25 mm`，对应一整圈约 `0.208915 m`。这个值只是当前工程参数，不代表已经完成负载下的实测标定；Pi 和 STM32 必须使用同一套最终半径参数，避免 `0x182` 速度与位置里程不一致。

V1 使用一个与 STM32 一致的有效轮半径参数：

```yaml
wheel_radius_m: 0.03325
```

如果后续测得左右轮有效半径不同，再扩展为左右独立参数；在此之前不得在 Pi 端私自使用另一套半径。

## 4. 反馈同步与时间

STM32 每 20 ms 冻结一个反馈快照。Pi 必须按 `feedback_seq` 聚合 `0x181/0x182/0x183/0x184/0x186`，禁止把不同序号的左右轮位置拼成同一个快照。

每个快照记录两种时间：

- `device_time_ms`：用于计算相邻有效快照的运动时间 `dt`；
- Linux monotonic receive time：用于反馈超时、心跳超时和诊断。

当设备时间缺失、倒退或间隔不合理时，当前快照不得积分；可使用受限的主机单调时钟作为降级诊断时间，但不能掩盖反馈时间异常。

序号规则：

- 丢失、重复或倒序快照不得重复积分；
- 模块化序号比较遵循 CAN 协议定义；
- 主机反馈超过 100 ms 未更新时进入安全状态；
- 心跳超过 200 ms 未推进时进入安全状态。

## 5. 差速后轴里程计

先把左右轮位置差换算为实际距离：

```text
d_left  = sign_left  * delta_left_position_mrad  / 1000 * wheel_radius_m
d_right = sign_right * delta_right_position_mrad / 1000 * wheel_radius_m
```

其中 `sign_left` 和 `sign_right` 只用于纠正编码器物理安装方向，默认值不能未经实机前进测试就写死。验收时要求车辆前进时左右 `d_left`、`d_right` 均为正，后退时均为负。

后轮轮距使用交接文档的当前参数：

```text
track_m = 0.120
```

差速积分：

```text
d_center = (d_left + d_right) / 2
d_yaw    = (d_right - d_left) / track_m
```

使用中点航向积分：

```text
theta_mid = theta + d_yaw / 2
x += d_center * cos(theta_mid)
y += d_center * sin(theta_mid)
theta += d_yaw
```

角度归一化到 `[-pi, pi)`，四元数由最终 yaw 生成。

## 6. 速度与位置的职责

- `0x183/0x184`：主要用于真实距离和 pose 积分；
- `0x182`：用于发布/校验轮速 twist、有效性和位置差分一致性；
- `0x188`：仅为控制输出诊断，不作为车速或里程；
- `0x180/0x181/0x186`：用于安全状态、故障、反馈新鲜度和 odom 有效性。

当左右位置均有效时，pose 使用位置差积分。任一位置无效时不得用无效位置推进 pose；仍可在 `0x182` 有效时发布 best-effort 轮速/底盘速度，但必须标记 odom 数据不完整。该逻辑不能解除 STM32 的运动安全门。

## 7. ROS 接口

发布：

```text
/wheel_odom                 现有底盘反馈接口
/odom                       nav_msgs/msg/Odometry（若现有架构需要）
TF: odom -> base_link       由 wheel odom 统一发布
```

消息字段：

- `header.stamp` 使用对应反馈快照的主机接收时间；
- `header.frame_id = odom`；
- `child_frame_id = base_link`；
- pose 使用米和四元数；
- twist 使用 `0x182` 或由位置差计算的一致速度；
- 反馈不完整、超时、故障或编码器无效时，通过状态/诊断和协方差明确标记，不伪造正常运动数据。

初始真实 CAN 运行参数：

```text
backend          = socketcan
can_interface    = can0
allow_motion     = false
wheel_radius_m   = 0.03325    # 待最终实测确认
track_m          = 0.120
```

容器必须能访问主机的 `can0`；应在 Pi 上确认 Docker 的网络模式或 SocketCAN 暴露方式后再启动真实后端。ROS 运行时统一使用 `rmw_fastrtps_cpp`。

## 8. 安全约束

里程计是只读反馈路径，不得因为收到有效编码器帧而自动进入 DRIVE。

以下任一条件成立时：

- `CALIBRATION_REQUIRED`；
- `drive_allowed=0`；
- STM32 fault 或 fault_latched；
- CAN error frame、Bus-Off、Error Passive 或 TX 失败；
- 反馈/心跳超时；
- 左右编码器无效；
- 序号重复、倒序或反馈快照不完整；

只允许保持安全停止/诊断状态，不得发送速度命令或恢复运动权限。初次 ROS 实机验证始终使用 `allow_motion=false`。

## 9. 测试与验收

### 9.1 单元测试

覆盖：

1. `56000 count/rev` 和 `mrad -> m` 换算；
2. 半径 `0.03325 m` 时一整圈距离约 `0.208915 m`；
3. 正反向符号修正；
4. 左右位置按相同 `feedback_seq` 配对；
5. 重复、倒序和丢帧不重复积分；
6. 直行、原地旋转、圆弧运动积分；
7. 反馈超时、CAN 错误和编码器无效时安全降级；
8. `allow_motion=false` 下不会发送速度命令。

### 9.2 Pi 容器只读实机验证

在 `humble-dev` 内以 `socketcan/can0/allow_motion=false` 启动，确认：

```text
0x180..0x186 持续接收
/hardware/status 正常
/hardware/feedback 正常
/wheel_odom 单调更新
odom -> base_link TF 存在
```

此阶段不启动标定，不发送前进、后退、转向命令，不烧录 STM32。

### 9.3 运动验收前置条件

只有 STM32 标定成功、`fault_code=0`、`drive_allowed=1`、编码器有效且 CAN 无新错误后，才允许进行抬轮低速测试。每项动作之间发送完整全零 STOP，并记录原始 CAN 帧和 odom 输出。

验收重点：

- 一整圈换算距离与 `2*pi*r` 一致；
- 前进时 x 正向增长，后退时 x 负向增长；
- 左右轮差分产生正确 yaw 符号；
- odom 距离单位为米，不能出现以 count 或 mrad 为单位的输出；
- 任意安全条件异常都不能自动恢复运动。

## 10. 未决事项

1. 实机确认左右编码器方向符号；
2. 确认 Pi Docker 内 `can0` 的可见方式；
3. 最终确认 STM32 与 Pi 使用的有效轮半径数值；
4. 确认现有 `wheel_odom` 是否已经包含 `nav_msgs/Odometry` 和 TF，避免重复发布。

