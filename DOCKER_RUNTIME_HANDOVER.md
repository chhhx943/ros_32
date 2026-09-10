# Pi ROS Docker 运行环境交接说明

更新时间：2026-09-08

## 结论

Pi 上只有 `humble-dev` 是主 ROS 容器。

`humble-camera`、`imu-dev` 和 `lidar-dev` 都是设备容器，不是主容器。它们不得运行底盘控制、真实 wheel odom、robot_localization EKF 或主 TF 链路。

镜像名可能都显示为 `humble-dev:camera` 或 `humble-dev:local`，不能用镜像名判断角色，必须以容器名和实际启动命令为准。

## 当前容器角色与状态

| 容器 | 角色 | 当前状态 | 运行边界 |
|---|---|---|---|
| `humble-dev` | 唯一主 ROS 容器 | `Up` | CAN、`car_hardware_adapter`、wheel odom、EKF、主 ROS 应用和主诊断 |
| `humble-camera` | 相机设备容器 | `Up` | 仅 `car_camera_driver/camera_node` |
| `imu-dev` | IMU 设备容器 | `Up` | 仅 `im900_serial_node`，发布 `/vendor/imu` |
| `lidar-dev` | 雷达/激光雷达设备容器 | `Up`，容器已启动 | 仅雷达驱动；不得运行主底盘控制链路 |

### 已核对的设备容器配置

`lidar-dev` 的实际配置为：

- 镜像：`humble-dev:local`
- `network=host`
- `ipc=host`
- 用户：`c`
- 工作目录：`/home/c/robot_ws`
- 工作区挂载：`/home/c/robot_ws:/home/c/robot_ws`
- 设备映射：`/dev/ttyUSB0:/dev/car_lidar`
- 容器命令：`bash`
- 当前状态：容器已启动；默认进程为交互式 `bash`

在 `/home/c/robot_ws/deploy` 中没有找到可直接确认的雷达启动脚本或服务定义，因此当前不能安全推断其启动命令。启动前需要先确认实际雷达型号、驱动包、参数文件和 topic 名称。

注意：`humble-dev` 当前也存在 `/dev/ydlidar`、`/dev/car_lidar`、`/dev/camera_front` 映射。雷达驱动只能选择一个运行时 owner：不能同时在 `humble-dev` 和 `lidar-dev` 打开同一设备或发布同一雷达 topic，避免设备占用冲突和重复 topic。

当前已知设备容器命令：

- `humble-camera`：`car_camera_driver camera_node`
- `imu-dev`：`car_sensor_adapters im900_serial_node`
- `lidar-dev`：仅确认容器为交互式 `bash`，尚未确认雷达驱动启动命令

以下是历史备份容器，不属于当前运行环境，不应启动：

- `humble-dev-ttyusb-backup`
- `humble-dev-unmapped-backup`

## ROS 主链路所有权

主链路必须集中在 `humble-dev`：

```text
STM32 CAN
  -> can0
  -> car_hardware_adapter       [humble-dev]
  -> /car/wheel_odom             [humble-dev]
  -> robot_localization EKF      [humble-dev]
  -> /odom                        [humble-dev]
  -> odom -> base_link TF         [EKF，唯一发布者]
```

设备容器只提供设备数据：

- `humble-camera`：相机 topic
- `imu-dev`：`/vendor/imu`
- `lidar-dev`：未来确认后提供雷达 topic

## IMU DDS 传输配置

Pi 当前使用 ROS 2 Humble 自带的 Fast RTPS 2.6.12。由于 `imu-dev` 与
`humble-dev` 采用 host network，且默认 Fast DDS 会同时考虑 UDP 与共享内存，
IMU 原始 topic 曾出现“ROS graph 可发现、订阅端无样本”的跨容器故障。

当前运行合同是：IMU 发布端和主容器订阅端均加载同一份 UDP-only profile，
不依赖共享内存传输。profile 文件为：

```text
/home/c/robot_ws/fastdds_udp_only.xml
```

Fast RTPS 2.6 使用的环境变量是：

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/home/c/robot_ws/fastdds_udp_only.xml
```

profile 关闭 builtin transports，仅启用 UDPv4，并保留 `127.0.0.1` 与 Pi 的
`192.168.123.32` 地址。`imu-dev` 仍只运行 `im900_serial_node` 并发布
`/vendor/imu`；`imu_adapter_node` 必须在 `humble-dev` 中以相同环境变量运行，
将数据归一化为 `/car/imu/data`。不在 `imu-dev` 中运行 adapter、EKF 或 TF。
`imu-dev` 仅映射 `/dev/ttyAMA0`，并显式丢弃 `CAP_NET_RAW`；CAN raw socket
权限只保留给 `humble-dev` 的主链路。

验证要求：

```bash
docker exec imu-dev printenv FASTRTPS_DEFAULT_PROFILES_FILE
docker exec humble-dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/c/robot_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/c/robot_ws/fastdds_udp_only.xml
  ros2 topic hz /vendor/imu
  ros2 topic hz /car/imu/data
'
```

验收基线为 `/vendor/imu` 与 `/car/imu/data` 均约 30 Hz，消息帧为
`imu_link`，且 `imu-dev` 中不存在 `imu_adapter_node`、`hardware_adapter_node`、
`robot_localization` 或 `amcl` 进程。共享内存不是当前 IMU 跨容器链路的必需路径；
如后续重新启用共享内存，必须先完成等价的双端样本验收。

禁止在 `humble-camera`、`imu-dev` 或 `lidar-dev` 中启动：

- `hardware_adapter_node`
- `localization_system_main`
- 独立 CAN odom 节点
- 第二个 EKF
- 第二个 `odom -> base_link` TF 发布者

## CAN 所有权

`can0` 属于主 ROS 运行链路，正常使用时由 `humble-dev` 中的 `car_hardware_adapter` 访问 SocketCAN。

只读诊断命令：

```bash
docker exec humble-dev ip -details -statistics link show can0
docker exec humble-dev candump -tz -x can0
```

诊断时不要在多个容器中重复启动 adapter，也不要用 `cansend` 替代 ROS 控制链路。

## 主容器环境

`humble-dev` 已确认具备：

- ROS 2 Humble
- `robot_localization`
- `/opt/ros/humble/lib/librl_lib.so`
- `ekf_node`
- `iproute2`
- `can-utils`
- `can0` 访问能力，500 kbps

检查命令：

```bash
docker exec humble-dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/c/robot_ws/install/setup.bash
  ros2 pkg prefix robot_localization
  ros2 pkg executables robot_localization | grep ekf_node
  ip -details link show can0
'
```

## 推荐检查顺序

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
docker inspect -f '{{.State.Status}} exit={{.State.ExitCode}}' lidar-dev
```

主容器恢复：

```bash
docker start humble-dev
docker exec -it humble-dev bash
```

进入主容器后：

```bash
source /opt/ros/humble/setup.bash
source /home/c/robot_ws/install/setup.bash
```

真实底盘验证保持：

```text
backend=socketcan
can_interface=can0
allow_motion=false
```

当前 Pi -> STM32 的 CAN ACK/TX failure 是独立链路问题，不应通过在相机、IMU 或雷达容器中另起 adapter 绕过。

## 启动前检查清单

```bash
# 只能有一个主 adapter
docker exec humble-dev pgrep -af hardware_adapter_node || true
docker exec humble-camera pgrep -af hardware_adapter_node || true
docker exec imu-dev pgrep -af hardware_adapter_node || true

# 主容器确认 CAN 和反馈
docker exec humble-dev candump -n 6 can0

# 主容器确认 ROS 图
docker exec humble-dev bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/c/robot_ws/install/setup.bash
  ros2 node list
  ros2 topic list | grep -E "wheel_odom|/odom|hardware/status|hardware/feedback|tf"
'
```

若 `hardware_adapter_node` 出现在 `humble-camera`、`imu-dev` 或 `lidar-dev`，应先停止重复实例，再继续测试。

## 当前处理结论

本次已启动 `lidar-dev` 容器，但尚未启动雷达驱动进程；没有启动标定，没有发送运动命令，也没有改变 STM32 固件或 Pi 的控制逻辑。下一步若要真正启用雷达，应先确认雷达型号、驱动包、参数文件、输出 topic 以及由 `humble-dev` 还是 `lidar-dev` 独占设备。
