import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("car_bringup")
    config_dir = os.path.join(bringup_dir, "config")

    # Container boundary: IMU and YDLIDAR drivers run in imu-dev/lidar-dev.
    # humble-dev receives only their ROS topics and owns the CAN adapter,
    # localization, monitoring, and the local TF authority.
    return LaunchDescription([
        Node(
            package="car_hardware_adapter",
            executable="hardware_adapter_node",
            namespace="car",
            name="hardware_adapter",
            parameters=[os.path.join(config_dir, "hardware_adapter.yaml")],
            output="screen",
        ),
        Node(
            package="car_sensor_adapters",
            executable="imu_adapter_node",
            namespace="car",
            name="imu_adapter",
            parameters=[os.path.join(config_dir, "sensors.yaml")],
            output="screen",
        ),
        Node(
            package="car_sensor_adapters",
            executable="lidar_adapter_node",
            namespace="car",
            name="lidar_adapter",
            parameters=[os.path.join(config_dir, "sensors.yaml")],
            output="screen",
        ),
        Node(
            package="car_device_monitor",
            executable="device_monitor_node_main",
            namespace="car",
            name="device_monitor",
            parameters=[os.path.join(config_dir, "sensors.yaml")],
            output="screen",
        ),
        Node(
            package="car_localization",
            executable="localization_system_main",
            namespace="car",
            parameters=[os.path.join(config_dir, "localization.yaml")],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="c8_base_to_imu_tf",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
                "--frame-id", "base_link", "--child-frame-id", "imu_link",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="c8_base_to_laser_link_tf",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.02",
                "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
                "--frame-id", "base_link", "--child-frame-id", "laser_link",
            ],
            output="screen",
        ),
    ])
