#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get directories
    robotino_hal_share = get_package_share_directory('robotino_hal')
    params_path = os.path.join(robotino_hal_share, 'config', 'robot_params.yaml')

    # Load parameters
    with open(params_path, 'r') as f:
        config = yaml.safe_load(f)

    # Core HAL nodes
    battery_monitor = Node(
        package='robotino_hal',
        executable='battery_monitor',
        name='battery_monitor',
        parameters=[config],
        output='screen'
    )

    sensor_monitor = Node(
        package='robotino_hal',
        executable='sensor_monitor',
        name='sensor_monitor',
        parameters=[config],
        output='screen'
    )

    motor_control = Node(
        package='robotino_hal',
        executable='motor_control',
        name='motor_control',
        parameters=[config],
        output='screen'
    )

    odometry_monitor = Node(
        package='robotino_hal',
        executable='odometry_monitor',
        name='odometry_monitor',
        parameters=[config],
        output='screen'
    )

    camera_monitor = Node(
        package='robotino_hal',
        executable='camera_monitor',
        name='camera_monitor',
        parameters=[config],
        output='screen'
    )

    # RPLidar nodes
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )

    # Static transform for LiDAR - this needs to start first
    lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Group static transforms to start first
    transform_nodes = TimerAction(
        period=1.0,
        actions=[lidar_transform]
    )

    # Group HAL nodes to start next
    hal_nodes = TimerAction(
        period=2.0,
        actions=[
            battery_monitor,
            sensor_monitor,
            motor_control,
            odometry_monitor,
            camera_monitor,
        ]
    )

    # Group sensor nodes to start last
    sensor_nodes = TimerAction(
        period=3.0,
        actions=[rplidar_node]
    )

    return LaunchDescription([
        transform_nodes,  # Start transforms first
        hal_nodes,       # Then HAL nodes
        sensor_nodes,    # Then sensors
    ])