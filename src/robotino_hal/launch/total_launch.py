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
    rplidar_params_path = os.path.expanduser('/home/robotino/robotinoMovement/robotino_lidar/config/rplidar_params.yaml')

    # Load parameters
    with open(params_path, 'r') as f:
        config = yaml.safe_load(f)

    # Create the core HAL nodes
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
        parameters=[rplidar_params_path],
        output='screen'
    )

    # Add a rotated base frame (180 degrees around Z axis)
    rotated_base_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rotated_base_publisher',
        arguments=['0', '0', '0', '0', '0', '3.14159', 'base_link', 'rotated_base_link']
    )

    # Update the lidar transform to connect to the rotated frame
    lidar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'rotated_base_link', 'laser_frame']
    )

    # Group HAL nodes to start first
    hal_nodes = TimerAction(
        period=3.0,
        actions=[
            battery_monitor,
            sensor_monitor,
            motor_control,
            odometry_monitor,
            camera_monitor,
        ]
    )

    # Group sensor nodes to start next
    sensor_nodes = TimerAction(
        period=5.0,
        actions=[
            rplidar_node,
            rotated_base_transform,  # Add the rotated base transform
            lidar_transform,
        ]
    )

    return LaunchDescription([
        hal_nodes,
        sensor_nodes,
    ])