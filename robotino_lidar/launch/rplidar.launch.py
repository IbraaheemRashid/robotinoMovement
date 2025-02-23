#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_path = os.path.expanduser('~/robotino_lidar/config/rplidar_params.yaml')
    
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[params_path],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=['0', '0', '0.05', '0', '0', '3.14159', 'base_link', 'laser']
        )
    ])
