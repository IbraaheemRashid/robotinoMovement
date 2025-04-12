#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Super simple launch file
    social_navigation_node = Node(
        package='robotino_hal',
        executable='social_navigation',
        name='social_navigation',
        output='screen'
    )
    
    return LaunchDescription([
        social_navigation_node
    ])