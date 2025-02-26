#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get directories
    robotino_hal_share = get_package_share_directory('robotino_hal')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Config paths
    nav2_params_path = os.path.join(robotino_hal_share, 'config', 'nav2_params.yaml')
    
    # Include the Nav2 launch file with our custom parameters
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'false',
        }.items()
    )
    
    return LaunchDescription([
        nav2_bringup_launch,
    ])