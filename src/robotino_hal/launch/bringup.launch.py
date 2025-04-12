#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    robotino_hal_share = get_package_share_directory('robotino_hal')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_file = '/home/mir/map/base.yaml'

    nav2_params_file = os.path.join(robotino_hal_share, 'config', 'nav2_params.yaml')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            #'params_file': nav2_params_file,
            'use_sim_time': 'false',
        }.items()
    )

    return LaunchDescription([
        nav2_bringup_launch,
    ])

