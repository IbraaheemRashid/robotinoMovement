#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import SetParameter
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('robotino_hal').find('robotino_hal')
    urdf_path = os.path.join(pkg_share, 'description', 'robotino.urdf')
    
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Set robot_description in the global namespace
    set_robot_description = SetParameter(
        name='robot_description',
        value=robot_description
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher (now uses global parameter)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        set_robot_description,
        robot_state_publisher,
        joint_state_publisher,
    ])