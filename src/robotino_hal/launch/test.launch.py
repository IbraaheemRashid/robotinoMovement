from launch import LaunchDescription
from launch.actions import TimerAction
import os

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    robotino_hal_share = get_package_share_directory('robotino_hal')
    params_path = os.path.join(robotino_hal_share, 'config', 'robot_params.yaml')

    print(f'\nLooking for parameter file at: {params_path}')

    if not os.path.exists(params_path):
        raise FileNotFoundError(f'Parameters file not found at {params_path}')

    with open(params_path, 'r') as f:
        config = yaml.safe_load(f)

    nav_node = Node(
        package='robotino_hal',
        executable='navigation',
        name='simple_navigator',
        parameters=[config],
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']
    )

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

    nav_delayed = TimerAction(
        period=5.0,
        actions=[nav_node]
    )

    print('\nLaunching nodes in sequence: hal_nodes -> navigation')

    return LaunchDescription([
        hal_nodes,
        nav_delayed,
    ])
