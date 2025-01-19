from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    prefix_path = get_package_prefix('robotino_hal')
    mock_server_path = os.path.join(prefix_path, '..', '..', 'src', 'robotino_hal', 'test', 'mock_robotino_server.py')
    
    if not os.path.exists(mock_server_path):
        raise FileNotFoundError(f"Mock server not found at {mock_server_path}")

    mock_server = ExecuteProcess(
        cmd=['python3', mock_server_path],
        name='mock_server',
        output='screen'
    )

    common_params = {
        'robotino_ip': 'localhost',  # Remove port from IP
        'robotino_port': 8080,       # Add separate port parameter
        'timeout': 1.0,
    }

    battery_monitor = Node(
        package='robotino_hal',
        executable='battery_monitor',
        name='battery_monitor',
        parameters=[{
            **common_params,
            'update_rate': 1.0,
        }],
        output='screen'
    )

    sensor_monitor = Node(
        package='robotino_hal',
        executable='sensor_monitor',
        name='sensor_monitor',
        parameters=[{
            **common_params,
            'update_rate': 10.0,
            'num_sensors': 9
        }],
        output='screen'
    )

    motor_control = Node(
        package='robotino_hal',
        executable='motor_control',
        name='motor_control',
        parameters=[{
            **common_params,
            'max_linear_speed': 0.8,
            'max_angular_speed': 1.5,
            'control_rate': 50.0
        }],
        output='screen'
    )

    odometry_monitor = Node(
        package='robotino_hal',
        executable='odometry_monitor',
        name='odometry_monitor',
        parameters=[{
            **common_params,
            'update_rate': 20.0
        }],
        output='screen'
    )

    delayed_nodes = TimerAction(
        period=3.0,
        actions=[
            battery_monitor,
            sensor_monitor,
            motor_control,
            odometry_monitor
        ]
    )

    return LaunchDescription([
        mock_server,
        delayed_nodes
    ])