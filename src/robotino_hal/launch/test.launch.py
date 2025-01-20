from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():
    # Get paths
    robotino_hal_share = get_package_share_directory('robotino_hal')
    params_path = os.path.join(robotino_hal_share, 'config', 'robot_params.yaml')
    
    # Debug prints
    print(f"\nLooking for parameter file at: {params_path}")
    
    if not os.path.exists(params_path):
        raise FileNotFoundError(f"Parameters file not found at {params_path}")
    
    # Load parameters
    with open(params_path, 'r') as f:
        config = yaml.safe_load(f)
        print(f"\nLoaded parameters: {config}")
    
    # Create the nodes
    nav_node = Node(
        package='robotino_hal',
        executable='navigation',
        name='simple_navigator',  # Changed name to match class name
        parameters=[config],
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']
    )
    
    # Other nodes remain the same...
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
    
    # Start nodes with delays
    hal_nodes = TimerAction(
        period=3.0,
        actions=[
            battery_monitor,
            sensor_monitor,
            motor_control,
            odometry_monitor
        ]
    )
    
    nav_delayed = TimerAction(
        period=5.0,
        actions=[nav_node]
    )
    
    # Create mock server config
    mock_server = ExecuteProcess(
        cmd=['python3', os.path.join(get_package_prefix('robotino_hal'), 
             '..', '..', 'src', 'robotino_hal', 'test', 'mock_robotino_server.py')],
        name='mock_server',
        output='screen'
    )
    
    print("\nLaunching nodes in sequence: mock_server -> hal_nodes -> navigation")
    
    return LaunchDescription([
        mock_server,
        hal_nodes,
        nav_delayed
    ])