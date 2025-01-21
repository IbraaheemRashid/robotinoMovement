from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    robotino_hal_dir = get_package_share_directory('robotino_hal')
    
    # Load parameters
    nav2_config = os.path.join(robotino_hal_dir, 'config', 'nav2_params.yaml')
    robot_params = os.path.join(robotino_hal_dir, 'config', 'robot_params.yaml')
    
    # Your existing HAL nodes
    battery_monitor = Node(
        package='robotino_hal',
        executable='battery_monitor',
        name='battery_monitor',
        parameters=[robot_params],
        output='screen'
    )
    
    sensor_monitor = Node(
        package='robotino_hal',
        executable='sensor_monitor',
        name='sensor_monitor',
        parameters=[robot_params],
        output='screen'
    )
    
    motor_control = Node(
        package='robotino_hal',
        executable='motor_control',
        name='motor_control',
        parameters=[robot_params],
        output='screen'
    )
    
    odometry_monitor = Node(
        package='robotino_hal',
        executable='odometry_monitor',
        name='odometry_monitor',
        parameters=[robot_params],
        output='screen'
    )
    
    # Nav2 nodes
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config]
    )
    
    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_config],
        output='screen'
    )
    
    nav2_bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config]
    )
    
    # Start HAL nodes
    hal_nodes = [
        battery_monitor,
        sensor_monitor,
        motor_control,
        odometry_monitor
    ]
    
    # Start Nav2 nodes
    nav2_nodes = [
        nav2_controller,
        nav2_planner,
        nav2_behaviors,
        nav2_bt
    ]
    
    return LaunchDescription(hal_nodes + nav2_nodes)