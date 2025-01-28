from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    robotino_hal_dir = get_package_share_directory('robotino_hal')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Load parameters
    nav2_config = os.path.join(robotino_hal_dir, 'config', 'nav2_params.yaml')
    robot_params = os.path.join(robotino_hal_dir, 'config', 'robot_params.yaml')
    urdf_file = os.path.join(robotino_hal_dir, 'description', 'urdf', 'robotino.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

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

    # Nav2 nodes with lifecycle management
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/cmd_vel', '/robotino/cmd_vel'),
            ('/odom', '/robotino/odom')
        ]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config]
    )

    recoveries_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator']}
        ]
    )

    # AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config]
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_config]
    )

    # Lifecycle manager for map and localization
    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Group all nodes
    hal_group = GroupAction([
        robot_state_pub,
        battery_monitor,
        sensor_monitor,
        motor_control,
        odometry_monitor
    ])

    nav2_group = GroupAction([
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager,
        amcl,
        map_server,
        lifecycle_manager_loc
    ])

    return LaunchDescription([
        declare_use_sim_time,
        hal_group,
        nav2_group
    ])