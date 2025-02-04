from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the launch directory
    robotino_hal_dir = FindPackageShare('robotino_hal')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robotino_hal_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robotino_hal_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup'
    )
    
    # Include your existing HAL nodes launch
    hal_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robotino_hal',
                executable='battery_monitor',
                name='battery_monitor',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='robotino_hal',
                executable='sensor_monitor',
                name='sensor_monitor',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='robotino_hal',
                executable='motor_control',
                name='motor_control',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='robotino_hal',
                executable='odometry_monitor',
                name='odometry_monitor',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='robotino_hal',
                executable='camera_monitor',
                name='camera_monitor',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='robotino_hal',
                executable='lidar_monitor',
                name='lidar_monitor',
                parameters=[params_file],
                output='screen'
            ),
        ]
    )
    
    # Include the Nav2 launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition
        }.items()
    )
    
    # State publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': os.path.join(robotino_hal_dir, 'urdf', 'robotino.urdf.xacro')
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    
    # Add the actions
    ld.add_action(robot_state_publisher)
    ld.add_action(hal_nodes)
    ld.add_action(nav2_bringup_launch)
    
    return ld