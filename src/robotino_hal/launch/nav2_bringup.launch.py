from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Get the package directory
    robotino_hal_dir = get_package_share_directory('robotino_hal')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Paths to files
    urdf_file = os.path.join(robotino_hal_dir, 'description', 'robotino.urdf')
    params_file = os.path.join(robotino_hal_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(robotino_hal_dir, 'config', 'rviz_config.rviz')

    # Launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to Nav2 parameters file'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
    'map',
    default_value=os.path.join(robotino_hal_dir, 'maps', 'test1.yaml'),
    description='Path to the map YAML file'
)

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': False
        }]
    )

    # Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'False',
            'map': LaunchConfiguration('map'),
        }.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        robot_state_publisher,
        nav2_launch,
        rviz,
    ])
