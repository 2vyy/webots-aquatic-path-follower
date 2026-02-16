import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    usv_nav_pkg_dir = get_package_share_directory('usv_navigation')

    # Arguments
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare the params file argument
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(usv_nav_pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(usv_nav_pkg_dir, 'config', 'mapper_params_online_async.yaml'),
        description='Full path to the SLAM Toolbox parameters file'
    )

    # SLAM Toolbox — provides the map -> odom TF
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ],
    )

    # Include the standard Nav2 Bringup launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        declare_params_file,
        declare_slam_params_file,
        slam_toolbox_node,
        nav2_launch,
    ])