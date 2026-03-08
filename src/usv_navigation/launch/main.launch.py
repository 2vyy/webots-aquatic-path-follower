"""
Unified USV Launch File
=======================
Supports both simulation (Webots) and real hardware (BlueBoat via MAVROS).

Usage:
  Simulation:  ros2 launch usv_navigation main.launch.py use_sim:=true
  Hardware:    ros2 launch usv_navigation main.launch.py use_sim:=false
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Arguments ---
    use_sim = LaunchConfiguration('use_sim')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Set to "true" for Webots simulation, "false" for real BlueBoat hardware'
    )

    # --- Package Paths ---
    sim_pkg_dir = get_package_share_directory('usv_simulation')
    nav_pkg_dir = get_package_share_directory('usv_navigation')

    # =========================================================================
    # SIMULATION MODE: Launch Webots + Simulator Bridge
    # =========================================================================
    sim_launch = GroupAction(
        condition=IfCondition(use_sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_pkg_dir, 'launch', 'robot_launch.py')
                ),
            ),
        ]
    )

    # =========================================================================
    # HARDWARE MODE: Launch MAVROS + Robot State Publisher
    # =========================================================================
    hw_launch = GroupAction(
        condition=UnlessCondition(use_sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg_dir, 'launch', 'hardware_launch.py')
                ),
            ),
        ]
    )

    # =========================================================================
    # NAVIGATION (always launched, after a delay for clock/sensors to start)
    # =========================================================================
    nav_launch = TimerAction(
        period=5.0,  # Wait for sim clock or hardware sensors to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim,
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim,
        sim_launch,
        hw_launch,
        nav_launch,
    ])
