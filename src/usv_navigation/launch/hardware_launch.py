"""
Hardware Launch File
====================
Launches the real BlueBoat hardware bridge using MAVROS.
This is the counterpart to robot_launch.py (which launches Webots).

Topics published (matching the simulation bridge):
  - /scan          (sensor_msgs/LaserScan)   — from real LiDAR driver
  - /odom          (nav_msgs/Odometry)        — from MAVROS
  - /cmd_vel       (geometry_msgs/Twist)      — consumed by MAVROS

Prerequisites:
  - blueos-ros2 extension running on the companion computer
  - MAVROS installed: sudo apt install ros-${ROS_DISTRO}-mavros
  - Real LiDAR driver node running (e.g. sllidar_ros2 or similar)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg_dir = get_package_share_directory('usv_description')
    robot_description_path = os.path.join(description_pkg_dir, 'urdf', 'usv.urdf')

    # --- Robot State Publisher (same URDF, but use_sim_time=False) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(robot_description_path).read(),
            'use_sim_time': False
        }]
    )

    # --- MAVROS Node ---
    # Connects to ArduRover via MAVLink.
    # fcu_url: serial port on the companion computer (adjust for your setup)
    # gcs_url: optional ground station forwarding
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://:14550@',  # Default BlueOS MAVLink endpoint
            'gcs_url': '',
            'system_id': 1,
            'component_id': 1,
            'target_system_id': 1,
            'target_component_id': 1,
        }],
        # Remap MAVROS velocity output to match our standard /cmd_vel interface
        remappings=[
            ('/mavros/setpoint_velocity/cmd_vel_unstamped', '/cmd_vel'),
        ]
    )

    # --- Placeholder for Real LiDAR Driver ---
    # Uncomment and configure when you have a physical LiDAR:
    #
    # lidar_node = Node(
    #     package='sllidar_ros2',       # or 'rplidar_ros' depending on sensor
    #     executable='sllidar_node',
    #     name='lidar',
    #     output='screen',
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB0',
    #         'frame_id': 'laser_link',
    #     }],
    #     remappings=[
    #         ('/scan', '/scan'),  # Already standard
    #     ]
    # )

    return LaunchDescription([
        robot_state_publisher,
        mavros_node,
        # lidar_node,  # Uncomment when real LiDAR is connected
    ])
