import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    # 1. PATHS
    package_dir = get_package_share_directory('usv_simulation')
    description_pkg_dir = get_package_share_directory('usv_description')
    
    # Path to your .wbt file (Ensure you moved it here!)
    world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt')
    
    # Path to the URDF we just created
    robot_description_path = os.path.join(description_pkg_dir, 'urdf', 'usv.urdf')

    # 2. WEBOTS INSTANCE
    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=False
    )

    # 3. ROBOT DRIVER (The Bridge)
    # This reads the URDF, finds the <plugin> tag, and loads your Python script
    my_robot_driver = WebotsController(
        robot_name='my_robot', # IMPORTANT: Must match the "name" field of the Robot node in your .wbt file
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(robot_description_path).read(),
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        robot_state_publisher,
        # Shutdown logic: If Webots closes, kill the driver
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])