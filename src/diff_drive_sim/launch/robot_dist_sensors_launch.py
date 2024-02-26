import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('diff_drive_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot_dist_sensors.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'diff_drive_dist_sensors.wbt')
    )

    webots_robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    obstacle_avoider = Node(
        package='diff_drive_sim',
        executable='obstacle_avoider',
    )

    return LaunchDescription([
        webots,
        webots_robot_driver,
        obstacle_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])