import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node


def generate_launch_description():
    
    package_dir = get_package_share_directory('mecanum_robot_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot.urdf')
    with open(robot_description_path, 'r') as desc:
        robot_description = desc.read()

    ## Webots Launcher
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'mecanum_robot.wbt')
    )

    ## Robot Driver
    robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    ## RVIZ
    rviz2_config_path = os.path.join(package_dir, 'resource', 'rviz_config.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz2_config_path],
        remappings=[('/map', 'map'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')],
    )

    ## Robot frames and transforms nodes
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.075', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
        arguments=[robot_description_path],
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list':["wheels_encoders"]}],
    )

    ## Robot odometry
    odometry_publisher = Node(
        package='mecanum_robot_sim',
        executable='odometry_publisher',
        remappings=[('/odom', '/wheel/odometry')],
    )

    return LaunchDescription([
        webots,
        robot_driver,
        rviz2,
        footprint_publisher,
        robot_state_publisher,
        joint_state_publisher,
        odometry_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])