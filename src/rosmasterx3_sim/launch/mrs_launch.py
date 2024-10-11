import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    num_robots = 2
    nodes = []

    package_dir = get_package_share_directory('rosmasterx3_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'rosmasterx3_multi_robot.urdf')
    with open(robot_description_path, 'r') as desc:
        robot_description = desc.read()

    nodes.append(DeclareLaunchArgument(
            'world',
            default_value='multi_robot.wbt',
            description='World file'
    ))
    nodes.append(DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
    ))

    ## Webots and Robot Nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
    )
    nodes.append(webots)
    nodes.append(webots._supervisor)

    ## RVIZ
    rviz2_config_path = os.path.join(package_dir, 'resource', 'rviz_mrs_config.rviz')
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
    nodes.append(rviz2)

    for i in range(1,num_robots + 1):

        robot_namespace = "robot" + str(i)

        robot_driver = WebotsController(
            robot_name=robot_namespace,
            parameters=[
                {'robot_description': robot_description_path},
            ],
            namespace=robot_namespace
        )

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': robot_namespace + "/"
            }],
            arguments=[robot_description_path],
            namespace=robot_namespace,
        )

        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'source_list':["joint_states"]}],
            namespace=robot_namespace,
        )

        optical_frame_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '1.57', '3.14', '1.57', robot_namespace + '/camera', robot_namespace + '/optical_frame'],
            namespace=robot_namespace,
        )

        nodes.append(robot_driver)
        nodes.append(robot_state_publisher)
        nodes.append(joint_state_publisher)
        nodes.append(optical_frame_publisher)

    nodes.append(launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    ))

    return LaunchDescription(nodes)