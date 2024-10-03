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

    robot1_namespace = "robot1"
    robot2_namespace = "robot2"

    package_dir = get_package_share_directory('rosmasterx3_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'rosmasterx3_multi_robot.urdf')
    # with open(robot_description_path, 'r') as desc:
    #     robot_description = desc.read()
        
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    ## Webots and Robot Nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
    )

    robot_driver1 = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        namespace=robot1_namespace
    )
    robot_driver2 = WebotsController(
        robot_name='robot2',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        namespace=robot2_namespace
    )

    leader_follower_controller = Node(
        package='rosmasterx3_sim',
        executable='leader_follower_controller',
        parameters=[
            {'robot1_initial_pose': [0.0,0.5,0.0]},
            {'robot2_initial_pose': [0.0,-0.5,0.0]},
            {'d_goal': 0.5},
            {'alpha_goal': 0.0},
            {'theta_f_goal': 0.0},
            {'K': [1.0,1.0,1.0]},
        ],
        remappings=[
            ('robot1/pose','robot1/webots_pose'),
            ('robot2/pose','robot2/webots_pose')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='multi_robot.wbt',
            description='World file'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,
        robot_driver1,
        robot_driver2,

        leader_follower_controller,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])