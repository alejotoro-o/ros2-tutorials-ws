import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo
from launch.events import Shutdown, matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)
    use_slam_cartographer = LaunchConfiguration('slam_cartographer', default=False)
    
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
        condition=launch.conditions.IfCondition(use_rviz)
    )

    ## Robot frames and transforms nodes
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.075', '0', '0', '0', 'base_footprint', 'base_link'],
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

    ## Sensor Fusion
    # ekf_params = os.path.join(package_dir, 'resource', 'ekf_params.yaml')
    # ekf_sensor_fusion = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     output='screen',
    #     parameters=[ekf_params],
    # )
    ekf_sensor_fusion = Node(
        package='mecanum_robot_sim',
        executable='ekf_node',
        parameters=[
            {'model_noise': [0.01,0.0,0.0,
                             0.0,0.01,0.0,
                             0.0,0.0,0.6]},
            {'sensor_noise': 0.001}
        ]
    )

    ## SLAM
    ## SLAM Toolbox
    toolbox_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params.yaml')
    start_async_slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            toolbox_params,
            {
                'use_sim_time': True,
            }
        ],
    )

    # Automatically Transition: Unconfigured -> Inactive (Configure)
    slam_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
    )

    # Automatically Transition: Inactive -> Active (Activate)
    slam_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
    )

    ## Cartographer
    cartographer_config_dir = os.path.join(package_dir, 'resource')
    cartographer_config_basename = 'cartographer_params.lua'
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', cartographer_config_basename],
        condition=launch.conditions.IfCondition(use_slam_cartographer)
    )

    grid_executable = 'cartographer_occupancy_grid_node'
    cartographer_grid = Node(
        package='cartographer_ros',
        executable=grid_executable,
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05'],
        condition=launch.conditions.IfCondition(use_slam_cartographer))

    ## Navigation
    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_path, 'launch', 'navigation_launch.py')),
        condition=launch.conditions.IfCondition(use_nav))
    
    ## Wait for controller before launching nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=[
            rviz2,
            odometry_publisher,
            ekf_sensor_fusion,
            start_async_slam_toolbox_node,
            slam_configure_event,
            slam_activate_event,
            cartographer,
            cartographer_grid,
            navigation,
        ]
    )

    return LaunchDescription([
        webots,
        robot_driver,
        footprint_publisher,
        robot_state_publisher,
        joint_state_publisher,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])