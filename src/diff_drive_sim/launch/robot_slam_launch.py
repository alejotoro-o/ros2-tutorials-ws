import os
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch.events import Shutdown, matches_action
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    package_dir = get_package_share_directory('diff_drive_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'diff_drive_imu_lidar.urdf')
    
    with open(robot_description_path, 'r') as desc:
        robot_description = desc.read()

    ## Static Transforms & State Publishers
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.05', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        arguments=[robot_description_path],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ["wheels_encoders"]}],
    )

    odometry_publisher = Node(
        package='diff_drive_sim',
        executable='odometry_publisher',
        remappings=[('/odom', '/wheel/odometry')],
    )

    sensor_fusion = Node(
        package='diff_drive_sim',
        executable='ekf_node',
        parameters=[
            {'model_noise': [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.8]},
            {'sensor_noise': 0.001}
        ],
        remappings=[('/filtered_odom', '/odom')],
    )

    ## RVIZ
    rviz2_config_path = os.path.join(package_dir, 'resource', 'slam.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz2_config_path],
    )

    ## SLAM (Lifecycle Node)
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

    ## Webots Launcher
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'diff_drive_sensor_fusion.wbt'),
    )

    robot_driver = WebotsController(
        robot_name='robot',
        parameters=[{'robot_description': robot_description_path}]
    )

    # Wait for controller before launching visualization and SLAM
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=[
            rviz2,
            start_async_slam_toolbox_node,
            slam_configure_event,
            slam_activate_event,
        ]
    )

    return LaunchDescription([
        webots,
        robot_driver,
        
        footprint_publisher,
        robot_state_publisher,
        joint_state_publisher,
        odometry_publisher,
        sensor_fusion,

        waiting_nodes,

        # Shutdown when Webots exits
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])