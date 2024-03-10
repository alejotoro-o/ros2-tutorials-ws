import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    package_dir = get_package_share_directory('diff_drive_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'diff_drive_imu_lidar.urdf')
    with open(robot_description_path, 'r') as desc:
        robot_description = desc.read()
        

    ## Robot frames and transforms nodes
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
    odometry_publisher = Node(
        package='diff_drive_sim',
        executable='odometry_publisher',
        remappings=[('/odom', '/wheel/odometry')],
    )

    ## RVIZ
    rviz2_config_path = os.path.join(package_dir, 'resource', 'odometry.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz2_config_path],
    )

    ## Robot Localization - Sensor Fusion
    sensor_fusion_config_path = os.path.join(package_dir, 'resource', 'ekf.yaml')
    sensor_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[sensor_fusion_config_path, 
                    {'use_sim_time': False}]
    )

    ## Webots and Robot Nodes
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'diff_drive_sensor_fusion.wbt'),
    )
    robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        robot_driver,
        
        footprint_publisher,
        robot_state_publisher,
        joint_state_publisher,
        odometry_publisher,

        rviz2,
        sensor_fusion,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])