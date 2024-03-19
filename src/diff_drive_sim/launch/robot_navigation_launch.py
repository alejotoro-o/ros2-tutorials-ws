import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node


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
    ## Sensor Fusion
    sensor_fusion = Node(
        package='diff_drive_sim',
        executable='ekf_node',
        parameters=[
            {'model_noise': [0.01,0.0,0.0,
                             0.0,0.01,0.0,
                             0.0,0.0,0.8]},
            {'sensor_noise': 0.001}
        ],
        remappings=[('/filtered_odom', '/odom')],
    )


    ## RVIZ
    rviz2_config_path = os.path.join(package_dir, 'resource', 'navigation.rviz')
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

    ## SLAM
    toolbox_params = os.path.join(package_dir, 'resource', 'slam_toolbox_params_nav.yaml')
    slam_toolbox = Node(
        parameters=[toolbox_params,
                    {'use_sim_time': True}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    ## Navigation
    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_path, 'launch', 'navigation_launch.py'))
    )

    ## Webots and Robot Nodes
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'diff_drive_navigation.wbt'),
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
        sensor_fusion,

        rviz2,
        slam_toolbox,
        navigation,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])