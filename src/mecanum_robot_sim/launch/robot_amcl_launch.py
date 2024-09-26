import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
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
    rviz2_config_path = os.path.join(package_dir, 'resource', 'rviz_amcl_config.rviz')
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

    ## Navigation Map Server and AMCL
    map_file_path = os.path.join(package_dir, 'resource', 'map.yaml')
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}]
    )

    amcl_params = os.path.join(package_dir, 'resource', 'amcl_params.yaml')
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[amcl_params],
    )

    lifecycle_nodes = ['map_server', 'amcl']
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )

    
    ## Wait for controller before launching nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=[
            rviz2,
            odometry_publisher,
            ekf_sensor_fusion,
            map_server,
            amcl,
            lifecycle_manager
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