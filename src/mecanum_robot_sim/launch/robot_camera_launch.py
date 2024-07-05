import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)
    use_rtabmap = LaunchConfiguration('rtabmap', default=False)
    
    package_dir = get_package_share_directory('mecanum_robot_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot_camera.urdf')
    with open(robot_description_path, 'r') as desc:
        robot_description = desc.read()

    ## Webots Launcher
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'mecanum_robot_camera.wbt')
    )

    ## Robot Driver
    robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    ## RVIZ
    rviz2_config_path = os.path.join(package_dir, 'resource', 'rviz_camera_config.rviz')
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
    slam_toolbox = Node(
        parameters=[toolbox_params,
                    {'use_sim_time': True}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam_toolbox)
    )

    ## RTABmap
    rtabmap_launch_path = get_package_share_directory('rtabmap_launch')
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_path, 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            "rtabmap_args":"--delete_db_on_start",
            "frame_id":"base_link",
            "rgb_topic":"/camera/image_color",
            "depth_topic":"/range_finder/image",
            "camera_info_topic":"/camera/camera_info",
            "approx_sync":"True",
            "rviz":"False",
            "visual_odometry":"False",
            "odom_topic":"/filtered_odom",
            #"queue_size":"40"
        }.items(),
        condition=launch.conditions.IfCondition(use_rtabmap))

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
            rtabmap,
            slam_toolbox,
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