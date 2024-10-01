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

    use_nav = LaunchConfiguration('nav', default=False)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)
    use_rtabmap = LaunchConfiguration('rtabmap', default=False)

    package_dir = get_package_share_directory('rosmasterx3_sim')
    robot_description_path = os.path.join(package_dir, 'resource', 'rosmasterx3.urdf')
    with open(robot_description_path, 'r') as desc:
        robot_description = desc.read()
        
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    ## Robot frames and transforms nodes
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
        parameters=[{'source_list':["joint_states"]}],
    )
    optical_frame_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '1.57', '3.14', '1.57', 'camera', 'optical_frame'],
    )

    ## Webots and Robot Nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True,
    )
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
            "frame_id":"base_footprint",
            "rgb_topic":"/camera/image_color",
            "depth_topic":"/range_finder/image",
            "camera_info_topic":"/camera/camera_info",
            "approx_sync":"True",
            "rviz":"False",
            "visual_odometry":"False",
            "odom_topic":"/odom",
            #"queue_size":"20"
        }.items(),
        condition=launch.conditions.IfCondition(use_rtabmap)
    )

    ## Navigation
    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_path, 'launch', 'navigation_launch.py')),
        condition=launch.conditions.IfCondition(use_nav)
    )

    ## Wait for controller before launching nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=[
            rviz2,
            rtabmap,
            slam_toolbox,
            navigation,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='rosmasterx3.wbt',
            description='World file'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,
        robot_driver,

        robot_state_publisher,
        joint_state_publisher,
        optical_frame_publisher,

        waiting_nodes,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])