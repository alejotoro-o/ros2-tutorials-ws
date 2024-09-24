import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('diff_drive_sim_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(package_dir, 'worlds', 'diff_drive.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + world
        }.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable',
                     'qos_overrides./model/vehicle_green.subscriber.reliability': 'reliable'}],
        output='screen'
    )

    ## RVIZ
    rviz2_config_path = os.path.join(package_dir, 'resource', 'diff_drive.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz2_config_path],
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        rviz
    ])