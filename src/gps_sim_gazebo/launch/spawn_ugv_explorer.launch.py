#!/usr/bin/env python3
"""
Launch file: spawn_ugv_csiro.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # ----- Paquetes y rutas -----
    pkg_robot_simulator = get_package_share_directory('gps_sim_gazebo')
    pkg_ros_gz_sim      = get_package_share_directory('ros_gz_sim')
    models_path = os.path.join(pkg_robot_simulator, 'models')
    worlds_path = os.path.join(pkg_robot_simulator, 'worlds')

    # ----- Argumentos -----
    declare_model_folder = DeclareLaunchArgument(
        'model_folder', default_value='EXPLORER_R2_SENSOR_CONFIG_1',
        description='Carpeta del modelo dentro de models/ a spawnear')
    declare_world = DeclareLaunchArgument(
        'world', default_value='tugbot_depot.sdf',
        description='Archivo .sdf del mundo dentro de worlds/<world_folder>/')
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='explorer_r2_sensor_config_1')
    # declare_x = DeclareLaunchArgument('x', default_value='-8.0') #For Rubicon World
    declare_x = DeclareLaunchArgument('x', default_value='0.0') # For empty World
    declare_y = DeclareLaunchArgument('y', default_value='-0.0')
    # declare_z = DeclareLaunchArgument('z', default_value='4.4') #For Rubicon world
    declare_z = DeclareLaunchArgument('z', default_value='0.4') # For empty World
    declare_R = DeclareLaunchArgument('R', default_value='0.0')
    declare_P = DeclareLaunchArgument('P', default_value='0.0')
    declare_Y = DeclareLaunchArgument('Y', default_value='0.0')
    declare_gui = DeclareLaunchArgument('gui', default_value='true')

    model_folder = LaunchConfiguration('model_folder')
    world        = LaunchConfiguration('world')
    robot_name   = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x'); y = LaunchConfiguration('y'); z = LaunchConfiguration('z')
    R = LaunchConfiguration('R'); P = LaunchConfiguration('P'); Y = LaunchConfiguration('Y')

    # ----- Rutas a SDFs -----
    model_sdf = PathJoinSubstitution([models_path, model_folder, 'model.sdf'])
    world_sdf = PathJoinSubstitution([worlds_path, world])

    gz_args_gui      = [world_sdf, ' -r -v 4']
    gz_args_headless = [world_sdf, ' -r -v 4 -s']

    # ----- Variables de entorno -----
    env_actions = [
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', worlds_path),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     models_path),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',     worlds_path),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([worlds_path])),
        AppendEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            PathJoinSubstitution([worlds_path])),
    ]

    # ----- Lanzar Gazebo -----
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args_gui}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args_headless}.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    # ----- Spawn del robot -----
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', name='spawn_robot',
        output='screen',
        arguments=[
            '-file', model_sdf, '-name', robot_name,
            '-x', x, '-y', y, '-z', z,
            '-R', R, '-P', P, '-Y', Y,
            '-allow_renaming', 'true',
        ],
    )

    # ----- Puente ROS2 <-> Ignition -----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            # ROS2 -> Ignition
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Ignition -> ROS2
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
    )

    # ----- NavSat Haversine -> coordenadas cartesianas locales -----
    navsat_to_cartesian = Node(
        package='gps_sim_gazebo',
        executable='navsat_to_cartesian',
        name='navsat_to_cartesian',
        output='screen',
    )

    # ----- Fusion: GPS position + IMU orientation -> /pose -----
    pose_fuser = Node(
        package='gps_sim_gazebo',
        executable='pose_fuser',
        name='pose_fuser',
        output='screen',
    )

    # ----- Control de posicion en lazo cerrado -----
    position_controller = Node(
        package='gps_sim_gazebo',
        executable='position_controller',
        name='position_controller',
        output='screen',
        parameters=[{
            # Tipo de controlador: pid | turn_drive | bang_bang | pure_pursuit
            'controller_type': 'pid',
            # Meta (setpoint) en metros
            'goal_x': 0.0,
            'goal_y': 0.0,
            # PID — ganancias lineal (distancia)
            'kp_linear': 1.0,
            'ki_linear': 0.0,
            'kd_linear': 0.125,
            # PID — ganancias angular (orientacion)
            'kp_angular': 3.0,
            'ki_angular': 0.0,
            'kd_angular': 0.125,
            # Limites de velocidad
            'max_vel_lin': 0.5,
            'max_vel_ang': 1.0,
            # Tolerancia para considerar meta alcanzada [m]
            'goal_tolerance': 0.1,
            # turn_drive — umbral de angulo [rad] (~12 deg)
            'angle_threshold': 0.2,
            # bang_bang — zona muerta angular [rad] (~6 deg)
            'dead_zone': 0.1,
            # pure_pursuit — distancia de lookahead [m]
            'lookahead_distance': 1.0,
            # pure_pursuit — velocidad lineal constante [m/s]
            'pure_pursuit_speed': 0.3,
        }],
    )

    return LaunchDescription([
        declare_model_folder, declare_world,
        declare_robot_name,
        declare_x, declare_y, declare_z,
        declare_R, declare_P, declare_Y,
        declare_gui,
        *env_actions,
        gz_sim_gui, gz_sim_headless,
        spawn_robot,
        bridge,
        navsat_to_cartesian,
        pose_fuser,
        position_controller,
    ])