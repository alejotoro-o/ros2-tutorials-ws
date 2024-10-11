from setuptools import find_packages, setup

package_name = 'rosmasterx3_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
## Meshes
data_files.append(('share/' + package_name + '/meshes', ['meshes/base_link_X3_mod.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/laser_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/camera_link.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/front_right_wheel_X3.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/front_left_wheel_X3.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/back_right_wheel_X3.STL']))
data_files.append(('share/' + package_name + '/meshes', ['meshes/back_left_wheel_X3.STL']))
## Worlds
data_files.append(('share/' + package_name + '/worlds', ['worlds/rosmasterx3.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/multi_robot.wbt']))
## Launch
data_files.append(('share/' + package_name + '/launch', ['launch/rosmaster_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/multi_robot_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/mrs_launch.py']))
## Resource
data_files.append(('share/' + package_name + '/resource', ['resource/rosmasterx3.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/slam_toolbox_params.yaml']))
data_files.append(('share/' + package_name + '/resource', ['resource/rviz_config.rviz']))
data_files.append(('share/' + package_name + '/resource', ['resource/rosmasterx3_multi_robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/rviz_mrs_config.rviz']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alejotoro.o@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "leader_follower_controller = rosmasterx3_sim.leader_follower_controller:main"
        ],
    },
)
