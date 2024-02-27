from setuptools import find_packages, setup

package_name = 'diff_drive_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
## Launch files
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_dist_sensors_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_rviz_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_lidar_launch.py']))
## World files
data_files.append(('share/' + package_name + '/worlds', ['worlds/diff_drive.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/diff_drive_dist_sensors.wbt']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/diff_drive_lidar.wbt']))
## Resource files
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot_dist_sensors.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/diff_drive.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/diff_drive_lidar.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/urdf_view.rviz']))
data_files.append(('share/' + package_name + '/resource', ['resource/lidar_scan.rviz']))

data_files.append(('share/' + package_name, ['package.xml']))

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
            'robot_driver = diff_drive_sim.webots_robot_driver:main',

            'obstacle_avoider = diff_drive_sim.obstacle_avoider:main',
        ],
    },
)
