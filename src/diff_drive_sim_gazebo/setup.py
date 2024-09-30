from setuptools import find_packages, setup

package_name = 'diff_drive_sim_gazebo'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
## Worlds
data_files.append(('share/' + package_name + '/worlds', ['worlds/diff_drive.sdf']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/diff_drive_empty.sdf']))
## Launch
data_files.append(('share/' + package_name + '/launch', ['launch/diff_drive_launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/diff_drive_urdf_launch.py']))
## Resource
data_files.append(('share/' + package_name + '/resource', ['resource/diff_drive.rviz']))
data_files.append(('share/' + package_name + '/resource', ['resource/diff_drive_urdf.rviz']))
data_files.append(('share/' + package_name + '/resource', ['resource/diff_drive.urdf']))

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
        ],
    },
)
