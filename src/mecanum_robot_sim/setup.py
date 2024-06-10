from setuptools import find_packages, setup

package_name = 'mecanum_robot_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
## Launch Files
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
## Resource files
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
## World files
data_files.append(('share/' + package_name + '/worlds', ['worlds/mecanum_robot.wbt']))

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
