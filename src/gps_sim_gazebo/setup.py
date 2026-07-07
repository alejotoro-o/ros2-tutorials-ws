from setuptools import setup
import os
from glob import glob

package_name = 'gps_sim_gazebo'

def package_files(directory):
    """Recursively collect all files under a directory."""
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [file_path]))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*')),
]

# Add recursive worlds and models
data_files += package_files('worlds')
data_files += package_files('models')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro',
    maintainer_email='your_email@example.com',
    description='GPS simulation in Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navsat_to_cartesian = gps_sim_gazebo.navsat_to_cartesian:main',
            'pose_fuser = gps_sim_gazebo.pose_fuser:main',
            'position_controller = gps_sim_gazebo.position_controller:main',
            'setpoint_client = gps_sim_gazebo.setpoint_client:main',
        ],
    },
)
