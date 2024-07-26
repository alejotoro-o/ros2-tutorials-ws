from setuptools import find_packages, setup

package_name = 'lifecycle_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejandro',
    maintainer_email='alejotoro.o@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "talker1 = lifecycle_nodes.talker1:main",
            "talker2 = lifecycle_nodes.talker2:main",
            "listener = lifecycle_nodes.listener:main",
            "lifecycle_client = lifecycle_nodes.lifecycle_client:main"
        ],
    },
)
