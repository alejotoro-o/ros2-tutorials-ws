from setuptools import find_packages, setup

package_name = 'action_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'action = action_package.counter_action_server:main',
            'client = action_package.counter_action_client:main',

            'adv_action = action_package.advanced_counter_action_server:main',
            'adv_client = action_package.advanced_counter_action_client:main'
        ],
    },
)
