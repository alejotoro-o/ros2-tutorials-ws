from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    talker1 = Node(
        package="lifecycle_nodes",
        executable="talker1"
    )

    talker2 = Node(
        package="lifecycle_nodes",
        executable="talker2"
    )

    listener = Node(
        package="lifecycle_nodes",
        executable="listener"
    )

    return LaunchDescription([
        talker1,
        talker2,
        listener
    ])