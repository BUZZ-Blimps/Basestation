from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Blimp',
            namespace='Blimp1',
            executable='Blimp'
        )
    ])
