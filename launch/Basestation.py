from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='BasestationV3',
            executable='Basestation',
            output='screen',
            emulate_tty=True,
        )
    ])
