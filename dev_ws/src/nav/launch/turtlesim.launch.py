from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            node_executable='turtlesim_node',
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='nav',
            node_executable='turtlepid',
            output="screen",
            emulate_tty=True
        )
    ])
