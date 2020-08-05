from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav',
            node_executable='mux',
            output="screen",
            emulate_tty=True
        ),
        Node(
             package='nav',
             node_executable='motors',
             output="screen",
             emulate_tty=True
         ),
    ])
