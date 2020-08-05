from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
             package='nav',
             node_executable='imu',
             output="screen",
             emulate_tty=True
         ),
        Node(
            package='nav',
            node_executable='dwm',
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='nav',
            node_executable='posefusion',
            output="screen",
            emulate_tty=True
        )
    ])
