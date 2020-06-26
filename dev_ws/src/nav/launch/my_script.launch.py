from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav',
            node_executable='imu',
            output="screen"
        ),
        Node(
            package='nav',
            node_executable='mover'
        ),
#         Node(
#             package='key_teleop',
#             node_executable='key_teleop',
#             node_name='keyx',
#         )
    ])