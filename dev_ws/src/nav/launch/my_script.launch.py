from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
#         Node(
#             package='nav',
#             node_executable='imu',
#             output="screen"
#         ),
        Node(
            package='nav',
            node_executable='mux'
        ),
        Node(
            package='nav',
            node_executable='posefusion'
        ),
#         Node(
#             package='nav',
#             node_executable='motors'
#         ),
    ])