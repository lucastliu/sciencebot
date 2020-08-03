import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class PoseFusion(Node):
    """
    Node to combine sensor information into a single Pose msg
    Currently just fills in x, y position from DWM &
    theta from imu.

    Potential entrypoint for more advanced sensor fusion,
    i.e. Kalman filter, AMCL
    """
    def __init__(self):
        super().__init__('pose_fusion')

        self.pose = Pose()

        self.xy = self.create_subscription(
            Pose,
            'xy',
            self.xy_callback,
            qos_profile_sensor_data)
        self.xy  # prevent unused variable warning

        self.heading = self.create_subscription(
            Pose,
            'heading',
            self.heading_callback,
            qos_profile_sensor_data)
        self.heading  # prevent unused variable warning

        self.publisher = self.create_publisher(Pose, 'pose', qos_profile_sensor_data)
        self.i = 0
        self.get_logger().info('Pose Fusion Node Live')

    def xy_callback(self, data):
        self.pose.x = data.x
        self.pose.y = data.y
        self.publisher.publish(self.pose)
        self.log_pose()

    def heading_callback(self, data):
        self.pose.theta = data.theta
        self.publisher.publish(self.pose)
        self.log_pose()

    def log_pose(self):
        self.i += 1
        if self.i >= 10:
            self.i = 0
            self.get_logger().info('X: {:06.3f}  Y: {:06.3f}  Theta: {:06.3f}'
                                   .format(self.pose.x, self.pose.y, self.pose.theta))


def main(args=None):
    rclpy.init(args=args)

    pf = PoseFusion()

    rclpy.spin(pf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
