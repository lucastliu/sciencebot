import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time


class PoseFusion(Node):

    def __init__(self):
        super().__init__('pose_fusion')

        self.pose = Pose()
        
        self.xy = self.create_subscription(
            Pose,
            'xy',
            self.xy_callback,
            10)
        self.xy  # prevent unused variable warning

        self.heading = self.create_subscription(
            Pose,
            'heading',
            self.heading_callback,
            10)
        self.heading  # prevent unused variable warning

        self.publisher = self.create_publisher(Pose, 'pose', 10)     # CHANGE
        
    def xy_callback(self, data):
        self.pose.x = data.x
        self.pose.y = data.y
        self.publisher.publish(self.pose)
        
    def heading_callback(self, data):
        self.pose.theta = data.theta
        self.publisher.publish(self.pose)


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

