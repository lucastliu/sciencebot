import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose

from nav.imu.imu_integrated_movement import *


class IMU(Node):

    def __init__(self):
        super().__init__('imu')
        self.publisher_ = self.create_publisher(Pose, 'heading', 10)     # CHANGE
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.theta = getYaw("/home/pi/pi-bno055/getbno055")
        self.publisher_.publish(msg)
        self.get_logger().info('Heading: "%s"' % msg.theta)


def main(args=None):
    rclpy.init(args=args)

    imu = IMU()

    rclpy.spin(imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



