import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tutorial_interfaces.msg import Num    # CHANGE

from nav.imu.imu_integrated_movement import *

class IMU(Node):

    def __init__(self):
        super().__init__('imu')
        self.publisher_ = self.create_publisher(String, 'heading', 10)     # CHANGE
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = str(getYaw("/home/pi/pi-bno055/getbno055"))                                         # CHANGE          
        self.publisher_.publish(msg)
        self.get_logger().info('Heading: "%s"' % msg.data)  # CHANGE 
        self.i += 1


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



