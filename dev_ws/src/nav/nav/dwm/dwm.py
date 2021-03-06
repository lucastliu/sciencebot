import time
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from custom_interfaces.msg import Num
from turtlesim.msg import Pose

from nav.dwm.DWMTag import DWMTag


class DWM(Node):

    def __init__(self):
        super().__init__('dwm')
        self.publisher_ = self.create_publisher(Pose, 'xy', qos_profile_sensor_data)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.tag = DWMTag(port_name="/dev/ttyACM0")
        self.get_logger().info('DWM Node Live')

    def timer_callback(self):
        self.tag.update_position()
        msg = Pose()
        position = self.tag.get_pos()
        msg.x = position[0]
        msg.y = position[1]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    dwm = DWM()

    rclpy.spin(dwm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dwm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
