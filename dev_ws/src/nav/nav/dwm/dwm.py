import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tutorial_interfaces.msg import Num    # CHANGE

from nav.dwm.DWMTag import DWMTag

class DWM(Node):

    def __init__(self):
        super().__init__('dwm')
        self.publisher_ = self.create_publisher(String, 'position', 10)     # CHANGE
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.tag = DWMTag(port_name="/dev/ttyACM0")

    def timer_callback(self):
        self.tag.update_position()
        msg = String()
        msg.data = str(self.tag.get_pos())                                        # CHANGE          
        self.publisher_.publish(msg)
        self.get_logger().info('Beacons: "%s"' % msg.data)  # CHANGE 
        self.i += 1


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



