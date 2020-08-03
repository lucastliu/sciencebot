import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist


class Mux(Node):
    """
    Intermediate node to choose between different motor command sources
    Currently setup to favor manual control input
    """
    def __init__(self):
        super().__init__('mux')

        self.autonomous = self.create_subscription(
            Twist,
            'auto_vel',
            self.autonomous_callback,
            qos_profile_sensor_data)
        self.autonomous  # prevent unused variable warning

        self.manual = self.create_subscription(
            Twist,
            'key_vel',
            self.manual_callback,
            qos_profile_sensor_data)

        self.manual  # prevent unused variable warning
        self.block_duration = 0
        self.manual_time = time.time()

        p = qos_profile_sensor_data
        p.depth = 1
        self.publisher = self.create_publisher(Twist, 'cmd_vel', p)
        self.get_logger().info('Mux Node Live')

    def manual_callback(self, twist):
        self.manual_time = time.time()
        self.block_duration = 5
        self.publisher.publish(twist)

    def autonomous_callback(self, twist):
        time_since_manual_cmd = time.time() - self.manual_time
        if time_since_manual_cmd >= self.block_duration:
            self.block_duration = 0  # stop blocking
            self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    mux = Mux()

    rclpy.spin(mux)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mux.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
