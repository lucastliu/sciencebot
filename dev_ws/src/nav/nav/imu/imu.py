import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose

import time
import board
import busio
import adafruit_bno055


class IMU(Node):

    def __init__(self):
        super().__init__('imu')
        
        # Use these lines for I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # User these lines for UART
        # uart = busio.UART(board.TX, board.RX)
        # sensor = adafruit_bno055.BNO055_UART(uart)
        while not self.sensor.calibrated:
            self.get_logger().info('IMU not calibrated')
            self.get_logger().info(str(self.sensor.calibration_status))
            time.sleep(1)
        
        self.get_logger().info('Success: IMU Calibration ')
        time.sleep(1)
        
        self.publisher_ = self.create_publisher(Pose, 'heading', 10)     # CHANGE
        timer_period = .1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pos_pub = self.create_publisher(Pose, 'imu_position', 10)     # CHANGE


        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.accel = 0.5 * timer_period * timer_period # p = .5at^2

        self.i = 0

    def timer_callback(self):
        msg = Pose()
        self.position_update()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.publisher_.publish(msg)
        self.i += 1
        if self.i > 10:
            self.i = 0
            self.get_logger().info('IMU Pose: "%s"' % msg)
        
    def position_update(self):
        x = self.sensor.linear_acceleration[0]
        y = self.sensor.linear_acceleration[1]
        theta = self.sensor.euler[0]
        
        if x:
            self.x = x
        if y:
            self.y = y
        if theta:
            self.theta = theta



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



