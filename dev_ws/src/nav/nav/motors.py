import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import time

import RPi.GPIO as GPIO
import time
from nav.SerialMotor import SerialMotor
from nav.motor_constants import *


class Motors(Node):

    def __init__(self):
        super().__init__('motors')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.sm = SerialMotor("/dev/ttyACM1")
        
    def convert_velocity_to_power(self, vel):
        neg = False
        if vel < 0:
            vel = -1 * vel
            neg = True
        
        power = 1.3375 * vel - 0.5473
        
        if abs(power) > 1:
            power = power/power
            self.get_logger().info('Exceeded maximum wheel power')
        if abs(power) < .5:
            self.get_logger().info('Low wheel power')
            power = .5
        if neg:
            power = -1 * power
            
        return power
    
    def listener_callback(self, twist):
        self.get_logger().info('Twist  Linear: %.2f Angular: %.2f' % (twist.linear.x, twist.angular.z)) # CHANGE
        WHEEL_BASE = .15  # distance between wheels, meters
        RADIUS = .035  # wheel radius, meters
        right_vel = (-1 * twist.linear.x + twist.angular.z * WHEEL_BASE / 2.0) / RADIUS  #right
        left_vel = (-1 * twist.linear.x - twist.angular.z * WHEEL_BASE / 2.0) / RADIUS   # left
        self.get_logger().info('Velocity  Right: %.3f Left: %.3f' % (right_vel, left_vel)) # CHANGE

        #  convert desired velocity to wheel power, -1 to 1
        
        #  hard cap top and bottom ranges
        
        right_power = convert_velocity_to_power(right_vel)
        left_power = convert_velocity_to_power(left_vel)
        self.get_logger().info('Power Right: %.3f Left: %.3f' % (right_power, left_power)) # CHANGE

        self.sm.set_motor(3, right_power)  # right
        self.sm.set_motor(4, left_power)  # left
        

def main(args=None):
    rclpy.init(args=args)

    motors = Motors()

    rclpy.spin(motors)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
