# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        
        #change this to directly affect LR motors
        #          setWheelVelocity((int) ((received.linear + received.rotation) * 100), (int) ((received.linear - received.rotation) * 100));
        
        self.sm = SerialMotor("/dev/ttyACM1")

    def listener_callback(self, twist):
        self.get_logger().info('Teleop Command  Linear: %.2f Angular: %.2f' % (twist.linear.x, twist.angular.z)) # CHANGE

        if twist.linear.x and twist.angular.z:
            self.sm.set_motor(3,(-1*twist.linear.x + twist.angular.z)/2)
            self.sm.set_motor(4,(-1*twist.linear.x - twist.angular.z)/2)
        else:
            self.sm.set_motor(3,(-1*twist.linear.x + twist.angular.z))
            self.sm.set_motor(4,(-1*twist.linear.x - twist.angular.z))
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
