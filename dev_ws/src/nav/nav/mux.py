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
# limitations under the License.c

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time

from rclpy.qos import qos_profile_sensor_data

class Mux(Node):

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
            self.block_duration = 0 # stop blocking
            self.publisher.publish(twist)
            #self.get_logger().info('Twist  Linear: %.2f Angular: %.2f' % (twist.linear.x, twist.angular.z)) # CHANGE



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

