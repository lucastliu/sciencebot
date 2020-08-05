import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import MoveTo, Tune
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav.pid import PID

from rclpy.qos import qos_profile_sensor_data
from nav.motors.SerialMotor import SerialMotor

from nav.controller_base import ControllerBase


class Bang(ControllerBase):

    def __init__(self):
        super().__init__('bang', Pose, 'pose', MoveTo, 'move_to')
        self.group = MutuallyExclusiveCallbackGroup()

        self.sm = SerialMotor("/dev/ttyACM1")

        self.r = 1.0
        self.angle_diff = 10
        self.old = [0.0, 0.0]
        self.twist = Twist()

        self.get_logger().info('BB Node Live')

    def pose_callback(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.angle = self.angle_convert(math.radians(pose.theta % 360.0))

    def action_callback(self, goal_handle):
        self.get_logger().info('Executing Move To...')
        self.x_dest = goal_handle.request.x_dest
        self.y_dest = goal_handle.request.y_dest
        self.r = self.get_distance()
        self.get_logger().info('Start r: {0}'.format(self.r))
        self.old = [0, 0]

        # return x,y during each cycle
        # return final position
        feedback_msg = MoveTo.Feedback()

        while self.r > 0.2:

            self.angular_correction()
            self.linear_correction()

            # give feedback
            feedback_msg.x_curr = self.x
            feedback_msg.y_curr = self.y
            goal_handle.publish_feedback(feedback_msg)

            # update while condition
            self.r = self.get_distance()

        # stop motors
        self.sm.set_motor(3, 0)  # right
        self.sm.set_motor(4, 0)  # left

        goal_handle.succeed()

        result = MoveTo.Result()
        result.x_final = self.x
        result.y_final = self.y

        self.angle_diff = 2 * math.pi

        self.get_logger().info('Finished Move To')

        return result

    def linear_smooth(self, d):
        if d > 1:
            return 1
        elif d < .6:
            return .6
        else:
            return d

    def angular_smooth(self, a):
        return .7
        #y = a / math.pi
        #if y < .5:
        #    y = .5
        #return y

    def linear_correction(self):
        self.r = self.get_distance()
        speed = self.linear_smooth(self.r)
        print("Speed: {}".format(speed))
        L = .3*self.old[0] + .7*speed
        R = .3*self.old[1] + .7*speed
        self.sm.set_motor(4, R)  # left
        self.sm.set_motor(3, L)  # right

        L = self.r/4
        time.sleep(L)
        self.old = [L, R]

    def angular_correction(self):
        while abs(self.angle_diff) > math.pi / 45:
            self.calculate_closest_turn()
            spin = self.angular_smooth(abs(self.angle_diff))
            print("Spin: {}".format(spin))
            if self.angle_diff < 0:
                R = -1*spin
                L = spin

            else:
                R = spin
                L = -1*spin

            R = .2*self.old[0] + .8*R
            L = .2*self.old[1] + .8*L
            self.sm.set_motor(4, R)
            self.sm.set_motor(3, L)

            self.old = [L, R]


def main(args=None):
    rclpy.init(args=args)

    try:
        server = Bang()
        executor = MultiThreadedExecutor()
        executor.add_node(server)

        try:
            executor.spin()

        finally:
            executor.shutdown()
            server.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
