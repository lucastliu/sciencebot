import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import MoveTo, Tune
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from rclpy.qos import qos_profile_sensor_data
from nav.motors.SerialMotor import SerialMotor

from nav.controller_base import ControllerBase


class Mixed(ControllerBase):
    """
    Controller implementation with
    completely mixed angular and linear control
    """
    def __init__(self):
        super().__init__('mixed', Pose, 'pose', MoveTo, 'move_to')

        self.sm = SerialMotor("/dev/ttyACM1")

        self.x_dest = 0.0
        self.y_dest = 0.0
        self.r = 99.99
        self.old = [0.0, 0.0]
        self.curr = [0.0, 0.0]
        self.twist = Twist()
        self.T = 0.0

        self.get_logger().info('Mixed Controller Node Live')

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
        self.curr = [0, 0]
        self.T = 0.0

        feedback_msg = MoveTo.Feedback()

        while self.r > 0.15:

            # calculate errors
            self.r = self.get_distance()
            # make adjustments to motors instructions
            self.linear_correction()

            self.calculate_closest_turn()
            self.angular_correction()

            # send new commands to motors
            self.move()

            # give feedback
            feedback_msg.x_curr = self.x
            feedback_msg.y_curr = self.y
            goal_handle.publish_feedback(feedback_msg)

        # stop motors
        self.sm.set_motor(3, 0)
        self.sm.set_motor(4, 0)

        # close out ROS action
        goal_handle.succeed()
        result = MoveTo.Result()
        result.x_final = self.x
        result.y_final = self.y
        self.get_logger().info('Finished Move To')

        return result

    def move(self):
        self.smoothing()
        print("Move Speed: {}".format(str(self.curr)))
        if(abs(self.curr[0]) > 1 or abs(self.curr[1]) > 1):
            print('bad motor limit')
            print(str(self.curr))
            return
        self.sm.set_motor(3, self.curr[0])  # left
        self.sm.set_motor(4, self.curr[1])  # right
        time.sleep(self.T)

    def smoothing(self):
        self.curr[0] = .15*self.old[0] + .85*self.curr[0]
        self.curr[1] = .15*self.old[1] + .85*self.curr[1]
        self.old = self.curr  # update old

    def linear_smooth(self, d):
        d *= .7

        if d > .7:
            return .7
        elif d < .1:
            return .1
        else:
            return d

    def angular_smooth(self, a):
        y = 1 - (a / (math.pi*20))
        if y > .7:
            return .7
        return y

    def linear_correction(self):
        self.r = self.get_distance()
        speed = self.linear_smooth(self.r)
        print("Initial Speed: {}".format(speed))
        self.curr = [speed, speed]

    def angular_correction(self):
        if abs(self.angle_diff) > (math.pi / 10):  # focus on turning entirely
            print('ANGULAR')
            spin = .6
            if self.angle_diff < 0:
                self.curr = [spin, -1 * spin]
            else:
                self.curr = [-1 * spin, spin]

            self.T = .05

        else:
            print('LINEAR')
            pull = self.angular_smooth(abs(self.angle_diff))
            if self.angle_diff > 0:
                self.curr[1] *= pull
            else:
                self.curr[0] *= pull

            # set duration of instruction
            self.T = self.r / 4
            if self.T < .02:
                self.T = .02


def main(args=None):
    rclpy.init(args=args)

    try:
        server = Mixed()
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
