import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import MoveTo, Tune
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from nav.pid import PID
from nav.controller_base import ControllerBase


class PositionPID(ControllerBase):
    """
    Designed to operate with ROS turtlesim.
    Moves turtle to desired heading using
    a linear PID controller, and
    an angular PID controller.
    """
    def __init__(self):
        super().__init__('turtle_position_pid', Pose, 'turtle1/pose', Tune, 'move_to')

        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.r = 99.99
        self.twist = Twist()

        self.get_logger().info('Turtle PID Control Live')

    def pose_callback(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.angle = pose.theta

    def action_callback(self, goal_handle):
        self.get_logger().info('Executing Move To...')
        L = goal_handle.request.linear
        A = goal_handle.request.angular
        self.angle_pid = PID(kp=A[0], ki=A[1], kd=A[2])  # Ex vals: 6 0 0
        self.distance_pid = PID(kp=L[0], ki=L[1], kd=L[2])  # Ex vals: 1.5 0 0
        self.x_dest = goal_handle.request.x_dest
        self.y_dest = goal_handle.request.y_dest
        self.r = self.get_distance()
        self.angle_diff = 2 * math.pi
        self.get_logger().info('Start r: {0}'.format(self.r))

        # return x,y during each cycle
        # return final position
        feedback_msg = Tune.Feedback()

        while self.r > 0.1:

            # calculate
            self.linear_correction()
            self.angular_correction()

            # publish velocity updates
            self.publisher.publish(self.twist)

            # give feedback
            feedback_msg.x_curr = self.x
            feedback_msg.y_curr = self.y
            goal_handle.publish_feedback(feedback_msg)

            # update while condition
            self.r = self.get_distance()

        # stop motors
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

        goal_handle.succeed()

        result = Tune.Result()
        result.x_final = self.x
        result.y_final = self.y

        self.get_logger().info('Finish Move To')
        return result

    def linear_correction(self):
        pid_dist = self.distance_pid.update(self.r)
        self.twist.linear.x = pid_dist

    def angular_correction(self):
        self.angle_diff = self.steering_angle() - self.angle

        if self.angle_diff > math.pi:
            self.angle_diff = self.angle_diff - 2*math.pi

        if self.angle_diff < -1*math.pi:
            self.angle_diff = self.angle_diff + 2*math.pi

        pid_angle = self.angle_pid.update(self.angle_diff)
        self.twist.angular.z = pid_angle


def main(args=None):
    rclpy.init(args=args)

    try:
        server = PositionPID()
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
