import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import Heading
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from nav.pid import PID
from nav.controller_base import ControllerBase


class TurtleTurnPID(ControllerBase):
    """
    Designed to operate with ROS turtlesim.
    Rotates turtle to desired heading using
    angular PID controller.
    """
    def __init__(self):
        super().__init__('turtle_turn_pid', Pose, 'turtle1/pose', Heading, 'heading')

        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        self.twist = Twist()

        self.get_logger().info('Turtle Heading PID Node Live')

    def pose_callback(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.angle = pose.theta

    def action_callback(self, goal_handle):
        self.get_logger().info('Executing Turn...')
        A = goal_handle.request.angular
        self.angle_pid = PID(kp=A[0], ki=A[1], kd=A[2])
        self.dest_angle = math.radians(goal_handle.request.dest_angle % 360)
        self.dest_angle = self.angle_convert(self.dest_angle)

        feedback_msg = Heading.Feedback()

        while abs(self.angle_diff) > math.pi / 180:

            # calculate
            self.angular_correction()

            # publish velocity updates
            self.publisher.publish(self.twist)

            # give feedback
            feedback_msg.curr = self.angle_diff
            goal_handle.publish_feedback(feedback_msg)

        # stop motors
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

        goal_handle.succeed()

        result = Heading.Result()
        result.final = math.degrees(self.angle)
        self.angle_diff = 2 * math.pi
        self.get_logger().info('Finished Turn')

        return result

    def angular_correction(self):
        self.calculate_closest_turn
        pid_angle = self.angle_pid.update(self.angle_diff)
        self.twist.angular.z = pid_angle


def main(args=None):
    rclpy.init(args=args)

    try:
        server = TurtleTurnPID()
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
