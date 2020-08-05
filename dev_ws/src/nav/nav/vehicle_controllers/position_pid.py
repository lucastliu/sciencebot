import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from custom_interfaces.action import MoveTo, Tune
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav.pid import PID

from nav.controller_base import ControllerBase


class PositionPID(ControllerBase):
    """
    Server that handles positioning requests.
    Uses 2 PID controllers, one for angular velocity,
    one for linear.

    Very difficult to find proper constants for PID.
    """
    def __init__(self):
        super().__init__('bang', Pose, 'pose', Tune, 'move_to')

        p = qos_profile_sensor_data
        p.depth = 1
        self.publisher = self.create_publisher(Twist, 'auto_vel', p)

        self.x_dest = 0.0
        self.y_dest = 0.0
        self.r = 99.99

        self.twist = Twist()

        self.get_logger().info('PID Node Live')

    def pose_callback(self, pose):
        self.x = pose.x
        self.y = pose.y
        self.angle = self.angle_convert(math.radians(pose.theta % 360.0))

    def action_callback(self, goal_handle):
        self.get_logger().info('Executing Move To...')
        L = goal_handle.request.linear
        A = goal_handle.request.angular
        self.angle_pid = PID(kp=A[0], ki=A[1], kd=A[2], imax=.3)  # Ex: 6 0 0
        self.distance_pid = PID(kp=L[0], ki=L[1], kd=L[2], imax=.3)  # Ex: .1 .005 0
        self.x_dest = goal_handle.request.x_dest
        self.y_dest = goal_handle.request.y_dest
        self.r = self.get_distance()
        self.get_logger().info('Start r: {0}'.format(self.r))

        # return x,y during each cycle
        # return final position
        feedback_msg = Tune.Feedback()

        while self.r > 0.4:

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
        self.get_logger().info('Finished Move To')
        return result

    def linear_correction(self):
        pid_dist = self.distance_pid.update(self.r)
        self.twist.linear.x = pid_dist

    def angular_correction(self):
        self.calculate_closest_turn()

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
