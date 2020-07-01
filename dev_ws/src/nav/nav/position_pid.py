import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from tutorial_interfaces.action import MoveTo
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav import pid.PID


class PositionPID(Node):

    def __init__(self):
        super().__init__('position_pid')

        self._action_server = ActionServer(
            self,
            MoveTo,
            'move_to',
            self.move_to_callback)

        self.subscription = self.create_subscription(
            Pose,
            'pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Twist, 'auto_vel', 10)

        self.angle_pid = PID()
        self.distance_pid = PID()

        self.x_dest = 0.0
        self.y_dest = 0.0
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.r = 999.99

        self.twist = Twist()

    def pose_callback(self, pose):
        self.get_logger().info('Pose: %s' % (pose))  # CHANGE
        self.x = pose.x
        self.y = pose.y
        self.angle = pose.theta

    def move_to_callback(self, goal_handle):
        self.get_logger().info('Executing Move To...')
        self.x_dest = goal_handle.request.x_dest
        self.y_dest = goal_handle.request.y_dest
        self.r = self.get_distance()

        # return x,y during each cycle
        # return final position
        feedback_msg = MoveTo.Feedback()

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

        goal_handle.succeed()

        result = MoveTo.Result()
        result.x_final = self.x
        result.y_final = self.y
        return result

    def get_distance(self):
        return math.sqrt(
            math.pow(self.x_dest - self.x, 2)
            + math.pow(self.y_dest - self.y, 2)
            )

    def get_angle(self):
        r_x = self.r * math.cos(self.angle)
        r_y = self.r * math.sin(self.angle)

        xim = self.x + r_x
        yim = self.y + r_y

        c = math.sqrt(
            math.pow(self.x_dest - xim, 2)
            + math.pow(self.y_dest - yim, 2)
            )

        if xim > self.x_dest:
            alpha = math.acos(
                (2 * math.pow(self.r, 2) - math.pow(c, 2))
                / (2 * math.pow(self.r, 2))
                )
        else:
            alpha = 2 * math.pi * math.acos(
                (2*math.pow(self.r, 2)
                 - math.pow(c, 2)) / (2 * math.pow(self.r, 2))
                )

        return alpha

# may need trigger stops in these 2 methods 
    def linear_correction(self):
        pid_dist = self.distance_pid.update(self.r)
        self.twist.linear.x = pid_dist

    def angular_correction(self):
        angle = self.get_angle()
        pid_angle = self.angle_pid.update(angle)
        self.twist.angular.z = pid_angle


def main(args=None):
    rclpy.init(args=args)

    server = PositionPID()

    rclpy.spin(server)


if __name__ == '__main__':
    main()
