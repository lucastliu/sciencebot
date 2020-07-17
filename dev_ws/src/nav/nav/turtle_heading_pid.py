import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tutorial_interfaces.action import Heading
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav.pid import PID


class PositionPID(Node):

    def __init__(self):
        super().__init__('imu_pid')
        
        self.group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            node=self,
            action_type=Heading,
            action_name='heading',
            execute_callback=self.move_to_callback,
            callback_group=self.group)

        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic='turtle1/pose',
            callback=self.pose_callback,
            qos_profile=4,
            callback_group=self.group)

        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)

        self.angle_diff = 50
        self.angle = 0.0
        self.dest_angle = 0.0

        self.twist = Twist()
        
        self.get_logger().info('Turtle Heading PID Node Live')
    def pose_callback(self, pose):
        #self.get_logger().info('Pose: %s' % (pose))  # CHANGE
        self.x = pose.x
        self.y = pose.y
        if pose.theta >= 0:
            self.angle = pose.theta
        else:
            self.angle = 2*math.pi + pose.theta

    def move_to_callback(self, goal_handle):
        self.get_logger().info('Executing Turn...')
        A = goal_handle.request.angular
        self.angle_pid = PID(kp=A[0], ki=A[1], kd=A[2])  # .05 .0008 0     .7 m/s movement
        self.dest_angle = math.radians(goal_handle.request.dest_angle)

        # return x,y during each cycle
        # return final position
        feedback_msg = Heading.Feedback()

        while abs(self.angle_diff) > math.pi / 180:

            # calculate
            self.angular_correction()

            # publish velocity updates
            self.publisher.publish(self.twist)
            
            # give feedback
            feedback_msg.curr = self.angle_diff
            goal_handle.publish_feedback(feedback_msg)
            #print(feedback_msg)

        #stop motors
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)
        
        goal_handle.succeed()

        result = Heading.Result()
        result.final = math.degrees(self.angle)
        self.angle_diff = 10
        self.get_logger().info('Finish Turn')

        return result

    def angular_correction(self):
        self.angle_diff = self.dest_angle - self.angle
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
