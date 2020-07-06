import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tutorial_interfaces.action import MoveTo
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav.pid import PID


class PositionPID(Node):

    def __init__(self):
        super().__init__('position_pid')
        
        self.group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            node=self,
            action_type=MoveTo,
            action_name='move_to',
            execute_callback=self.move_to_callback,
            callback_group=self.group)

        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic='pose',
            callback=self.pose_callback,
            qos_profile=10,
            callback_group=self.group)

        self.publisher = self.create_publisher(Twist, 'auto_vel', 10)
        

        self.angle_pid = PID(kp=1.2, ki=0, kd=0)
        self.distance_pid = PID(kp=1.2, ki=0, kd=0)

        self.x_dest = 0.0
        self.y_dest = 0.0
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.r = 999.99

        self.twist = Twist()

    def pose_callback(self, pose):
        #self.get_logger().info('Pose: %s' % (pose))  # CHANGE
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

        while self.r > 0.2:

            # calculate
            self.linear_correction()
            self.angular_correction()

            # publish velocity updates
            self.publisher.publish(self.twist)
            
            # give feedback
            feedback_msg.x_curr = self.x
            feedback_msg.y_curr = self.y
            goal_handle.publish_feedback(feedback_msg)
            #print(feedback_msg)

            # update while condition
            self.r = self.get_distance()
            
            # yield thread
            time.sleep(0.0)

        goal_handle.succeed()

        result = MoveTo.Result()
        result.x_final = self.x
        result.y_final = self.y
        
        #stop motors
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

        return result

    def get_distance(self):
        return math.sqrt(
            math.pow(self.x_dest - self.x, 2)
            + math.pow(self.y_dest - self.y, 2)
            )

    def get_angle(self):
        r_x = self.r * math.cos(math.radians(self.angle))
        r_y = self.r * math.sin(math.radians(self.angle))

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
