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

class Bang(Node):

    def __init__(self):
        super().__init__('position_pid')
        
        self.group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            node=self,
            action_type=Tune,
            action_name='move_to',
            execute_callback=self.move_to_callback,
            callback_group=self.group)

        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic='pose',
            callback=self.pose_callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.group)

        self.sm = SerialMotor("/dev/ttyACM1")
        self.get_logger().info('Motors Node Live')
        

        self.x_dest = 0.0
        self.y_dest = 0.0
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.r = 1.0
        self.angle_diff = 10

        self.twist = Twist()
        
        self.get_logger().info('PID Node Live')
    def pose_callback(self, pose):
        #self.get_logger().info('Pose: %s' % (pose))  # CHANGE
        self.x = pose.x
        self.y = pose.y
        temp = math.radians(pose.theta % 360.0) #carefully consider trig options
        
        if temp < math.pi:
            temp = -1*temp
        else:
            temp = 2*math.pi - temp
            
        self.angle = temp

    def move_to_callback(self, goal_handle):
        self.get_logger().info('Executing Move To...')
        self.x_dest = goal_handle.request.x_dest
        self.y_dest = goal_handle.request.y_dest
        self.r = self.get_distance()
        self.get_logger().info('Start r: {0}'.format(self.r))

        # return x,y during each cycle
        # return final position
        feedback_msg = Tune.Feedback()

        while self.r > 0.4:

            # calculate

            self.angular_correction()
            self.linear_correction()
            
            # give feedback
            feedback_msg.x_curr = self.x
            feedback_msg.y_curr = self.y
            goal_handle.publish_feedback(feedback_msg)
            #print(feedback_msg)

            # update while condition
            self.r = self.get_distance()

        #stop motors
        self.sm.set_motor(3, 0)  # right
        self.sm.set_motor(4, 0)  # left

        
        goal_handle.succeed()

        result = Tune.Result()
        result.x_final = self.x
        result.y_final = self.y
        

        
        self.get_logger().info('Finish Move To')


        return result

    def get_distance(self):
        return math.sqrt(
            math.pow(self.x_dest - self.x, 2)
            + math.pow(self.y_dest - self.y, 2)
            )

    def linear_correction(self):
        self.r = self.get_distance()
        self.sm.set_motor(3, .6)  # right
        self.sm.set_motor(4, .6)  # left
        L = self.r
        time.sleep(L)
        
        self.sm.set_motor(3, 0)  # right
        self.sm.set_motor(4, 0)  # left
        
        

    def angular_correction(self):
        while abs(self.angle_diff) > math.pi / 45:
            self.angle_diff = self.angle - self.steering_angle()
            
            if self.angle_diff > math.pi:
                self.angle_diff = self.angle_diff - 2*math.pi

            if self.angle_diff < -1*math.pi:
                self.angle_diff = self.angle_diff + 2*math.pi
            
            if self.angle_diff > 0:
                self.sm.set_motor(3, -.5)  # right
                self.sm.set_motor(4,.5)  # left
            else:
                self.sm.set_motor(3, .5)  # right
                self.sm.set_motor(4, -.5)  # left
                
        self.sm.set_motor(3, 0)  # right
        self.sm.set_motor(4, 0)  # left
        
    def steering_angle(self):
        return math.atan2(self.y_dest - self.y, self.x_dest - self.x)


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
