import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import Heading
from custom_interfaces.msg import Cpid
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav.pid import PID

from rclpy.qos import qos_profile_sensor_data
from nav.motors.SerialMotor import SerialMotor

class Bturn(Node):

    def __init__(self):
        super().__init__('imu_pid')
        
        self.group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            node=self,
            action_type=Heading,
            action_name='heading',
            execute_callback=self.move_to_callback,
            callback_group=self.group)

        self.subscription = self.create_subscription(
            msg_type=Pose,
            topic='pose',
            callback=self.pose_callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.group)

        self.angle_diff = 50
        self.angle = 0.0
        self.dest_angle = 0.0
        self.sm = SerialMotor("/dev/ttyACM1")

        
        self.get_logger().info('Bturn Node Live')
    def pose_callback(self, pose):
        #self.get_logger().info('Pose: %s' % (pose))  # CHANGE
        self.x = pose.x
        self.y = pose.y
        temp = math.radians(pose.theta % 360)
      
        
        if temp < math.pi:
            temp = -1*temp
        else:
            temp = 2*math.pi - temp
            
        self.angle = temp
            
        #print("theta: {0} angle: {1}".format(pose.theta, self.angle))

    def move_to_callback(self, goal_handle):
        self.get_logger().info('Executing Turn...')
        temp = math.radians(goal_handle.request.dest_angle % 360)
        if temp < math.pi:
            temp = -1*temp
        else:
            temp = 2*math.pi - temp
        
        self.dest_angle = temp

        # return x,y during each cycle
        # return final position
        feedback_msg = Heading.Feedback()
        while abs(self.angle_diff) > math.pi / 45:
            self.angle_diff = self.angle - self.dest_angle
            
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
            
        # give feedback
        feedback_msg.curr = math.degrees(self.angle_diff) #self.angle_diff
        goal_handle.publish_feedback(feedback_msg)
        #print(feedback_msg)
        
        goal_handle.succeed()

        result = Heading.Result()
        result.final = math.degrees(self.angle)
        self.angle_diff = 10
        self.get_logger().info('Finish Turn')

        return result



def main(args=None):
    rclpy.init(args=args)
    
    try:
        server = Bturn()
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
