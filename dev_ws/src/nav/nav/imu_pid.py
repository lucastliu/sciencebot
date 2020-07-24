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

class PositionPID(Node):

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
        
        p = qos_profile_sensor_data
        p.depth = 1
        self.publisher = self.create_publisher(Twist, 'auto_vel', qos_profile_sensor_data)
        self.pid_publisher = self.create_publisher(Cpid, 'hpid', 1) 

        self.angle_diff = 50
        self.angle = 0.0
        self.dest_angle = 0.0

        self.twist = Twist()
        self.cpid = Cpid()
        
        self.get_logger().info('Heading PID Node Live')
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
        A = goal_handle.request.angular
        self.angle_pid = PID(kp=A[0], ki=A[1], kd=A[2], imax=.3)  # .1 .005 0     .7 m/s movement       [.1 0 11 100 degree, one overshoot (max .9), depth=5]
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

            # calculate
            self.angular_correction()
            
            self.cpid.prop = self.angle_pid.P
            self.cpid.intg = self.angle_pid.I
            self.cpid.derv = self.angle_pid.D
            
            # publish velocity updates
            self.publisher.publish(self.twist)
            self.pid_publisher.publish(self.cpid)
            
            # give feedback
            feedback_msg.curr = math.degrees(self.angle) #self.angle_diff
            goal_handle.publish_feedback(feedback_msg)
            #print(feedback_msg)

        #stop motors
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)
        self.angle_diff = self.angle - self.dest_angle
        
        goal_handle.succeed()

        result = Heading.Result()
        result.final = math.degrees(self.angle)
        self.angle_diff = 10
        self.get_logger().info('Finish Turn')

        return result

    def angular_correction(self):
        self.angle_diff = self.angle - self.dest_angle
        
        if self.angle_diff > math.pi:
            self.angle_diff = self.angle_diff - 2*math.pi

        if self.angle_diff < -1*math.pi:
            self.angle_diff = self.angle_diff + 2*math.pi
            
        pid_angle = self.angle_pid.update(self.angle_diff)
        print("self diff: {0}".format(self.angle_diff))
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
