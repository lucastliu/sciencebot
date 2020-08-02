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
        super().__init__('b2')
        
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
        

        self.x_dest = 0.0
        self.y_dest = 0.0
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.r = 1.0
        self.angle_diff = 10
        self.old = [0.0, 0.0]
        self.curr = [0.0, 0.0]
        self.twist = Twist()
        self.T = 0.0
        
        self.get_logger().info('BB Node Live')
    def pose_callback(self, pose):
        #self.get_logger().info('Pose: %s' % (pose))  # CHANGE
        self.x = pose.x
        self.y = pose.y
        self.angle = self.angle_convert(math.radians(pose.theta % 360.0))

    def move_to_callback(self, goal_handle):
        self.get_logger().info('Executing Move To...')
        self.x_dest = goal_handle.request.x_dest
        self.y_dest = goal_handle.request.y_dest
        self.r = self.get_distance()
        self.get_logger().info('Start r: {0}'.format(self.r))
        self.old = [0, 0]
        self.curr = [0, 0]
        self.T = 0.0

        feedback_msg = Tune.Feedback()


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
            #print(feedback_msg)


        # stop motors
        self.sm.set_motor(3, 0)  # right
        self.sm.set_motor(4, 0)  # left

        # close out ROS action
        goal_handle.succeed()
        result = Tune.Result()
        result.x_final = self.x
        result.y_final = self.y
        self.get_logger().info('Finish Move To')


        return result

    def calculate_closest_turn(self):
        """
        Minimum angle needed to turn is never more than pi radians (180 degrees). 
        No need to turn the long way around.
        """
        
        a = self.angle - self.steering_angle()
        if a > math.pi:
            a -= 2*math.pi

        elif a < -1*math.pi:
            a += 2*math.pi
            
        self.angle_diff = a
        #print(self.angle_diff)
        
    def angle_convert(self, a):
        """
        Convert raw angle heading (radians) to proper co-ordinate system
        """
        if a > math.pi:
            a *= -1

        else:
            a = 2*math.pi - a
            
        return a
            
    def get_distance(self):
        return math.sqrt(
            math.pow(self.x_dest - self.x, 2)
            + math.pow(self.y_dest - self.y, 2)
            )
            
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
        self.old = self.curr # update old

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
        if abs(self.angle_diff) > (math.pi / 10): # focus on turning entirely
            print('ANGULAR')
            spin = .6
            if self.angle_diff > 0:
                self.curr = [spin, -1 * spin]
            else:
                self.curr = [-1 * spin, spin]
            
            self.T = .05
#             self.sm.set_motor(3, self.curr[0])  # left
#             self.sm.set_motor(4, self.curr[1])  # right
#             self.closest_turn()
        else:
            print('LINEAR') # -.1, -.4
            pull = self.angular_smooth(abs(self.angle_diff))
            if self.angle_diff > 0:
                #self.curr[0] -= .1
                self.curr[1] *= pull
            else:
                self.curr[0] *= pull
                #self.curr[1] -= .1
                    
            self.T = self.r / 4
            if self.T < .02:
                self.T = .02
        
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
