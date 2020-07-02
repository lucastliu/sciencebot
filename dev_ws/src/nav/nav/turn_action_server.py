import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from tutorial_interfaces.action import Turn
from geometry_msgs.msg import Twist


class TurnActionServer(Node):

    def __init__(self):
        super().__init__('turn_action_server')
        self._action_server = ActionServer(
            self,
            Turn,
            'turn',
            self.execute_callback)
        
        self.publisher = self.create_publisher(Twist, 'auto_vel', 10)     # CHANGE
            

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Turn.Feedback()
        feedback_msg.turns_completed = 0

        for i in range(0, goal_handle.request.turns):
            twist = Twist()
            twist.linear.x = 1.0
            twist.angular.z = -0.75
            self.publisher.publish(twist)
            time.sleep(2)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            time.sleep(0.5)
            
            feedback_msg.turns_completed = i + 1
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.turns_completed))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Turn.Result()
        result.turns_done = feedback_msg.turns_completed
        return result
        
def main(args=None):
    rclpy.init(args=args)

    turn_action_server = TurnActionServer()

    rclpy.spin(turn_action_server)


if __name__ == '__main__':
    main()
