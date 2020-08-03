import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from custom_interfaces.action import Heading


class PositionPIDClient(Node):
    """
    Client script for a heading action. Requests a desired heading in degrees.
    Receives current heading feedback during action,
    as well as final angle after action is completed.

    Action request must be processed by a heading server
    """
    def __init__(self):
        super().__init__('heading_pid_client')
        self._action_client = ActionClient(self, Heading, 'heading')

    def send_goal(self, dest_angle, angular):
        goal_msg = Heading.Goal()
        goal_msg.dest_angle = dest_angle
        goal_msg.angular = angular
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Final Position: {0}'
                               .format(result.final))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: {0}'
                               .format(feedback.curr))


def main(args=None):
    rclpy.init(args=args)

    action_client = PositionPIDClient()
    dest_angle = float(input("Desired Angle: "))
    angular = [float(item) for item in input("Enter Angular PID Constants : ").split()] 
    action_client.send_goal(dest_angle, angular)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
