import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from tutorial_interfaces.action import MoveTo


class PositionPIDClient(Node):

    def __init__(self):
        super().__init__('position_pid_client')
        self._action_client = ActionClient(self, MoveTo, 'move_to')

    def send_goal(self, x_dest, y_dest):
        goal_msg = MoveTo.Goal()
        goal_msg.x_dest = x_dest
        goal_msg.y_dest = y_dest

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
        self.get_logger().info('Final Positon: {0},  {1}'
                               .format(result.x_final, result.y_final))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback: {0},  {1}'
                               .format(result.x_curr, result.y_curr))


def main(args=None):
    rclpy.init(args=args)

    action_client = PositionPIDClient()

    action_client.send_goal(1.8, 1.8)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
