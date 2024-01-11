import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from interfaces_tutorial.action import Counter


class CounterActionClient(Node):

    def __init__(self):

        super().__init__('counter_action_client')
        self._action_client = ActionClient(self, Counter, 'counter')
        self.goal_handle = None

    def send_goal(self, goal):

        goal_msg = Counter.Goal()
        goal_msg.range = goal

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    def cancel_goal(self):

        if self.goal_handle:
            self.get_logger().info('Canceling goal')
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')


def control_loop(node):

    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    while 1:

        goal = input()

        if goal == 'c':
            node.cancel_goal()
            continue
        elif goal == 'q':
            break

        node.send_goal(int(goal))


def main(args=None):

    rclpy.init(args=args)

    action_client = CounterActionClient()

    control_loop(action_client)

    rclpy.shutdown()


if __name__ == '__main__':
    main()