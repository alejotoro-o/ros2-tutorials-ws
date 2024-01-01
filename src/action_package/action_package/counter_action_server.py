import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from interfaces_tutorial.action import Counter


class CounterActionServer(Node):

    def __init__(self):

        super().__init__('counter_action_server')
        self._action_server = ActionServer(self, Counter, 'counter', self.execute_callback)

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')

        feedback_msg = Counter.Feedback()
        feedback_msg.partial_sequence = []

        for i in range(0, goal_handle.request.range):

            feedback_msg.partial_sequence.append(i)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Counter.Result()
        result.sequence = feedback_msg.partial_sequence

        return result


def main(args=None):

    rclpy.init(args=args)

    fibonacci_action_server = CounterActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()