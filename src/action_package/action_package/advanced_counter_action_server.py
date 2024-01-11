import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from interfaces_tutorial.action import Counter


class CounterActionServer(Node):

    def __init__(self):

        super().__init__('counter_action_server')
        self._action_server = ActionServer(self, Counter, 'counter',
                                           self.execute_callback,
                                           cancel_callback=self.cancel_callback)
        self.result = Counter.Result()
        self.goal_handle = None # New

    async def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')

        # New
        if self.goal_handle:
            
            self.get_logger().info('New goal received, aborting previous goal')
            self.goal_handle.abort()

        self.goal_handle = goal_handle

        feedback_msg = Counter.Feedback()
        feedback_msg.partial_sequence = []

        for i in range(0, goal_handle.request.range):

            feedback_msg.partial_sequence.append(i)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)

            # New
            if self.goal_handle.is_cancel_requested:

                self.goal_handle.canceled()
                self.goal_handle = None
                self.result.sequence = []
                self.get_logger().info("Goal canceled")
                return self.result

            # New
            if self.goal_handle != goal_handle:

                self.result.sequence = []
                # self.get_logger().info('New goal received, aborting previous goal')
                return self.result

        self.goal_handle.succeed()

        self.get_logger().info("Goal finished")
        
        self.result.sequence = feedback_msg.partial_sequence

        self.goal_handle = None

        return self.result
    
    # New
    def cancel_callback(self, goal_handle):

        self.get_logger().info('Received cancel request')

        return CancelResponse.ACCEPT


def main(args=None):

    rclpy.init(args=args)

    counter_action_server = CounterActionServer()

    rclpy.spin(counter_action_server, executor=MultiThreadedExecutor())


if __name__ == '__main__':
    main()