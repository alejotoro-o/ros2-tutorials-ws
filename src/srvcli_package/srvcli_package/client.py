import sys

import rclpy
from rclpy.node import Node

from interfaces_tutorial.srv import AddTwoInts

class ClientAsync(Node):

    def __init__(self):

        super().__init__('client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):

        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main():

    rclpy.init()

    client = ClientAsync()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    client.get_logger().info('Result of add_two_ints: for %d + %d = %d' % (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()