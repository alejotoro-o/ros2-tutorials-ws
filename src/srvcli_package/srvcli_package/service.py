import rclpy
from rclpy.node import Node

from interfaces_tutorial.srv import AddTwoInts


class Service(Node):

    def __init__(self):
        
        super().__init__('service')
        self.service = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):

        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():

    rclpy.init()

    service = Service()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()