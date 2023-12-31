import rclpy
from rclpy.node import Node

from interfaces_tutorial.msg import Num # Original: from std_msgs.msg import String

class PublisherCi(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Num, 'topic', 10) # Original: self.publisher = self.create_publisher(String, 'topic', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):

        msg = Num() # Original: msg = String()

        msg.num = self.i # Original: msg.data = 'Hello World: %d' % self.i

        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num) # Original: msg.data

        self.i += 1

def main(args=None):

    rclpy.init(args=args)

    publisher = PublisherCi()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()