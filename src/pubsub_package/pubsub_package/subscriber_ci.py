import rclpy
from rclpy.node import Node

from interfaces_tutorial.msg import Num # Original: from std_msgs.msg import String

class SubscriberCi(Node):

    def __init__(self):
        super().__init__('subscriber')

        self.subscription = self.create_subscription(Num, 'topic', self.listener_callback, 10) # Original: String

        self.subscription

    def listener_callback(self, msg):

        self.get_logger().info('I heard: "%d"' % msg.num) # Original: msg.data

def main(args=None):

    rclpy.init(args=args)

    subscriber = SubscriberCi()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()