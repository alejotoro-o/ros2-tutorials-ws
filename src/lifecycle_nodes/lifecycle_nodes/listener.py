import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):

        super().__init__('listener')

        self.create_subscription(String, "msg_topic", self.subscription_callback, 10)

    def subscription_callback(self, msg):

        self.get_logger().info(msg.data)

def main(args=None):

    rclpy.init(args=args)

    listener_node = Listener()

    rclpy.spin(listener_node)

    listener_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()