import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard msg: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    listener_node = Listener()
    
    try:
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        listener_node.get_logger().info("Execution interrupted by user (CTRL+C).")
    finally:
        # Cleanly shut down node and rclpy
        listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
