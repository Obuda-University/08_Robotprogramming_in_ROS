import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker_node = Talker()

    try:
        rclpy.spin(talker_node)
    except KeyboardInterrupt:
        talker_node.get_logger().info("Execution interrupted by user (CTRL+C).")
    finally:
        # Cleanly shut down node and rclpy
        talker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    