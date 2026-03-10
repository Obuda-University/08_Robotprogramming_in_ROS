import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class TurtlesimController(Node):
    def __init__(self) -> None:
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def go_straight(self, speed, distance):
        # Create and publish msg
        vel_msg = Twist()
        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        # Set loop rate
        loop_rate = self.create_rate(100, self.get_clock()) # Hz

        # Calculate time
        T = abs(distance / speed)

        # Publish first msg and note time when to stop
        self.twist_pub.publish(vel_msg)
        # self.get_logger().info('Turtle started.')
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        # Publish msg while the calculated time is up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            # self.get_logger().info('On its way...')
            rclpy.spin_once(self)   # loop rate

        # turtle arrived, set velocity to 0
        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)
        # self.get_logger().info('Arrived to destination.')

    def turn(self, omega, angle):
        pass
    
    def draw_square(self, speed, omega, a):
        pass
    
    def draw_poly(self, speed, omega, N, a):
        pass

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    tc.go_straight(1.0, 2.0)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
