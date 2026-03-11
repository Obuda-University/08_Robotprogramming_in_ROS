import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtlesimController(Node):
    """ROS2 node to control the Turtlesim robot using open-loop time-based commands."""
    def __init__(self) -> None:
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.rate_hz: int = 100
        self.get_logger().info("Turtlesim Controller Node has been started.")

    def _move_turtle(self, linear_vel: float, angular_vel: float, duration: float) -> None:
        """Internal Private method to publish velocity commands for a specific duration."""
        vel_msg = Twist()
        vel_msg.linear.x = float(linear_vel)
        vel_msg.angular.z = float(angular_vel)
        
        # Set loop rate
        loop_rate = self.create_rate(self.rate_hz, self.get_clock())
        
        # Publish first message and note the target stop time
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=duration)

        # Publish message until the calculated time is up
        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)
            
        # Ensure the turtle stops immediately after the loop
        self._stop_turtle()
        
    def _stop_turtle(self) -> None:
        """Publishes a zero-velocity Twist message to stop the robot."""
        vel_msg = Twist()
        self.twist_pub.publish(vel_msg)

    def go_straight(self, speed: float, distance: float) -> None:
        """Moves the turtle straight."""
        self.get_logger().info(f'Moving straight for {distance} units at speed {speed}.')
        # Maintain direction based on distance sign
        actual_speed: float = speed if distance > 0 else -speed
        duration: float = abs(distance / speed)
        
        self._move_turtle(actual_speed, 0.0, duration)

    def turn(self, omega: float, angle: float) -> None:
        "Turns the turtle in place."
        self.get_logger().info(f'Turning {angle:.2f} radians at angular speed {omega}.')
        # Maintain rotational direction based on angle sign
        actual_omega: float = omega if angle > 0 else -omega
        duration: float = abs(angle / omega)
        
        self._move_turtle(0.0, actual_omega, duration)
    
    def draw_square(self, speed: float, omega: float, a: float) -> None:
        """Draws a square with side length 'a'."""
        self.get_logger().info(f'Drawing a square with side length {a}.')
        # A square has 4 sides ad 90-degree (pi/2) turns
        for _ in range(4):
            self.go_straight(speed, a)
            self.turn(omega, math.pi / 2.0)
    
    def draw_poly(self, speed: float, omega: float, N: int, a: float) -> None:
        """Draws a regular polygon with 'N' sides of length 'a'."""
        if N < 3:
            self.get_logger().warn('A polygon must have at least 3 sides.')
            return

        self.get_logger().info(f'Drawing a {N}-sided plygon with side length {a}.')
        # The exterior turn angle for a regular polygon is 360 degrees / N
        turn_angle: float = (2.0 * math.pi) / N
        for _ in range(N):
            self.go_straight(speed, a)
            self.turn(omega, turn_angle)
        

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()

    try:
        # 1. Test basic movement
        tc.go_straight(1.0, 2.0)
        tc.turn(1.0, math.pi / 2.0)
        
        # 2. Test the composite shape functions
        tc.draw_square(2.0, 1.5, 3.0)
        tc.draw_poly(2.0, 1.5, 6, 2.0)  # Hexagon
        
    except KeyboardInterrupt:
        tc.get_logger().info("Execution interrupted by user (CTRL+C).")
    finally:
        # Cleanly shut down node and rclpy
        tc._stop_turtle()
        tc.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
