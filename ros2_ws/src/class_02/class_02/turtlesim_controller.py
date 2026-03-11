import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtlesimController(Node):
    """ROS2 node to control the Turtlesim robot using open-loop time-based commands."""
    def __init__(self) -> None:
        super().__init__('turtlesim_controller')
        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.rate_hz: int = 100
        self.get_logger().info("Turtlesim Controller Node has been started. Using open-loop.")

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
        

class ProportionalController(Node):
    def __init__(self) -> None:
        super().__init__('closed_loop_controller')
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Subscriber for pose feedback
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # State variable to store the current pose
        self.current_pose: Pose | None = None
        self.rate_hz: int = 100
        self.get_logger().info("Turtlesim Controller Node has been started. Using closed-loop.\nWaiting for data...")

    def pose_callback(self, message: Pose) -> None:
        """Callback function to update the robot's current pose."""
        self.current_pose = message
        
    def wait_for_pose(self) -> None:
        """Helper to ensure we have pose data before calculating movements."""
        while self.current_pose is None and rclpy.ok():
            rclpy.spin_once(self)
        self.get_logger().info("Pose data received.")

    def get_distance(self, goal_x: float, goal_y: float) -> float:
        """Calculates the Euclidean distance from current pose to goal."""
        return math.sqrt(
            (goal_x - self.current_pose.x)**2 +
            (goal_y - self.current_pose.y)**2
        )
        
    def get_steering_angle(self, goal_x: float, goal_y: float) -> float:
        """Calculates the angle from the current pose to the goal."""
        return math.atan2(
            goal_y - self.current_pose.y,
            goal_x - self.current_pose.x
        )
        
    def go_to_goal(self, goal_x: float, goal_y: float, distance_tolerance: float = 0.1) -> None:
        """Moves the turtle to a specific (x, y) coordinate using P-Control."""
        self.wait_for_pose()
        self.get_logger().info(f"Navigating to: ({goal_x}, {goal_y})")
        
        vel_msg = Twist()
        
        # Proportional controller gains
        K_lienar: float = 1.5
        K_angular: float = 4.0
        
        loop_rate = self.create_rate(self.rate_hz, self.get_clock())
        
        # Keep calculating and publishing velocities until we are close enough
        while self.get_distance(goal_x, goal_y) >= distance_tolerance and rclpy.ok():
            # 1. Proportional Linear Velocity
            vel_msg.linear.x = K_lienar * self.get_distance(goal_x, goal_y)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            
            # 2. Proportional Angular Velocity
            target_angle = self.get_steering_angle(goal_x, goal_y)
            angle_error = target_angle - self.current_pose.theta
            
            # Normalize angle error to be between -pi and pi
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = K_angular * angle_error
            
            # 3. Publish the velocity
            self.cmd_pub.publish(vel_msg)
            
            rclpy.spin_once(self)  # Spin to get incoming pose messages
        
        # 4. Stop the robot once it reaches the target
        self.get_logger().info("Yay! Goal reached.")
        self.stop_turtle()
        
    def stop_turtle(self) -> None:
        """Publishes a zero-velocity Twist message to stop the robot."""
        vel_msg = Twist()
        self.cmd_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()
    pc = ProportionalController()

    try:
        # 1. Test basic movement
        #tc.go_straight(1.0, 2.0)
        #tc.turn(1.0, math.pi / 2.0)
        
        # 2. Test the composite shape functions
        #tc.draw_square(2.0, 1.5, 3.0)
        #tc.draw_poly(2.0, 1.5, 6, 2.0)  # Hexagon
        
        # 3. Test with coordinates on the map
        pc.go_to_goal(8.0, 8.0)
        pc.go_to_goal(2.0, 8.0)
        pc.go_to_goal(5.5, 5.5) # Center
        
    except KeyboardInterrupt:
        tc.get_logger().info("Execution interrupted by user (CTRL+C).")
    finally:
        # Cleanly shut down node and rclpy
        #tc._stop_turtle()
        pc.stop_turtle()
        #tc.destroy_node()
        pc.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
