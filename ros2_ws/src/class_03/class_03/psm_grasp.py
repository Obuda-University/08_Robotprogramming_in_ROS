import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from matplotlib import pyplot as plt


class PSM(Node):
    def __init__(self):
        super().__init__('psm_grasp')
        self.servo_cp_pub = self.create_publisher(
            PoseStamped,
            '/PSM1/servo_cp',
            10)

        self.servo_jaw_pub = self.create_publisher(
            JointState,
            '/PSM1/jaw/servo_jp',
            10)

        # TCP pose
        self.measured_cp = None
        self.measured_cp_sub = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_measured_cp,
            10)

        # Jaw
        self.measured_jaw = None
        self.measured_jaw_sub = self.create_subscription(
            JointState,
            '/PSM1/jaw/measured_js',
            self.cb_measured_jaw,
            10)

        # Marker
        self.marker = None
        self.marker_sub = self.create_subscription(
            Marker,
            '/dummy_target_marker',
            self.cb_marker,
            10)

    # Callback for Marker
    def cb_marker(self, msg):
        self.marker = msg
        #self.get_logger().info(msg)

    # Callback for TCP pose
    def cb_measured_cp(self, msg):
        self.measured_cp = msg
        #self.get_logger().info(msg)

    # Callback for jaws angle
    def cb_measured_jaw(self, msg):
        self.measured_jaw = msg
        #self.get_logger().info(msg)

    def move_tcp_to(self, target, v, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        tcp_curr = np.array([self.measured_cp.pose.position.x,
                             self.measured_cp.pose.position.y,
                             self.measured_cp.pose.position.z])
        tcp_des = np.array(target)

        distance = np.linalg.norm(tcp_des - tcp_curr)
        T = distance / v
        N = int(math.floor(T / dt))

        tr_x = np.linspace(tcp_curr[0], tcp_des[0], N)
        tr_y = np.linspace(tcp_curr[1], tcp_des[1], N)
        tr_z = np.linspace(tcp_curr[2], tcp_des[2], N)



        # Plotting
        #tr_t = np.linspace(0, T, N)
        #plt.plot(tr_t, tr_x)
        #plt.plot(tr_t, tr_y)
        #plt.plot(tr_t, tr_z)
        #plt.show()
        #fig = plt.figure(figsize=(8,6))
        #ax = fig.add_subplot(111, projection='3d')
        #ax.scatter(tr_x, tr_y, tr_z,  color='red', marker='o')
        #ax.set_xlabel('X')
        #ax.set_ylabel('Y')
        #ax.set_zlabel('Z')
        #plt.show()

        self.get_logger().info('Starting navigation...')

        loop_rate = self.create_rate(1.0 / dt, self.get_clock()) # Hz
        for i in range(N):
            msg = self.measured_cp
            msg.pose.position.x = tr_x[i]
            msg.pose.position.y = tr_y[i]
            msg.pose.position.z = tr_z[i]
            self.servo_cp_pub.publish(msg)
            rclpy.spin_once(self)

        self.get_logger().info('Finsihed navigation.')

    def move_tcp_circular(self, center, radius, duration, dt):
        # Wait for pos to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        # Calculate number of steps
        N = int(math.floor(duration / dt))
        
        # Generate angles from 0 to 2*Pi
        thetas = np.linspace(0, 2.0 * np.pi, N)

        self.get_logger().info('Starting circular navigation...')

        loop_rate = self.create_rate(1.0 / dt, self.get_clock()) # Hz
        for i in range(N):
            # Copy current pose to maintain orientation
            msg = self.measured_cp 
            
            # Update X and Y based on the circle formula, keep Z constant
            msg.pose.position.x = center[0] + radius * math.cos(thetas[i])
            msg.pose.position.y = center[1] + radius * math.sin(thetas[i])
            msg.pose.position.z = center[2]
            
            self.servo_cp_pub.publish(msg)
            rclpy.spin_once(self)

        self.get_logger().info('Finished circular navigation.')

    def move_jaw_to(self, target, omega, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.measured_jaw is None and rclpy.ok():
            self.get_logger().info('Waiting for jaw pos...')
            rclpy.spin_once(self)

        distance = abs(target - self.measured_jaw.position[0])
        T = distance / omega
        N = int(math.floor(T / dt))

        tr_jaw = np.linspace(self.measured_jaw.position[0], target, N)

        self.get_logger().info('Starting moving jaws...')

        loop_rate = self.create_rate(1.0 / dt, self.get_clock()) # Hz
        for i in range(N):
            msg = self.measured_jaw
            msg.position = [tr_jaw[i]]
            self.servo_jaw_pub.publish(msg)
            rclpy.spin_once(self)

        self.get_logger().info('Finsihed moving jaws.')

    def grasp_marker(self, v, omega, dt):
        # Wait for the marker position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.marker is None and rclpy.ok():
            self.get_logger().info('Waiting for marker pos...')
            rclpy.spin_once(self)

        # Open jaws
        self.move_jaw_to(target=0.8, omega=omega, dt=dt)

        # Go to marker
        target = np.array([self.marker.pose.position.x,
                           self.marker.pose.position.y,
                           self.marker.pose.position.z + 0.008])

        self.move_tcp_to(target=target, v=v, dt=dt)

        # Close jaws
        self.move_jaw_to(target=0.0, omega=omega, dt=dt)

def main(args=None):
    rclpy.init(args=args)
    psm = PSM()

    #Reset the arm
    center_point = [0.0, 0.0, -0.12]
    psm.move_tcp_to(center_point, 0.01, 0.01)
    psm.move_jaw_to(0.0, 0.1, 0.01)

    # Execute circular traj
    # Rad = 3cm, Dur = 5s
    psm.move_tcp_circular(center=center_point, radius=0.03, duration=5.0, dt=0.01)

    #psm.move_tcp_to(target=[0.0, 0.05, -0.12], v=0.01, dt=0.01)
    #psm.move_jaw_to(target=0.8, omega=0.2, dt=0.01)
    #psm.grasp_marker(v=0.005, omega=0.05, dt=0.01)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    psm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
