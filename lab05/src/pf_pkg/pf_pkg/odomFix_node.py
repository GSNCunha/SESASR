import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

class OdomFixNode(Node):
    def __init__(self):
        super().__init__('odom_fix_node')

        # Initialization of position and orientation
        self.position = np.array([0.0, 0.0, 0.0])  # Initial position (x, y, z)
        self.orientation = 0.0  # Initial orientation (theta, in radians)

        # Initialization of velocities (with default values)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Variables to control simulation time
        self.dt = 0.01  # Time interval for each update (100 Hz)

        # Subscription to the velocity topic
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher on the new topic /odom_fixed
        self.publisher = self.create_publisher(Odometry, '/odom_fixed', 10)

        # Timer to execute the update every dt seconds
        self.create_timer(self.dt, self.update_odometry)

    def cmd_vel_callback(self, msg):
        """
        Receives the robot's linear velocity (msg.linear.x) and angular velocity (msg.angular.z).
        """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_odometry(self):
        """
        Updates position and orientation based on the velocities and publishes the /odom_fixed topic.
        """
        # Calculate the new position using a simple kinematic model
        delta_x = self.linear_velocity * np.cos(self.orientation) * self.dt
        delta_y = self.linear_velocity * np.sin(self.orientation) * self.dt
        delta_orientation = self.angular_velocity * self.dt

        # Update position and orientation
        self.position[0] += delta_x
        self.position[1] += delta_y
        self.orientation += delta_orientation

        # Ensure orientation stays within the range -pi to pi
        if self.orientation > np.pi:
            self.orientation -= 2 * np.pi
        elif self.orientation < -np.pi:
            self.orientation += 2 * np.pi

        # Create the odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        # Set position
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = 0.0

        # Set orientation using quaternions
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(self.orientation / 2)
        msg.pose.pose.orientation.w = np.cos(self.orientation / 2)

        # Publish the adjusted odometry topic
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
