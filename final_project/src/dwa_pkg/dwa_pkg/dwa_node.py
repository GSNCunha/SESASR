import rclpy
from rclpy.node import Node
import numpy as np
from dwa_pkg.dwa import *
from dwa_pkg.utils import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class DWANode(Node):  # Renamed to avoid overwriting the imported DWA class
    def __init__(self):
        super().__init__('dwa_node')

        # Initial robot pose, goal pose, and obstacles
        self.init_pose = np.array([0.0, 0.0, 0.0])  # initial x, y, theta of the robot

        self.goal_pose = np.array([0.0, 0.0])  # Default goal pose for initialization

        # Initialize the DWA controller
        self.controller = DWA(
            dt=0.1,
            sim_time=2.0,
            v_samples=10,
            w_samples=20,
            goal_dist_tol=0.2,
            collision_tol=0.2,
            weight_angle= 0.1,
            weight_vel= 10,
            weight_obs= 0.1,
            obstacles_map=np.empty((0, 2)),
            init_pose=self.init_pose,
            max_linear_acc=0.5,
            max_ang_acc= 3.2,
            max_lin_vel= 0.22,  # m/s
            min_lin_vel= 0.0,  # m/s
            max_ang_vel= 2.84,  # rad/s
            min_ang_vel=-2.84,  # rad/s
            radius=0.3,  # m
        )

        self.laser_sensor = LaserScanSensor(
            max_dist=3.5,   # Maximum range to consider (meters)
            min_dist=0.1,   # Minimum range to consider (meters)
            num_points=30,  # Number of downsampled points
            tot_num_points=270  # Total points in a full 360-degree scan
        )

        # Timer to execute the controller periodically
        self.timer = self.create_timer(0.1, self.update_robot_pose)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Tracking the state
        self.done = False
        self.robot_poses = None

    def odom_callback(self, msg):
        """
        Callback to update the robot pose using odometry data.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)  # Extract yaw (theta)
        self.controller.robot.pose = np.array([x, y, theta])

    def update_robot_pose(self):


        dist_to_goal = np.linalg.norm(self.controller.robot.pose[0:2] - self.goal_pose)
        if dist_to_goal < self.controller.goal_dist_tol:
            print("Goal reached!")

        u = self.controller.compute_cmd(self.goal_pose, self.controller.robot.pose, self.controller.obstacles )

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = u[0]
        cmd_vel_msg.angular.z = u[1]
        self.get_logger().info(f"v: {u[0]}, w: {u[1]}")
        self.cmd_vel_pub.publish(cmd_vel_msg)




    def scan_callback(self, msg):
        """
        Callback to process LiDAR data and update obstacles in DWA controller.
        """
        # Process LiDAR ranges
        ranges = np.array(msg.ranges)
        filtered_ranges = self.laser_sensor.process_data(ranges)

        # Convert ranges to obstacle coordinates
        robot_pose = [self.controller.robot.pose[0], self.controller.robot.pose[1], self.controller.robot.pose[2]]
        obstacles = range_to_obstacles(filtered_ranges, robot_pose, self.laser_sensor.num_points)

        # Update obstacles in DWA controller
        self.controller.obstacles = obstacles

    def goal_pose_callback(self, msg):
        #self.get_logger().info(f"Received Odometry message")
        """
        Callback to update the goal pose based on the incoming message.
        """
        # Extract the goal position from the Odometry message
        self.goal_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        #self.get_logger().info(f"Updated goal pose: {self.goal_pose}")

        # Directly set the goal pose in the controller (if 'set_goal' does not exist)
        self.controller.goal_pose = self.goal_pose





def main(args=None):
    rclpy.init(args=args)
    dwa_node = DWANode()
    rclpy.spin(dwa_node)
    dwa_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()