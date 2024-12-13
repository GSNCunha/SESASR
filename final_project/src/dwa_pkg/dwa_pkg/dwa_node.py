import rclpy
from rclpy.node import Node
import numpy as np
from dwa_pkg.dwa import *
from dwa_pkg.utils import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


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
            weight_angle=0.1,
            weight_vel=0.2,
            weight_obs=0.08,
            obstacles_map=np.empty((0, 2)),
            init_pose=self.init_pose,
            max_linear_acc=0.5,
            max_ang_acc=3.2,
            max_lin_vel=0.5,  # m/s
            min_lin_vel=0.0,  # m/s
            max_ang_vel=3.0,  # rad/s
            min_ang_vel=-3.0,  # rad/s
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

        # Tracking the state
        self.done = False
        self.robot_poses = None

    def update_robot_pose(self):
        if not self.done:
            # Check if the goal is not [0.0, 0.0] with a tolerance
            if not np.allclose(self.goal_pose, [0.0, 0.0], atol=1e-3):  # Tolerance of 1e-3 for small precision differences
                self.done, self.robot_poses = self.controller.go_to_pose(self.goal_pose)

                if self.done:
                    self.get_logger().info("Goal reached!")
                else:
                    self.get_logger().info(f"Updating robot pose. Current pose: {self.controller.robot.pose}")
            ##else:
                ##self.get_logger().info("Goal is at the origin, skipping movement.")


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
        self.controller.obstacles_map = obstacles

    def goal_pose_callback(self, msg):
        self.get_logger().info(f"Received Odometry message")
        """
        Callback to update the goal pose based on the incoming message.
        """
        # Extract the goal position from the Odometry message
        self.goal_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.get_logger().info(f"Updated goal pose: {self.goal_pose}")

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
