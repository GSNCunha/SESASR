import rclpy
from rclpy.node import Node
import numpy as np
from dwa_pkg.dwa import *
from dwa_pkg.utils import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray


class DWANode(Node):  
    def __init__(self):
        super().__init__('dwa_node')

        # Initial robot pose, goal pose, and obstacles
        self.init_pose = np.array([0.0, 0.0, 0.0])

        self.goal_pose = np.array([2, 0])
        self.status = None
        # Initialize the DWA controller
        self.controller = DWA(
            dt=0.1,
            sim_time=2.0,
            v_samples=30,
            w_samples=20,
            goal_dist_tol=0.2,
            collision_tol=0.15,
            weight_angle=0.1,
            weight_vel= 0.6,
            weight_obs=0.05,
            weight_target=0.01,
            obstacles_map=np.empty((0, 2)),
            init_pose=self.init_pose,
            max_linear_acc=0.5,
            max_ang_acc=3.2,
            max_lin_vel=0.22,
            min_lin_vel=0.0,
            max_ang_vel=2.84,
            min_ang_vel=-2.84,
            radius=0.3,
        )

        self.laser_sensor = LaserScanSensor(
            max_dist=3.5,
            min_dist=0.1,
            num_points=30,
            tot_num_points=360
        )

        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacle_markers', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(1/15, self.update_robot_pose)
        self.done = False
        self.robot_poses = None

        # Counter for interactions
        self.interaction_count = 0

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = 2 * np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.controller.robot.pose = np.array([x, y, theta])

    def update_robot_pose(self):
        # Increment the counter
        self.interaction_count += 1

        # Check if interaction count has exceeded 3000
        if self.interaction_count > 3000:
            #self.get_logger().info("Timeout reached! Stopping the robot.")
            self.status = "timeout"
            self.stop_robot()
            return
        
        dist_to_goal = np.linalg.norm(self.controller.robot.pose[0:2] - self.goal_pose)

        if self.status == "collision":
            self.stop_robot()
            self.task_result = "collision"
            return
        if self.interaction_count % 50 == 0:
            self.get_logger().info(f"Distance_to_goal:{dist_to_goal}")


        if dist_to_goal < self.controller.goal_dist_tol:
            self.status = "goal_reached"
            print("Goal reached!")

        u = self.controller.compute_cmd(self.goal_pose, self.controller.robot.pose, self.controller.obstacles)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = u[0]
        cmd_vel_msg.angular.z = u[1]
        self.controller.robot.vel = u
        self.get_logger().info(f"v: {u[0]},goal:{dist_to_goal}")
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
    def stop_robot(self):
        # Send a stop command to the robot
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def scan_callback(self, msg):
        # Process LiDAR ranges
        ranges = np.array(msg.ranges)
        filtered_ranges = self.laser_sensor.process_data(ranges)
        
        if np.min(filtered_ranges) < self.controller.collision_tol:
            self.status = "collision"
            #self.get_logger().info("Collision detected. Robot stopped.")
            return
        # Convert ranges to obstacle coordinates
        robot_pose = [self.controller.robot.pose[0], self.controller.robot.pose[1], self.controller.robot.pose[2]]
        obstacles = range_to_obstacles(filtered_ranges, robot_pose, self.laser_sensor.num_points)

        # Update obstacles in DWA controller
        self.controller.obstacles = obstacles

        # Publish obstacles to RViz
        self.publish_obstacle_markers(obstacles)

    def goal_pose_callback(self, msg):
        self.goal_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.controller.goal_pose = self.goal_pose

    def publish_obstacle_markers(self, obstacles):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "odom"  # Ensure the frame matches your setup
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2  # Marker size
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0  # Red color
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Full opacity
            marker.lifetime.sec = 1  # Short lifespan to refresh markers dynamically
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        


def main(args=None):
    rclpy.init(args=args)
    dwa_node = DWANode()
    rclpy.spin(dwa_node)
    rclpy.shutdown()
    if dwa_node.status:
            print(f"Task result: {dwa_node.task_result}")
    else:
        print("Task ended without a result.")
        dwa_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
