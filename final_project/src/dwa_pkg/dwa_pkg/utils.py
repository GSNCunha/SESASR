import numpy as np
import math


class Differential_drive_robot():
    def __init__(self, 
                init_pose,
                max_linear_acc = 0.8,
                max_ang_acc = 100 * math.pi /180,
                max_lin_vel = 1.0, # m/s
                min_lin_vel = 0.0, # m/s
                max_ang_vel = 3.0, # rad/s 
                min_ang_vel = -3.0, # rad/s 
                radius = 0.3, # radius for circular robot
                ):
        
        #initialization
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])
        
        # kinematic properties
        self.max_linear_acc = max_linear_acc
        self.max_ang_acc = max_ang_acc
        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

        # size
        self.radius = radius # circular shape

        # trajectory initialization
        self.trajectory = np.array([init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0]).reshape(1, -1)

    def update_state(self, u, dt):
        """
        Compute next pose of the robot according to differential drive kinematics rule (platform level equation).
        Save velocity and pose in the overall trajectory list of configurations.
        """

        if u is list:
            u = np.array(u)

        self.vel = u

        next_x = self.pose[0] + self.vel[0] * math.cos(self.pose[2]) * dt
        next_y = self.pose[1] + self.vel[0] * math.sin(self.pose[2]) * dt
        next_th = self.pose[2] + self.vel[1] * dt
        self.pose = np.array([next_x, next_y, next_th])

        traj_state = np.array([next_x, next_y, next_th, self.vel[0], self.vel[1]]).reshape(1, -1)
        self.trajectory = np.concatenate([self.trajectory, traj_state], axis=0)

        return self.pose

def calc_nearest_obs(robot_pose, obstacles, obstacle_max_dist=3):
    """
    Filter obstacles: find the ones in the local map considered for obstacle avoidance.
    """
    nearest_obs = []
    
    for obs in obstacles:
        temp_dist_to_obs = np.linalg.norm(robot_pose[0:2]-obs)

        if temp_dist_to_obs < obstacle_max_dist :
            nearest_obs.append(obs)

    return np.array(nearest_obs)

class LaserScanSensor:
    def __init__(
        self,
        max_dist,
        min_dist,
        num_points,
        tot_num_points,
    ):
        self.max_dist = max_dist
        self.min_dist = min_dist
        self.tot_num_points = tot_num_points
        self.num_points = num_points

    def process_data(self, ranges):
        """
        Process raw LiDAR ranges:
        - Remove NaN and Inf values
        - Saturate the distances between min_dist and max_dist
        - Filter to keep only `num_points` ranges, choosing the minimum distance for each sector
        """
        # Step 1: Replace NaN and Inf
        ranges = np.array(ranges)
        ranges[np.isnan(ranges)] = self.max_dist  # Replace NaNs with max_dist
        ranges[np.isinf(ranges)] = self.min_dist  # Replace Infs with min_dist

        # Step 2: Saturate the ranges between min_dist and max_dist
        ranges = np.clip(ranges, self.min_dist, self.max_dist)

        # Step 3: Downsample the ranges to num_points
        # Divide the ranges into angular sectors and take the minimum value in each sector
        sector_size = int(np.ceil(self.tot_num_points / self.num_points))
        filtered_ranges = [
            np.min(ranges[i:i + sector_size])
            for i in range(0, self.tot_num_points, sector_size)
        ]

        # Ensure the output has exactly num_points
        filtered_ranges = np.array(filtered_ranges[:self.num_points])

        return filtered_ranges

def range_to_obstacles(ranges, robot_pose, num_rays, fov=2 * np.pi):
    """
    Convert LiDAR scan ranges into [x, y] obstacle coordinates in the map frame.
    Parameters:
    - ranges: Filtered LiDAR ranges
    - robot_pose: [x, y, theta] of the robot in the map frame
    - num_rays: Number of rays (points) to consider
    - fov: Field of view of the LiDAR (default: 360° or 2π radians)
    Returns:
    - end_points: A (num_rays x 2) array with [x, y] obstacle positions
    """
    robot_x, robot_y, robot_angle = robot_pose

    # Define the start angle and step angle for the rays
    start_angle = robot_angle - fov / 2
    step_angle = fov / num_rays

    # Compute the [x, y] coordinates of each obstacle
    end_points = np.zeros((num_rays, 2))
    for i, r in enumerate(ranges):
        angle = start_angle + i * step_angle
        x = robot_x + r * np.cos(angle)
        y = robot_y + r * np.sin(angle)
        end_points[i] = [x, y]

    return end_points

def normalize_angle(theta):
    """
    Normalize angles to the range [-π, π).
    If theta is an array, normalize each element.
    """
    # If theta is an array, apply normalize_angle element-wise
    if isinstance(theta, np.ndarray):
        return np.vectorize(lambda x: (x % (2 * np.pi) - np.pi) if x > np.pi else x)(theta)
    else:
        # Scalar version (as before)
        return (theta % (2 * np.pi)) - np.pi if theta > np.pi else theta

def normalize(arr: np.ndarray):
    """ normalize array of values """
    if np.isclose(np.max(arr) - np.min(arr), 0.0):
        return np.zeros_like(arr)
    else:
        return (arr - np.min(arr)) / (np.max(arr) - np.min(arr))

def sigmoid(x: np.ndarray):
  """ compute sigmoid smoothing activation of a given array of values"""
  return 1/(1 + np.exp(-x)) 