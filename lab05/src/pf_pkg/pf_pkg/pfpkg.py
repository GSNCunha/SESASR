import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from landmark_msgs.msg import LandmarkArray
import numpy as np
from pf_pkg.pf import RobotPF
from pf_pkg.probabilistic_models import landmark_range_bearing_model, sample_velocity_motion_model
from pf_pkg.utils import  (
    simple_resample,
    residual_resample,
    stratified_resample,
    systematic_resample,
    state_mean,
)
import yaml
from visualization_msgs.msg import Marker, MarkerArray



def load_landmarks(yaml_file):
    """
    Load landmarks from a YAML file.
    """
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    landmarks = {
        'id': data['landmarks']['id'],
        'x': data['landmarks']['x'],
        'y': data['landmarks']['y'],
        'z': data['landmarks']['z'],
    }
    return landmarks


class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # Declare parameters
        self.declare_parameter('resampling_strategy', 'systematic')
        self.declare_parameter('init_strategy', 'uniform')
        self.declare_parameter('landmarks_yaml', '/home/thilevin/sesasr/SESASR/lab05/src/pf_pkg/pf_pkg/landmarks.yaml')

        # Load landmarks
        landmarks_yaml = self.get_parameter('landmarks_yaml').get_parameter_value().string_value
        self.landmarks = load_landmarks(landmarks_yaml)
        self.get_logger().info(f"Loaded landmarks: {self.landmarks}")

        # Particle filter setup
        self.pf = RobotPF(
            dim_x=3,  # [x, y, theta]
            dim_u=2,  # [v, w]
            eval_gux=sample_velocity_motion_model,
            resampling_fn=self.dynamic_resampling,
            boundaries=[(-3.0, 3.0), (-3.0, 3.0), (-np.pi, np.pi)],  # Environment boundaries
            N=2000
        )
        self.initialize_particles()

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LandmarkArray, '/landmarks', self.landmarks_callback, 10)
        

        # Publisher
        self.pub_pf = self.create_publisher(Odometry, '/pf', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/particles', 10)


        # Timer for prediction step (20 Hz)
        self.create_timer(0.05, self.run_prediction)

        # Internal state
        self.latest_twist = None
        self.dt = 0.05  # Time step (20 Hz)

    def initialize_particles(self):
        """
        Initialize particles based on the chosen strategy.
        """
        init_strategy = self.get_parameter('init_strategy').get_parameter_value().string_value

        if init_strategy == "uniform":
            for i in range(self.pf.dim_x):
                self.pf.particles[:, i] = np.random.uniform(
                    self.pf.boundaries[i][0], self.pf.boundaries[i][1], self.pf.N
                )
        elif init_strategy == "gaussian":
            # Example: Gaussian initialization around [5.0, 5.0, 0.0] with standard deviations [1.0, 1.0, 0.1]
            init_pose = [5.0, 5.0, 0.0]
            std = [1.0, 1.0, 0.1]
            for i in range(self.pf.dim_x):
                self.pf.particles[:, i] = np.random.normal(init_pose[i], std[i], self.pf.N)
        else:
            raise ValueError(f"Unknown initialization strategy: {init_strategy}")

        self.pf.weights = np.ones(self.pf.N) / self.pf.N

    def publish_particles(self):
        """
        Publish particles as a MarkerArray for visualization in RViz.
        """
        marker_array = MarkerArray()
        for i, particle in enumerate(self.pf.particles):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "odom"
            marker.ns = "particles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = particle[0]
            marker.pose.position.y = particle[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05  # Size of the sphere
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue
            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.pub_markers.publish(marker_array)


    def cmd_vel_callback(self, msg):
        self.latest_twist = [msg.linear.x, msg.angular.z]

    def landmarks_callback(self, msg):
        """
        Update particle weights based on landmark measurements.
        """
        sigma_z = [0.026, 0.1]  # Measurement noise (range, bearing)
        for landmark_msg in msg.landmarks:
            z = [landmark_msg.range, landmark_msg.bearing]
            landmark_id = landmark_msg.id
            print(f">>>Landmark ID: {landmark_id}<<<<")
            # Find the corresponding landmark position
            landmark_idx = self.landmarks['id'].index(landmark_id)
            print(f">>>>>Landmark Index: {landmark_idx}<<<<<<")
            landmark_pos = [
                self.landmarks['x'][landmark_idx],
                self.landmarks['y'][landmark_idx]
            ]
            # print(f"Landmark Position: {landmark_pos}")

            # Update particles with measurements
            # print(f"Updating particles with z: {z}, sigma_z: {sigma_z}")
            self.pf.update(
                z=z,
                sigma_z=sigma_z,
                eval_hx=landmark_range_bearing_model,
                hx_args=(landmark_pos, sigma_z)
            )
            
        self.pf.normalize_weights()
        # print("Weights normalized")

        # Dynamically select and pass the resampling function
        if self.pf.neff() < self.pf.N / 2:
            resampling_fn = self.dynamic_resampling()
            print("Resampling particles")
            self.pf.resampling(resampling_fn)  #No need to pass weights explicitly

        # Estimate the state
        self.pf.estimate()
        # print(f"Estimated state: {self.pf.mu}")

        # Publish the estimated state
        # print("Publishing estimate")
        self.publish_estimate()
        self.publish_particles()


    def run_prediction(self):
        """
        Predict the next particle state based on the latest velocity command.
        """
        if self.latest_twist is None:
            return
        u = self.latest_twist
        sigma_u = np.array([0.03, 0.03, 0.03, 0.03, 0.03, 0.03])  # Noise for velocity model
        self.pf.predict(u=u, sigma_u=sigma_u, g_extra_args=(self.dt,))

    def dynamic_resampling(self):
        """
        Dynamically choose the resampling strategy.
        """
        resampling_strategy = self.get_parameter('resampling_strategy').get_parameter_value().string_value

        strategy_map = {
            "simple": simple_resample,
            "residual": residual_resample,
            "stratified": stratified_resample,
            "systematic": systematic_resample,
        }

        if resampling_strategy not in strategy_map:
            raise ValueError(f"Unknown resampling strategy: {resampling_strategy}")

        return strategy_map[resampling_strategy]  # Return the correct resampling function



    def publish_estimate(self):
        """
        Publish the estimated state to the /pf topic.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.pose.pose.position.x = self.pf.mu[0]
        odom_msg.pose.pose.position.y = self.pf.mu[1]
        odom_msg.pose.pose.orientation.z = np.sin(self.pf.mu[2] / 2)
        odom_msg.pose.pose.orientation.w = np.cos(self.pf.mu[2] / 2)
        self.pub_pf.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()