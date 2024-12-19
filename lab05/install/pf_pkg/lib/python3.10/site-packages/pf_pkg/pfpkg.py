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
    residual,
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
        # self.declare_parameter('resampling_strategy', 'systematic')
        # self.declare_parameter('init_strategy', 'uniform')
        self.declare_parameter('landmarks_yaml', '/root/SESASR/lab05/src/pf_pkg/pf_pkg/landmarks1.yaml')

        # Load landmarks
        landmarks_yaml = self.get_parameter('landmarks_yaml').get_parameter_value().string_value
        self.landmarks = load_landmarks(landmarks_yaml)
        self.get_logger().info(f"Loaded landmarks: {self.landmarks}")

        # Particle filter setup
        self.pf = RobotPF(
            dim_x=3,  # [x, y, theta]
            dim_u=2,  # [v, w]
            eval_gux=sample_velocity_motion_model,
            resampling_fn= None,
             boundaries=[(-0.9, 2.7), (-1.7, 1.2), (-np.pi, np.pi)],
            N=2000
        )
        self.pf.initialize_particles()

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LandmarkArray, 'camera/landmarks', self.landmarks_callback, 10)

        # Publisher
        self.pub_pf = self.create_publisher(Odometry, '/pf_home', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/particles_home', 10)
        self.pub_landmark_markers = self.create_publisher(MarkerArray, '/landmark_markers', 10)

        # Timer for prediction step (20 Hz)
        self.create_timer(0.05, self.run_prediction)


        self.publish_landmarks()


        # Internal state
        self.latest_twist = None
        self.dt = 0.05  # Time step (20 Hz)
        
    def cmd_vel_callback(self, msg):
        self.latest_twist = [msg.linear.x, msg.angular.z]

    def publish_landmarks(self):
        """
        Publish the landmarks as MarkerArray for visualization in RViz.
        """
        markers = MarkerArray()
        for i, landmark_id in enumerate(self.landmarks['id']):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = self.landmarks['x'][i]
            marker.pose.position.y = self.landmarks['y'][i]
            marker.pose.position.z = self.landmarks['z'][i]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.a = 1.0  # Opacidade
            marker.color.r = 1.0  # Vermelho
            marker.color.g = 0.0  # Verde
            marker.color.b = 0.0  # Azul

            markers.markers.append(marker)

        self.pub_landmark_markers.publish(markers)
        
    def landmarks_callback(self, msg):
        """
        Update particle weights based on landmark measurements.
        """
        sigma_z = [0.05, 0.2]  # Measurement noise (range, bearing)
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
            if z is not None:
                self.pf.update(
                    z,
                    sigma_z,
                    landmark_range_bearing_model,
                    hx_args=(landmark_pos, sigma_z)
                )
            
            self.pf.normalize_weights()
            
            neff = self.pf.neff()   
            
            if neff < self.pf.N / 2:
                self.pf.resampling(resampling_fn=systematic_resample, resampling_args=(self.pf.weights,))
                assert np.allclose(self.pf.weights, 1 / self.pf.N)
                
            self.pf.estimate(mean_fn=state_mean, residual_fn=residual, angle_idx=2)
            self.publish_estimate()
            self.publish_particles()
            
            
    def run_prediction(self):
            """
            Predict the next particle state based on the latest velocity command.
            """
            if self.latest_twist is None:
                return
            
            u = self.latest_twist
            sigma_u = np.array([2, 2])  # Noise for velocity model
            self.pf.predict(u=u, sigma_u=sigma_u, g_extra_args=(self.dt,))  
            self.pf.estimate(mean_fn=state_mean, residual_fn=residual, angle_idx=2)
            # self.publish_estimate()
            # self.publish_particles()
            
    def publish_estimate(self):
        """
        Publish the current particle filter estimate.
        """
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.pf.mu[0]
        msg.pose.pose.position.y = self.pf.mu[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = np.sin(self.pf.mu[2] / 2)
        msg.pose.pose.orientation.w = np.cos(self.pf.mu[2] / 2)
        self.pub_pf.publish(msg)
    
    def publish_particles(self):
        
        particles = self.pf.particles
        markers = MarkerArray()
        for i in range(particles.shape[0]):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = particles[i, 0]
            marker.pose.position.y = particles[i, 1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.z = np.sin(particles[i, 2] / 2)
            marker.pose.orientation.w = np.cos(particles[i, 2] / 2)
            marker.scale.x = 0.1
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 0.8
            markers.markers.append(marker)
        self.pub_markers.publish(markers)   
                    
def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
            
    
 
            
    