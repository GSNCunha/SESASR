import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from landmark_msgs.msg import LandmarkArray  # Assuming LandmarkArray is the correct type
from EKF_package.ekf import *
from EKF_package.probabilistic_models import *
import yaml

class Ekf(Node):
    def __init__(self):
        super().__init__('EKF_node')

        # Load landmarks from YAML
        self.landmarks = self.load_landmarks()

        # Initialize EKF with initial state and covariance
        self.initial_state = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001]
        self.initial_covariance = np.eye(5) * 0.1  # Matriz identidade escalada 5x5
        self.qt = np.eye(2) * 0.1  # Process noise covariance
        self.mt = np.eye(2) * 0.1  # Measurement noise covariance

        eval_gux, eval_Gt, eval_Vt = velocity_mm_simpy()

        self.ekf = RobotEKF(
            dim_x=len(self.initial_state),  # Set dim_x based on the length of initial_state
            dim_u=2,  # Adjust based on your control vector's dimension if needed
            eval_gux=eval_gux,
            eval_Gt=eval_Gt,
            eval_Vt=eval_Vt
        )
        self.ekf.mu = np.array(self.initial_state)   # Set initial state

        # Subscribe to odometry and landmark topics
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LandmarkArray, '/landmarks', self.landmark_callback, 10)

        # Publisher for EKF output
        self.ekf_publisher = self.create_publisher(Odometry, '/ekf', 10)
        
        # Timer for EKF update (20 Hz)
        timer_period = 1/20  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize control variables
        self.v = 0.0
        self.w = 0.0

    def load_landmarks(self):
        with open('/home/gsncunha/SESASR/lab04/install/turtlebot3_perception/share/turtlebot3_perception/config/landmarks.yaml', 'r') as file:
            data = yaml.safe_load(file)

        # Create a dictionary mapping each 'id' to its corresponding 'x' and 'y' coordinates
        landmarks = {id: {'x': x, 'y': y} for id, x, y in zip(data['landmarks']['id'], data['landmarks']['x'], data['landmarks']['y'])}
        
        return landmarks


    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def landmark_callback(self, msg: LandmarkArray):
        odom_msg = Odometry()
        
        for landmark in msg.landmarks:
            id = landmark.id
            range = landmark.range
            bearing = landmark.bearing

            # Retrieve the landmark position by ID, if it exists
            if id in self.landmarks:
                lx = self.landmarks[id]['x']
                ly = self.landmarks[id]['y']

                # Measurement update with the observed landmark
                measurement = [range, bearing]
                eval_hx, eval_Ht = landmark_sm_simpy()
                print("mu", self.ekf.mu)  


                self.ekf.update(
                measurement,
                eval_hx,
                eval_Ht,
                self.qt,
                (self.ekf.mu[0], self.ekf.mu[1], self.ekf.mu[2], self.ekf.mu[3], self.ekf.mu[4], lx, ly),  # state with v and w
                (self.ekf.mu[0], self.ekf.mu[1], self.ekf.mu[2], self.ekf.mu[3], self.ekf.mu[4], lx, ly),
                residual=np.subtract
            )

            else:
                self.get_logger().warn(f"Landmark with ID {id} not found in YAML configuration.")

        # Populate and publish Odometry message
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        estimated_state = self.ekf.mu

        odom_msg.pose.pose.position.x = estimated_state[0]
        odom_msg.pose.pose.position.y = estimated_state[1]
        odom_msg.pose.pose.orientation.z = estimated_state[2]
        odom_msg.twist.twist.linear.x = estimated_state[3]
        odom_msg.twist.twist.angular.z = estimated_state[4]

        
        self.ekf_publisher.publish(odom_msg)

    def timer_callback(self):
        # Define control input
        control_input = []
        
        # Define control noise standard deviations (e.g., 0.1 for each as a placeholder)
        sigma_u = [0.1, 0.1]  # Ensure this is always a list or array

        
        # Define the time step (dt), this could come from the system's clock or a fixed time interval
        dt = 1.0 / 20.0  # Assuming a 20 Hz update rate, adjust as needed
        
        # Perform the EKF prediction step with the control input, noise, and time step
        self.ekf.predict(control_input, sigma_u, (dt, ))




def main(args=None):
    rclpy.init(args=args)
    ekf = Ekf()  # Correct initialization here
    rclpy.spin(ekf)
    ekf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
