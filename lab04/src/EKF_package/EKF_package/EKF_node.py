import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from landmark_msgs.msg import LandmarkArray  # Assuming LandmarkArray is the correct type
from ekf import *
import yaml
from probabilistic_models import *

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('EKF_node')


        self.landmarks = self.load_landmarks()

        # Initialize EKF with initial state and covariance
        initial_state = [0.0, 0.0, 0.0]
        initial_covariance = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
        self.ekf = RobotEKF(initial_state, initial_covariance)

        # Subscribe to odometry topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LandmarkArray, '/landmarks', self.landmark_callback, 10)
        self.ekf_publisher = self.create_publisher(Odometry, '/ekf', 10)
        
        # Timer for EKF update
        timer_period = 1/20  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize control variables
        self.v = 0.0
        self.w = 0.0

    def load_landmarks(self):
        # Load the landmarks from the YAML file
        with open('/home/gsncunha/SESASR/lab04/install/turtlebot3_perception/share/turtlebot3_perception/config/landmarks.yaml', 'r') as file:
            data = yaml.safe_load(file)
        landmarks = {'id': data['id'], 'x': data['x'], 'y': data['y']}
        return landmarks

    def odom_callback(self, msg: Odometry):
        # Update linear and angular velocities from odometry
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
        
        # Predict state with control input
        control_input = [self.v, self.w]
        self.ekf.predict(control_input)

    def landmark_callback(self, msg: LandmarkArray):
        odom_msg = Odometry()
        # Loop through each landmark in the message and update the EKF
        for landmark in msg.landmarks:
            id = landmark.id
            range = landmark.range
            bearing = landmark.bearing

            # Get the landmark coordinates from the loaded data
            lx = self.landmarks['x'][id]
            ly = self.landmarks['y'][id]

            # Use the range and bearing to compute the measurement update
            # Update the EKF with the range and bearing of the observed landmark
            measurement = [range, bearing, lx, ly]  # Adjust depending on your EKF implementation
            self.ekf.update(measurement)

        # Publish the Odometry message

        odom_msg.header.stamp = self.get_clock().now().to_msg()  # Set current time
        estimated_state = self.ekf.get_state()

        odom_msg.pose.pose.position.x = estimated_state[0]  # x position
        odom_msg.pose.pose.position.y = estimated_state[1]  # y position
        odom_msg.pose.pose.orientation.z = estimated_state[2]  # Orientation (theta), assuming 2D pose
    
        self.ekf_publisher.publish(odom_msg)


    def timer_callback(self):
        # Placeholder measurement update
        measurement = [self.v, self.w]  # Replace with actual measurement data
        self.ekf.update(measurement)

        # Log estimated state
        estimated_state = self.ekf.get_state()
        self.get_logger().info(f'Estimated State: {estimated_state}')


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Clean up
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
