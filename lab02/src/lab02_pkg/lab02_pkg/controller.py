import rclpy
import tf_transformations
import math
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.movements = 0
        self.movements_set = 1
        self.angle = 0
        self.actual_angle = 0
        self.left_turn = 0
        self.right_turn = 0

    def odom_callback(self, msg:  Odometry):
        msg.pose.pose.orientation

        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)

        self.angle = yaw
        pass

    def scan_callback(self, msg):
        # Handle LaserScan message
        msg_control_velocity = Twist()
        msg_get_position = Odometry()
        if self.left_turn == 1 or self.right_turn == 1:
            if self.left_turn == 1:
                if self.actual_angle > (3*math.pi/2):
                    
                if self.angle < (self.actual_angle + math.pi/2):
                    msg_control_velocity.angular.z = 0.5
                elif self.angle >= (self.actual_angle + math.pi/2):
                    msg_control_velocity.angular.z = 0.0
                    self.left_turn = 0
            if self.right_turn == 1:
                if self.angle > (self.actual_angle - math.pi/2):
                    msg_control_velocity.angular.z = -0.5
                elif self.angle <= (self.actual_angle - math.pi/2):
                    msg_control_velocity.angular.z = 0.0
                    self.right_turn = 0

        else:

            if msg.ranges[0] < 0.8:  # Adjusted LaserScan range access
                msg_control_velocity.linear.x = 0.0
                if msg.ranges[90] > msg.ranges[270]:
                    self.left_turn = 1
                    self.actual_angle = self.angle
                    msg_control_velocity.angular.z = 0.5
                elif msg.ranges[90] < msg.ranges[270]:
                    self.right_turn = 1
                    self.actual_angle = self.angle
                    msg_control_velocity.angular.z = -0.5
            else:
                msg_control_velocity.linear.x = 0.22

        self.publisher_.publish(msg_control_velocity)
        self.get_logger().info(f'Publishing: X = "{msg_control_velocity.linear.x}" sensor = "{msg.ranges[0]}"')

    def timer_callback(self):
        # Periodic operations can be placed here if needed
        pass


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
