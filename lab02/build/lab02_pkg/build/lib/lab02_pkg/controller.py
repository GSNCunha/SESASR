from dataclasses import dataclass
import rclpy
import tf_transformations
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

@dataclass
class OperationMode:
    name: str = "hallway"
    side_length: float = 0.0
    front_length: float = 0.0
    go_front: float = 0.0
    x_position: float = 0.0
    y_position: float = 0.0

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.movements = 0
        self.movements_set = 1
        self.angle = 0
        self.actual_angle = 0
        self.left_turn = 0
        self.right_turn = 0
        self.orientation = "East"
        self.location_x = 0.0
        self.location_y = 0.0

        # Initialize operation_mode
        self.operation_mode = OperationMode()

    def odom_callback(self, msg: Odometry):
        # Retrieve quaternion and convert to Euler angles
        quat = [
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        self.angle = yaw

        # Save position coordinates to self.location_x and self.location_y
        self.location_x = msg.pose.pose.position.x
        self.location_y = msg.pose.pose.position.y




    def scan_callback(self, msg):
        msg_control_velocity = Twist()

        def radians_to_go(initial_angle, actual_angle, side_of_turn):
            if side_of_turn == "left":
                if initial_angle > (3 * math.pi / 2):
                    if actual_angle > (3 * math.pi / 2):
                        return math.pi / 2 - (actual_angle - initial_angle)
                    else:
                        return math.pi / 2 - (2 * math.pi - initial_angle) - actual_angle
                else:
                    return initial_angle + math.pi / 2 - actual_angle
            elif side_of_turn == "right":
                if initial_angle < (math.pi / 2):
                    if actual_angle < (math.pi / 2):
                        return math.pi / 2 - (initial_angle - actual_angle)
                    else:
                        return math.pi / 2 - initial_angle - (2 * math.pi - actual_angle)
                return actual_angle - (initial_angle - math.pi / 2)
            return 0.0
        
        def change_orientation(side):
            if self.orientation == "East":
                if side == "right":
                    self.orientation = "South"  # Fixed
                elif side == "left":
                    self.orientation = "North"  # Fixed
            elif self.orientation == "North":
                if side == "right":
                    self.orientation = "East"  # Fixed
                elif side == "left":
                    self.orientation = "West"  # Fixed
            elif self.orientation == "West":
                if side == "right":
                    self.orientation = "North"  # Fixed
                elif side == "left":
                    self.orientation = "South"  # Fixed
            elif self.orientation == "South":
                if side == "right":
                    self.orientation = "West"  # Fixed
                elif side == "left":
                    self.orientation = "East"  # Fixed




        if self.operation_mode.name == "NONE":
            if msg.ranges[90] < 1.5 and msg.ranges[270] < 1.5:
                self.operation_mode.name = "hallway"
                self.get_logger().info(f'I am in a Hallway!')
            elif msg.ranges[90] < 1.5:
                self.operation_mode.name = "left_saloon"
            elif msg.ranges[270] < 1.5:
                self.operation_mode.name = "right_saloon"
            else:
                self.operation_mode.name = "saloon"

        if self.operation_mode.name == "hallway":

            if msg.ranges[90] > 1.5:
                self.get_logger().info('Left side open!')
                if msg.ranges[0] < 2.5:
                    self.operation_mode.name = "turn_left"
                    self.actual_angle = self.angle
                    self.operation_mode.side_length = msg.ranges[270]
                    self.operation_mode.front_length = msg.ranges[0]
                    self.get_logger().info('I need to turn left!')
                else:
                    msg_control_velocity.linear.x = 0.22

            elif msg.ranges[270] > 1.5:
                self.get_logger().info('Right side open!')
                if msg.ranges[0] < 2.5:
                    self.operation_mode.name = "turn_right"
                    self.actual_angle = self.angle
                    self.operation_mode.side_length = msg.ranges[90]
                    self.operation_mode.front_length = msg.ranges[0]
                    self.get_logger().info('I need to turn right!')
                else:
                    msg_control_velocity.linear.x = 0.22
            else:
                msg_control_velocity.linear.x = 0.22



        if self.operation_mode.name == "turn_left":
            turn_time = 2*math.pi*(self.operation_mode.front_length/2)/4/0.1
            angular_velocity = math.pi / 2 / turn_time

            radians = radians_to_go(self.actual_angle, self.angle, "left")

            if radians > 0:
                msg_control_velocity.angular.z = angular_velocity
                msg_control_velocity.linear.x = 0.10
            else:
                change_orientation("left")
                self.get_logger().info(f'Finish curve! angle= {self.angle}')

                self.operation_mode.name = "go_front"
                self.get_logger().info(f'Lets go front!')
                if self.operation_mode.side_length > self.operation_mode.front_length/2:
                    self.operation_mode.go_front = self.operation_mode.side_length - (self.operation_mode.front_length)/2 + 0.4
                else:
                    self.operation_mode.go_front = 0.4
                self.operation_mode.x_position = self.location_x
                self.operation_mode.y_position = self.location_y
                self.operation_mode.side_length = 0.0
                self.operation_mode.front_length = 0.0

        if self.operation_mode.name == "turn_right":
                    turn_time = 2*math.pi*(self.operation_mode.front_length/2)/4/0.1
                    angular_velocity = math.pi / 2 / turn_time

                    radians = radians_to_go(self.actual_angle, self.angle, "right")

                    if radians > 0:
                        msg_control_velocity.angular.z = - angular_velocity
                        msg_control_velocity.linear.x = 0.10
                    else:
                        change_orientation("right")
                        self.get_logger().info(f'Finish curve! angle= {self.angle}')

                        self.operation_mode.name = "go_front"
                        self.get_logger().info(f'Lets go front!')
                        if self.operation_mode.side_length > self.operation_mode.front_length:
                            self.operation_mode.go_front = self.operation_mode.side_length - self.operation_mode.front_length + 0.2
                        else:
                            self.operation_mode.go_front = 0.2
                        self.operation_mode.x_position = self.location_x
                        self.operation_mode.y_position = self.location_y
                        self.operation_mode.side_length = 0.0
                        self.operation_mode.front_length = 0.0


        if self.operation_mode.name == "go_front":
            if self.orientation == "East":
                if (self.location_x - self.operation_mode.x_position) < self.operation_mode.go_front:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.side_length = 0.0
                    self.operation_mode.front_length = 0.0
            if self.orientation == "North":
                if (self.location_y - self.operation_mode.y_position) < self.operation_mode.go_front:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.side_length = 0.0
                    self.operation_mode.front_length = 0.0
            if self.orientation == "West":
                if (self.operation_mode.x_position - self.location_x) < self.operation_mode.go_front:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.side_length = 0.0
                    self.operation_mode.front_length = 0.0

            if self.orientation == "South":
                if (self.operation_mode.y_position - self.location_y) < self.operation_mode.go_front:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.side_length = 0.0
                    self.operation_mode.front_length = 0.0


        self.publisher_.publish(msg_control_velocity)
        #self.get_logger().info(f'Publishing: X = "{msg_control_velocity.linear.x}" sensor = "{msg.ranges[0]}"')

    def timer_callback(self):
        # Periodic operations can be placed here if needed
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
