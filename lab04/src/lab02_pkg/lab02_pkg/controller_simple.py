from dataclasses import dataclass
import rclpy
import tf_transformations
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

FRONT_VELO = 0.2
INITIAL_ORIENTATION = "East"
FRONT_LIMIT = 1
ANGULAR_VELOCITY_HIGH = 0.2
ANGULAR_VELOCITY_LOW = 0.05

@dataclass
class OperationMode:
    name: str = "FRONT"

class Controller(Node):

    def __init__(self):
    
        super().__init__('controller')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.angle = 0
        self.orientation = INITIAL_ORIENTATION

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

       

        def normalize_angles(angle):
            if angle < 0.0:
                return angle + math.pi*2
            elif angle > math.pi*2:
                return angle - math.pi*2
            else:
                return angle
            
        def radians_to_go(actual_angle, side_of_turn):
            if self.orientation == "East":
                initial_angle = 2*math.pi
            elif self.orientation == "North":
                initial_angle = math.pi/2
            elif self.orientation == "West":
                initial_angle = math.pi
            elif self.orientation == "South":
                initial_angle = math.pi*3/2

            normalized_actual_angle = normalize_angles(actual_angle)



            if side_of_turn == "left":
                if initial_angle >= (3 * math.pi / 2):
                    if normalized_actual_angle > (3 * math.pi / 2)/1.1:
                        return math.pi / 2 - (normalized_actual_angle - initial_angle)
                    else:
                        return math.pi / 2 - (2 * math.pi - initial_angle) - normalized_actual_angle
                else:
                    return initial_angle + math.pi / 2 - normalized_actual_angle
                
            elif side_of_turn == "right":
                if initial_angle == (math.pi / 2):
                    if normalized_actual_angle < (math.pi / 2)*1.1:
                        return math.pi / 2 - (initial_angle - normalized_actual_angle)
                    else:
                        return math.pi / 2 - initial_angle - (2 * math.pi - normalized_actual_angle)
                else:
                    if initial_angle == 2*math.pi:
                        if normalized_actual_angle < math.pi/2:
                            return math.pi/2
                        else:
                            return normalized_actual_angle - (initial_angle - math.pi / 2)
                    else:
                        return normalized_actual_angle - (initial_angle - math.pi / 2)
            return 0.0
        

        def change_orientation(side):
            if self.orientation == "East":
                if side == "right":
                    self.orientation = "South" 
                elif side == "left":
                    self.orientation = "North"  
            elif self.orientation == "North":
                if side == "right":
                    self.orientation = "East"  
                elif side == "left":
                    self.orientation = "West"  
            elif self.orientation == "West":
                if side == "right":
                    self.orientation = "North"  
                elif side == "left":
                    self.orientation = "South"  
            elif self.orientation == "South":
                if side == "right":
                    self.orientation = "West" 
                elif side == "left":
                    self.orientation = "East"  

        if msg.ranges[0] < FRONT_LIMIT and self.operation_mode.name == "FRONT":
            if msg.ranges[90] < msg.ranges[270]:
                self.operation_mode.name = "turn_right"
            else:
                self.operation_mode.name = "turn_left"
        elif self.operation_mode.name != "turn_left" and self.operation_mode.name != "turn_right":
            msg_control_velocity.linear.x = FRONT_VELO


        if self.operation_mode.name == "turn_left":

            radians = radians_to_go(self.angle, "left")

            if radians > 0:

                if radians < 0.4:
                    msg_control_velocity.angular.z =  ANGULAR_VELOCITY_LOW
                msg_control_velocity.angular.z = ANGULAR_VELOCITY_HIGH

            else:
                change_orientation("left")
                self.get_logger().info(f'Finish curve! angle= {self.angle*180/(math.pi)}')

                self.operation_mode.name = "FRONT"


        if self.operation_mode.name == "turn_right":

            radians = radians_to_go(self.angle, "right")

            if radians > 0:
                if radians < 0.4:
                    msg_control_velocity.angular.z =  -ANGULAR_VELOCITY_LOW
                msg_control_velocity.angular.z = -ANGULAR_VELOCITY_HIGH

            else:
                change_orientation("right")
                self.get_logger().info(f'Finish curve! angle= {self.angle*180/(math.pi)}')
                self.operation_mode.name = "FRONT"


        self.publisher_.publish(msg_control_velocity)
       
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
