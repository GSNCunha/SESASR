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
    name: str = "NONE"
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
        self.angle = 0
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




        if self.operation_mode.name == "NONE":
            if msg.ranges[90] < 1.5 and msg.ranges[270] < 1.5:
                if msg.ranges[0] > 1:
                    self.operation_mode.name = "hallway"
                    self.get_logger().info(f'I am in a Hallway!')
                else: 
                    self.operation_mode.name = "dead_end"
                    self.get_logger().info(f'No way to go! I may go back')
            elif msg.ranges[90] < 1.5 and msg.ranges[270] > 1.5:
                if msg.ranges[0] > 3.5:
                    self.operation_mode.name = "right_saloon"
                    self.get_logger().info(f'I have a saloon in the right')
                else:
                    self.operation_mode.name = "turn_right"
                    self.operation_mode.front_length = msg.ranges[0]
                    self.get_logger().info('I need to turn right!')
            elif msg.ranges[90] > 1.5 and msg.ranges[270] < 1.5:
                if msg.ranges[0] > 3.5:
                    self.operation_mode.name = "left_saloon"
                    self.get_logger().info(f'I have a saloon in the left')
                else:
                    self.operation_mode.name = "turn_left"
                    self.operation_mode.front_length = msg.ranges[0]
                    self.get_logger().info('I need to turn left!')
            else:
                if msg.ranges[0] < 0.5:
                    self.get_logger().info(f'I need to Turn, why not right?')
                    self.operation_mode.name = "turn_right"
                    self.operation_mode.front_length = msg.ranges[0]
                    self.get_logger().info('I need to turn right!')
                else:
                    self.operation_mode.name = "saloon"
                    self.get_logger().info(f'I am in a saloon')

        if self.operation_mode.name == "hallway":

            if msg.ranges[90] > 1.5:
                self.get_logger().info('Left side open!')
                if msg.ranges[0] < 3.5:
                    self.operation_mode.name = "turn_left"
                    self.operation_mode.front_length = msg.ranges[0]
                    self.get_logger().info('I need to turn left!')
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0
                
                    msg_control_velocity.linear.x = 0.22

            elif msg.ranges[270] > 1.5:
                self.get_logger().info('Right side open!')
                if msg.ranges[0] < 3.5:
                    self.operation_mode.name = "turn_right"
                    self.operation_mode.front_length = msg.ranges[0]
                    self.get_logger().info('I need to turn right!')
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0

                    msg_control_velocity.linear.x = 0.22
            else:
                if msg.ranges[0] > 0.8:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0



        if self.operation_mode.name == "turn_left":
            turn_time = 2*math.pi*(self.operation_mode.front_length/2)/4/0.2
            angular_velocity = math.pi / 2 / turn_time

            radians = radians_to_go(self.angle, "left")

            if radians > 0:
                if msg.ranges[0] < 0.5:
                    self.get_logger().info(f'Dangerous Curve! Lets stop')
                    self.operation_mode.name = "angular_turn_left"
                else:
                    if radians < 0.175:
                        turn_time = 2*math.pi*(self.operation_mode.front_length/2)/4/0.08
                        angular_velocity = math.pi / 2 / turn_time
                        msg_control_velocity.angular.z =  angular_velocity
                        msg_control_velocity.linear.x = 0.08

                    msg_control_velocity.angular.z = angular_velocity
                    msg_control_velocity.linear.x = 0.2
            else:
                change_orientation("left")
                self.get_logger().info(f'Finish curve! angle= {self.angle*180/(math.pi)}')

                self.operation_mode.name = "go_front"
                self.get_logger().info(f'Lets go front!')

                self.operation_mode.go_front = 0.4
                self.operation_mode.x_position = self.location_x
                self.operation_mode.y_position = self.location_y
                self.operation_mode.front_length = 0.0

        if self.operation_mode.name == "angular_turn_left":

            radians = radians_to_go(self.angle, "left")

            if radians > 0:

                if radians < 0.175:
                    msg_control_velocity.angular.z =  0.2
                msg_control_velocity.angular.z = 0.5

            else:
                change_orientation("left")
                self.get_logger().info(f'Finish curve! angle= {self.angle*180/(math.pi)}')

                self.operation_mode.name = "NONE"
                self.operation_mode.go_front = 0.0
                self.operation_mode.x_position = 0.0
                self.operation_mode.y_position = 0.0
                self.operation_mode.front_length = 0.0

        if self.operation_mode.name == "turn_right":
                    turn_time = 2*math.pi*(self.operation_mode.front_length/2)/4/0.2
                    angular_velocity = math.pi / 2 / turn_time

                    radians = radians_to_go(self.angle, "right")

                    if radians > 0:
                        if any(value < 0.5 for value in msg.ranges[350:360] + msg.ranges[0:11]):
                            self.get_logger().info('Dangerous Curve! Lets stop')
                            self.operation_mode.name = "angular_turn_right"

                        else:
                            if radians < 0.175:
                                turn_time = 2*math.pi*(self.operation_mode.front_length/2)/4/0.08
                                angular_velocity = math.pi / 2 / turn_time
                                msg_control_velocity.angular.z = - angular_velocity
                                msg_control_velocity.linear.x = 0.08

                            msg_control_velocity.angular.z = - angular_velocity
                            msg_control_velocity.linear.x = 0.2
                    else:
                        change_orientation("right")
                        self.get_logger().info(f'Finish curve! angle= {self.angle*180/(math.pi)}')

                        self.operation_mode.name = "go_front"
                        self.get_logger().info(f'Lets go front!')
                        self.operation_mode.go_front = 0.4
                        self.operation_mode.x_position = self.location_x
                        self.operation_mode.y_position = self.location_y
                        self.operation_mode.front_length = 0.0

        if self.operation_mode.name == "angular_turn_right":

            radians = radians_to_go(self.angle, "right")

            if radians > 0:
                if radians < 0.175:
                    msg_control_velocity.angular.z =  0.2
                msg_control_velocity.angular.z = -0.5

            else:
                change_orientation("right")
                self.get_logger().info(f'Finish curve! angle= {self.angle*180/(math.pi)}')

                self.operation_mode.name = "NONE"
                self.operation_mode.go_front = 0.0
                self.operation_mode.x_position = 0.0
                self.operation_mode.y_position = 0.0
                self.operation_mode.front_length = 0.0

        if self.operation_mode.name == "go_front":
            if self.orientation == "East":
                if (self.location_x - self.operation_mode.x_position) < self.operation_mode.go_front and msg.ranges[0] > 0.5:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0
            if self.orientation == "North":
                if (self.location_y - self.operation_mode.y_position) < self.operation_mode.go_front and msg.ranges[0] > 0.5:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0
            if self.orientation == "West":
                if (self.operation_mode.x_position - self.location_x) < self.operation_mode.go_front and msg.ranges[0] > 0.5:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0

            if self.orientation == "South":
                if (self.operation_mode.y_position - self.location_y) < self.operation_mode.go_front and msg.ranges[0] > 0.5:
                    msg_control_velocity.linear.x = 0.22
                else:
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0

        if self.operation_mode.name == "left_saloon":
            
            if msg.ranges[90] > 1.5 and msg.ranges[0] > 1.5:
                msg_control_velocity.linear.x = 0.22
            else:
                self.operation_mode.name = "NONE"
                self.operation_mode.go_front = 0.0
                self.operation_mode.x_position = 0.0
                self.operation_mode.y_position = 0.0
                self.operation_mode.front_length = 0.0

        if self.operation_mode.name == "right_saloon":
            
            if msg.ranges[270] > 1.5 and msg.ranges[0] > 1.5:
                msg_control_velocity.linear.x = 0.22
            else:
                self.operation_mode.name = "NONE"
                self.operation_mode.go_front = 0.0
                self.operation_mode.x_position = 0.0
                self.operation_mode.y_position = 0.0
                self.operation_mode.front_length = 0.0  

        if self.operation_mode.name == "saloon":
            
            if msg.ranges[90] >= 1.5 and msg.ranges[0] >= 1 and msg.ranges[270] >= 1.5:
                msg_control_velocity.linear.x = 0.22
            else:
                self.operation_mode.name = "NONE"
                self.operation_mode.go_front = 0.0
                self.operation_mode.x_position = 0.0
                self.operation_mode.y_position = 0.0
                self.operation_mode.front_length = 0.0


        if self.operation_mode.name == "dead_end":
            normalized_actual_angle = normalize_angles(self.angle)
            if self.orientation == "North":
                if normalized_actual_angle < 3*math.pi/2*0.85:
                    msg_control_velocity.angular.z = 1.5
                elif normalized_actual_angle < 3*math.pi/2:
                    msg_control_velocity.angular.z = 0.3
                else:
                    self.orientation = "South"
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0
            elif self.orientation == "South":
                if normalized_actual_angle > math.pi/2/0.85:
                    msg_control_velocity.angular.z = -1.5
                elif normalized_actual_angle > math.pi/2:
                    msg_control_velocity.angular.z = -0.3
                else:
                    self.orientation = "North"
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0

            elif self.orientation == "West":
                if normalized_actual_angle < 2*math.pi*0.85  and normalized_actual_angle > math.pi/2:
                    msg_control_velocity.angular.z = 1.5
                elif normalized_actual_angle < 2*math.pi and normalized_actual_angle > math.pi/2:
                    msg_control_velocity.angular.z = 0.2
                else:
                    self.orientation = "East"
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
                    self.operation_mode.front_length = 0.0

            elif self.orientation == "East":
                if normalized_actual_angle > math.pi/0.85 or normalized_actual_angle < math.pi/2:
                    msg_control_velocity.angular.z = -1.5
                elif normalized_actual_angle > math.pi:
                    msg_control_velocity.angular.z = -0.2
                else:
                    self.orientation = "West"
                    self.operation_mode.name = "NONE"
                    self.operation_mode.go_front = 0.0
                    self.operation_mode.x_position = 0.0
                    self.operation_mode.y_position = 0.0
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
