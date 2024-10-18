import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('Localization')
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_topic',
            self.listener_callback,
            10)
        self.subscription 

        self.subscription_reset = self.create_subscription(
            Bool,
            'reset',
            self.reset_callback,
            10)
        self.subscription_reset 

        self.location = type('Location', (), {})() 
        self.location.x = 0.0
        self.location.y = 0.0

    def listener_callback(self, msg):
        self.location.x += msg.linear.x
        self.location.y += msg.linear.y

        pose_msg = Pose()
        pose_msg.position.x = self.location.x
        pose_msg.position.y = self.location.y
        pose_msg.position.z = 0.0 

        self.publisher_.publish(pose_msg)

        self.get_logger().info(f'Updated Location: X = "{self.location.x}", Y = "{self.location.y}"')

    def reset_callback(self, msg):
        if msg.data:
            self.location.x = 0.0
            self.location.y = 0.0

            pose_msg = Pose()
            pose_msg.position.x = float(self.location.x)
            pose_msg.position.y = float(self.location.y)
            pose_msg.position.z = 0.0

            self.publisher_.publish(pose_msg)

            self.get_logger().info(f'Reset!')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
