import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose

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

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
