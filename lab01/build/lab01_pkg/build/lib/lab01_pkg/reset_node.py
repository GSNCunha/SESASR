import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('reset_node')
        self.publisher_ = self.create_publisher(Bool, '/reset', 10)
        self.subscription = self.create_subscription(
            Pose,
            'pose',
            self.pose_callback,
            10)
        self.subscription  

    def pose_callback(self, msg):
        if msg.position.x > 6 or msg.position.y > 6 or msg.position.x < -6 or msg.position.y < -6:
            reset_msg = Bool()
            reset_msg.data = True
            self.publisher_.publish(reset_msg)
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
