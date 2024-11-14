# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ekf import *

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('EKF_node')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        timer_period = 1/20  # 20 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.v = 0.0
        self.w = 0.0

        
    def timer_callback(self):
        timer_period = 1

    def odom_callback(self, msg: Odometry):
        # Extract the linear and angular velocity from the odometry message
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
