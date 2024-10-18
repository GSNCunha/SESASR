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

from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.movements = 0
        self.movements_set = 1

    def timer_callback(self):
        msg = Twist()
        
        if self.movements < self.movements_set:
            if self.i == 0:
                msg.linear.x = 1.0
                self.movements += 1
            elif self.i == 1:
                msg.linear.y = 1.0
                self.movements += 1
            elif self.i == 2:
                msg.linear.x = -1.0
                self.movements += 1
            elif self.i == 3:
                msg.linear.y = -1.0
                self.movements += 1

        if self.movements == self.movements_set:
            self.movements_set +=1
            self.movements = 0
            if self.i == 3:
                self.i = 0
            else:
                self.i += 1




        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: X = "{msg.linear.x}" Y = "{msg.linear.y}"')

        



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
