import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        
        self.declare_parameter('param1', 'world')
        param1 = self.get_parameter('param1').get_parameter_value().string_value
        
        self.declare_parameter('param2', '2.0')
        param2 = self.get_parameter('param2').get_parameter_value().double_value
