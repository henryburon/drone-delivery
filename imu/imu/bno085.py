import rclpy
from rclpy.node import Node



class BNO085(Node):
    def __init__(self):
        super().__init__('bno085')

        # Timers
        self.timer = self.create_timer(1/100, self.timer_callback)

    
    def timer_callback(self):
        self.get_logger().info('Hello World!')




def bn085_entry(args=None):
    rclpy.init(args=args)
    node = BNO085()
    rclpy.spin(node)
    rclpy.shutdown()