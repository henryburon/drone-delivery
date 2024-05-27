import rclpy
from rclpy.node import Node




class MotorControl(Node):
    def __init__(self):
      super().__init__('motor_control')

      self.create_timer(0.5, self.timer_callback)



    def timer_callback(self):
        print("Hello, world!")







def motor_control_entry(args=None):
   rclpy.init(args=args)
   node = MotorControl()
   rclpy.spin(node)
   rclpy.shutdown()








