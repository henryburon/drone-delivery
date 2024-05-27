import rclpy
from rclpy.node import Node
import board, busio
import adafruit_pca9685


class MotorControl(Node):
    def __init__(self):
      super().__init__('motor_control')

      self.timer = self.create_timer(0.5, self.timer_callback)
      self.arm_esc_timer = self.create_timer(0.01, self.arm_esc_callback)

      # set up the PCA9685 board
      i2c = busio.I2C(board.SCL, board.SDA)
      self.pca = adafruit_pca9685.PCA9685(i2c)
      self.pca.frequency = 50
      self.duty_cycle = 2800

      self.led_channel_0 = self.pca.channels[0]
      self.led_channel_2 = self.pca.channels[2]

      # arm the ESC first
      self.esc_armed = False



      self.flag = False

    # put the ESC through the arming sequence, then do nothing
    def arm_esc_callback(self):
        if not self.esc_armed:
            self.led_channel_0.duty_cycle = self.duty_cycle
            self.led_channel_2.duty_cycle = self.duty_cycle
            self.duty_cycle += 2
            self.get_logger().info("Arming ESC: " + str(self.duty_cycle))
        
            if self.duty_cycle >= 3000:
                self.esc_armed = True
                self.get_logger().info("ESC armed!")

    def timer_callback(self):
        if self.esc_armed == True:
            self.get_logger().info("ESC armed. Waiting for user input!")

            if self.flag == False:
               self.flag = True
               self.led_channel_0.duty_cycle = 3680
            
            else:
                self.flag = False
                self.led_channel_0.duty_cycle = 2900








def motor_control_entry(args=None):
   rclpy.init(args=args)
   node = MotorControl()
   rclpy.spin(node)
   rclpy.shutdown()








