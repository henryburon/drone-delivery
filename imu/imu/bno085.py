import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C



class BNO085(Node):
    def __init__(self):
        super().__init__('bno085')

        # Set up device with Jetson Nano
        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.bno = BNO08X_I2C(i2c, address=0x4A)

        # Enable base features
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        # Begin the sensor's self-calibration routine
        self.bno.begin_calibration()

        # Timers
        self.timer = self.create_timer(1/100, self.timer_callback)

    
    def timer_callback(self):
        self.get_logger().info('Hello World!')

        # Get Acceleration
        accel_x, accel_y, accel_z = self.bno.acceleration
        self.get_logger().info(str(accel_x))




def bn085_entry(args=None):
    rclpy.init(args=args)
    node = BNO085()
    rclpy.spin(node)
    rclpy.shutdown()