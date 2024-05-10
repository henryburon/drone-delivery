import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_LINEAR_ACCELERATION
)
from adafruit_bno08x.i2c import BNO08X_I2C
from sensor_msgs.msg import Imu, MagneticField #, Quaternion


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
        self.bno.enable_feature(BNO_REPORT_GRAVITY)
        self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        # Begin the sensor's self-calibration routine
        self.bno.begin_calibration()

        # Timers
        self.timer = self.create_timer(1/20, self.timer_callback)

        # Publishers
        # could eventually make a custom message type, but this is simpler for now
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

    
    def timer_callback(self):

        # Read sensor values
        accel_x, accel_y, accel_z = self.bno.acceleration # acceleration
        gyro_x, gyro_y, gyro_z = self.bno.gyro # gyro
        mag_x, mag_y, mag_z = self.bno.magnetic # magnetometer
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion # rotation vector quaternion
        grav_x, grav_y, grav_z = self.bno.gravity # gravity
        lin_acc_x, lin_acc_y, lin_acc_z = self.bno.linear_acceleration # linear acceleration

        imu_msg = Imu()
        imu_msg.linear_acceleration.x = lin_acc_x
        imu_msg.linear_acceleration.y = lin_acc_y
        imu_msg.linear_acceleration.z = lin_acc_z
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        self.imu_pub.publish(imu_msg)





def bn085_entry(args=None):
    rclpy.init(args=args)
    node = BNO085()
    rclpy.spin(node)
    rclpy.shutdown()