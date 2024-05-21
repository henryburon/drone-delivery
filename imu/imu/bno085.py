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
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import QuaternionStamped, PointStamped


class BNO085(Node):
    def __init__(self):
        super().__init__('bno085')

        # Set up device with Raspberry Pi I2C
        i2c = busio.I2C(board.SCL, board.SDA)
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
        self.timer = self.create_timer(1/5, self.timer_callback)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.quat_pub = self.create_publisher(QuaternionStamped, 'imu/quat', 10)
        self.acc_pub = self.create_publisher(PointStamped, 'imu/acc', 10)
        self.grav_pub = self.create_publisher(PointStamped, 'imu/grav', 10)

    def timer_callback(self):

        # read sensor values each loop
        acc_x, acc_y, acc_z = self.bno.acceleration # acceleration
        gyro_x, gyro_y, gyro_z = self.bno.gyro # gyro
        mag_x, mag_y, mag_z = self.bno.magnetic # magnetometer
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion # rotation vector quaternion
        grav_x, grav_y, grav_z = self.bno.gravity # gravity
        lin_acc_x, lin_acc_y, lin_acc_z = self.bno.linear_acceleration # linear acceleration

        # make the IMU message
        imu_msg = Imu()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.linear_acceleration.x = lin_acc_x
        imu_msg.linear_acceleration.y = lin_acc_y
        imu_msg.linear_acceleration.z = lin_acc_z

        self.imu_pub.publish(imu_msg)

        # make the MagneticField message
        mag_msg = MagneticField()

        mag_msg.header.stamp = self.get_clock().now().to_msg()

        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        self.mag_pub.publish(mag_msg)

        # make the QuaternionStamped message
        quat_msg = QuaternionStamped()

        quat_msg.header.stamp = self.get_clock().now().to_msg()

        quat_msg.quaternion.x = quat_i
        quat_msg.quaternion.y = quat_j
        quat_msg.quaternion.z = quat_k
        quat_msg.quaternion.w = quat_real

        self.quat_pub.publish(quat_msg)

        # make the Acceleration (PointStamped) message
        acc_msg = PointStamped()

        acc_msg.header.stamp = self.get_clock().now().to_msg()

        acc_msg.point.x = acc_x
        acc_msg.point.y = acc_y
        acc_msg.point.z = acc_z

        self.acc_pub.publish(acc_msg)

        # make the Gravity (PointStamped) message
        grav_msg = PointStamped()

        grav_msg.header.stamp = self.get_clock().now().to_msg()

        grav_msg.point.x = grav_x
        grav_msg.point.y = grav_y
        grav_msg.point.z = grav_z
        
        self.grav_pub.publish(grav_msg)



def bn085_entry(args=None):
    rclpy.init(args=args)
    node = BNO085()
    rclpy.spin(node)
    rclpy.shutdown()
