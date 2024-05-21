import board
import busio
import time
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GRAVITY,
    BNO_REPORT_LINEAR_ACCELERATION
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_GRAVITY)
bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)



while True:
      accel_x, accel_y, accel_z = bno.acceleration
      print(accel_x, accel_y, accel_z)

      gyro_x, gyro_y, gyro_z = bno.gyro
      print(gyro_x, gyro_y, gyro_z)

      mag_x, mag_y, mag_z = bno.magnetic
      print(mag_x, mag_y, mag_z)

      quat_i, quat_j, quat_k, quat_real = bno.quaternion
      print(quat_i, quat_j, quat_k, quat_real)

      grav_x, grav_y, grav_z = bno.gravity
      print(grav_x, grav_y, grav_z)

      lin_acc_x, lin_acc_y, lin_acc_z = bno.linear_acceleration
      print(lin_acc_x, lin_acc_y, lin_acc_z)

      