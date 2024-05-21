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

i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)



while True:
      accel_x, accel_y, accel_z = bno.acceleration
      print(accel_x, accel_y, accel_z)

      gyro_x, gyro_y, gyro_z = bno.gyro
      print(gyro_x, gyro_y, gyro_z)

      time.sleep(0.5)