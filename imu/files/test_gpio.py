import board
import busio
import adafruit_pca9685

print("1")

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

print("2")

pca.frequency = 50
motor_channel = pca.channels[0]


# Increase brightness:
for i in range(0xffff):
    motor_channel.duty_cycle = i