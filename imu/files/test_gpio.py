import board
import busio
import adafruit_pca9685

print("1")

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

print("2")

pca.frequency = 50
motor_channel = pca.channels[0]

while True:
   motor_channel.duty_cycle = 0x7fff


# while True:
#    motor_channel.duty_cycle = 0xAFC8 # in decimal, this is 39000
