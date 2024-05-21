import board
import busio
import adafruit_pca9685
from time import sleep

print("1")

i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

print("2")

pca.frequency = 50
motor_channel = pca.channels[0]


print("3")

motor_channel.duty_cycle = 0x9858 # in decimal, this is 39000
sleep(1)
motor_channel.duty_cycle = 0
sleep(1)

print("ending")