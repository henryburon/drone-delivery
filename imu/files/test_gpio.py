import board
import busio
import adafruit_pca9685
import time
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

#3567


# this code is to control a brushless motor with ESC

# set the frequency of the PWM signal to 50Hz
pca.frequency = 50

# make the motor go forward
led_channel = pca.channels[2]

# while True:
#       led_channel.duty_cycle = 0xDF9
#       time.sleep(5)
#       led_channel.duty_cycle = 0x000
#       time.sleep(1)


# Increase brightness:
# for i in range(0, 0xffff, 500):
#     time.sleep(1.5)
#     led_channel.duty_cycle = i
#     print(i)


# set to 3000 for a second, then 4000
led_channel.duty_cycle = 0xBB8
time.sleep(1)
led_channel.duty_cycle = 0xFA0
time.sleep(5)


