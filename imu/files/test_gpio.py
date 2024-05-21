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
led_channel = pca.channels[0]

# Increase brightness:
# for i in range(0xffff):
#     time.sleep(0.01)
#     led_channel.duty_cycle = i
#     print(i)


# while True:
#     led_channel.duty_cycle = 0xDEF

# make a while loop that lasts 7 seconds



# led_channel.duty_cycle = 0
# time.sleep(3)

# led_channel.duty_cycle = 0xDEF
# time.sleep(7)

# led_channel.duty_cycle = 0
# time.sleep(3)



start = time.time()
while time.time() - start < 10:
    led_channel.duty_cycle = 0xE10

led_channel.duty_cycle = 0