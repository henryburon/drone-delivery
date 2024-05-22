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

# Increase brightness:
# for i in range(0, 0xffff, 4):
#     time.sleep(0.01)
#     led_channel.duty_cycle = i
#     print(i)

for i in range(0x9C4, 0xBB8, 2):
    time.sleep(0.01)
    led_channel.duty_cycle = i
    print(i)




try:
   print("starting!")

   # # initialize by setting to 3000
   # led_channel.duty_cycle = 0xBB8
   # time.sleep(0.5)

   # set to 4000
   led_channel.duty_cycle = 0xFA0
   time.sleep(15)

   print("ending!")

except KeyboardInterrupt:
      led_channel.duty_cycle = 0x000
      print("stopped!")