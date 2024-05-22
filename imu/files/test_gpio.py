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

# ARM the ESC first. This is entirely the ESC, not the PCA9685
for i in range(0xB54, 0xBB8, 2): # safe: 2500 - 3000
    time.sleep(0.01)
    led_channel.duty_cycle = i
    print(i)

print("armed!")


try:
   print("starting!")

   # set to 4000
   led_channel.duty_cycle = 0xFA0
   time.sleep(3)
   led_channel.duty_cycle = 0x1194
   time.sleep(3)
   led_channel.duty_cycle = 0xFA0
   time.sleep(3)

   print("ending!")
   time.sleep(5)

except KeyboardInterrupt:
      led_channel.duty_cycle = 0x000
      print("stopped!")