import board
import busio
import adafruit_pca9685
import time
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)


# set the frequency of the PWM signal to 50Hz
pca.frequency = 50

# make the motor go forward
# led_channel_0 = pca.channels[0]
led_channel_2 = pca.channels[2]

# ARM the ESC first. This is entirely the ESC, not the PCA9685
# to arm the ESC, the min range I can get it to work at is 2800 - 3000, skipping 2
# could potentially get a smaller range when skipping fewer numbers
for i in range(2800, 3000, 2): # safe: 2500 - 3000
   time.sleep(0.01)
   # led_channel_0.duty_cycle = i
   led_channel_2.duty_cycle = i
   print(i)

print("armed!")



while True:
   # ask the user to input an integer
   user_input = input("Enter a duty cycle (int): ")
   # led_channel_0.duty_cycle = int(user_input)
   led_channel_2.duty_cycle = int(user_input)
   print("Duty cycle set to: " + str(user_input))
   time.sleep(2)



# try:
#    print("starting!")

#    # set to 4000
#    led_channel.duty_cycle = 0xFA0
#    time.sleep(3)
#    led_channel.duty_cycle = 0x1194
#    time.sleep(3)
#    led_channel.duty_cycle = 0xFA0
#    time.sleep(3)

#    print("ending!")
#    time.sleep(5)

# except KeyboardInterrupt:
#       led_channel.duty_cycle = 0x000
#       print("stopped!")