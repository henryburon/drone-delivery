import board
import busio
import adafruit_pca9685

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

# Set the PWM frequency
pca.frequency = 50
motor_channel = pca.channels[0]

# Function to convert percentage to hex value
def percent_to_duty_cycle(percent):
    if not 0 <= percent <= 100:
        raise ValueError("Percent must be between 0 and 100")
    
    # Calculate the corresponding duty cycle value
    duty_cycle = int((percent / 100) * 0xFFFF)
    
    return duty_cycle

percent = 5
duty_cycle_value = percent_to_duty_cycle(percent)

print(f"Duty cycle value for {percent}% is {duty_cycle_value}")

# Set the motor channel's duty cycle in a loop
while True:
    motor_channel.duty_cycle = duty_cycle_value

# while True:
#    motor_channel.duty_cycle = 0xAFC8 # in decimal, this is 39000
