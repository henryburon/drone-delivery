import serial
import time
import board
import busio
import adafruit_pca9685
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

# Initialize I2C, PCA9685, BNO08X
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50

# Enable gyroscope sensor
bno.enable_feature(BNO_REPORT_GYROSCOPE)

# Define motor channels
servo_channel = pca.channels[0]
motor_channel_fl = pca.channels[1]
motor_channel_bl = pca.channels[2]
motor_channel_br = pca.channels[3]
motor_channel_fr = pca.channels[4]

us_to_duty_cycle = lambda us: int((us / 20000) * 65535)

# Arm motors
start_time = time.time()
while time.time() - start_time < 3:
    motor_channel_fr.duty_cycle = us_to_duty_cycle(1630) 
    motor_channel_bl.duty_cycle = us_to_duty_cycle(1630)
    motor_channel_br.duty_cycle = us_to_duty_cycle(1630)
    motor_channel_fl.duty_cycle = us_to_duty_cycle(1630)

# P-controller gain
Kp = 40
tol = 0.05

# Set up serial connection
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    print("Serial connection established")
except serial.SerialException as e:
    print(f"Could not open serial port: {e}")
    exit(1)

stabilize_mode = False

# Read serial and perform corresponding action
try:
    while True:
        
        gyro_x, gyro_y, gyro_z = bno.gyro
        p = Kp * gyro_z
        pwm = 1670 + p
        pwm = min(pwm, 2000)

        if stabilize_mode == True:
            # if it's spinning CCW...
            if gyro_z > 0 + tol:
                motor_channel_fr.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_bl.duty_cycle = us_to_duty_cycle(1630)

                motor_channel_fl.duty_cycle = us_to_duty_cycle(pwm)
                motor_channel_br.duty_cycle = us_to_duty_cycle(pwm)
            
            # if it's spinning CW...
            elif gyro_z < 0 - tol:
                motor_channel_fl.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_br.duty_cycle = us_to_duty_cycle(1630)

                #val = 1670 + (p*2)
                #val = min(val, 2000)
            
                motor_channel_fr.duty_cycle = us_to_duty_cycle(pwm)
                motor_channel_bl.duty_cycle = us_to_duty_cycle(pwm)

            else:
                motor_channel_fr.duty_cycle = us_to_duty_cycle(1630) 
                motor_channel_bl.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_br.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_fl.duty_cycle = us_to_duty_cycle(1630)

        # Check if there's data in the serial buffer
        while ser.in_waiting > 0:
            # Read one byte at a time, decode it, and strip any whitespace
            data = ser.readline().decode('utf-8').strip()
            print(f"Received data: {data}")

            # Perform corresponding action
            if data == '1':
                # IN winch
                continue
            elif data == '2':
                # STOP winch
                continue
            elif data == '3':
                # OUT winch
                continue
            elif data == '4':
                # OPEN tether
                ms = 1400
                open_dc = int((ms/20000) * 65535)
                servo_channel.duty_cycle = open_dc
                
            elif data == '5':
                # CLOSE tether hook
                ms = 318
                close_dc = int((ms/20000) * 65535)
                servo_channel.duty_cycle = close_dc

            elif data == '6':
                # STABILIZE ON
                stabilize_mode = True

            elif data == '7':
                # STABILIZE OFF
                stabilize_mode = False

                motor_channel_fr.duty_cycle = us_to_duty_cycle(1630) 
                motor_channel_bl.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_br.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_fl.duty_cycle = us_to_duty_cycle(1630)

            elif data == '8':
                # FORWARD motion
                motor_channel_fl.duty_cycle = us_to_duty_cycle(1580)
                motor_channel_bl.duty_cycle = us_to_duty_cycle(1580)
                motor_channel_br.duty_cycle = us_to_duty_cycle(1675)
                motor_channel_fr.duty_cycle = us_to_duty_cycle(1675)
                
            elif data == '9':
                # STOP motion 
                motor_channel_fr.duty_cycle = us_to_duty_cycle(1630) 
                motor_channel_bl.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_br.duty_cycle = us_to_duty_cycle(1630)
                motor_channel_fl.duty_cycle = us_to_duty_cycle(1630)
            else:
                print("Received something else")

except KeyboardInterrupt:
    motor_channel_fr.duty_cycle = us_to_duty_cycle(1630) 
    motor_channel_bl.duty_cycle = us_to_duty_cycle(1630)
    motor_channel_br.duty_cycle = us_to_duty_cycle(1630)
    motor_channel_fl.duty_cycle = us_to_duty_cycle(1630)
