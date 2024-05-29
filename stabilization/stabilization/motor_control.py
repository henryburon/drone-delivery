import rclpy
from rclpy.node import Node
import board, busio
import adafruit_pca9685

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import QuaternionStamped, PointStamped


class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # subscribers
        self.imu_data_sub = self.create_subscription(Imu, 'imu/data', self.imu_data_callback, 10)

        # timers
        self.timer = self.create_timer(0.10, self.timer_callback)
        self.arm_esc_timer = self.create_timer(0.01, self.arm_esc_callback)

        # set up the PCA9685 board
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = adafruit_pca9685.PCA9685(i2c)
        self.pca.frequency = 50
        self.duty_cycle = 2800

        self.led_channel_0 = self.pca.channels[0] # CW = channel 0
        self.led_channel_2 = self.pca.channels[2] # CCW = channel 2

        # arm the ESC first
        self.esc_armed = False

        # imu data structure
        self.imu_data = {}
        self.imu_data['linear_acceleration'] = [0, 0, 0]
        self.imu_data['gyro'] = [0, 0, 0]

        # other variables
        self.flag = True # set back to false after testing
        self.Kp = 10.0
        self.Ki = 0.0
        self.Kd = 0.0

    # put the ESC through the arming sequence as the first step
    def arm_esc_callback(self):
        if not self.esc_armed:
            self.led_channel_0.duty_cycle = self.duty_cycle
            self.led_channel_2.duty_cycle = self.duty_cycle
            self.duty_cycle += 2
            self.get_logger().info("Arming ESC: " + str(self.duty_cycle))
        
            if self.duty_cycle >= 3000:
                self.esc_armed = True
                self.get_logger().info("ESC armed!")

    # main timer callback
    def timer_callback(self):
        if self.esc_armed == True:
            self.get_logger().info("ESC armed. Waiting for user input!", once=True)

            self.stabilize_angle_z()

            # if self.flag == False:
            #    self.flag = True
            #    self.led_channel_0.duty_cycle = 3680
            #    self.led_channel_2.duty_cycle = 3680
            
            # else:
            #     self.flag = False
            #     self.led_channel_0.duty_cycle = 2900
                # self.led_channel_2.duty_cycle = 2900

            # log the imu data
            # self.get_logger().info("IMU data: ")
            # self.get_logger().info("Gyro: " + str(self.imu_data['gyro']))

    def stabilize_angle_z(self):

        # P controller
        error = 0.0 - self.imu_data['gyro'][2]

        # log the error
        self.get_logger().info("Error: " + str(error))

        # if we detect any error at all...
        if abs(error) >= 0.1:

            # if the box is rotating CW...
            if error > 0:
                # ensure CW-pushing motor is off
                self.led_channel_0.duty_cycle = 3200

                # determine the duty cycle for the CCW-pushing motor
                output_percent = self.Kp * abs(error)

                # convert the desired motor percentage to a duty cycle
                duty_cycle = int(self.percentage_to_duty_cycle(output_percent, 2))

                # log the duty cycle
                self.get_logger().info("Duty cycle: " + str(duty_cycle))

                # set the CCW-pushing motor's duty cycle
                self.led_channel_2.duty_cycle = duty_cycle

            # if the box is rotating CCW...
            else:
                # ensure CCW-pushing motor is off
                self.led_channel_2.duty_cycle = 3200

                # determine the duty cycle for the CW-pushing motor
                output_percent = self.Kp * abs(error)

                # convert the desired motor percentage to a duty cycle
                duty_cycle = int(self.percentage_to_duty_cycle(output_percent, 0))

                # log the duty cycle
                self.get_logger().info("Duty cycle: " + str(duty_cycle))

                # set the CW-pushing motor's duty cycle
                self.led_channel_0.duty_cycle = duty_cycle

        # safety feature: if the error is too large, just shut off the motors
        # elif abs(error) > 7.0:
        #     self.led_channel_0.duty_cycle = 3200
        #     self.led_channel_2.duty_cycle = 3200

        # if there is no error...
        else:
            # ensure both motors are off
            self.led_channel_0.duty_cycle = 3200
            self.led_channel_2.duty_cycle = 3200


                 
            







    # this will return the duty cycle value
    def percentage_to_duty_cycle(self, percentage, channel):

        if percentage == 0:
            return 3200

        percent_range = [0, 100]
        duty_cycle_range = [3600, 4500]

        scaled_value = (percentage - percent_range[0]) * (duty_cycle_range[1] - duty_cycle_range[0]) / (percent_range[1] - percent_range[0]) + duty_cycle_range[0]

        if channel == 0:
            scaled_value += 75


        return scaled_value

         











    # collect data from imu/data
    def imu_data_callback(self, msg):
        self.imu_data['linear_acceleration'] = [msg.linear_acceleration.x, 
                                                msg.linear_acceleration.y, 
                                                msg.linear_acceleration.z]
        
        # z axis is the one we care about, right now
        self.imu_data['gyro'] = [msg.angular_velocity.x,
                                 msg.angular_velocity.y,
                                 msg.angular_velocity.z]
                        






def motor_control_entry(args=None):
   rclpy.init(args=args)
   node = MotorControl()
   rclpy.spin(node)
   rclpy.shutdown()








