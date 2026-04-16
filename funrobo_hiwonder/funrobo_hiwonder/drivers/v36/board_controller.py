#!/usr/bin/env python3
import sys
import time
import numpy as np
from smbus2 import SMBus, i2c_msg
# from rpi_ws281x import PixelStrip
# from rpi_ws281x import Color as PixelColor


# Define constants
MOTOR_TYPE_JGB37_520_12V_110RPM = 3 # the magnetic ring generates 44 pulses per revolution, combined with a gear reduction ratio of 90 Default

ENCODER_MOTOR_MODULE_ADDR = 0x34
MOTOR_TYPE_ADDR = 20 #0x20
MOTOR_ENCODER_POLARITY_ADDR = 21 #0x21
ADC_BAT_ADDR = 0x00
MOTOR_ENCODER_TOTAL_ADDR = 60 #0x60
MOTOR_FIXED_PWM_ADDR = 31 #0x31 
MOTOR_FIXED_SPEED_ADDR = 51 #0x51 
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0


class BoardController:
    def __init__(self, i2c_addr=0x7A, i2c_bus=1):
        if sys.version_info.major == 2:
            print('Please run this program with Python 3!')
            sys.exit(0)

        self.i2c_addr = i2c_addr
        self.i2c_bus = i2c_bus
        self.motor_speed = [0, 0, 0, 0]
        self.servo_angle = [0, 0, 0, 0, 0, 0]
        self.servo_pulse = [0, 0, 0, 0, 0, 0]
        
        # Constants
        self.ADC_BAT_ADDR = 0
        self.SERVO_ADDR = 21
        self.MOTOR_ADDR = 31
        self.SERVO_CMD_ADDR = 40

        # Initialize I2C
        self.bus = SMBus(self.i2c_bus)

        self.initialize_motors()


    def initialize_motors(self):
        # initialize chassis motors
        try:
            time.sleep(1)
            self.bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_TYPE_ADDR, MotorType) # Set motor type
            time.sleep(0.5)
            self.bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)  # Set encoding polarity
            print('Encoder motor driver module has been initialized!! \n')
        except Exception as e:
            print(f"[ERROR] Failed to initialize motors: {e}")


    def set_motor_speed(self, speed: list):
        # uses MOTOR_FIXED_SPEED_ADDR to set speed
        try:
            if len(speed) != 4:
                raise ValueError("Speed list must contain exactly 4 values.")
            speed = np.clip(speed, -100, 100)
            speed_ = [int(s) for s in speed]
            self.bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR, speed_)
            self.motor_speed = speed_
        except Exception as e:
            print(f"[ERROR] Failed to set motor speed: {e}")


    def get_motor_speed(self, index):
        return self.motor_speed


    def close(self):
        """Closes the I2C bus"""
        self.bus.close()