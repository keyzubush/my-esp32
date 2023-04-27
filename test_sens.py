from machine import Pin, PWM
from machine import I2C
from time import sleep
import time
import sys

from hcsr04 import HCSR04
import mpu6050
import dcmotor

dc_motor_l = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), 1000))
dc_motor_r = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), 1000))

sensor = HCSR04(trigger_pin=2, echo_pin=18, echo_timeout_us=10000)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))     #initializing the I2C method for ESP32
mpu = mpu6050.accel(i2c)


for n in range(5):

    if n % 2 == 0:
        dc_motor_l.forward(100)
        dc_motor_r.forward(100)
    else:
        dc_motor_l.backwards(100)
        dc_motor_r.backwards(100)

    for i in range(5):
        time.sleep(0.1)
        distance = sensor.distance_cm()
        mpu.get_values()
        print('Distance:', distance, 'cm, ', mpu.get_values())

    dc_motor_l.stop()
    dc_motor_r.stop()
    time.sleep(1)

    if n % 2 == 0:
        dc_motor_l.forward(100)
        dc_motor_r.backwards(100)
    else:
        dc_motor_r.forward(100)
        dc_motor_l.backwards(100)

    for i in range(25):
        time.sleep(0.1)
        distance = sensor.distance_cm()
        mpu.get_values()
        print('Distance:', distance, 'cm, ', mpu.get_values())
    
    dc_motor_l.stop()
    dc_motor_r.stop()

dc_motor_l.stop()
dc_motor_r.stop()

del sys.modules['test_sens']

