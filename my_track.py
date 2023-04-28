from machine import Pin, PWM, ADC
from machine import I2C
from time import sleep
import time
import sys
import random

from hcsr04 import HCSR04
import mpu6050
import dcmotor

dc_motor_l = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), 1000))
dc_motor_r = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), 1000))

sensor = HCSR04(trigger_pin=2, echo_pin=18, echo_timeout_us=10000)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))     #initializing the I2C method for ESP32
mpu = mpu6050.accel(i2c)

line_pin = ADC(Pin(34))


state = "forward"
corr = 0
fw_count = 0

for i in range(10**6):
    
    # get sensors

    distance = -1
    while distance < 0: distance = sensor.distance_cm()
    accel = mpu.get_values()
    line = line_pin.read()
    if i % 100 == 0: print('Distance:', distance, 'cm, ', accel, line)

    # calculate state

    if distance < 10:
        state = "turn"
        fw_count = 0

    if distance > 50: state = "forward"

    if line > 4000: state = "stop"

    # do action

    if state == "forward":
        dc_motor_l.forward(80 - corr)
        dc_motor_r.forward(80 + corr)
        fw_count += 1
        if accel['GyZ'] < 0 and fw_count > 10: 
            corr += 1
        else:
            corr -= 1

    if state == "turn":
        if random.randrange(2) > 0:
            dc_motor_l.forward(50)
            dc_motor_r.backwards(50)
        else:
            dc_motor_r.forward(50)
            dc_motor_l.backwards(50)
        time.sleep(0.5)

    if state == "stop":
        dc_motor_l.stop()
        dc_motor_r.stop()
        time.sleep(1)
        dc_motor_l.forward(100)
        dc_motor_r.forward(100)


