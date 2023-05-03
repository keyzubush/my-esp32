from machine import Pin, PWM, ADC
from machine import I2C
from time import sleep
import time
import sys
import random

from hcsr04 import HCSR04
import mpu6050
import dcmotor

line_r_pin = ADC(Pin(34))
line_l_pin = ADC(Pin(35))

freq = 2000
dc_motor_l = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), freq))
dc_motor_r = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), freq))

sensor = HCSR04(trigger_pin=2, echo_pin=18)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = mpu6050.accel(i2c)
servo = PWM(Pin(4, mode=Pin.OUT), freq=50)

prev_state = state = "forward"
corr = fw_count = angle = 0
prev_distance = distance = 50
t = time.time_ns()
prev_line = line = 1000
line_trigger = True

for i in range(10**6):
    
    # get sensors

    distance = sensor.distance_cm()
    if distance < 0: distance = prev_distance
    else: prev_distance = distance
    accel = mpu.get_values()
    prev_line = line
    line_r = line_r_pin.read()
    line_l = line_l_pin.read()
    dt = (time.time_ns() - t)/10**9
    t = time.time_ns()

    # calculate state

    state = "line"

    # do action


    if state == "line":

        kP = 1
        kD = 10
        eOld = 0
        v = 70

        e = line_l - line_r
        p = e * kP
        d = (e - eOld) * kD
        u = p + d
        eOld = e

        dc_motor_l.forward(v + u)
        dc_motor_r.forward(v - u)

        time.sleep(0.001)

        print(line_l, line_r, u, e, p, d)