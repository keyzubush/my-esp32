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
e_old = 0

for i in range(10**6):

    # get sensors

    distance = sensor.distance_cm()
    if distance < 0: distance = prev_distance
    else: prev_distance = distance
    if i % 10 == 0: accel = mpu.get_values()
    prev_line = line
    line_r = line_r_pin.read()/1000
    line_l = line_l_pin.read()/1000
    dt = (time.time_ns() - t)/10**9
    t = time.time_ns()

    # calculate state

    state = "line"

    # do action


    if state == "line":

        kP = 25
        kD = 250
        v = 70

        e = line_l - line_r
        p = e * kP
        d = (e - e_old) * kD
        u = p + d
        e_old = e

        dc_motor_l.forward(v + u)
        dc_motor_r.forward(v - u)

        if i % 200 == 0:
            print(f"dt={dt}, l={line_l}, r={line_r}, u={u}, e={e}, p={p}, d={d}")