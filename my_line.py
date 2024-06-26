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
dc_motor_r = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), freq))
dc_motor_l = dcmotor.DCMotor(Pin(26, Pin.OUT), Pin(27, Pin.OUT), PWM(Pin(14), freq))

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
I = 0

for i in range(10**6):

    # get sensors

    # distance = sensor.distance_cm()
    # if distance < 0: distance = prev_distance
    # else: prev_distance = distance
    # if i % 10 == 0: accel = mpu.get_values()
    time.sleep_ms(2)
    
    prev_line = line
    line_r = line_r_pin.read()/4096
    line_l = line_l_pin.read()/4006
    dt = (time.time_ns() - t)/10**9
    t = time.time_ns()

    # calculate state

    state = "line"

    # do action


    if state == "line":

        kP = 100
        kI = 200
        kD = 1
        iMax = 25
        v = 80

        e = line_r - line_l
        P = e * kP 
        I += e * kI * dt
        D = (e - e_old) * kD / dt

        if(abs(I) > iMax): I = iMax * (1 if I > 0 else -1)

        u = P + I + D
        e_old = e

        dc_motor_l.move(v + u)
        dc_motor_r.move(v - u)

        if i % 20 == 0:
            print(f"dt={dt}, l={line_l}, r={line_r}, u={u}, e={e}, P={P}, I={I}, D={D}")
