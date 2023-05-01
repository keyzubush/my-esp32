from machine import Pin, PWM, ADC
from machine import I2C
from time import sleep
import time
import sys
import random

from hcsr04 import HCSR04
import mpu6050
import dcmotor

freq = 2000
dc_motor_l = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), freq))
dc_motor_r = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), freq))

sensor = HCSR04(trigger_pin=2, echo_pin=18)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = mpu6050.accel(i2c)
servo = PWM(Pin(4, mode=Pin.OUT), freq=50)
line_pin = ADC(Pin(34))

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
    line = line_pin.read()
    dt = (time.time_ns() - t)/10**9
    t = time.time_ns()

    # calculate state

    if distance < 15:
        state = "turn"
    else:
        state = "forward"

    if prev_state == "forward" and accel["GyX"] > 1750:
        state = "turn"

    # do action

    if state == "forward":
        dc_motor_l.forward(100 - corr)
        dc_motor_r.forward(100 + corr)
        if accel['GyZ'] < 0: 
            corr += 1
        else:
            corr -= 1

    if state == "turn":
        dc_motor_l.stop()
        dc_motor_r.stop()
        servo.duty(50)
        sleep(1)
        d_l = sensor.distance_cm()
        servo.duty(100)
        sleep(1)
        d_r = sensor.distance_cm()
        servo.duty(75)
        sleep(1)
        dc_motor_l.backwards(50)
        dc_motor_r.backwards(50)
        time.sleep(0.5)
        angle = 0
        turn_time = 0
        while angle < 5500 and turn_time < 5:
            accel = mpu.get_values()
            if d_r > d_l:
                dc_motor_l.forward(100 - corr)
                dc_motor_r.backwards(100 + corr)
            else:
                dc_motor_r.forward(100 + corr)
                dc_motor_l.backwards(100 - corr)
            angle += abs(accel['GyZ'])*0.1
            time.sleep(0.1)
            turn_time += 0.1
        angle = 0

    if state == "stop":
        dc_motor_l.stop()
        dc_motor_r.stop()
        time.sleep(1)
        dc_motor_l.forward(100)
        dc_motor_r.forward(100)


    # print
        
    if i % 10 == 0:
        print(f'd={distance}, accel={accel}, line={line}, st={state}, corr={corr}, dt={dt}')
        prev_state = state
