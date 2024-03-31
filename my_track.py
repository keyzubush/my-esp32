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
dc_motor_r = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), freq))
dc_motor_l = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), freq))

sensor = HCSR04(trigger_pin=2, echo_pin=18)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = mpu6050.accel(i2c)
servo = PWM(Pin(4, mode=Pin.OUT), freq=50)
line_pin = ADC(Pin(34))

prev_state = state = "forward"
corr = fw_count = angle = 0
prev_distance = distance = 50
t = time.time_ms()
e = 0

LINE = 1.5
LINE_TIMEOUT = 2.0 # sec
RIGHT_ANGLE = 5000
K_P = 10
K_D = 100
V = 100

i = 0
while True:
    i += 1
    
    # get sensors

    distance = sensor.distance_cm()
    distance = distance if distance > 0 else prev_distance
    accel = mpu.get_values()
    line_r = line_r_pin.read()/1000
    line_l = line_l_pin.read()/1000
    dt = (time.time_ms() - t) / 10**6
    t = time.time_ms()

    # calculate state

    if distance < 10 and state != "line" and state != "detour":
        state = "turn"
    elif distance < 15 and state == "line":
        state = "detour"
    elif prev_state == "detour" and abs(line_l - line_r) > LINE:
        pass

    elif abs(line_l - line_r) > 1 or prev_state == "line":
        if abs(line_l - line_r) > LINE:
            t_line = time.time_ms()
        if time.time_ms() - t_line > LINE_TIMEOUT:
            state = "forward"
    elif prev_state == "line":
    else:
        state = "forward"

    # do action

    if state == "forward":
        dc_motor_l.forward(V - u)
        dc_motor_r.forward(V + u)
        if accel['GyZ'] < 0: 
            u += 1
        else:
            u -= 1

    if state == "line":
        e = line_l - line_r
        p = e * K_P
        d = (e - e_old) * K_D
        u = p + d
        e_old = e
        dc_motor_l.forward(V + u)
        dc_motor_r.forward(V - u)

    if state == "search":
        angle += abs(accel['GyZ'])*dt
        if angle < RIGHT_ANGLE:
            dc_motor_l.forward(50)
            dc_motor_r.stop()
        else:
            dc_motor_l.stop()
            dc_motor_r.forward(50)
        if angle > RIGHT_ANGLE*3:
            state = prev_state = "forward"
            angle = 0

    if state == "detour":
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
        angle = 0
        while angle < RIGHT_ANGLE/2:
            accel = mpu.get_values()
            if d_r > d_l:
                dc_motor_l.forward(50)
                dc_motor_r.backwards(50)
            else:
                dc_motor_r.forward(50)
                dc_motor_l.backwards(50)
            time.sleep(0.1)
            angle += abs(accel['GyZ'])*0.1
        angle = 0
        dc_motor_l.forward(V - u)
        dc_motor_r.forward(V + u)


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
        while angle < RIGHT_ANGLE:
            accel = mpu.get_values()
            if d_r > d_l:
                dc_motor_l.forward(50)
                dc_motor_r.backwards(50)
            else:
                dc_motor_r.forward(50)
                dc_motor_l.backwards(50)
            time.sleep(0.1)
            angle += abs(accel['GyZ'])*0.1
        angle = 0

    if state == "stop":
        dc_motor_l.stop()
        dc_motor_r.stop()

    # print new state
        
    if state != prev_state:
        print(f"dt={dt}, l={line_l}, r={line_r}, u={u}, e={e}, p={p}, d={d}, st={state}")
        prev_state = state




