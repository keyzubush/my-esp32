from machine import Pin, PWM
from time import sleep
import time

import dcmotor

freq = 2000

dc_motor_l = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), freq))
dc_motor_r = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), freq))

def run_servo():
    servo = PWM(Pin(4, mode=Pin.OUT), freq=50)
    servo.duty(50)
    sleep(1)
    servo.duty(100)
    sleep(1)
    servo.duty(75)
    sleep(1)

def run_motor():
    for i in range(10, 100, 10):
        s = i 
        dc_motor_l.forward(s)
        dc_motor_r.forward(s)
        print(s)
        time.sleep(1)
        dc_motor_l.stop()
        dc_motor_r.stop()
        time.sleep(1)
        #dc_motor_l.backwards(100-s)
        #dc_motor_r.backwards(100-s)
        #time.sleep(1)
        #dc_motor_l.stop()
        #dc_motor_r.stop()
        #time.sleep(5)

run_servo()
run_motor()

import sys
del sys.modules['my_motor']
