from machine import Pin, PWM
from time import sleep
import time

import dcmotor

dc_motor_l = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), 1000))
dc_motor_r = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), 1000))

def run_servo():
    servo = PWM(Pin(4, mode=Pin.OUT), freq=50)
    servo.duty(50)
    sleep(1)
    servo.duty(100)
    sleep(1)
    servo.duty(75)
    sleep(1)

def run_motor():
    for i in range(600, 1000, 50):
        s = i 
        dc_motor_l.forward(s)
        dc_motor_r.forward(s)
        time.sleep(3)
        dc_motor_l.stop()
        dc_motor_r.stop()
        time.sleep(1)
        dc_motor_l.backwards(s)
        dc_motor_r.backwards(s)
        time.sleep(3)
        dc_motor_l.stop()
        dc_motor_r.stop()
        time.sleep(2)

run_servo()
run_motor()
