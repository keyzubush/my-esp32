from machine import Pin, PWM, ADC
from machine import I2C
from time import sleep
import time
import sys
import random

from hcsr04 import HCSR04
import mpu6050
import dcmotor

            
import time
from machine import Pin
from philips import RC5_IR

freq = 2000
dc_motor_r = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), freq))
dc_motor_l = dcmotor.DCMotor(Pin(26, Pin.OUT), Pin(27, Pin.OUT), PWM(Pin(14), freq))

v = 100

while True:

    data = input("input =")

    if data == 'w':
        print('Forward.')
        dc_motor_l.move(v)
        dc_motor_r.move(v)
    elif data == "s":
        print('Downward.')
        dc_motor_l.move(-v)
        dc_motor_r.move(-v)
    elif data == "d":
        print('Right.')
        dc_motor_l.move(v)
        dc_motor_r.move(-v)
    elif data == "a":
        print('Left.')
        dc_motor_l.move(-v)
        dc_motor_r.move(v)
    elif data == "x":
        print('Up.')
        v += 5
        if v > 100: v = 100
    elif data == "z":
        print('Down.')
        v -= 5
        if v < 0: v = 0
    elif data == "q":
        print('Stop.')
        dc_motor_l.stop()
        dc_motor_r.stop() 
    else:
        print('No action')

    time.sleep_ms(10)
