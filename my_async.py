import time, sys, random

from machine import Pin, PWM, ADC
from machine import I2C
import uasyncio

from hcsr04 import HCSR04
import mpu6050
import dcmotor

freq = 2000
dc_motor_l = dcmotor.DCMotor(Pin(25, Pin.OUT), Pin(33, Pin.OUT), PWM(Pin(32), freq))
dc_motor_r = dcmotor.DCMotor(Pin(27, Pin.OUT), Pin(26, Pin.OUT), PWM(Pin(14), freq))

sensor = HCSR04(trigger_pin=15, echo_pin=18)
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
mpu = mpu6050.accel(i2c)
servo = PWM(Pin(4, mode=Pin.OUT), freq=50)
line_r_pin = ADC(Pin(34))
line_l_pin = ADC(Pin(35))

prev_state = state = "forward"
corr = fw_count = angle = 0
prev_distance = distance = 0
radar = [50.0]*11
t = time.time_ms()

LINE = 1.5
LINE_TIMEOUT = 2.0 # sec
ACCEL_DEGREE = 55 
K_P = 25
K_D = 250
V = 70

async def print_debug():
    while True:
        if state != prev_state:
            print(f"dt={dt}, l={line_l}, r={line_r}, u={u}, e={e}, p={p}, d={d}, st={state}")
        await uasyncio.sleep_ms(100)

async def blink(led):
    while True:
        if state == "forward": period_ms = 1000
        if state == "line":    period_ms = 500
        if state == "detour":  period_ms = 250
        if state == "turn":    period_ms = 100
        led.on()
        await uasyncio.sleep_ms(5)
        led.off()
        await uasyncio.sleep_ms(period_ms)

async def sensors():
    global distance, accel, line_l, line_r, dt
    while True:
        distance = sensor.distance_cm()
        distance = distance if distance > 0 else prev_distance
        accel = mpu.get_values()
        line_r = line_r_pin.read()/1000
        line_l = line_l_pin.read()/1000
        dt = (time.time_ms() - t) / 10**6
        t = time.time_ms()
        await uasyncio.sleep_ms(20)

async def head():
    global radar
    servo.duty(50)
    await uasincio.sleep_ms(500)
    for i in range(10 + 1):
        radar[i] = distance
        servo.duty(50 + i*5)
        await uasincio.sleep_ms(100)
    servo.duty(75)
    await uasincio.sleep_ms(500)

async def drive_angle(angle):
    a = 0
    while abs(a) < abs(ACCEL_DEGREE*angle):
        if angle > 0:
            dc_motor_l.forward(50)
            dc_motor_r.backwards(50)
        else:
            dc_motor_r.forward(50)
            dc_motor_l.backwards(50)
        a += accel['GyZ'] < 0: 
        await uasincio.sleep_ms(20)



async def calculate_state():
    global state
    while True:
        if distance < 10 and state != "line":
            state = "turn"
        elif distance < 20 and state == "line":
            state = "detour"
        elif prev_state == "detour" and abs(line_l - line_r) > LINE:
            state = "line"
        elif abs(line_l - line_r) > LINE or prev_state == "line":
            if abs(line_l - line_r) > LINE:
                t_line = time.time_ms()
            if time.time_ms() - t_line > LINE_TIMEOUT:
                state = "forward"
        elif prev_state == "line":
            state = "line"
        else:
            state = "forward"
        
        prev_state = state
        await uasincio.sleep_ms(10)

async def drive_forward():
    u = 0
    while state == "forward":
        dc_motor_l.forward(V - u)
        dc_motor_r.forward(V + u)
        if accel['GyZ'] < 0: 
            u += 1
        else:
            u -= 1
        await uasyncio.sleep_ms(10)

async def drive():
    while True:
        if state == "forward": await drive_forward()
        if state == "line":    await drive_line()
        if state == "detour":  await drive_detour()
        if state == "turn":    await drive_turn()
        if state == "search":  await drive_search()
        await uasincio.sleep_ms(10)

async def drive_line():
    e = e_old = 0
    u = 0
    while state == "line":
        e = line_l - line_r
        p = e * K_P
        d = (e - e_old) * K_D
        u = p + d
        e_old = e
        dc_motor_l.forward(V + u)
        dc_motor_r.forward(V - u)
        await uasincio.sleep_ms(10)

async def drive_search():
    while state == "search":
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
        await uasincio.sleep_ms(10)

async def drive_detour():
    while state == "detour":
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

async def drive_turn():
    if state == "turn":
        dc_motor_l.stop()
        dc_motor_r.stop()
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


async def main():
    uasyncio.create_task(blink())
    uasyncio.create_task(sensors())
    uasyncio.create_task(calculate_state())
    uasyncio.create_task(drive())
    await uasyncio.sleep_ms(600_000)

main()
dc_motor_l.stop()
dc_motor_r.stop()