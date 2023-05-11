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
led = Pin(2, Pin.OUT)

prev_state = "forward"
state = "forward"
prev_distance = distance = 50
radar = [50.0]*11
ticks = time.ticks_ms()
dt = 0.01
line_l = line_r = 0
accel = {}

N_RADAR = len(radar)
LINE = 1.5
LINE_TIMEOUT = 2000
ACCEL_DEGREE = 55
K_P = 25
K_D = 250
V = 70

async def print_debug():
    while True:
        print(f"dt={dt}, l={line_l}, r={line_r}, st={state}")
        await uasyncio.sleep_ms(500)

async def blink():
    while True:
        if   state == "forward": period_ms = 1000
        elif state == "line":    period_ms = 500
        elif state == "detour":  period_ms = 250
        elif state == "turn":    period_ms = 100
        elif state == "angle":   period_ms = 50
        led.on()
        await uasyncio.sleep_ms(5)
        led.off()
        await uasyncio.sleep_ms(period_ms)

async def sensors():
    global distance, accel, line_l, line_r, dt, ticks
    while True:
        distance = sensor.distance_cm()
        distance = distance if distance > 0 else prev_distance
        accel = mpu.get_values()
        line_r = line_r_pin.read()/1000
        line_l = line_l_pin.read()/1000
        dt = time.ticks_diff(time.ticks_ms(), ticks) / 10**3
        ticks = time.ticks_ms()
        await uasyncio.sleep_ms(20)

async def head():
    global radar
    servo.duty(50)
    await uasyncio.sleep_ms(500)
    for i in range(N_RADAR):
        radar[i] = distance
        servo.duty(int(50 + i*(50/(N_RADAR-1))))
        await uasyncio.sleep_ms(100)
    servo.duty(75)
    await uasyncio.sleep_ms(500)

async def calculate_state():
    global state, prev_state, line_l, line_r
    while True:
        if distance < 10 and state != "line":
            state = "turn"
        elif distance < 20 and state == "line":
            state = "detour"
        elif state == "forward" and accel['GyX'] > 1750:
            state = "obstacle"
        elif prev_state == "detour" and abs(line_l - line_r) > LINE:
            state = "line"
        elif abs(line_l - line_r) > LINE or prev_state == "line":
            if abs(line_l - line_r) > LINE:
                t_line = time.ticks_ms()
            if time.ticks_diff(time.ticks_ms(), t_line) > LINE_TIMEOUT:
                state = "forward"
        elif prev_state == "line":
            state = "line"
        else:
            state = "forward"

        prev_state = state
        await uasyncio.sleep_ms(10)

async def drive_angle(angle):
    a = 0
    dc_motor_l.stop()
    dc_motor_r.stop()
    while abs(a) < abs(ACCEL_DEGREE*angle) and state == "detour":
        if angle > 0:
            dc_motor_l.forward(50)
            dc_motor_r.backwards(50)
        else:
            dc_motor_r.forward(50)
            dc_motor_l.backwards(50)
        a += accel['GyZ']
        await uasyncio.sleep_ms(20)

async def drive_forward(period_ms):
    u = 0
    timer = 0
    while state in ["forward", "detour"] and timer < period_ms:
        dc_motor_l.forward(V - u)
        dc_motor_r.forward(V + u)
        if accel['GyZ'] < 0:
            u += 1
        else:
            u -= 1
        await uasyncio.sleep_ms(10)
        timer += dt

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
        await uasyncio.sleep_ms(10)

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
        await uasyncio.sleep_ms(10)

async def drive_detour():
    await drive_angle(45)
    await drive_forward(1000)
    await drive_angle(-90)
    await drive_forward(1000)

async def drive_turn():
    dc_motor_l.backwards(50)
    dc_motor_r.backwards(50)
    await uasyncio.sleep_ms(500)
    await head()
    if radar[0] > radar[-1]:
        await drive_angle(-90)
    else:
        await drive_angle(90)

async def drive_obstacle():
    dc_motor_l.backwards(50)
    dc_motor_r.backwards(50)
    await uasyncio.sleep_ms(500)
    await drive_detour()

async def drive():
    while True:
        if   state == "forward": await drive_forward(60_000)
        elif state == "line":    await drive_line()
        elif state == "detour":  await drive_detour()
        elif state == "turn":    await drive_turn()
        elif state == "obstacle":await drive_obstacle()
        elif state == "search":  await drive_search()
        await uasyncio.sleep_ms(10)

async def main():
    uasyncio.create_task(sensors())
    uasyncio.create_task(print_debug())
    uasyncio.create_task(blink())
    uasyncio.create_task(calculate_state())
    uasyncio.create_task(drive())
    await uasyncio.sleep_ms(600_000)

uasyncio.run(main())
dc_motor_l.stop()
dc_motor_r.stop()
