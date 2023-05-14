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

prev_distance = distance = 50
radar = [50.0]*11
ticks = time.ticks_ms()
dt = 0.01
line_l = line_r = 0
accel = {}

N_RADAR = len(radar)
LINE = 2.5
LINE_TIMEOUT = 2000
ACCEL_DEGREE = 55
K_P = 25
K_D = 250
V = 70

class State():
    FORWARD = "forward"
    LINE    = "line"
    DETOUR  = "detour"
    TURN    = "turn"
    ROTATE  = "rotate"

state = State.FORWARD

async def print_debug():
    print("call print_debug")
    await uasyncio.sleep_ms(500)
    i = 0
    prev_state = state
    while True:
        i += 1
        if i % 100 == 0 or state != prev_state:
            print(f"t={ticks}, d={distance}, l={line_l}, r={line_r}, gyx={accel['GyX']}, gyz={accel['GyZ']}, st={state}")
            prev_state = state
        await uasyncio.sleep_ms(10)

async def blink():
    print("call blink")
    while True:
        if   state == State.FORWARD: period_ms = 300
        elif state == State.LINE:    period_ms = 50
        elif state == State.DETOUR:  period_ms = 100
        elif state == State.TURN:    period_ms = 75
        elif state == State.ROTATE:  period_ms = 200
        led.on()
        await uasyncio.sleep_ms(5)
        led.off()
        await uasyncio.sleep_ms(period_ms)

async def sensors():
    print("call sensors")
    global distance, accel, line_l, line_r, dt, ticks
    while True:
        distance = sensor.distance_cm()
        distance = distance if distance > 0 else prev_distance
        accel = mpu.get_values()
        line_r = line_r_pin.read()/1000
        line_l = line_l_pin.read()/1000
        dt = time.ticks_diff(time.ticks_ms(), ticks) / 10**3
        ticks = time.ticks_ms()
        await uasyncio.sleep_ms(10)

async def head():
    print("call head")
    global radar
    if state not in [State.TURN]: return
    servo.duty(50)
    await uasyncio.sleep_ms(500)
    for i in range(N_RADAR):
        radar[i] = distance
        servo.duty(int(50 + i*(50/(N_RADAR-1))))
        await uasyncio.sleep_ms(200)
    servo.duty(75)

async def calculate_state():
    print("call calculate_state")
    global state
    while True:
        if distance < 10 and state not in [State.LINE, State.DETOUR]:
            state = State.TURN
            print("d < 10")
        elif distance < 20 and state == State.LINE:
            state = State.DETOUR
            print("d < 20")
        elif accel['GyX'] > 3000 and state == State.FORWARD:
            state = State.TURN
            print("gyx > 3000")
        elif abs(line_l - line_r) > LINE or state in [State.LINE, State.DETOUR]:
            if abs(line_l - line_r) > LINE:
                t_line = time.ticks_ms()
                if state != State.DETOUR: state = State.LINE
            if time.ticks_diff(time.ticks_ms(), t_line) > LINE_TIMEOUT:
                state = State.FORWARD
                print("> LINE_TIMEOUT")
        elif state in [State.LINE, State.TURN, State.DETOUR]:
            pass
        else:
            state = State.FORWARD

        await uasyncio.sleep_ms(10)

async def drive_angle(angle):
    print("call drive_angle")
    a = 0
    dc_motor_l.stop()
    dc_motor_r.stop()
    await uasyncio.sleep_ms(100)
    while abs(a) < abs(ACCEL_DEGREE*angle):
        if angle > 0:
            dc_motor_l.forward(V)
            dc_motor_r.backwards(V)
        else:
            dc_motor_r.forward(V)
            dc_motor_l.backwards(V)
        a += accel['GyZ']*dt
        await uasyncio.sleep_ms(20)

async def drive_forward(period_ms=1000):
    print("call drive_forward")
    u = 0
    timer = 0
    while timer < period_ms/1000:
        dc_motor_l.forward(V - u)
        dc_motor_r.forward(V + u)
        if accel['GyZ'] < 0:
            u += 1
        else:
            u -= 1
        await uasyncio.sleep_ms(10)
        timer += dt
    print("end drive_forward")

async def drive_backwards(period_ms):
    print("call drive_backwards")
    u = 0
    timer = 0
    while timer < period_ms/1000:
        dc_motor_l.backwards(V + u)
        dc_motor_r.backwards(V - u)
        if accel['GyZ'] < 0:
            u += 1
        else:
            u -= 1
        await uasyncio.sleep_ms(10)
        timer += dt
    dc_motor_l.stop()
    dc_motor_r.stop()
    print("end drive_backwards")

async def drive_line():
    print("call drive_line")
    e = e_old = 0
    u = 0
    while state == State.LINE:
        e = line_l - line_r
        p = e * K_P
        d = (e - e_old) * K_D
        u = p + d
        e_old = e
        dc_motor_l.forward(V + u)
        dc_motor_r.forward(V - u)
        await uasyncio.sleep_ms(10)

async def drive_detour():
    print("call drive_detour")
    global state
    await drive_angle(45)
    await drive_forward(2500)
    await drive_angle(-90)
    state = State.FORWARD
    await uasyncio.sleep_ms(20)

async def drive_turn():
    print("call drive_turn")
    await drive_backwards(500)
    await head()
    if radar[0] > radar[-1]:
        await drive_angle(-90)
    else:
        await drive_angle(90)
    state = State.FORWARD
    await uasyncio.sleep_ms(20)

async def drive():
    print("call drive")
    while True:
        if   state == State.FORWARD: await drive_forward()
        elif state == State.LINE:    await drive_line()
        elif state == State.DETOUR:  await drive_detour()
        elif state == State.TURN:    await drive_turn()
        await uasyncio.sleep_ms(10)

async def main():
    print("call main")
    uasyncio.create_task(sensors())
    uasyncio.create_task(print_debug())
    uasyncio.create_task(blink())
    uasyncio.create_task(calculate_state())
    uasyncio.create_task(drive())
    await uasyncio.sleep_ms(600_000)

uasyncio.run(main())
dc_motor_l.stop()
dc_motor_r.stop()
