# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# This example uses an L9110 H-bridge driver to run a DC Motor using two PWM pins.
#  https://www.adafruit.com/product/4489

# Hardware setup:
#   DC motor via L9110 H-bridge driver on two PWM pins that are on their own channels
#   e.g., RP2040 Pico pins GP28, GP27

import time
import board
import pwmio
from adafruit_motor import motor

PWM_PIN_A = board.D26  # pick any pwm pins on their own channels
PWM_PIN_B = board.D25
PWM_PIN_C = board.D32  # pick any pwm pins on their own channels
PWM_PIN_D = board.D33


# DC motor setup
# DC Motors generate electrical noise when running that can reset the microcontroller in extreme
# cases. A capacitor can be used to help prevent this.

pwm_c = pwmio.PWMOut(PWM_PIN_C, frequency=50)
pwm_d = pwmio.PWMOut(PWM_PIN_D, frequency=50)
motor0 = motor.DCMotor(pwm_c, pwm_d)


pwm_a = pwmio.PWMOut(PWM_PIN_A, frequency=50)
pwm_b = pwmio.PWMOut(PWM_PIN_B, frequency=50)
motor1 = motor.DCMotor(pwm_a, pwm_b)



import socketpool
import wifi
from adafruit_httpserver import Server, Request, Response

pool = socketpool.SocketPool(wifi.radio)
server = Server(pool, debug=True)


@server.route("/forward")
def base(request: Request):
    motor0.throttle = 1.0
    motor1.throttle = 1.0
    return Response(request, "Forward")
@server.route("/backward")
def base(request: Request):
    motor0.throttle = -1.0
    motor1.throttle = -1.0
    return Response(request, "Backward")
@server.route("/left")
def base(request: Request):
    motor0.throttle = -1.0
    motor1.throttle = 1.0
    return Response(request, "Left")
@server.route("/right")
def base(request: Request):
    motor0.throttle = 1.0
    motor1.throttle = -1.0
    return Response(request, "Right")
@server.route("/leftstep")
def base(request: Request):
    motor0.throttle = -1.0
    motor1.throttle = 1.0
    time.sleep(0.2)
    motor0.throttle = 0
    motor1.throttle = 0
    return Response(request, "LeftStep")
@server.route("/rightstep")
def base(request: Request):
    motor0.throttle = 1.0
    motor1.throttle = -1.0
    time.sleep(0.2)
    motor0.throttle = 0
    motor1.throttle = 0
    return Response(request, "RightStep")
@server.route("/stop")
def base(request: Request):
    motor0.throttle = 0
    motor1.throttle = 0
    return Response(request, "Stop")

server.serve_forever(str(wifi.radio.ipv4_address))

