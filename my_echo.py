# Complete project details at https://RandomNerdTutorials.com/micropython-hc-sr04-ultrasonic-esp32-esp8266/
from hcsr04 import HCSR04
from time import sleep

# ESP32
echo_right = HCSR04(trigger_pin=15, echo_pin=19, echo_timeout_us=10000)
echo_front = HCSR04(trigger_pin=2, echo_pin=5, echo_timeout_us=10000)
# ESP8266
#sensor = HCSR04(trigger_pin=12, echo_pin=14, echo_timeout_us=10000)

while True:
    distance_right = echo_right.distance_cm()
    distance_front = echo_front.distance_cm()
    print(distance_front, distance_right)
    sleep(0.1)
