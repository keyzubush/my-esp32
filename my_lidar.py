import utime
from machine import I2C
from machine import Pin
print("A")
import time
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
from vl53l1x import VL53L1X
print("B")
distance = VL53L1X(i2c)
print("C")
while True:
    print("range: mm ", distance.read())
    utime.sleep(1)
