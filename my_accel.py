from machine import I2C
from machine import Pin
import time
import mpu6050
i2c = I2C(0, scl=Pin(22), sda=Pin(21))     #initializing the I2C method for ESP32
#i2c = I2C(scl=Pin(5), sda=Pin(4))       #initializing the I2C method for ESP8266
mpu = mpu6050.accel(i2c)
while True:
    mpu.get_values()
    print(mpu.get_values())
    time.sleep(0.1)
                                                                