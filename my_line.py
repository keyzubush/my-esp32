from machine import Pin, ADC
import time

line = ADC(Pin(34))

for i in range(600):
    line_val = line.read()
    print("line =", line_val)
    time.sleep(0.1)
