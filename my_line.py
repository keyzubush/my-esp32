from machine import Pin, ADC
import time

line_r = ADC(Pin(34))
line_l = ADC(Pin(35))

for i in range(1200):
    line_l_val = line_l.read()
    line_r_val = line_r.read()
    print(f"line_l = {line_l_val}, line_r = {line_r_val}")
    time.sleep(0.1)
