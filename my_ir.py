import time
from machine import Pin
from philips import RC5_IR


def callback(data, addr, ctrl):
    print('Call')
    if data < 0:
        print('Repeat code.')
    else:
        print('Data {:02x} Addr {:04x}'.format(data, addr))

ir = RC5_IR(Pin(23, Pin.IN), callback)
while True:
    time.sleep_ms(500)
    print('Mark')
