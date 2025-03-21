# my-esp32

./esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
./esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 ../esp32-20230426-v1.20.0.bin
screen /dev/ttyUSB0 115200

import network
sta_if = network.WLAN(network.STA_IF)
ap_if = network.WLAN(network.AP_IF)
sta_if.active(True)
sta_if.connect('<your SSID>', '<your key>')
sta_if.ifconfig()

#

import webrepl
webrepl.start()

#

# MicroPyhton

webrepl-master/webrepl_cli.py -p 493944 boot.py 192.168.1.239:boot.py

webrepl-master/webrepl_cli.py -p 493944 192.168.1.239

# CircuitPython

./esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x0 cam.bin

f = open('settings.toml', 'w')
f.write('CIRCUITPY_WIFI_SSID = "wifissid"\n')
f.write('CIRCUITPY_WIFI_PASSWORD = "wifipassword"\n')
f.write('CIRCUITPY_WEB_API_PASSWORD = "webpassword"\n')
f.close()

5x8 !!!

# esp32-cam <-> esp320devkitv1 UART

tx IO12, rx IO13 =x= tx TX2, rx RX2

>>> import board
>>> import busio
>>> uart = busio.UART(board.IO12, board.IO13, baudrate=9600)
>>> uart.write("xxx")

>>> import board
>>> import busio
>>> uart = busio.UART(board.TX2, board.RX2, baudrate=9600)
>>> uart.read()
b'xxx'
