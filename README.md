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

webrepl-master/webrepl_cli.py -p 493944 boot.py 192.168.1.239:boot.py

