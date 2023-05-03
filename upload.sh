#!/bin/bash

# webrepl-master/webrepl_cli.py -p 493944 192.168.1.239

# import sys
# sys.exit()

# del sys.modules['my_line']

# import machine
# machine.reset()

webrepl-master/webrepl_cli.py -p 493944 dcmotor.py 192.168.1.239:dcmotor.py
webrepl-master/webrepl_cli.py -p 493944 mpu6050.py 192.168.1.239:mpu6050.py
webrepl-master/webrepl_cli.py -p 493944 hcsr04.py 192.168.1.239:hcsr04.py
webrepl-master/webrepl_cli.py -p 493944 my_motor.py 192.168.1.239:my_motor.py
webrepl-master/webrepl_cli.py -p 493944 my_accel.py 192.168.1.239:my_accel.py
webrepl-master/webrepl_cli.py -p 493944 my_echo.py 192.168.1.239:my_echo.py
webrepl-master/webrepl_cli.py -p 493944 my_line.py 192.168.1.239:my_line.py
webrepl-master/webrepl_cli.py -p 493944 my_track.py 192.168.1.239:my_track.py
webrepl-master/webrepl_cli.py -p 493944 my_robot.py 192.168.1.239:my_robot.py
webrepl-master/webrepl_cli.py -p 493944 my_track2.py 192.168.1.239:my_track2.py