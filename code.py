print("Hello World!")

import time
import os
import wifi
import microcontroller

def seconds_to_iso_time(s, tz):
    st = time.localtime(s)
    return f"{st[0]:04d}-{st[1]:02d}-{st[2]:02d}T{st[3]:02d}:{st[4]:02d}:{st[5]:02d}{tz:+03}:00"

time.sleep(3)  # wait for serial
print(f"{'-'*25}")

last = 0
start = time.monotonic()
while True:
    if (time.monotonic() - last) >= 1:
        try:
            wifi.radio.connect(os.getenv("CIRCUITPY_WIFI_SSID"), os.getenv("CIRCUITPY_WIFI_PASSWORD"))
        except:
            print(f".", end="")
        print(f"{seconds_to_iso_time(time.time(), -5)} [wifi connected={wifi.radio.connected} ipv4={wifi.radio.ipv4_address}]")
        last = time.monotonic()
