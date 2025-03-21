from ulab import numpy as np

import espcamera
import board
import busio

cam = espcamera.Camera(
    data_pins=board.CAMERA_DATA,
    external_clock_pin=board.CAMERA_XCLK,
    pixel_clock_pin=board.CAMERA_PCLK,
    vsync_pin=board.CAMERA_VSYNC,
    href_pin=board.CAMERA_HREF,
    powerdown_pin=board.CAMERA_PWDN,
    reset_pin=None,
    i2c=board.I2C(),
    external_clock_frequency=20_000_000,
    pixel_format=espcamera.PixelFormat.RGB565,
    frame_size=espcamera.FrameSize.R96X96,
    grab_mode=espcamera.GrabMode.LATEST)

grey = " .:-=+*#%@"

while True:
    input("Press Enter")
    ascii = []
    buf_orig = cam.take()
    buf = bytearray(buf_orig)
    rgb565 = np.frombuffer(buf, dtype=np.uint16)
    rgb565.byteswap()
    rgb565.shape = (96, 96) 
    # to RGB555
    red   = (rgb565 & np.full(rgb565.shape, 0b1111100000000000, dtype=np.uint16)) // np.full(rgb565.shape, 2**11) 
    green = (rgb565 & np.full(rgb565.shape, 0b0000011111100000, dtype=np.uint16)) // np.full(rgb565.shape, 2**6)
    blue  = (rgb565 & np.full(rgb565.shape, 0b0000000000011111, dtype=np.uint16)) // np.full(rgb565.shape, 2**0)
    bw = np.asarray(red + green + blue, dtype=np.uint16)
    red = np.asarray(red, dtype=np.uint16)
    print(red[48][48])
    print(green[48][48])
    print(blue[48][48])
    print(bw[48][48])
    for i in range(0, 96, 2):
        s = []
        for j in range(0, 96, 1):
            # s.append(grey[(bw[i][j] * len(grey)) // 97])
            s.append(grey[(red[i][j] * len(grey)) // 33])
        ascii.append(''.join(s) + "\n")
    print(''.join(ascii))


