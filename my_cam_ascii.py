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
grey.reverse()
cam_shape = (96, 96)
red_mask   = np.full(cam_shape, 0b1111100000000000, dtype=np.uint16)
green_mask = np.full(cam_shape, 0b0000011111100000, dtype=np.uint16)
blue_mask  = np.full(cam_shape, 0b0000000000011111, dtype=np.uint16)
red_shift   = np.full(cam_shape, 2**8) 
green_shift = np.full(cam_shape, 2**3) 
blue_shift  = np.full(cam_shape, 2**3) 


while True:
    input("Press Enter")
    ascii = []
    buf_orig = cam.take()
    buf = bytearray(buf_orig)
    rgb565 = np.frombuffer(buf, dtype=np.uint16)
    rgb565.byteswap(inplace=True)
    rgb565.shape = cam_shape
    # to RGB555
    red   = np.asarray((rgb565 & red_mask)   / red_shift,   dtype=np.uint8)
    green = np.asarray((rgb565 & green_mask) / green_shift, dtype=np.uint8)
    blue  = np.asarray((rgb565 & blue_mask)  * blue_shift,  dtype=np.uint8)
 
    o = np.asarray(red, dtype=np.int16) + green + blue
 
    print(o[48][48])
    for i in range(0, cam_shape[0], 2):
        s = []
        for j in range(0, cam_shape[1], 1):
            s.append(grey[(o[i][j] * len(grey)) // (256*3)])
        ascii.append(''.join(s) + "\n")
    print(''.join(ascii))


