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
    grab_mode=espcamera.GrabMode.WHEN_EMPTY)

cam.saturation = 2

# grey = " .:-=+*#%@"
grey = "@%#*+=-:. "
cam_shape = (96, 96)
red_mask   = np.full(cam_shape, 0b1111100000000000, dtype=np.uint16)
green_mask = np.full(cam_shape, 0b0000011111100000, dtype=np.uint16)
blue_mask  = np.full(cam_shape, 0b0000000000011111, dtype=np.uint16)
shift_8    = np.full(cam_shape, 2**8) 
shift_3    = np.full(cam_shape, 2**3) 


while True:
    input("Press Enter")
    buf_orig = cam.take()
    buf = bytearray(buf_orig)
    rgb565 = np.frombuffer(buf, dtype=np.uint16)
    rgb565.byteswap(inplace=True)
    rgb565.shape = cam_shape
    # to RGB555
    red   = np.asarray((rgb565 & red_mask)   // shift_8, dtype=np.uint8)
    green = np.asarray((rgb565 & green_mask) // shift_3, dtype=np.uint8)
    blue  = np.asarray((rgb565 & blue_mask)  *  shift_3, dtype=np.uint8)
 
    o = np.asarray(red, dtype=np.int16) + green + blue
    r = np.asarray(np.clip(np.asarray(red, dtype=np.int16) * 2 - green - blue, 0, 256), dtype=np.int16)
    g = np.asarray(np.clip(np.asarray(green, dtype=np.int16) * 2 - red - blue, 0, 256), dtype=np.int16)

    ascii = []
    print(o[48][48])
    for i in range(0, cam_shape[0], 2):
        s = []
        for j in range(0, cam_shape[1], 1):
            s.append(grey[(o[i][j] * len(grey)) // (256*3+1)])
        ascii.append(''.join(s) + "\n")
    print(''.join(ascii))

    ascii = []
    print(r[48][48])
    for i in range(0, cam_shape[0], 2):
        s = []
        for j in range(0, cam_shape[1], 1):
            s.append(grey[(r[i][j] * len(grey)) // (256+1)])
        ascii.append(''.join(s) + "\n")
    print(''.join(ascii))

    ascii = []
    print(g[48][48])
    for i in range(0, cam_shape[0], 2):
        s = []
        for j in range(0, cam_shape[1], 1):
            s.append(grey[(g[i][j] * len(grey)) // (256+1)])
        ascii.append(''.join(s) + "\n")
    print(''.join(ascii))

