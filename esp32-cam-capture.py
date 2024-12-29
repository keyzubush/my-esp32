import camera
camera.deinit()
camera.init(0, format=camera.RGB565, framesize=camera.FRAME_96X96, fb_location=camera.PSRAM)
camera.saturation(-2)
buf = camera.capture()
bs = bytearray(buf)

grey = " .:-=+*#%@"

for i in range(0, 96, 6):
    s = []
    for j in range(0, 96, 3):
        index = 2 * (i * 96 + j)
        r = (bs[index] & 0b11111000) >> 3
        g = 8 * (bs[index] & 0b00000111) + ((bs[index+1] & 0b11100000) >> 5)
        b = (bs[index+1] & 0b00011111)
        r = hex((r * 16) // 32)[2:]
        g = hex((g * 16) // 64)[2:]
        b = hex((b * 16) // 32)[2:]
        v = r + g + b
        s.append(grey[(v * 10) // 48])
    print(''.join(s))


