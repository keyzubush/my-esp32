import camera
camera.init(0, format=camera.RGB565, framesize=camera.FRAME_96X96, fb_location=camera.PSRAM)
camera.saturation(-2)

def get_picture():
    buf = camera.capture()
    bs = bytearray(buf)
    grey = " .:-=+*#%@"
    ascii = []
    for i in range(0, 96, 2):
        s = []
        for j in range(0, 96, 1):
            index = 2 * (i * 96 + j)
            r = (bs[index] & 0b11111000) >> 3
            g = 8 * (bs[index] & 0b00000111) + ((bs[index+1] & 0b11100000) >> 5)
            b = (bs[index+1] & 0b00011111)
            r = (r * 16) // 32
            g = (g * 16) // 64
            b = (b * 16) // 32
            v = r + g + b
            s.append(grey[(v * 10) // 48])
        ascii.append(''.join(s) + "<br>")
    return '<code>' + ''.join(ascii) + '</code>'

import machine

import socket
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]

s = socket.socket()
s.bind(addr)
s.listen(1)

print('listening on', addr)

while True:
    cl, addr = s.accept()
    print('client connected from', addr)
    cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
    cl.send(get_picture())
    cl.close()

