import board, asyncio, busio, json, espcamera, time, io
from ulab import numpy as np

i2c = board.I2C()
i2c.unlock()

cam = espcamera.Camera(
    data_pins=board.CAMERA_DATA,
    external_clock_pin=board.CAMERA_XCLK,
    pixel_clock_pin=board.CAMERA_PCLK,
    vsync_pin=board.CAMERA_VSYNC,
    href_pin=board.CAMERA_HREF,
    powerdown_pin=board.CAMERA_PWDN,
    reset_pin=None,
    i2c=i2c,
    external_clock_frequency=20_000_000,
    pixel_format=espcamera.PixelFormat.GRAYSCALE,
    frame_size=espcamera.FrameSize.R96X96,
    framebuffer_count=2,
    grab_mode=espcamera.GrabMode.WHEN_EMPTY)
cam.saturation = 2
cam_shape = (96, 96)

TIMEOUT = 0.05
STRIPE = 24
# 
R = 0.7
Kp = 0.65
Ki = 0.0
Kd = 0.1
# uart = busio.UART(board.TX2, board.RX2, baudrate=115200, timeout = TIMEOUT)
uart = busio.UART(board.IO12, board.IO13, baudrate=115200, timeout = TIMEOUT)
MAXBUF = 2048

msg_tx = {'id': 0, 'ts': 0, 'left': 0, 'right': 0, 'duration': -1}
msg_rx = {'id': 0, 'ts': 0, 'distance': 999, 'speed': 0, 'angle': 0}

def msg_debug(msg, checks):
    msg_prev = msg.copy()
    def msg_debug_inner(msg):
        nonlocal msg_prev
        for c in checks:
            if abs(msg['ts'] - msg_prev['ts']) > 5.0:
                print(msg)
                msg_prev = msg.copy()
                break
    return msg_debug_inner

msg_debug_tx = msg_debug(msg_tx, ['left', 'right'])
msg_debug_rx = msg_debug(msg_rx, ['distance', 'speed', 'angle'])

async def camera():
    global msg_rx, msg_tx
    t_prev = time.monotonic()
    E_prev = 0.0
    Ei = 0.0
    while True:
        buf_orig = cam.take(1)
        buf = bytearray(buf_orig)
        gray = np.frombuffer(buf, dtype=np.uint8)
        gray = gray.reshape(cam_shape)
        min_col = list(np.argmin(gray, axis = 1))
        #
        E = (float(sum(min_col[-STRIPE:]))/STRIPE - cam_shape[1]/2) / cam_shape[1]
        # print(E)
        t = time.monotonic()
        dt = t - t_prev
        Ei += E * dt
        U = Kp * E + Ki * Ei + Kd * (E - E_prev)/dt
        E_prev = E
        t_prev = t
        #
        msg_tx['left']  = min(max(R + U, -1.0), 1.0)
        msg_tx['right'] = min(max(R - U, -1.0), 1.0)
        await asyncio.sleep(TIMEOUT)   

async def uart_write(uart):
    global msg_rx, msg_tx
    while True:
        msg_tx['id'] += 1
        msg_tx['ts'] = time.monotonic()
        msg_debug_tx(msg_tx)
        uart.write(json.dumps(msg_tx).encode('utf-8') + "\n")
        await asyncio.sleep(TIMEOUT)   

async def uart_read(uart):
    global msg_rx, msg_tx   
    while True:
        try:
            b = uart.read(MAXBUF)
            if b:
                text = io.StringIO(b)
                for line in text:
                    msg_rx = json.loads(line)
                    msg_debug_rx(msg_rx)
        except:
            print("read error:", b)
        await asyncio.sleep(TIMEOUT)  

async def main():
    await asyncio.gather(
        asyncio.create_task(camera()),
        asyncio.create_task(uart_write(uart)),
        asyncio.create_task(uart_read(uart)))

asyncio.run(main())
cam.deinit()
