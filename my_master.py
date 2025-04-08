import board, asyncio, busio, json, espcamera
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
STRIPE = 8
# uart = busio.UART(board.TX2, board.RX2, baudrate=115200, timeout = TIMEOUT)
uart = busio.UART(board.IO12, board.IO13, baudrate=115200, timeout = TIMEOUT)

msg_tx = {'id': 0, 'timestamp': 0, 'left': 0, 'right': 0, 'duration': -1, 'hash': 0}
msg_rx = {'id': 0, 'timestamp': 0, 'distance': 999, 'speed': 0, 'angle': 0, 'hash': 0}

def msg_debug(msg, checks):
    msg_prev = msg
    def msg_debug_inner(msg):
        nonlocal msg_prev
        for c in checks:
            if msg_prev[c] != msg[c]:
                print(msg)
                msg_prev = msg.copy()
                break
    return msg_debug_inner

msg_debug_tx = msg_debug(msg_tx, ['left', 'right'])
msg_debug_rx = msg_debug(msg_rx, ['distance', 'speed', 'angle'])

async def camera():
    global msg_rx, msg_tx
    while True:
        buf_orig = cam.take(1)
        buf = bytearray(buf_orig)
        gray = np.frombuffer(buf, dtype=np.uint8)
        gray.shape = cam_shape
        min_col = list(np.argmin(np.asarray(gray, dtype=np.int16), axis = 1))
        delta = (sum(gray[-STRIPE:])/STRIPE - cam_shape[1]/2) / cam_shape[1]
        msg_tx['left']  = 0.5 - delta
        msg_tx['right'] = 0.5 + delta
        await asyncio.sleep(TIMEOUT)   

async def uart_write(uart):
    global msg_rx, msg_tx
    while True:
        msg_tx['id'] += 1
        msg_tx['timestamp'] = time.time()
        msg_debug_tx(msg_tx)
        uart.write(json.dumps(msg_tx).encode('utf-8'))
        await asyncio.sleep(TIMEOUT)   

async def uart_read(uart):
    global msg_rx, msg_tx
    while True:
        ba = uart.read()
        if ba:
            msg_rx = json.loads(ba.decode('uft-8'))
            msg_debug_rx(msg_rx)
        await asyncio.sleep(TIMEOUT)   

async def main():
    await asyncio.gather(
        asyncio.create_task(camera()),
        asyncio.create_task(uart_write(uart)),
        asyncio.create_task(uart_read(uart)))

asyncio.run(main())
cam.deinit()

