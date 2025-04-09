import asyncio, json, time
import numpy as np

TIMEOUT = 1
STRIPE = 8

msg_tx = {'id': 0, 'timestamp': 0, 'left': 0, 'right': 0, 'duration': -1, 'hash': 0}
msg_rx = {'id': 0, 'timestamp': 0, 'distance': 999, 'speed': 0, 'angle': 0, 'hash': 0}

uart = None
cam_shape = (96, 96)

def msg_debug(msg, checks):
    msg_prev = msg.copy()
    def msg_debug_inner(msg):
        nonlocal msg_prev
        for c in checks:
            # print(msg)
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
        # buf_orig = cam.take(1)
        # buf = bytearray(buf_orig)
        # gray = np.frombuffer(buf, dtype=np.uint8)
        gray = np.full((96*96,), fill_value=10, dtype=np.uint8)
        gray = gray.reshape(cam_shape)
        min_col = list(np.argmin(gray, axis = 1))
        delta = (float(sum(min_col[-STRIPE:]))/STRIPE - cam_shape[1]/2) / cam_shape[1]
        msg_tx['left']  = 0.5 - delta
        msg_tx['right'] = 0.5 + delta
        msg_debug_tx(msg_tx)
        await asyncio.sleep(0.3)   

async def uart_write(uart):
    global msg_rx, msg_tx
    while True:
        msg_tx['id'] += 1
        msg_tx['timestamp'] = time.time()
        msg_tx['left'] = 9
        # input(msg_tx)
        msg_debug_tx(msg_tx)
        # uart.write(json.dumps(msg_tx).encode('utf-8'))
        await asyncio.sleep(0.4)   

async def uart_read(uart):
    global msg_rx, msg_tx
    while True:
        # ba = uart.read()
        ba = '{"id": 0, "timestamp": 0, "distance": 999, "speed": 0, "angle": 0, "hash": 0}'
        if ba:
            msg_rx = json.loads(ba)
            msg_debug_rx(msg_rx)
        await asyncio.sleep(TIMEOUT)   

async def main():
    await asyncio.gather(
        asyncio.create_task(camera()),
        asyncio.create_task(uart_write(uart)),
        asyncio.create_task(uart_read(uart)))

asyncio.run(main())
cam.deinit()

