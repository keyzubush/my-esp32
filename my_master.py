import board, asyncio, busio, json

TIMEOUT = 0.05
# uart = busio.UART(board.TX2, board.RX2, baudrate=115200, timeout = TIMEOUT)
uart = busio.UART(board.IO12, board.IO13, baudrate=115200, timeout = TIMEOUT)

msg_rx = {'id': 0, 'timestamp': 0, 'left': 0, 'right': 0, 'duration': -1, 'hash': 0}
msg_tx = {'id': 0, 'timestamp': 0, 'distance': 999, 'speed': 0, 'angle': 0, 'hash': 0}

async def uart_write(uart):
    global msg_rx, msg_tx
    while True:
        interval = random.uniform(0.25, 2.5)
        uart.write(json.dumps(msg).encode('utf-8'))
        print('WRITE:', msg)
        msg['id'] += 1
        msg['hash'] = hash(msg['id'])
        await asyncio.sleep(interval)   

async def uart_read(uart):
    global msg_rx, msg_tx
    while True:
        ba = uart.read()
        if ba:
            print('READ:', json.loads(ba.decode('uft-8')))
        await asyncio.sleep(TIMEOUT)   

async def main():
    uw_task = asyncio.create_task(uart_write(uart))
    ur_task = asyncio.create_task(uart_read(uart))
    await asyncio.gather(uw_task, ur_task)

asyncio.run(main())
