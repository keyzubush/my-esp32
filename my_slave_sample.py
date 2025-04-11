import board, asyncio, busio, json, random, io

TIMEOUT = 0.5
uart = busio.UART(board.TX2, board.RX2, baudrate=115200, timeout = TIMEOUT)
# uart = busio.UART(board.IO12, board.IO13, baudrate=115200, timeout = TIMEOUT)

msg = {'id': 1234, 'left': 75, 'right': 100, 'hash': 0}

async def uart_write(uart):
    global msg
    while True:
        interval = random.uniform(0.25, 2.5)
        uart.write(json.dumps(msg).encode('utf-8') + "\n")
        print('WRITE:', msg)
        msg['id'] += 1
        msg['hash'] = hash(msg['id'])
        await asyncio.sleep(interval)   

async def uart_read(uart):
    while True:
        b = uart.read()
        if b:
            bas = io.StringIO(b)
            for ba in bas:
                if ba:
                    print('READ:', json.loads(ba))
        await asyncio.sleep(TIMEOUT)   

async def main():
    uw_task = asyncio.create_task(uart_write(uart))
    ur_task = asyncio.create_task(uart_read(uart))
    await asyncio.gather(uw_task, ur_task)

asyncio.run(main())
