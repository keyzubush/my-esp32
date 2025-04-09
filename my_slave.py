import board, asyncio, busio, json, pwmio, time
from adafruit_motor import motor

PWM_PIN_A = board.D26
PWM_PIN_B = board.D25
PWM_PIN_C = board.D32
PWM_PIN_D = board.D33

pwm_c = pwmio.PWMOut(PWM_PIN_C, frequency=50)
pwm_d = pwmio.PWMOut(PWM_PIN_D, frequency=50)
motor0 = motor.DCMotor(pwm_c, pwm_d)

pwm_a = pwmio.PWMOut(PWM_PIN_A, frequency=50)
pwm_b = pwmio.PWMOut(PWM_PIN_B, frequency=50)
motor1 = motor.DCMotor(pwm_a, pwm_b)

TIMEOUT = 0.05
uart = busio.UART(board.TX2, board.RX2, baudrate=115200, timeout = TIMEOUT)
# uart = busio.UART(board.IO12, board.IO13, baudrate=115200, timeout = TIMEOUT)

msg_rx = {'id': 0, 'timestamp': 0, 'left': 0, 'right': 0, 'duration': -1, 'hash': 0}
msg_tx = {'id': 0, 'timestamp': 0, 'distance': 999, 'speed': 0, 'angle': 0, 'hash': 0}

def msg_debug(msg, checks):
    msg_prev = msg.copy()
    def msg_debug_inner(msg):
        nonlocal msg_prev
        for c in checks:
            if abs(msg_prev[c] - msg[c]) > 0.25:
                print(msg)
                msg_prev = msg.copy()
                break
    return msg_debug_inner

msg_debug_rx = msg_debug(msg_rx, ['left', 'right'])
msg_debug_tx = msg_debug(msg_tx, ['distance', 'speed', 'angle'])

async def motor():
    global msg_rx, msg_tx
    while True:
        motor0.throttle = msg_rx['left']
        motor1.throttle = msg_rx['right']
        await asyncio.sleep(TIMEOUT)   

async def uart_write(uart):
    global msg_rx, msg_tx
    while True:
        msg_tx['id'] += 1
        msg_tx['timestamp'] = time.monotonic()
        msg_debug_tx(msg_tx)
        uart.write(json.dumps(msg_tx).encode('utf-8'))
        await asyncio.sleep(TIMEOUT)   

async def uart_read(uart):
    global msg_rx, msg_tx
    while True:
        try:
            ba = uart.read()
            if ba:
                msg_rx = json.loads(ba.decode('uft-8'))
                msg_debug_rx(msg_rx)
        except:
            pass
        await asyncio.sleep(TIMEOUT)   

async def main():
    await asyncio.gather(
        asyncio.create_task(motor()),
        asyncio.create_task(uart_write(uart)),
        asyncio.create_task(uart_read(uart)))

asyncio.run(main())

