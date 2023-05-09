import uasyncio

async def blink(led, period_ms):
    while True:
        led.on()
        await uasyncio.sleep_ms(5)
        led.off()
        await uasyncio.sleep_ms(period_ms)

async def main(led):
    uasyncio.create_task(blink(led, 400))
    await uasyncio.sleep_ms(10_000)

# Running on a generic board
from machine import Pin
uasyncio.run(main(Pin(2, Pin.OUT)))
