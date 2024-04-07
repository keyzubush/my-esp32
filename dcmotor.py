from machine import Pin, PWM

class DCMotor:
    def __init__(self, pin1, pin2, enable_pin, min_duty=600, max_duty=1000):
        self.pin1=pin1
        self.pin2=pin2
        self.enable_pin=enable_pin
        self.min_duty = min_duty
        self.max_duty = max_duty

    def forward(self, speed):
        self.speed = speed
        self.enable_pin.duty(self.duty_cycle())
        self.pin1.value(1)
        self.pin2.value(0)

    def backwards(self, speed):
        self.speed = speed
        self.enable_pin.duty(self.duty_cycle())
        self.pin1.value(0)
        self.pin2.value(1)

    def move(self, speed):
        self.speed = abs(speed)
        self.enable_pin.duty(self.duty_cycle())
        self.pin1.value(1 if speed > 0 else 0)
        self.pin2.value(0 if speed > 0 else 1)

    def stop(self):
        self.enable_pin.duty(0)
        self.pin1.value(0)
        self.pin2.value(0)

    def duty_cycle(self):
        if self.speed < 0:
            duty_cycle = 0
        elif self.speed > 100:
            duty_cycle = self.max_duty
        else:
            duty_cycle = int(self.min_duty + (self.max_duty - self.min_duty)*(self.speed/100))
        return duty_cycle
