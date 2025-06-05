import gpiod
import time

class MotorController:
    def __init__(self):
        self.chip = gpiod.Chip('gpiochip0')  # Default GPIO chip
        
        self.IN1 = self.chip.get_line(270)  # Pin 16 (GPIO4)
        self.IN2 = self.chip.get_line(228)  # Pin 18 (GPIO5)
        
        self.IN3 = self.chip.get_line(229)  # Pin 22 (GPIO6)
        self.IN4 = self.chip.get_line(262) # Pin 24 (GPIO10)
        
        # Set lines as output
        self.IN1.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN2.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN3.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN4.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)

    def motor_control(self, motor, direction):
        if motor == "A":
            if direction == "forward":
                self.IN1.set_value(1)
                self.IN2.set_value(0)
            elif direction == "backward":
                self.IN1.set_value(0)
                self.IN2.set_value(1)
            else:  # stop
                self.IN1.set_value(0)
                self.IN2.set_value(0)
        elif motor == "B":
            if direction == "forward":
                self.IN3.set_value(1)
                self.IN4.set_value(0)
            elif direction == "backward":
                self.IN3.set_value(0)
                self.IN4.set_value(1)
            else:  # stop
                self.IN3.set_value(0)
                self.IN4.set_value(0)
        

if __name__ == "__main__":
    motor = MotorController()

    motor.motor_control("A", "forward")
    motor.motor_control("B", "forward")
    time.sleep(2)
    motor.motor_control("A", "stop")
    motor.motor_control("B", "stop")

