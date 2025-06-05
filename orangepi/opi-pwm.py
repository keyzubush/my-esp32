import gpiod
import time

class PWMMotor:
    def __init__(self, pin1, pin2, chip, pwm_freq=1000):
        """Initialize a motor with PWM capability"""
        self.pin1 = chip.get_line(pin1)
        self.pin2 = chip.get_line(pin2)
        self.pin1.request(consumer="MOTOR_PWM", type=gpiod.LINE_REQ_DIR_OUT)
        self.pin2.request(consumer="MOTOR_PWM", type=gpiod.LINE_REQ_DIR_OUT)
        self.pwm_freq = pwm_freq
        self.current_speed = 0
        self.stop()
        
    def __call__(self, speed):
        """Control motor with speed from -1.0 to 1.0"""
        self.set_speed(speed)
    
    def set_speed(self, speed):
        """Set motor speed (-1.0 to 1.0) using software PWM"""
        speed = max(-1.0, min(1.0, speed))  # Clamp to [-1.0, 1.0]
        self.current_speed = speed
        
        if speed == 0:
            self.stop()
        elif speed > 0:
            self._pwm_control(forward=True, duty_cycle=speed)
        else:
            self._pwm_control(forward=False, duty_cycle=abs(speed))
    
    def _pwm_control(self, forward, duty_cycle):
        """Software PWM implementation"""
        period = 1.0 / self.pwm_freq
        on_time = period * duty_cycle
        off_time = period - on_time
        
        if forward:
            self.pin1.set_value(1)
            self.pin2.set_value(0)
        else:
            self.pin1.set_value(0)
            self.pin2.set_value(1)
        
        # For real PWM effect you would need to implement timing here
        # This is simplified - consider using hardware PWM for better performance
        time.sleep(on_time)
        self.stop()
        time.sleep(off_time)
    
    def stop(self):
        """Stop the motor"""
        self.pin1.set_value(0)
        self.pin2.set_value(0)
    
    def cleanup(self):
        """Release GPIO resources"""
        self.stop()
        self.pin1.release()
        self.pin2.release()

class MotorController:
    def __init__(self):
        self.chip = gpiod.Chip('gpiochip0')
        
        # Initialize motors with PWM capability
        self.left = PWMMotor(269, 270, self.chip)  # Left motor (GPIO4, GPIO5)
        self.right = PWMMotor(267, 268, self.chip)  # Right motor (GPIO6, GPIO10)
        
    def set_speeds(self, left_speed, right_speed):
        """Set speeds for both motors (-1.0 to 1.0)"""
        self.left(left_speed)
        self.right(right_speed)
        
    def stop_all(self):
        """Stop both motors"""
        self.left.stop()
        self.right.stop()
        
    def cleanup(self):
        """Cleanup all resources"""
        self.left.cleanup()
        self.right.cleanup()
        self.chip.close()

# Example usage
if __name__ == "__main__":
    controller = MotorController()
    
    try:
        # Ramp up speed
        for speed in [0.3, 0.6, 0.9]:
            controller.set_speeds(speed, speed)
            print(f"Running at {speed*100}% speed")
            time.sleep(2)
        
        # Rotate
        controller.set_speeds(0.5, -0.5)
        time.sleep(1)
        
        controller.stop_all()
        
    finally:
        controller.cleanup()