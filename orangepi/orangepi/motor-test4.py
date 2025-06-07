import gpiod
import sys
import termios
import tty
import select
import threading
import time

class MotorController:
    def __init__(self):
        self.chip = gpiod.Chip('gpiochip0')  # Default GPIO chip
        
        # Initialize GPIO pins
        self.IN1 = self.chip.get_line(268)  
        self.IN2 = self.chip.get_line(267)  
        self.IN3 = self.chip.get_line(269)  
        self.IN4 = self.chip.get_line(270)          
        # Set lines as output
        self.IN1.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN2.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN3.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN4.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        
        # Motor control variables
        self.motor_a_speed = 0  # Range: -100 (full backward) to 100 (full forward)
        self.motor_b_speed = 0
        self.running = True
        
        # Start PWM thread
        self.pwm_thread = threading.Thread(target=self._pwm_control_loop)
        self.pwm_thread.daemon = True
        self.pwm_thread.start()

    def _pwm_control_loop(self):
        """Pseudo-PWM control loop running in a separate thread"""
        while self.running:
            # Control Motor A
            if self.motor_a_speed > 0:
                # Forward with PWM
                self.IN1.set_value(1)
                self.IN2.set_value(0)
                time.sleep(abs(self.motor_a_speed) / 10000.0)
                self.IN1.set_value(0)
                time.sleep((100 - abs(self.motor_a_speed)) / 10000.0)
            elif self.motor_a_speed < 0:
                # Backward with PWM
                self.IN1.set_value(0)
                self.IN2.set_value(1)
                time.sleep(abs(self.motor_a_speed) / 10000.0)
                self.IN2.set_value(0)
                time.sleep((100 - abs(self.motor_a_speed)) / 10000.0)
            else:
                # Stop
                self.IN1.set_value(0)
                self.IN2.set_value(0)
                time.sleep(0.01)  # Small delay to prevent busy waiting
            
            # Control Motor B
            if self.motor_b_speed > 0:
                # Forward with PWM
                self.IN3.set_value(1)
                self.IN4.set_value(0)
                time.sleep(abs(self.motor_b_speed) / 10000.0)
                self.IN3.set_value(0)
                time.sleep((100 - abs(self.motor_b_speed)) / 10000.0)
            elif self.motor_b_speed < 0:
                # Backward with PWM
                self.IN3.set_value(0)
                self.IN4.set_value(1)
                time.sleep(abs(self.motor_b_speed) / 10000.0)
                self.IN4.set_value(0)
                time.sleep((100 - abs(self.motor_b_speed)) / 10000.0)
            else:
                # Stop
                self.IN3.set_value(0)
                self.IN4.set_value(0)
                time.sleep(0.01)  # Small delay to prevent busy waiting

    def set_motor_speed(self, motor, speed):
        """Set motor speed (-100 to 100)"""
        speed = max(-100, min(100, speed))  # Clamp speed to -100..100 range
        if motor == "A":
            self.motor_a_speed = speed
        elif motor == "B":
            self.motor_b_speed = speed

    def stop_all(self):
        """Stop both motors"""
        self.motor_a_speed = 0
        self.motor_b_speed = 0

    def cleanup(self):
        """Cleanup GPIO resources"""
        self.running = False
        self.pwm_thread.join()
        self.stop_all()
        self.IN1.release()
        self.IN2.release()
        self.IN3.release()
        self.IN4.release()
        self.chip.close()

def get_key():
    """Get a single key press without requiring Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1).lower()
        else:
            key = None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main_control_loop(controller):
    """Main control loop for WASD keys with speed control"""
    print("Motor Controller Ready")
    print("Controls: W-forward, S-backward, A-left, D-right, Space-stop, Q-quit")
    print("1-9: Set speed (1=slowest, 9=fastest)")
    
    current_speed = 50  # Default speed (50%)
    
    while True:
        key = get_key()
        
        if not key:
            continue
            
        if key == 'q':
            break
        elif key in ['1', '2', '3', '4', '5', '6', '7', '8', '9']:
            # Set speed (10% to 90% in 10% increments)
            current_speed = (int(key) * 10)
            print(f"Speed set to {current_speed}%")
        elif key == 'w':
            # Forward both motors
            controller.set_motor_speed("A", current_speed)
            controller.set_motor_speed("B", current_speed)
            print(f"Moving FORWARD at {current_speed}% speed")
        elif key == 's':
            # Backward both motors
            controller.set_motor_speed("A", -current_speed)
            controller.set_motor_speed("B", -current_speed)
            print(f"Moving BACKWARD at {current_speed}% speed")
        elif key == 'a':
            # Turn left (right motor forward, left motor backward)
            controller.set_motor_speed("A", -current_speed)
            controller.set_motor_speed("B", current_speed)
            print(f"Turning LEFT at {current_speed}% speed")
        elif key == 'd':
            # Turn right (left motor forward, right motor backward)
            controller.set_motor_speed("A", current_speed)
            controller.set_motor_speed("B", -current_speed)
            print(f"Turning RIGHT at {current_speed}% speed")
        elif key == ' ':
            # Stop all motors
            controller.stop_all()
            print("STOPPED")

if __name__ == "__main__":
    controller = MotorController()
    try:
        main_control_loop(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
        print("\nController stopped and resources released")


