import gpiod
import sys
import termios
import tty
import select

def setup_pwm(chip_path, channel, frequency, duty_cycle):
    try:
        # Export the PWM channel
        with open(f"{chip_path}/export", 'w') as f:
            f.write(str(channel))
        
        # Set period (nanoseconds)
        period_ns = int(1e9 / frequency)
        with open(f"{chip_path}/pwm{channel}/period", 'w') as f:
            f.write(str(period_ns))
        
        # Set duty cycle (nanoseconds)
        duty_ns = int(period_ns * duty_cycle / 100)
        with open(f"{chip_path}/pwm{channel}/duty_cycle", 'w') as f:
            f.write(str(duty_ns))
        
        # Enable PWM
        with open(f"{chip_path}/pwm{channel}/enable", 'w') as f:
            f.write('1')
        
        return True
    except Exception as e:
        print(f"Error setting up PWM: {e}")
        return False

def change_duty_cycle(chip_path, channel, duty_cycle):
    try:
        # Read current period
        with open(f"{chip_path}/pwm{channel}/period", 'r') as f:
            period_ns = int(f.read())
        
        # Calculate and set new duty cycle
        duty_ns = int(period_ns * duty_cycle / 100)
        with open(f"{chip_path}/pwm{channel}/duty_cycle", 'w') as f:
            f.write(str(duty_ns))
    except Exception as e:
        print(f"Error changing duty cycle: {e}")

class MotorController:
    def __init__(self):
        self.chip = gpiod.Chip('gpiochip0')  # Default GPIO chip
        
        self.PWM1 = self.chip.get_line(270)  
        self.PWM2 = self.chip.get_line(267)  
        # Initialize GPIO pins
        self.IN1 = self.chip.get_line(257)  
        self.IN2 = self.chip.get_line(228)  
        self.IN3 = self.chip.get_line(260)  
        self.IN4 = self.chip.get_line(76)          
        # Set lines as output
        self.IN1.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN2.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN3.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.IN4.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        #
        self.PWM1.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        self.PWM2.request(consumer="MOTOR", type=gpiod.LINE_REQ_DIR_OUT)
        
        # Set initial state
        self.stop_all()

    def control_motor(self, motor, direction):
        """Control a single motor"""
        self.PWM1.set_value(1)
        self.PWM2.set_value(1)
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

    def stop_all(self):
        """Stop both motors"""
        self.control_motor("A", "stop")
        self.control_motor("B", "stop")

    def cleanup(self):
        """Cleanup GPIO resources"""
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
    """Main control loop for WASD keys"""
    print("Motor Controller Ready")
    print("Controls: W-forward, S-backward, A-left, D-right, Space-stop, Q-quit")
    
    while True:
        key = get_key()
        
        if not key:
            continue
            
        if key == 'q':
            break
        elif key == 'w':
            # Forward both motors
            controller.control_motor("A", "forward")
            controller.control_motor("B", "forward")
            print("Moving FORWARD")
        elif key == 's':
            # Backward both motors
            controller.control_motor("A", "backward")
            controller.control_motor("B", "backward")
            print("Moving BACKWARD")
        elif key == 'a':
            # Turn left (right motor forward, left motor backward)
            controller.control_motor("A", "backward")
            controller.control_motor("B", "forward")
            print("Turning LEFT")
        elif key == 'd':
            # Turn right (left motor forward, right motor backward)
            controller.control_motor("A", "forward")
            controller.control_motor("B", "backward")
            print("Turning RIGHT")
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
