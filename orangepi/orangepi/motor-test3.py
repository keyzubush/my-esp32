import gpiod
import sys
import termios
import tty
import select

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
        
        # Set initial state
        self.stop_all()

    def control_motor(self, motor, direction):
        """Control a single motor"""
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
