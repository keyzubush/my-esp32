import gpiod
import sys
import termios
import tty
import select
import os

from mpu6050 import *

class MotorController:
    def __init__(self):
        self.chip = gpiod.Chip('gpiochip0')  # Default GPIO chip
        
        # PWM chip paths (adjust these according to your system)
        self.PWM_CHIP_PATH = "/sys/class/pwm/pwmchip0"
        self.PWM1_CHANNEL = 1  # Channel for motor A
        self.PWM2_CHANNEL = 4  # Channel for motor B
        
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
        
        # Set initial state
        self.current_speed = 95  # Default speed (50%)
        self.stop_all()
        
        # Setup PWM
        self.setup_pwm()

    def setup_pwm(self):
        """Initialize PWM channels with default frequency"""
        try:
            # Setup PWM channels with 1kHz frequency
            if not os.path.exists(f"{self.PWM_CHIP_PATH}/pwm{self.PWM1_CHANNEL}"):
                with open(f"{self.PWM_CHIP_PATH}/export", 'w') as f:
                    f.write(str(self.PWM1_CHANNEL))
            
            if not os.path.exists(f"{self.PWM_CHIP_PATH}/pwm{self.PWM2_CHANNEL}"):
                with open(f"{self.PWM_CHIP_PATH}/export", 'w') as f:
                    f.write(str(self.PWM2_CHANNEL))
            
            # Set frequency to 1kHz (period = 1,000,000 ns)
            period_ns = 1000000
            for channel in [self.PWM1_CHANNEL, self.PWM2_CHANNEL]:
                with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/period", 'w') as f:
                    f.write(str(period_ns))
                
                # Set initial duty cycle (50%)
                with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/duty_cycle", 'w') as f:
                    f.write(str(int(period_ns * 0.95)))
                
                # Enable PWM
                with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/enable", 'w') as f:
                    f.write('1')
                    
        except Exception as e:
            print(f"Error setting up PWM: {e}")
            raise

    def set_speed(self, channel, speed_percent):
        """Set speed for both motors (0-100%)"""
        try:
            self.current_speed = max(0, min(100, speed_percent))  # Clamp to 0-100
            
            # Read current period
            with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/period", 'r') as f:
                period_ns = int(f.read())
            
            # Calculate and set new duty cycle
            duty_ns = int(period_ns * self.current_speed / 100)
            
            with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/duty_cycle", 'w') as f:
                f.write(str(duty_ns))
                    
            print(f"Speed set to {self.current_speed}%")
            
        except Exception as e:
            print(f"Error changing speed: {e}")

    def control_motor(self, motor, direction):
        """Control a single motor's direction"""
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
        """Cleanup GPIO and PWM resources"""
        self.stop_all()
        
        # Release GPIO lines
        self.IN1.release()
        self.IN2.release()
        self.IN3.release()
        self.IN4.release()
        
        # Disable PWM channels
        try:
            for channel in [self.PWM1_CHANNEL, self.PWM2_CHANNEL]:
                with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/enable", 'w') as f:
                    f.write('0')
                with open(f"{self.PWM_CHIP_PATH}/unexport", 'w') as f:
                    f.write(str(channel))
        except Exception as e:
            print(f"Error cleaning up PWM: {e}")
            
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
    """Main control loop for WASD keys and speed control"""

    mpu = MPU6050()
    mpu.calibrate()
    mpu.start()

    print("Motor Controller Ready")
    print("Direction Controls: W-forward, S-backward, A-left, D-right, Space-stop, Q-quit")
    print("Speed Controls: 1-9 (10%-90%), 0 (100%)")
   
    i = 0
    speed = 95
    while True:
        key = get_key()

        data = mpu.get_data()
 
        if not key:
            if i % 5 == 0:
                print(f"Distance: {data['distance']:.2f}m, Integrated Gyro Z: {data['integrated_gyro_z']:.2f}°")
            i += 1
            continue
            
        if key == 'q':
            break
        elif key in ['w', 's', 'a', 'd', ' ']:
            # Direction control
            if key == 'w':
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
        elif key in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            # Speed control
            speed = int(key) * 10
            if key == '0':
                speed = 100  # 0 means 100% speed
            controller.set_speed(controller.PWM1_CHANNEL, speed)
            controller.set_speed(controller.PWM2_CHANNEL, speed)

if __name__ == "__main__":
    controller = MotorController()
    try:
        main_control_loop(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
        print("\nController stopped and resources released")

