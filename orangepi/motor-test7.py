#!/usr/bin/env python3

import gpiod
import sys
import termios
import tty
import select
import os
import math
import time
from smbus2 import SMBus
from threading import Thread, Lock
import numpy as np

# MPU6050 Configuration
SLAVE_ADDR = 0x68
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
GYRO_Z = 0x47
ACCL_X = 0x3B
ACCL_Y = 0x3D
ACCL_Z = 0x3F

class MPU6050:
    def __init__(self, bus_num=3):
        self.bus = SMBus(bus_num)
        self.bus.write_byte_data(SLAVE_ADDR, PWR_MGMT_1, 0)
        self.gyro_z_cal = 0
        self.angle_z = 0
        self.lock = Lock()
        self.running = True
        self.calibration_samples = []
        
    def read_word(self, addr):
        h = self.bus.read_byte_data(SLAVE_ADDR, addr)
        l = self.bus.read_byte_data(SLAVE_ADDR, addr+1)
        val = (h << 8) + l
        return val
    
    def read_word_i2c(self, addr):
        val = self.read_word(addr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
    
    def calibrate(self, duration=3):
        """Calibrate the gyro by collecting samples over duration seconds"""
        print(f"Calibrating MPU6050 for {duration} seconds...")
        start_time = time.time()
        samples = []
        
        while time.time() - start_time < duration:
            raw_z = self.read_word_i2c(GYRO_Z)
            samples.append(raw_z)
            time.sleep(0.02)  # 50Hz sampling
            
        self.gyro_z_cal = np.mean(samples)
        print(f"Calibration complete. Offset: {self.gyro_z_cal:.2f}")
    
    def update(self):
        """Thread function to continuously update sensor readings"""
        prev_time = time.time()
        
        while self.running:
            current_time = time.time()
            delta_time = current_time - prev_time
            prev_time = current_time
            
            # Read gyro Z-axis
            raw_z = self.read_word_i2c(GYRO_Z)
            adj_z = (raw_z - self.gyro_z_cal) / 131.0  # Convert to deg/s
            self.angle_z += adj_z * delta_time
            
            # Read accelerometer (not used in this implementation but available)
            acc_x = self.read_word_i2c(ACCL_X) / 16384.0
            acc_y = self.read_word_i2c(ACCL_Y) / 16384.0
            acc_z = self.read_word_i2c(ACCL_Z) / 16384.0
            
            with self.lock:
                # Keep angle between -180 and 180 for easier interpretation
                if self.angle_z > 180:
                    self.angle_z -= 360
                elif self.angle_z < -180:
                    self.angle_z += 360
            
            time.sleep(0.02)  # ~50Hz update rate
    
    def get_rotation(self):
        """Get current Z-axis rotation angle"""
        with self.lock:
            return self.angle_z
    
    def stop(self):
        self.running = False

class MotorController:
    def __init__(self):
        self.chip = gpiod.Chip('gpiochip0')
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
        
        # Motor control variables
        self.current_speed = 90  # Default speed (50%)
        self.target_speed = 0
        self.direction = "stop"
        self.correction_active = False
        self.correction_factor = 0
        self.max_correction = 0.3  # Max 30% speed difference for correction
        
        # Setup PWM
        self.setup_pwm()
        self.stop_all()
        
        # Initialize MPU6050
        self.mpu = MPU6050()
        self.mpu.calibrate()  # 3-second calibration
        self.sensor_thread = Thread(target=self.mpu.update)
        self.sensor_thread.start()
    
    def setup_pwm(self):
        """Initialize PWM channels with default frequency"""
        try:
            if not os.path.exists(f"{self.PWM_CHIP_PATH}/pwm{self.PWM1_CHANNEL}"):
                with open(f"{self.PWM_CHIP_PATH}/export", 'w') as f:
                    f.write(str(self.PWM1_CHANNEL))
            
            if not os.path.exists(f"{self.PWM_CHIP_PATH}/pwm{self.PWM2_CHANNEL}"):
                with open(f"{self.PWM_CHIP_PATH}/export", 'w') as f:
                    f.write(str(self.PWM2_CHANNEL))
            
            period_ns = 1000000  # 1kHz frequency
            for channel in [self.PWM1_CHANNEL, self.PWM2_CHANNEL]:
                with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/period", 'w') as f:
                    f.write(str(period_ns))
                
                with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/duty_cycle", 'w') as f:
                    f.write(str(int(period_ns * 0.5)))
                
                with open(f"{self.PWM_CHIP_PATH}/pwm{channel}/enable", 'w') as f:
                    f.write('1')
                    
        except Exception as e:
            print(f"Error setting up PWM: {e}")
            raise
    
    def set_speed(self, speed_percent):
        """Set target speed for both motors (0-100%)"""
        self.target_speed = max(0, min(100, speed_percent))
        self.update_motor_speeds()
        print(f"Speed set to {self.target_speed}%")
    
    def update_motor_speeds(self):
        """Apply current speed with correction factor to motors"""
        if self.direction == "stop":
            left_speed = 0
            right_speed = 0
        else:
            base_speed = self.target_speed * (self.current_speed / 100)
            
            if self.correction_active and self.direction in ["forward", "backward"]:
                # Apply correction only when moving forward/backward
                correction = self.correction_factor * self.max_correction * base_speed
                left_speed = base_speed - correction
                right_speed = base_speed + correction
                
                # Clamp speeds to 0-100%
                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))
            else:
                left_speed = right_speed = base_speed
        
        try:
            period_ns = 1000000  # 1kHz frequency
            left_duty = int(period_ns * left_speed / 100)
            right_duty = int(period_ns * right_speed / 100)
            
            with open(f"{self.PWM_CHIP_PATH}/pwm{self.PWM1_CHANNEL}/duty_cycle", 'w') as f:
                f.write(str(left_duty))
            with open(f"{self.PWM_CHIP_PATH}/pwm{self.PWM2_CHANNEL}/duty_cycle", 'w') as f:
                f.write(str(right_duty))
                
        except Exception as e:
            print(f"Error updating motor speeds: {e}")
    
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
        self.direction = "stop"
        self.control_motor("A", "stop")
        self.control_motor("B", "stop")
        self.update_motor_speeds()
    
    def update_correction(self):
        """Update motor correction based on Z-axis rotation"""
        if not self.correction_active or self.direction == "stop":
            self.correction_factor = 0
            return
        
        angle = self.mpu.get_rotation()
        
        # Only correct if we're drifting (more than 5 degrees off)
        if abs(angle) > 5:
            # Proportional correction factor (0 to 1)
            self.correction_factor = max(-1, min(1, angle / 30.0))  # Full correction at 30 degrees
            self.update_motor_speeds()
        else:
            self.correction_factor = 0
    
    def toggle_correction(self):
        """Toggle automatic correction on/off"""
        self.correction_active = not self.correction_active
        state = "ON" if self.correction_active else "OFF"
        print(f"Auto-correction {state}")
        if not self.correction_active:
            self.correction_factor = 0
            self.update_motor_speeds()
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop_all()
        self.mpu.stop()
        self.sensor_thread.join()
        
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
    print("Motor Controller with MPU6050 Z-Axis Correction")
    print("Direction Controls: W-forward, S-backward, A-left, D-right, Space-stop, Q-quit")
    print("Speed Controls: 1-9 (10%-90%), 0 (100%)")
    print("C: Toggle auto-correction (currently OFF)")
    
    while True:
        key = get_key()
        
        if not key:
            # Update correction continuously when active
            if controller.correction_active and controller.direction != "stop":
                controller.update_correction()
            continue
            
        if key == 'q':
            break
        elif key == 'c':
            controller.toggle_correction()
        elif key in ['w', 's', 'a', 'd', ' ']:
            # Direction control
            if key == 'w':
                controller.direction = "forward"
                controller.control_motor("A", "forward")
                controller.control_motor("B", "forward")
                print("Moving FORWARD")
            elif key == 's':
                controller.direction = "backward"
                controller.control_motor("A", "backward")
                controller.control_motor("B", "backward")
                print("Moving BACKWARD")
            elif key == 'a':
                controller.direction = "left"
                controller.control_motor("A", "backward")
                controller.control_motor("B", "forward")
                print("Turning LEFT")
            elif key == 'd':
                controller.direction = "right"
                controller.control_motor("A", "forward")
                controller.control_motor("B", "backward")
                print("Turning RIGHT")
            elif key == ' ':
                controller.stop_all()
                print("STOPPED")
            
            # Update motor speeds after direction change
            controller.update_motor_speeds()
        elif key in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            # Speed control
            speed = int(key) * 10
            if key == '0':
                speed = 100
            controller.set_speed(speed)

if __name__ == "__main__":
    controller = MotorController()
    try:
        main_control_loop(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()
        print("\nController stopped and resources released")
