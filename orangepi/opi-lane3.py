import cv2
import numpy as np
import gpiod
import time
import math
import threading

# ===== Enhanced Motor Controller with PWM Thread =====
class MotorController:
    def __init__(self):
        try:
            self.chip = gpiod.Chip('gpiochip0')  # Default GPIO chip
            
            # Initialize GPIO pins - using same pins as motor-test4.py
            self.IN1 = self.chip.get_line(268)  # GPIO pin 268
            self.IN2 = self.chip.get_line(267)  # GPIO pin 267
            self.IN3 = self.chip.get_line(269)  # GPIO pin 269
            self.IN4 = self.chip.get_line(270)  # GPIO pin 270
            
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
            
        except Exception as e:
            raise RuntimeError(f"Motor init failed: {str(e)}")

    def _pwm_control_loop(self):
        """Pseudo-PWM control loop running in a separate thread"""
        while self.running:
            # Control Motor A (Left motor - IN1/IN2)
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
            
            # Control Motor B (Right motor - IN3/IN4)
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

    def move(self, left_speed, right_speed):
        """Set motor speeds (-1.0 to 1.0 range)"""
        # Convert from -1.0..1.0 range to -100..100 range
        left_speed = max(-1.0, min(1.0, left_speed)) * 100
        right_speed = max(-1.0, min(1.0, right_speed)) * 100
        
        self.motor_a_speed = left_speed
        self.motor_b_speed = right_speed

    def cleanup(self):
        """Cleanup GPIO resources"""
        self.running = False
        if hasattr(self, 'pwm_thread'):
            self.pwm_thread.join()
        self.IN1.set_value(0)
        self.IN2.set_value(0)
        self.IN3.set_value(0)
        self.IN4.set_value(0)
        self.IN1.release()
        self.IN2.release()
        self.IN3.release()
        self.IN4.release()
        self.chip.close()

# ===== Robust Lane Follower =====
class LaneFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        self.cap.set(cv2.CAP_PROP_FPS, 20)  # Limit framerate
        
        self.motors = MotorController()
        self.KP, self.KI, self.KD = 6, 0.02, 4
        self.prev_error = 0
        self.integral = 0
        
        # ROI for 160x120
        self.src_points = np.float32([
            [40,70], [120,70], [150,110], [10,110]
        ])

    def _safe_mean(self, arr):
        """Handle empty arrays and NaN values"""
        return np.nan if len(arr) == 0 else np.nanmean(arr)

    def detect_lanes(self, binary_img):
        try:
            histogram = np.sum(binary_img[-40:,:], axis=0)
            midpoint = len(histogram)//2
            left_base = np.argmax(histogram[:midpoint])
            right_base = np.argmax(histogram[midpoint:]) + midpoint

            #nwindows = 5
            #margin = 30
            #minpix = 15
            nwindows = 2
            margin = 40
            minpix = 15
            
            nonzero = binary_img.nonzero()
            nonzeroy, nonzerox = nonzero[0], nonzero[1]
            
            leftx, lefty, rightx, righty = [], [], [], []
            
            for window in range(nwindows):
                win_y_low = binary_img.shape[0] - (window+1)*20
                win_y_high = binary_img.shape[0] - window*20
                
                # Left lane
                good_left = ((nonzeroy >= win_y_low) & 
                            (nonzeroy < win_y_high) & 
                            (nonzerox >= max(left_base-margin,0)) & 
                            (nonzerox < min(left_base+margin,binary_img.shape[1])))
                
                # Right lane
                good_right = ((nonzeroy >= win_y_low) & 
                             (nonzeroy < win_y_high) & 
                             (nonzerox >= max(right_base-margin,0)) & 
                             (nonzerox < min(right_base+margin,binary_img.shape[1])))
                
                leftx.extend(nonzerox[good_left])
                lefty.extend(nonzeroy[good_left])
                rightx.extend(nonzerox[good_right])
                righty.extend(nonzeroy[good_right])
                
                left_mean = self._safe_mean(nonzerox[good_left])
                right_mean = self._safe_mean(nonzerox[good_right])
                
                if not np.isnan(left_mean) and len(good_left) > minpix:
                    left_base = int(left_mean)
                if not np.isnan(right_mean) and len(good_right) > minpix:
                    right_base = int(right_mean)
            
            # Minimum points check
            if len(leftx) < 10 or len(rightx) < 10:
                return None, None
                
            return np.polyfit(lefty, leftx, 2), np.polyfit(righty, rightx, 2)
            
        except Exception as e:
            print(f"Lane detection error: {str(e)}")
            return None, None

    def compute_steering(self, left_fit, right_fit):
        if left_fit is None or right_fit is None:
            return 0
            
        try:
            y = 120  # Bottom of image
            left_x = left_fit[0]*y**2 + left_fit[1]*y + left_fit[2]
            right_x = right_fit[0]*y**2 + right_fit[1]*y + right_fit[2]
            return ((left_x + right_x)/2 - 80) / 80  # Normalized error
        except:
            return 0

    def run(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.05)
                    continue
                
                # Processing pipeline
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
                
                left_fit, right_fit = self.detect_lanes(thresh)
                error = self.compute_steering(left_fit, right_fit)
                
                # NaN-safe PID
                if not math.isnan(error):
                    self.integral = 0.95*self.integral + error
                    derivative = error - self.prev_error
                    correction = self.KP*error + self.KI*self.integral + self.KD*derivative
                    self.prev_error = error
                else:
                    correction = 0
                
                # Motor control with new PWM system
                base_speed = 1
                left_speed = base_speed - correction
                right_speed = base_speed + correction
                self.motors.move(left_speed, right_speed)
                
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.motors.cleanup()
            self.cap.release()

if __name__ == "__main__":
    follower = LaneFollower()
    follower.run()

