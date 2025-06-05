import cv2
import numpy as np
import gpiod
import time
import math

# ===== Enhanced Motor Controller =====
class MotorController:
    def __init__(self, chip='gpiochip0', pins=None):
        self.pin_map = pins or {
            'IN1': 270, 'IN2': 228,  # Left Motor
            'IN3': 229, 'IN4': 262   # Right Motor
        }
        
        try:
            self.chip = gpiod.Chip(chip)
            self.lines = {name: self.chip.get_line(pin) 
                         for name, pin in self.pin_map.items()}
            for line in self.lines.values():
                line.request(consumer="MotorCtrl", type=gpiod.LINE_REQ_DIR_OUT)
            self.stop_all()
        except Exception as e:
            raise RuntimeError(f"Motor init failed: {str(e)}")

    def _safe_pwm(self, line, duty):
        """Robust PWM with duty cycle clamping"""
        line.set_value(1)
        time.sleep(0.005 * max(0, min(1, duty)))  # Clamped duty cycle
        line.set_value(0)

    def move(self, left_speed, right_speed):
        """NaN-safe motor control"""
        if not math.isnan(left_speed) and (-1 <= left_speed <= 1):
            if left_speed > 0:
                self._safe_pwm(self.lines['IN1'], left_speed)
            elif left_speed < 0:
                self._safe_pwm(self.lines['IN2'], abs(left_speed))

        if not math.isnan(right_speed) and (-1 <= right_speed <= 1):
            if right_speed > 0:
                self._safe_pwm(self.lines['IN3'], right_speed)
            elif right_speed < 0:
                self._safe_pwm(self.lines['IN4'], abs(right_speed))

    def cleanup(self):
        self.stop_all()
        for line in self.lines.values():
            line.release()
        self.chip.close()

# ===== Robust Lane Follower =====
class LaneFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        self.cap.set(cv2.CAP_PROP_FPS, 20)  # Limit framerate
        
        self.motors = MotorController()
        self.KP, self.KI, self.KD = 0.3, 0.005, 0.08
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

            nwindows = 5
            margin = 30
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
                    time.sleep(0.1)
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
                
                # Motor control
                base_speed = 0.6
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
