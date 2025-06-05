import cv2
import numpy as np
import gpiod
import time

# ===== Motor Controller Class =====
class MotorController:
    def __init__(self, chip='gpiochip0', pins=None):
        self.pin_map = pins or {
            'IN1': 270,  # GPIO4 (Orange Pi Zero 2W)
            'IN2': 228,  # GPIO5
            'IN3': 229,  # GPIO6
            'IN4': 262   # GPIO10
        }
        
        try:
            self.chip = gpiod.Chip(chip)
            self.lines = {
                'IN1': self.chip.get_line(self.pin_map['IN1']),
                'IN2': self.chip.get_line(self.pin_map['IN2']),
                'IN3': self.chip.get_line(self.pin_map['IN3']),
                'IN4': self.chip.get_line(self.pin_map['IN4'])
            }
            
            for line in self.lines.values():
                line.request(consumer="MotorCtrl", type=gpiod.LINE_REQ_DIR_OUT)
                
            self.stop_all()
        except Exception as e:
            raise RuntimeError(f"Motor init failed: {str(e)}")

    def _set_motor(self, motor, direction, speed=1.0):
        if motor not in ('A', 'B') or direction not in ('forward', 'backward', 'stop'):
            return

        # Motor A (IN1, IN2)
        if motor == 'A':
            if direction == 'forward':
                self._pwm_control(self.lines['IN1'], self.lines['IN2'], speed)
            elif direction == 'backward':
                self._pwm_control(self.lines['IN2'], self.lines['IN1'], speed)
            else:
                self.lines['IN1'].set_value(0)
                self.lines['IN2'].set_value(0)
        # Motor B (IN3, IN4)
        else:
            if direction == 'forward':
                self._pwm_control(self.lines['IN3'], self.lines['IN4'], speed)
            elif direction == 'backward':
                self._pwm_control(self.lines['IN4'], self.lines['IN3'], speed)
            else:
                self.lines['IN3'].set_value(0)
                self.lines['IN4'].set_value(0)

    def _pwm_control(self, positive, negative, duty):
        positive.set_value(1)
        negative.set_value(0)
        if duty < 1.0:
            time.sleep(0.01 * duty)
            positive.set_value(0)
            time.sleep(0.01 * (1.0 - duty))

    def move(self, left_speed, right_speed):
        """Direct speed control (-1.0 to 1.0 for each motor"""
        self._set_motor('A', 'forward' if left_speed >=0 else 'backward', abs(left_speed))
        self._set_motor('B', 'forward' if right_speed >=0 else 'backward', abs(right_speed))

    def stop_all(self):
        self._set_motor('A', 'stop')
        self._set_motor('B', 'stop')

    def cleanup(self):
        self.stop_all()
        for line in self.lines.values():
            line.release()
        self.chip.close()

# ===== Lane Detection System =====
class LaneFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)  # Limit framerate
        
        self.motors = MotorController()
        
        # PID Controller
        self.KP = 0.25
        self.KI = 0.01
        self.KD = 0.1
        self.prev_error = 0
        self.integral = 0
        
        # Region of Interest (Adjust based on camera mount)
        self.src_points = np.float32([
            [80, 140],   # Top-left
            [240, 140],  # Top-right
            [300, 220],  # Bottom-right
            [20, 220]    # Bottom-left
        ])
        
    def preprocess(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
        return thresh

    def warp_perspective(self, img):
        h, w = img.shape[:2]
        dst = np.float32([[0,0], [w,0], [w,h], [0,h]])
        M = cv2.getPerspectiveTransform(self.src_points, dst)
        return cv2.warpPerspective(img, M, (w,h))

    def detect_lanes(self, binary_img):
        histogram = np.sum(binary_img[binary_img.shape[0]//2:,:], axis=0)
        midpoint = len(histogram)//2
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        margin = 50
        minpix = 30
        
        nonzero = binary_img.nonzero()
        nonzeroy, nonzerox = nonzero[0], nonzero[1]
        
        leftx, lefty, rightx, righty = [], [], [], []
        
        for window in range(nwindows):
            win_y_low = binary_img.shape[0] - (window+1)*20
            win_y_high = binary_img.shape[0] - window*20
            
            left_win_x_low = left_base - margin
            left_win_x_high = left_base + margin
            right_win_x_low = right_base - margin
            right_win_x_high = right_base + margin
            
            good_left = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                        (nonzerox >= left_win_x_low) & (nonzerox < left_win_x_high)).nonzero()[0]
            good_right = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                         (nonzerox >= right_win_x_low) & (nonzerox < right_win_x_high)).nonzero()[0]
            
            leftx.extend(nonzerox[good_left])
            lefty.extend(nonzeroy[good_left])
            rightx.extend(nonzerox[good_right])
            righty.extend(nonzeroy[good_right])
            
            if len(good_left) > minpix:
                left_base = np.int(np.mean(nonzerox[good_left]))
            if len(good_right) > minpix:
                right_base = np.int(np.mean(nonzerox[good_right]))
        
        if len(leftx) == 0 or len(rightx) == 0:
            return None, None
            
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        return left_fit, right_fit

    def compute_steering(self, left_fit, right_fit):
        if left_fit is None or right_fit is None:
            return 0
            
        ploty = np.linspace(0, 239, 240)
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        
        lane_center = (left_fitx[-1] + right_fitx[-1]) / 2
        return (lane_center - 160) / 160  # Normalized error (-1 to 1)

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        correction = self.KP*error + self.KI*self.integral + self.KD*derivative
        self.prev_error = error
        return np.clip(correction, -1, 1)

    def run(self):
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # Process frame
                processed = self.preprocess(frame)
                warped = self.warp_perspective(processed)
                left_fit, right_fit = self.detect_lanes(warped)
                
                # Calculate steering
                error = self.compute_steering(left_fit, right_fit)
                correction = self.pid_control(error)
                
                # Motor control
                base_speed = 0.6
                left_speed = base_speed - correction
                right_speed = base_speed + correction
                self.motors.move(left_speed, right_speed)
                
                # Display (optional)
                cv2.imshow("Lane View", warped)
                if cv2.waitKey(1) == ord('q'):
                    break
                    
        finally:
            self.motors.cleanup()
            self.cap.release()
            cv2.destroyAllWindows()

# ===== Main Execution =====
if __name__ == "__main__":
    follower = LaneFollower()
    follower.run()
