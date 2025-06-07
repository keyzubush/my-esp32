import cv2
import numpy as np
import time

# Initialize camera (headless mode may require backend tweaks)
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    raise RuntimeError("Cannot open camera. Check /dev/video0 permissions.")

# Set resolution for consistency (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Read first frame
_, prev_frame = cap.read()
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# Feature detection parameters
feature_params = dict(
    maxCorners=200,
    qualityLevel=0.01,
    minDistance=30,
    blockSize=7
)

# Optical flow parameters
lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)

# Initial feature detection
prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)

# Calibration (100 pixels = 0.1 meter)
PIXEL_TO_METER = 0.1 / 100  

# Timing
prev_time = time.time()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read error")
            break

        current_time = time.time()
        time_elapsed = current_time - prev_time
        prev_time = current_time

        curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate optical flow
        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray, curr_gray, prev_pts, None, **lk_params
        )

        if curr_pts is not None:
            good_new = curr_pts[status == 1]
            good_old = prev_pts[status == 1]

            if len(good_new) > 1 and time_elapsed > 0:
                # --- Translation ---
                displacement_pixels = np.mean(np.linalg.norm(good_new - good_old, axis=1))
                displacement_meters = displacement_pixels * PIXEL_TO_METER
                translation_speed = displacement_meters / time_elapsed

                # --- Rotation ---
                centroid_new = np.mean(good_new, axis=0)
                centroid_old = np.mean(good_old, axis=0)
                
                vectors_new = good_new - centroid_new
                vectors_old = good_old - centroid_old
                
                angles_new = np.arctan2(vectors_new[:, 1], vectors_new[:, 0])
                angles_old = np.arctan2(vectors_old[:, 1], vectors_old[:, 0])
                
                angle_diffs = np.arctan2(
                    np.sin(angles_new - angles_old),
                    np.cos(angles_new - angles_old)
                )
                rotation_speed = np.mean(angle_diffs) / time_elapsed

                # Print results (could log to file instead)
                print(
                    f"Time: {current_time:.3f} | "
                    f"Translation: {translation_speed:.3f} m/s | "
                    f"Rotation: {np.degrees(rotation_speed):.2f} °/s"
                )

        # Update for next iteration
        prev_gray = curr_gray.copy()
        prev_pts = good_new.reshape(-1, 1, 2) if curr_pts is not None else \
                   cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)

        # Exit on Ctrl+C (no GUI to capture 'q' key)
        
except KeyboardInterrupt:
    print("Stopping...")
finally:
    cap.release()