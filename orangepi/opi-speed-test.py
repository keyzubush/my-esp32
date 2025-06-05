import cv2
import numpy as np
import time

# Initialize camera
cap = cv2.VideoCapture(1)
_, prev_frame = cap.read()
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# Parameters for Shi-Tomasi corner detection
feature_params = dict(
    maxCorners=200,
    qualityLevel=0.01,
    minDistance=30,
    blockSize=7
)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)

# Detect initial features
prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)

# Pixel-to-meter conversion (calibrate for your setup!)
# Example: 100 pixels = 0.1 meter (10 cm)
PIXEL_TO_METER = 0.1 / 100  

# For rotation calculation
prev_centroid = None
prev_angle = None
prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
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
        # Filter only valid points
        good_new = curr_pts[status == 1]
        good_old = prev_pts[status == 1]

        if len(good_new) > 1 and time_elapsed > 0:
            # ===== Translation Speed =====
            displacement_pixels = np.mean(np.linalg.norm(good_new - good_old, axis=1))
            displacement_meters = displacement_pixels * PIXEL_TO_METER
            translation_speed = displacement_meters / time_elapsed  # m/s

            # ===== Rotation Speed =====
            # 1. Compute centroid (center of mass)
            centroid_new = np.mean(good_new, axis=0)
            centroid_old = np.mean(good_old, axis=0)

            # 2. Compute vectors from centroid to each point
            vectors_new = good_new - centroid_new
            vectors_old = good_old - centroid_old

            # 3. Compute angles for all vectors
            angles_new = np.arctan2(vectors_new[:, 1], vectors_new[:, 0])
            angles_old = np.arctan2(vectors_old[:, 1], vectors_old[:, 0])

            # 4. Calculate angular differences (handle wrap-around)
            angle_diffs = np.arctan2(
                np.sin(angles_new - angles_old),
                np.cos(angles_new - angles_old)
            )

            # 5. Average rotation (radians)
            mean_rotation = np.mean(angle_diffs)
            rotation_speed = mean_rotation / time_elapsed  # rad/s

            print(
                f"Translation: {translation_speed:.2f} m/s | "
                f"Rotation: {np.degrees(rotation_speed):.2f} °/s"
            )

            # Visualization (optional)
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel().astype(int)
                c, d = old.ravel().astype(int)
                frame = cv2.line(frame, (a, b), (c, d), (0, 255, 0), 2)
                frame = cv2.circle(frame, (a, b), 3, (0, 0, 255), -1)

    # Update previous frame and points
    prev_gray = curr_gray.copy()
    prev_pts = good_new.reshape(-1, 1, 2) if curr_pts is not None else cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)

    # cv2.imshow("Optical Flow", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
