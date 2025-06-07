import cv2

# Open the camera (usually /dev/video0 is camera index 0)
cap = cv2.VideoCapture(1)  # Use 0, 1, etc., or a video file path

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Press 'q' to quit")

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the frame in a window
    cv2.imshow("Camera Feed", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
