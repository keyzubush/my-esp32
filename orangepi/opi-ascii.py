import cv2
import numpy as np
import time
import os

# ASCII character ramp (70 levels from dark to light)
ASCII_CHARS = "@%#*+=-:. "

def frame_to_ascii(frame, cols=80, rows=40):
    """Convert camera frame to ASCII art"""
    # Resize and convert to grayscale
    height, width = frame.shape[:2]
    cell_width = width / cols
    cell_height = height / rows
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ascii_frame = ""
    for i in range(rows):
        for j in range(cols):
            # Get average brightness in cell
            x1, y1 = int(j * cell_width), int(i * cell_height)
            x2, y2 = int((j+1) * cell_width), int((i+1) * cell_height)
            cell = gray[y1:y2, x1:x2]
            avg_brightness = np.mean(cell) if cell.size > 0 else 0
            
            # Map brightness to ASCII character (fixed parentheses)
            char_index = min(((avg_brightness / 255) * len(ASCII_CHARS)-1), len(ASCII_CHARS)-1)
            ascii_frame += ASCII_CHARS[int(char_index)]
        ascii_frame += "\n"
    return ascii_frame

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 20)  # Limit framerate
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            # Generate and display ASCII art
            ascii_art = frame_to_ascii(frame)
            os.system('clear' if os.name == 'posix' else 'cls')
            print(ascii_art)
            print("Press Ctrl+C to exit")
            
            # Control frame rate
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        cap.release()

if __name__ == "__main__":
    main()
