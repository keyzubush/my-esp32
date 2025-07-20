import cv2
import numpy as np
import time
import os

# ASCII character ramp (70 levels from dark to light)
ASCII_CHARS = "@%#*+=-:. "

def frame_to_ascii(frame, cols=40, rows=20):
    """Convert camera frame to ASCII art"""
    # Resize and convert to grayscale
    height, width = frame.shape[:2]
    cell_width = width / cols
    cell_height = height / rows
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    ascii_frame = []
    for i in range(rows):
        for j in range(cols):
            # Get average brightness in cell
            x1, y1 = int(j * cell_width), int(i * cell_height)
            x2, y2 = int((j+1) * cell_width), int((i+1) * cell_height)
            cell = gray[y1:y2, x1:x2]
            avg_brightness = np.mean(cell) if cell.size > 0 else 0
            
            # Map brightness to ASCII character (fixed parentheses)
            char_index = min(((avg_brightness / 255) * len(ASCII_CHARS)-1), len(ASCII_CHARS)-1)
            ascii_frame.append( ASCII_CHARS[int(char_index)] )
        ascii_frame.append( "\n" )
    return "".join(ascii_frame)

def main():
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 30)  # Limit framerate
   
    # List of common CAP_PROP_* properties
    props = {
        "CAP_PROP_POS_MSEC": cv2.CAP_PROP_POS_MSEC,
        "CAP_PROP_POS_FRAMES": cv2.CAP_PROP_POS_FRAMES,
        "CAP_PROP_POS_AVI_RATIO": cv2.CAP_PROP_POS_AVI_RATIO,
        "CAP_PROP_FRAME_WIDTH": cv2.CAP_PROP_FRAME_WIDTH,
        "CAP_PROP_FRAME_HEIGHT": cv2.CAP_PROP_FRAME_HEIGHT,
        "CAP_PROP_FPS": cv2.CAP_PROP_FPS,
        "CAP_PROP_FOURCC": cv2.CAP_PROP_FOURCC,
        "CAP_PROP_FRAME_COUNT": cv2.CAP_PROP_FRAME_COUNT,
        "CAP_PROP_FORMAT": cv2.CAP_PROP_FORMAT,
        "CAP_PROP_MODE": cv2.CAP_PROP_MODE,
        "CAP_PROP_BRIGHTNESS": cv2.CAP_PROP_BRIGHTNESS,
        "CAP_PROP_CONTRAST": cv2.CAP_PROP_CONTRAST,
        "CAP_PROP_SATURATION": cv2.CAP_PROP_SATURATION,
        "CAP_PROP_HUE": cv2.CAP_PROP_HUE,
        "CAP_PROP_GAIN": cv2.CAP_PROP_GAIN,
        "CAP_PROP_EXPOSURE": cv2.CAP_PROP_EXPOSURE,
        "CAP_PROP_AUTO_EXPOSURE": cv2.CAP_PROP_AUTO_EXPOSURE,
        "CAP_PROP_ZOOM": cv2.CAP_PROP_ZOOM,
        "CAP_PROP_FOCUS": cv2.CAP_PROP_FOCUS,
        "CAP_PROP_AUTOFOCUS": cv2.CAP_PROP_AUTOFOCUS,
        "CAP_PROP_BUFFERSIZE": cv2.CAP_PROP_BUFFERSIZE,
    }

    print("Current VideoCapture Properties:")
    for name, prop in props.items():
        value = cap.get(prop)
        print(f"{name}: {value}")

    time.sleep(10)

    try:
        while True:
            starttime = time.time()
            # ret, frame = cap.read()
            ret, big_frame = cap.read()
            frame = cv2.resize(big_frame, (160, 120))
            readtime = time.time()
            if not ret:
                print("sleep ", time.time() )
                time.sleep(0.1)
                continue
            
            # Generate and display ASCII art
            ascii_art = frame_to_ascii(frame)
            print("\033c", end="")
            # os.system('clear' if os.name == 'posix' else 'cls')
            print(ascii_art)
            print("Press Ctrl+C to exit ", readtime - starttime, time.time() )
            
            # Control frame rate
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        cap.release()

if __name__ == "__main__":
    main()
