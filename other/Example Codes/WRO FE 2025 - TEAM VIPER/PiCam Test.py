import cv2
import numpy as np

# Camera capture
cap = cv2.VideoCapture(0)

# HSV color range for object
lower = np.array([0, 100, 100])
upper = np.array([10, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create color mask
    mask = cv2.inRange(hsv, lower, upper)

    # Track largest object
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        (x, y, w, h) = cv2.boundingRect(c)
        cx, cy = x + w//2, y + h//2
        print(f"Object at X:{cx}, Y:{cy}")

cap.release()
