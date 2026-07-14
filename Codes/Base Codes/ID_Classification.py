import Hobot.GPIO as GPIO
import time
import serial
import struct
import smbus2
import threading
from huskylib import HuskyLensLibrary
# -----------------------------
# UART Setup
# -----------------------------
hl_right = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)
hl_left = HuskyLensLibrary("SERIAL", "/dev/ttyS3", 9600)
gyro = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)

print("Knock Left:", hl_left.knock())
print("Knock Right:", hl_right.knock())
print("Gyro open:", gyro.is_open)

# -----------------------------
# Switch to Color Recognition algorithm
# -----------------------------
print("LeftCam algo:", hl_left.algorthim("ALGORITHM_COLOR_RECOGNITION"))
print("RightCam algo:", hl_right.algorthim("ALGORITHM_COLOR_RECOGNITION"))

# -----------------------------
# Rename IDs at device level
# -----------------------------
print("LeftCam rename:", hl_left.setCustomName("Green", 1))
print("LeftCam rename:", hl_left.setCustomName("Red", 2))
print("RightCam rename:", hl_right.setCustomName("Green", 1))
print("RightCam rename:", hl_right.setCustomName("Red", 2))

# -----------------------------
# Primary camera variable
# -----------------------------
PRIMARY_CAMERA = "LeftCam"  # placeholder, you can assign dynamically later

# -----------------------------
# Main Loop
# -----------------------------
while True:
    # Left camera detections
    results = hl_left.requestAll()
    if results:
        for r in results:
            if hasattr(r, "ID"):
                print(f"[LeftCam] Valid ID{r.ID} detection at ({r.x},{r.y}) size ({r.width}x{r.height})")

    # Right camera detections
    results = hl_right.requestAll()
    if results:
        for r in results:
            if hasattr(r, "ID"):
                print(f"[RightCam] Valid ID{r.ID} detection at ({r.x},{r.y}) size ({r.width}x{r.height})")

    # Display camera ID labels in top-right corner
    # Camera 1 = primary, Camera 2 = secondary
    hl_left.customText("1" if PRIMARY_CAMERA == "LeftCam" else "2", 300, 10)
    hl_right.customText("1" if PRIMARY_CAMERA == "RightCam" else "2", 300, 10)

    time.sleep(0.2)
