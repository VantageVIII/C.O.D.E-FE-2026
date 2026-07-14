import time
import serial
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from huskylib import HuskyLensLibrary

hl_left = HuskyLensLibrary("SERIAL", "/dev/ttyS3", 9600)
gyro = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)

print("Knock Left:", hl_left.knock())
print("Gyro open:", gyro.is_open)

hl_left.algorthim("ALGORITHM_COLOR_RECOGNITION")
hl_left.setCustomName("Green", 1)
hl_left.setCustomName("Red", 2)

FOV = 60.0
WIDTH = 320
YAW = 0.0

def pixel_to_angle(x, width, fov_deg, yaw_deg):
    norm_x = (x - width/2) / width
    angle = norm_x * fov_deg + yaw_deg
    return angle

def classify_detection(angle, bbox_height, bbox_width, ID):
    row = 0 if angle > 0 else 1  # left=0, right=1
    col = 1  # default medium if single
    grid_id = row*3 + col + 1
    return {"ID":ID,"row":row,"col":col,"grid":grid_id,
            "height":bbox_height,"width":bbox_width}

colors = {1:"green", 2:"red"}

while True:
    detections = []
    results = hl_left.requestAll()

    if results:
        raw = []
        for r in results:
            if hasattr(r,"ID"):
                angle = pixel_to_angle(r.x, WIDTH, FOV, YAW)
                raw.append(classify_detection(angle, r.height, r.width, r.ID))

        if len(raw) == 2:
            left = [d for d in raw if d["row"] == 0]
            right = [d for d in raw if d["row"] == 1]

            if left and right:
                L = left[0]
                R = right[0]

                size_left = L["height"] * L["width"]
                size_right = R["height"] * R["width"]

                if size_left > size_right:
                    L["col"] = 0  # near
                    R["col"] = 2  # far
                elif size_right > size_left:
                    R["col"] = 0  # near
                    L["col"] = 2  # far

                # Occlusion check
                if size_left < 0.3 * size_right:
                    L["col"] = 2
                    R["col"] = 0
                elif size_right < 0.3 * size_left:
                    R["col"] = 2
                    L["col"] = 0

                # Update grid IDs
                L["grid"] = L["row"]*3 + L["col"] + 1
                R["grid"] = R["row"]*3 + R["col"] + 1
                detections = [L, R]

        elif len(raw) == 1:
            # Single pillar → Medium column
            raw[0]["col"] = 1
            raw[0]["grid"] = raw[0]["row"]*3 + raw[0]["col"] + 1
            detections = raw

    # Plot detections
    plt.clf()
    plt.xlim(-0.5,2.5)
    plt.ylim(-0.5,1.5)
    plt.gca().set_aspect('equal')
    plt.title("2x3 Grid Mapping (Left Camera)")
    plt.xlabel("Column (Near–Medium–Far)")
    plt.ylabel("Row (Left–Right)")

    # Draw grid lines
    for i in range(3):
        plt.axvline(i-0.5, color="gray", linestyle=":")
    for j in range(2):
        plt.axhline(j-0.5, color="gray", linestyle=":")

    # Draw detections
    for det in detections:
        color = colors.get(det["ID"],"black")
        plt.gca().add_patch(
            plt.Rectangle((det["col"]-0.4, det["row"]-0.4),
                          0.8, 0.8, facecolor=color, edgecolor="black")
        )
        plt.text(det["col"], det["row"], str(det["grid"]),
                 ha="center", va="center", fontsize=10, color="white")

    plt.savefig("arena.png")
    time.sleep(0.2)
