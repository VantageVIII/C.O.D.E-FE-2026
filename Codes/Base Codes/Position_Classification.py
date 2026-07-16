import time
import serial
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from huskylib import HuskyLensLibrary

# Initialize cameras
hl_left = HuskyLensLibrary("SERIAL", "/dev/ttyS3", 9600)   # Primary
hl_right = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)  # Secondary
gyro = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)

print("Knock Left:", hl_left.knock())
print("Knock Right:", hl_right.knock())
print("Gyro open:", gyro.is_open)

hl_left.algorthim("ALGORITHM_COLOR_RECOGNITION")
hl_right.algorthim("ALGORITHM_COLOR_RECOGNITION")
hl_left.setCustomName("Green", 1)
hl_left.setCustomName("Red", 2)
hl_right.setCustomName("Green", 1)
hl_right.setCustomName("Red", 2)

# Camera geometry
FOV = 60.0
WIDTH = 320
YAW_LEFT = +12.0
YAW_RIGHT = -12.0
PITCH = -12.0  # downward tilt

# Thresholds (replace with calibrated values!)
NEAR_THRESH = 100
MEDIUM_THRESH = 60
FAR_THRESH = 30

def classify_detection_global(x, width, fov_deg, yaw_deg, bbox_height, bbox_width, ID, cam):
    # Convert pixel to angle in camera frame
    norm_x = (x - width/2) / width
    local_angle = norm_x * fov_deg + yaw_deg

    # Normalize to global frame (remove yaw offset)
    global_angle = local_angle - yaw_deg

    # Row assignment based on global angle
    row = 0 if global_angle > 0 else 1

    # Column thresholds
    if bbox_height >= NEAR_THRESH:
        col = 0
    elif bbox_height >= MEDIUM_THRESH:
        col = 1
    else:
        col = 2

    grid_id = row*3 + col + 1
    return {"ID":ID,"row":row,"col":col,"grid":grid_id,
            "height":bbox_height,"width":bbox_width,
            "angle":global_angle,"cam":cam}

colors = {1:"green", 2:"red"}

while True:
    detections = []
    raw_left, raw_right = [], []

    # Collect from both cameras
    results_left = hl_left.requestAll()
    if results_left:
        for r in results_left:
            if hasattr(r,"ID"):
                raw_left.append(classify_detection_global(r.x, WIDTH, FOV, YAW_LEFT,
                                                          r.height, r.width, r.ID, "L"))

    results_right = hl_right.requestAll()
    if results_right:
        for r in results_right:
            if hasattr(r,"ID"):
                raw_right.append(classify_detection_global(r.x, WIDTH, FOV, YAW_RIGHT,
                                                           r.height, r.width, r.ID, "R"))

    # Primary = Left camera
    primary_dets = raw_left
    secondary_dets = [d for d in raw_right if d["angle"] < 0]  # inward half of right cam

    # Handle primary detections
    if len(primary_dets) == 2:
        L, R = primary_dets[0], primary_dets[1]
        size_L = L["height"] * L["width"]
        size_R = R["height"] * R["width"]

        if size_L > size_R:
            L["col"] = 0; R["col"] = 2
        else:
            R["col"] = 0; L["col"] = 2

        # Occlusion check
        if size_L < 0.3 * size_R:
            L["col"] = 2; R["col"] = 0
        elif size_R < 0.3 * size_L:
            R["col"] = 2; L["col"] = 0

        L["grid"] = L["row"]*3 + L["col"] + 1
        R["grid"] = R["row"]*3 + R["col"] + 1
        detections = [L, R]

    elif len(primary_dets) == 1:
        det = primary_dets[0]
        det["grid"] = det["row"]*3 + det["col"] + 1
        detections = [det]

        # Secondary confirmation check
        for sec in secondary_dets:
            if sec["ID"] == det["ID"]:
                primary_size = det["height"] * det["width"]
                secondary_size = sec["height"] * sec["width"]
                if secondary_size < 0.5 * primary_size:
                    # Hidden Far pillar confirmed
                    hidden = sec.copy()
                    hidden["col"] = 2  # Far
                    hidden["grid"] = hidden["row"]*3 + hidden["col"] + 1
                    detections.append(hidden)
                    print(f"Secondary confirmed hidden Far pillar behind {det['ID']}")

    # Plot detections
    plt.clf()
    plt.xlim(-0.5,2.5)
    plt.ylim(-0.5,1.5)
    plt.gca().set_aspect('equal')
    plt.title("2x3 Grid Mapping (Left Primary, Global Angle Correction)")
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
