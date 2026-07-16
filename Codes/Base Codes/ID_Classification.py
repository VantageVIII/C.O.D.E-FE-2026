import time
import serial
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from huskylib import HuskyLensLibrary

# -------------------------
# Hardware initialization
# -------------------------
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

# -------------------------
# Geometry and thresholds
# -------------------------
FOV = 60.0
WIDTH = 320
YAW_LEFT = +12.0
YAW_RIGHT = -12.0
PITCH = -12.0  # downward tilt

NEAR_THRESH = 100
MEDIUM_THRESH = 60
FAR_THRESH = 30

colors = {1: "green", 2: "red"}

# -------------------------
# Grid helpers
# -------------------------
def get_grid_map(orientation):
    """
    Returns a 2x3 grid_map[row][col] -> position id (1..6)
    orientation: "anticlockwise" or "clockwise"
    Anticlockwise:
        6 | 5 | 4
        3 | 2 | 1
    Clockwise:
        4 | 5 | 6
        1 | 2 | 3
    """
    if orientation == "anticlockwise":
        return [[6, 5, 4],
                [3, 2, 1]]
    else:
        return [[4, 5, 6],
                [1, 2, 3]]

def grid_id_from_row_col(row, col, orientation):
    # Defensive check to avoid IndexError
    if row not in (0, 1) or col not in (0, 1, 2):
        return None
    grid_map = get_grid_map(orientation)
    return grid_map[row][col]

# -------------------------
# Layout definitions
# -------------------------
# Anticlockwise layout definitions (IDs 1–36)
layout_definitions_anticlockwise = {
    1: [("green", 6)],
    2: [("red", 6)],
    3: [("green", 5)],
    4: [("red", 5)],
    5: [("green", 4)],
    6: [("red", 4)],
    7: [("green", 3)],
    8: [("red", 3)],
    9: [("green", 2)],
    10: [("red", 2)],
    11: [("green", 4)],
    12: [("red", 4)],
    13: [("green", 3), ("green", 4)],
    14: [("green", 3), ("red", 4)],
    15: [("red", 3), ("green", 4)],
    16: [("green", 3), ("red", 4)],
    17: [("red", 3), ("green", 4)],
    18: [("red", 3), ("red", 4)],
    19: [("green", 6), ("green", 1)],
    20: [("green", 6), ("red", 1)],
    21: [("red", 6), ("green", 1)],
    22: [("green", 6), ("red", 1)],
    23: [("red", 6), ("green", 1)],
    24: [("red", 6), ("red", 1)],
    25: [("green", 6), ("green", 4)],
    26: [("green", 6), ("red", 4)],
    27: [("red", 6), ("green", 4)],
    28: [("green", 6), ("red", 4)],
    29: [("red", 6), ("green", 4)],
    30: [("red", 6), ("red", 4)],
    31: [("green", 3), ("green", 1)],
    32: [("green", 3), ("red", 1)],
    33: [("red", 3), ("green", 1)],
    34: [("green", 3), ("red", 1)],
    35: [("red", 3), ("green", 1)],
    36: [("red", 3), ("red", 1)]
}

# Clockwise layout definitions (IDs 1–36)
# These follow the clockwise grid mapping you confirmed; duplicates are intentional.
layout_definitions_clockwise = {
    1: [("green", 4)],
    2: [("red", 4)],
    3: [("green", 5)],
    4: [("red", 5)],
    5: [("green", 6)],
    6: [("red", 6)],
    7: [("green", 1)],
    8: [("red", 1)],
    9: [("green", 2)],
    10: [("red", 2)],
    11: [("green", 6)],
    12: [("red", 6)],
    13: [("green", 1), ("green", 6)],
    14: [("green", 1), ("red", 6)],
    15: [("red", 1), ("green", 6)],
    16: [("green", 1), ("red", 6)],
    17: [("red", 1), ("green", 6)],
    18: [("red", 1), ("red", 6)],
    19: [("green", 4), ("green", 3)],
    20: [("green", 4), ("red", 3)],
    21: [("red", 4), ("green", 3)],
    22: [("green", 4), ("red", 3)],
    23: [("red", 4), ("green", 3)],
    24: [("red", 4), ("red", 3)],
    25: [("green", 4), ("green", 6)],
    26: [("green", 4), ("red", 6)],
    27: [("red", 4), ("green", 6)],
    28: [("green", 4), ("red", 6)],
    29: [("red", 4), ("green", 6)],
    30: [("red", 4), ("red", 6)],
    31: [("green", 1), ("green", 3)],
    32: [("green", 1), ("red", 3)],
    33: [("red", 1), ("green", 3)],
    34: [("green", 1), ("red", 3)],
    35: [("red", 1), ("green", 3)],
    36: [("red", 1), ("red", 3)]
}

# -------------------------
# Detection classification
# -------------------------
def classify_detection_global(x, width, fov_deg, yaw_deg, bbox_height, bbox_width, ID, cam, orientation):
    """
    Classify detection into row (0 top, 1 bottom) and col (0 near,1 medium,2 far),
    then map to grid position id according to orientation.
    """
    norm_x = (x - width / 2) / width
    local_angle = norm_x * fov_deg + yaw_deg
    global_angle = local_angle - yaw_deg

    row = 0 if global_angle > 0 else 1

    if bbox_height >= NEAR_THRESH:
        col = 0
    elif bbox_height >= MEDIUM_THRESH:
        col = 1
    else:
        col = 2

    grid_id = grid_id_from_row_col(row, col, orientation)
    return {
        "ID": ID,
        "row": row,
        "col": col,
        "grid": grid_id,
        "height": bbox_height,
        "width": bbox_width,
        "angle": global_angle,
        "cam": cam
    }

# -------------------------
# Matching logic with superset filtering
# -------------------------
def match_layout(detections, layout_definitions):
    """
    Return matched layout IDs. If a matched layout is a strict subset of another matched layout,
    the subset is removed so only the most complete layouts remain.
    """
    matches = []
    for layout_id, expected in layout_definitions.items():
        ok = True
        for color, pos in expected:
            found = any(det["grid"] == pos and colors.get(det["ID"]) == color for det in detections)
            if not found:
                ok = False
                break
        if ok:
            matches.append(layout_id)

    # Filter out subsets
    final_matches = []
    for m in matches:
        expected_m = set(layout_definitions[m])
        is_subset = False
        for n in matches:
            if m == n:
                continue
            expected_n = set(layout_definitions[n])
            if expected_m.issubset(expected_n):
                # If m is subset of n, drop m
                is_subset = True
                break
        if not is_subset:
            final_matches.append(m)

    return final_matches

# -------------------------
# Orientation selection
# -------------------------
# Set orientation to "anticlockwise" or "clockwise"
orientation = "anticlockwise"
layout_definitions = layout_definitions_anticlockwise if orientation == "anticlockwise" else layout_definitions_clockwise

# -------------------------
# Main loop
# -------------------------
while True:
    detections = []
    raw_left, raw_right = [], []

    # Collect from left camera (primary)
    results_left = hl_left.requestAll()
    if results_left:
        for r in results_left:
            if hasattr(r, "ID"):
                raw_left.append(classify_detection_global(
                    r.x, WIDTH, FOV, YAW_LEFT, r.height, r.width, r.ID, "L", orientation
                ))

    # Collect from right camera (secondary)
    results_right = hl_right.requestAll()
    if results_right:
        for r in results_right:
            if hasattr(r, "ID"):
                raw_right.append(classify_detection_global(
                    r.x, WIDTH, FOV, YAW_RIGHT, r.height, r.width, r.ID, "R", orientation
                ))

    # Primary = left camera detections
    primary_dets = raw_left
    # Secondary inward half of right cam (angle < 0)
    secondary_dets = [d for d in raw_right if d["angle"] < 0]

    # Handle two primary detections
    if len(primary_dets) == 2:
        L, R = primary_dets[0], primary_dets[1]
        size_L = L["height"] * L["width"]
        size_R = R["height"] * R["width"]

        # Heuristic: larger object is nearer (col=0), smaller is far (col=2)
        if size_L > size_R:
            L["col"] = 0
            R["col"] = 2
        else:
            R["col"] = 0
            L["col"] = 2

        # Occlusion adjustments
        if size_L < 0.3 * size_R:
            L["col"] = 2
            R["col"] = 0
        elif size_R < 0.3 * size_L:
            R["col"] = 2
            L["col"] = 0

        # Recompute grid ids after col adjustments
        L["grid"] = grid_id_from_row_col(L["row"], L["col"], orientation)
        R["grid"] = grid_id_from_row_col(R["row"], R["col"], orientation)
        detections = [L, R]

    # Handle single primary detection (possible hidden far pillar)
    elif len(primary_dets) == 1:
        det = primary_dets[0]
        # grid already set by classify_detection_global
        detections = [det]

        # Secondary confirmation check for hidden far pillar behind primary
        for sec in secondary_dets:
            if sec["ID"] == det["ID"]:
                primary_size = det["height"] * det["width"]
                secondary_size = sec["height"] * sec["width"]
                if secondary_size < 0.5 * primary_size:
                    # Hidden Far pillar confirmed: set col=2 (far) and recompute grid
                    hidden = sec.copy()
                    hidden["col"] = 2
                    hidden["grid"] = grid_id_from_row_col(hidden["row"], hidden["col"], orientation)
                    detections.append(hidden)
                    print(f"Secondary confirmed hidden Far pillar behind {det['ID']}")

    # If no primary detections, optionally consider secondary detections alone
    else:
        # Try to use right camera inward detections if left sees nothing
        if secondary_dets:
            # Use secondary detections as primary fallback
            for d in secondary_dets:
                # keep their grid as classified
                detections.append(d)

    # Identify matching Layout IDs (with superset filtering)
    matched_layouts = match_layout(detections, layout_definitions)
    if matched_layouts:
        print(f"Detected Layout IDs: {matched_layouts}")
    else:
        print("No matching Layout ID found yet")

    # -------------------------
    # Plot detections (2x3 schematic)
    # -------------------------
    plt.clf()
    plt.xlim(-0.5, 2.5)
    plt.ylim(-0.5, 1.5)
    plt.gca().set_aspect('equal')
    plt.title(f"2x3 Grid Mapping ({orientation})")
    plt.xlabel("Column (Near–Medium–Far)")
    plt.ylabel("Row (Top–Bottom)")

    for i in range(3):
        plt.axvline(i - 0.5, color="gray", linestyle=":")
    for j in range(2):
        plt.axhline(j - 0.5, color="gray", linestyle=":")

    for det in detections:
        # Use detected color for rectangle; fallback to black
        color = colors.get(det["ID"], "black")
        plt.gca().add_patch(
            plt.Rectangle((det["col"] - 0.4, det["row"] - 0.4),
                          0.8, 0.8, facecolor=color, edgecolor="black")
        )
        # Display the position grid id inside the cell
        plt.text(det["col"], det["row"], str(det["grid"]),
                 ha="center", va="center", fontsize=10, color="white")

    plt.savefig("arena.png")
    time.sleep(0.2)
