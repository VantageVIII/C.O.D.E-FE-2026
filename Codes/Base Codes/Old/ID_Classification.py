import time
import math
import collections
import serial
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from huskylib import HuskyLensLibrary

# -------------------------
# Hardware initialization
# -------------------------
hl_left = HuskyLensLibrary("SERIAL", "/dev/ttyS3", 9600)
hl_right = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)

try:
    gyro = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.05)
except Exception:
    gyro = None

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
HEIGHT = 240
YAW_LEFT = +12.0
YAW_RIGHT = -12.0

NEAR_AREA = 3000
MEDIUM_AREA_DEFAULT = 900

EWMA_ALPHA = 0.45
CONFIRM_FRAMES = 3
HYSTERESIS_AREA = 300

PILLAR_NORMAL_DEG = 90.0
APPROACH_ANGLE = 63.0

# Distance model constant (kept for optional use)
DISTANCE_MODEL_K = 1200.0

# Per-color calibration
COLOR_CALIBRATION = {"green": 1.0, "red": 1.0}
colors = {1: "green", 2: "red"}

# -------------------------
# Targeted fixes (tune these)
# -------------------------
# Static pixel offsets applied to x BEFORE angle->row mapping.
# Key: (cam, color_name, from_grid) -> pixels (positive moves detection right)
# Example: left camera, green initially read as grid 3 should be nudged right 18 pixels to become grid 6
TARGETED_PIXEL_OFFSETS = {
    ("L", "green", 3): 18,   # tune this value until green 3 -> 6 reliably
    ("L", "red",   5): 12    # tune this value if red at 5 needs lateral nudge
}

# Per-case medium thresholds (area). Smaller value makes medium easier to hit.
# Key: (cam, color_name, from_grid) -> medium_area_threshold
TARGETED_MEDIUM_THRESHOLDS = {
    ("L", "green", 6): 600,  # make green easier to classify as medium in cell 6
    ("L", "red",   5): 600   # make red easier to classify as medium in cell 5
}

# Restrict corrections to a camera if desired (None to allow both)
APPLY_CORRECTION_CAM = "L"

# -------------------------
# Helpers
# -------------------------
def get_grid_map(orientation):
    if orientation == "anticlockwise":
        return [[4, 5, 6],
                [1, 2, 3]]
    else:
        return [[1, 2, 3],
                [4, 5, 6]]

def grid_id_from_row_col(row, col, orientation):
    if row not in (0, 1) or col not in (0, 1, 2):
        return None
    return get_grid_map(orientation)[row][col]

def read_gyro_heading():
    if gyro is None:
        return None
    try:
        line = gyro.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return None
        for token in line.replace(',', ' ').split():
            try:
                return float(token) % 360.0
            except:
                continue
        return None
    except Exception:
        return None

def angle_compensation_factor(heading_deg):
    delta = abs((heading_deg - PILLAR_NORMAL_DEG + 180) % 360 - 180)
    delta = max(0.0, min(89.0, delta))
    cosd = math.cos(math.radians(delta))
    if cosd < 0.01:
        cosd = 0.01
    return 1.0 / cosd, delta

def estimate_distance_cm_from_area(area):
    if area <= 0:
        return float('inf')
    return DISTANCE_MODEL_K / math.sqrt(area)

# -------------------------
# State
# -------------------------
ewma_area = collections.defaultdict(float)
confirm_counts = collections.defaultdict(int)
last_committed = {}
last_candidate = {}

# -------------------------
# Classification with per-cell pixel offsets and thresholds
# -------------------------
def classify_detection_global(x, width, fov_deg, yaw_deg,
                              bbox_height, bbox_width, bbox_y,
                              ID, cam, orientation,
                              heading_override=None,
                              enable_targeted=True,
                              log=False):
    id_name = colors.get(ID, "unknown")
    raw_area = bbox_height * bbox_width
    raw_area *= COLOR_CALIBRATION.get(id_name, 1.0)

    heading = read_gyro_heading()
    if heading is None:
        heading = heading_override if heading_override is not None else APPROACH_ANGLE

    angle_factor, delta_deg = angle_compensation_factor(heading)
    compensated_area = raw_area * angle_factor

    key = (cam, ID)
    prev = ewma_area.get(key, 0.0)
    smoothed = compensated_area if prev == 0.0 else (EWMA_ALPHA * compensated_area + (1.0 - EWMA_ALPHA) * prev)
    ewma_area[key] = smoothed

    est_distance_cm = estimate_distance_cm_from_area(smoothed)

    # initial candidate using per-color default
    medium_threshold = MEDIUM_AREA_DEFAULT
    touching_bottom = (bbox_y + bbox_height) >= (HEIGHT - 3)
    if touching_bottom:
        candidate = "near"
    elif smoothed >= NEAR_AREA:
        candidate = "near"
    elif smoothed >= medium_threshold:
        candidate = "medium"
    else:
        candidate = "far"

    # compute initial row/col/grid from raw x (no pixel offset yet)
    norm_x = (x - width / 2) / width
    local_angle = norm_x * fov_deg + yaw_deg
    global_angle = local_angle - yaw_deg
    row = 0 if global_angle > 0 else 1
    col = 0 if candidate == "near" else (1 if candidate == "medium" else 2)
    grid_init = grid_id_from_row_col(row, col, orientation)

    # targeted per-case medium threshold override (re-evaluate candidate if needed)
    targeted_key = (cam, id_name, grid_init)
    targeted_threshold = TARGETED_MEDIUM_THRESHOLDS.get(targeted_key)
    if targeted_threshold is not None and candidate != "medium":
        if touching_bottom:
            candidate2 = "near"
        elif smoothed >= NEAR_AREA:
            candidate2 = "near"
        elif smoothed >= targeted_threshold:
            candidate2 = "medium"
        else:
            candidate2 = "far"
        if candidate2 != candidate:
            candidate = candidate2
            col = 0 if candidate == "near" else (1 if candidate == "medium" else 2)
            grid_init = grid_id_from_row_col(row, col, orientation)
            if log:
                print(f"[THRESH_OVERRIDE] cam={cam} color={id_name} grid_before={targeted_key[2]} used_threshold={targeted_threshold} -> candidate={candidate} grid_after={grid_init}")

    # targeted static pixel offset applied BEFORE angle->row mapping
    pixel_offset = 0
    if enable_targeted:
        if APPLY_CORRECTION_CAM is None or APPLY_CORRECTION_CAM == cam:
            px = TARGETED_PIXEL_OFFSETS.get((cam, id_name, grid_init))
            if px is not None:
                pixel_offset = px
                if log:
                    print(f"[PIXEL_OFFSET] cam={cam} color={id_name} grid={grid_init} px_offset={pixel_offset}")

    x_biased = x + pixel_offset

    # recompute row/angle using biased x (this is the key lateral correction)
    norm_x_biased = (x_biased - width / 2) / width
    local_angle_biased = norm_x_biased * fov_deg + yaw_deg
    global_angle_biased = local_angle_biased - yaw_deg
    row_biased = 0 if global_angle_biased > 0 else 1

    # final col uses candidate (near/medium/far) determined above
    col_final = 0 if candidate == "near" else (1 if candidate == "medium" else 2)
    grid_final = grid_id_from_row_col(row_biased, col_final, orientation)

    result = {
        "ID": ID,
        "ID_name": id_name,
        "row": row_biased,
        "col": col_final,
        "grid": grid_final,
        "height": bbox_height,
        "width": bbox_width,
        "area_smoothed": smoothed,
        "est_distance_cm": est_distance_cm,
        "angle": global_angle_biased,
        "cam": cam,
        "class": candidate,
        "touching_bottom": touching_bottom,
        "heading_used": heading,
        "delta_deg": delta_deg,
        "angle_compensation": angle_factor,
        "raw_x": x,
        "biased_x": x_biased,
        "pixel_offset_applied": pixel_offset,
        "medium_threshold_used": targeted_threshold if targeted_threshold is not None else medium_threshold
    }

    # confirmation and hysteresis (use the medium threshold actually used)
    hysteresis_threshold = result["medium_threshold_used"]
    last_cand = last_candidate.get(key)
    if last_cand != candidate:
        confirm_counts[key] = 1
        last_candidate[key] = candidate
    else:
        confirm_counts[key] += 1

    commit_now = False
    committed_last = last_committed.get(key)
    if committed_last is None:
        if confirm_counts[key] >= CONFIRM_FRAMES:
            commit_now = True
    elif committed_last != candidate:
        if confirm_counts[key] >= CONFIRM_FRAMES:
            if committed_last == "medium" and candidate == "far":
                if smoothed + HYSTERESIS_AREA < hysteresis_threshold:
                    commit_now = True
            elif committed_last == "far" and candidate == "medium":
                if smoothed - HYSTERESIS_AREA > hysteresis_threshold:
                    commit_now = True
            elif committed_last == "near" and candidate == "medium":
                if smoothed + HYSTERESIS_AREA < NEAR_AREA:
                    commit_now = True
            elif committed_last == "medium" and candidate == "near":
                if smoothed - HYSTERESIS_AREA > hysteresis_threshold:
                    commit_now = True
            else:
                commit_now = True

    if commit_now:
        last_committed[key] = candidate
        confirm_counts[key] = 0

    result["class"] = last_committed.get(key, candidate)

    if log:
        print(f"[LOG] cam={cam} ID={ID} color={id_name} smoothed={int(smoothed)} thr_used={int(result['medium_threshold_used'])} "
              f"grid_init={grid_init} grid_final={grid_final} px_off={pixel_offset} dist={est_distance_cm:.1f}cm class={result['class']}")

    return result

# -------------------------
# Matching logic (unchanged)
# -------------------------
def match_layout(detections, layout_definitions):
    matches = []
    for layout_id, expected in layout_definitions.items():
        ok = True
        for color, pos in expected:
            found = any(det["grid"] == pos and det["ID_name"] == color for det in detections)
            if not found:
                ok = False
                break
        if ok:
            matches.append(layout_id)
    final = []
    for m in matches:
        expected_m = set(layout_definitions[m])
        is_subset = False
        for n in matches:
            if m == n:
                continue
            expected_n = set(layout_definitions[n])
            if expected_m.issubset(expected_n):
                is_subset = True
                break
        if not is_subset:
            final.append(m)
    return final

# -------------------------
# Layout definitions (unchanged)
# -------------------------
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
layout_definitions_clockwise = {
    1: [("green", 4)],
    2: [("red", 4)],
    3: [("green", 5)],
    4: [("red", 5)],
    5: [("green", 6)],
    6: [("red", 6)],
    7: [("green", 1)],
    8: [("red", 1)],
    9: [("green", 2)], 10: [("red", 2)],
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

orientation = "anticlockwise"
layout_definitions = layout_definitions_anticlockwise if orientation == "anticlockwise" else layout_definitions_clockwise

# -------------------------
# Main loop
# -------------------------
while True:
    detections = []
    raw_left, raw_right = [], []

    results_left = hl_left.requestAll()
    if results_left:
        for r in results_left:
            if hasattr(r, "ID"):
                det = classify_detection_global(
                    r.x, WIDTH, FOV, YAW_LEFT,
                    r.height, r.width, r.y,
                    r.ID, "L", orientation,
                    heading_override=APPROACH_ANGLE,
                    enable_targeted=True,
                    log=True
                )
                raw_left.append(det)

    results_right = hl_right.requestAll()
    if results_right:
        for r in results_right:
            if hasattr(r, "ID"):
                det = classify_detection_global(
                    r.x, WIDTH, FOV, YAW_RIGHT,
                    r.height, r.width, r.y,
                    r.ID, "R", orientation,
                    heading_override=APPROACH_ANGLE,
                    enable_targeted=True,
                    log=True
                )
                raw_right.append(det)

    primary_dets = raw_left
    secondary_dets = [d for d in raw_right if d["angle"] < 0]

    if len(primary_dets) == 2:
        L, R = primary_dets[0], primary_dets[1]
        size_L = L["height"] * L["width"]
        size_R = R["height"] * R["width"]
        if size_L > size_R:
            L["col"] = 0; R["col"] = 2
        else:
            R["col"] = 0; L["col"] = 2
        if size_L < 0.3 * size_R:
            L["col"] = 2; R["col"] = 0
        elif size_R < 0.3 * size_L:
            R["col"] = 2; L["col"] = 0
        L["grid"] = grid_id_from_row_col(L["row"], L["col"], orientation)
        R["grid"] = grid_id_from_row_col(R["row"], R["col"], orientation)
        detections = [L, R]
    elif len(primary_dets) == 1:
        det = primary_dets[0]
        detections = [det]
        for sec in secondary_dets:
            if sec["ID"] == det["ID"]:
                primary_size = det["height"] * det["width"]
                secondary_size = sec["height"] * sec["width"]
                if secondary_size < 0.5 * primary_size:
                    hidden = sec.copy()
                    hidden["col"] = 2
                    hidden["grid"] = grid_id_from_row_col(hidden["row"], hidden["col"], orientation)
                    detections.append(hidden)
    else:
        detections.extend(secondary_dets)

    matched_layouts = match_layout(detections, layout_definitions)
    if matched_layouts:
        print(f"Detected Layout IDs: {matched_layouts}")
    else:
        print("No matching Layout ID found yet")

    # debug plot
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
        rect_color = det["ID_name"] if det["ID_name"] in ("green", "red") else "black"
        plt.gca().add_patch(
            plt.Rectangle((det["col"] - 0.4, det["row"] - 0.4),
                          0.8, 0.8, facecolo
                          =rect_color, edgecolor="black")
        )
        label = f"{det['grid']} {det['class']} {det['ID_name']}"
        debug = f"A={int(det['area_smoothed'])} thr={int(det['medium_threshold_used'])} px_off={int(det['pixel_offset_applied'])}"
        plt.text(det["col"], det["row"], f"{label}\n{debug}",
                 ha="center", va="center", fontsize=8, color="white")
    plt.savefig("arena.png")
    time.sleep(0.2)
