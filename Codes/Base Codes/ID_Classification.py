import time
from huskylib import HuskyLensLibrary

# -----------------------------
# Camera Setup
# -----------------------------
hl = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)
print("Knock:", hl.knock())

# -----------------------------
# Switch to Color Recognition algorithm
# -----------------------------
print(hl.algorthim("ALGORITHM_COLOR_RECOGNITION"))

# -----------------------------
# Rename IDs at device level
# -----------------------------
print(hl.setCustomName("Green", 1))  # Colour ID1
print(hl.setCustomName("Red", 2))    # Colour ID2

# -----------------------------
# Define exclusion zone (top-right corner)
# -----------------------------
EXCLUDE_X = 320 - 75   # assuming screen width ~320 px
EXCLUDE_Y = 0          # top edge
EXCLUDE_W = 75
EXCLUDE_H = 75

# -----------------------------
# Helper: check if detection is inside exclusion zone
# -----------------------------
def inside_exclusion(r):
    return (EXCLUDE_X <= r.x <= EXCLUDE_X + EXCLUDE_W) and (EXCLUDE_Y <= r.y <= EXCLUDE_Y + EXCLUDE_H)

# -----------------------------
# Main Loop
# -----------------------------
while True:
    results = hl.requestAll()
    if results:
        for r in results:
            if hasattr(r, "ID"):
                if inside_exclusion(r):
                    print(f"Ignored ID{r.ID} inside exclusion zone at ({r.x},{r.y})")
                else:
                    # HuskyLens itself now shows "Green:ID1" or "Red:ID2"
                    print(f"Valid ID{r.ID} detection at ({r.x},{r.y}) size ({r.width}x{r.height})")

    # Draw exclusion zone markers on HuskyLens screen
    hl.customText("X", EXCLUDE_X, EXCLUDE_Y)                        # top-left
    hl.customText("X", EXCLUDE_X + EXCLUDE_W, EXCLUDE_Y)            # top-right
    hl.customText("X", EXCLUDE_X, EXCLUDE_Y + EXCLUDE_H)            # bottom-left
    hl.customText("X", EXCLUDE_X + EXCLUDE_W, EXCLUDE_Y + EXCLUDE_H) # bottom-right

    time.sleep(0.2)
