import smbus2, time, Hobot.GPIO as GPIO
from huskylib import HuskyLensLibrary

# -----------------------------
# Colour Sensor Setup
# -----------------------------
bus_num = 0   # /dev/i2c-0
addr = 0x29   # TCS34725 default address
bus = smbus2.SMBus(bus_num)

COMMAND_BIT = 0x80
ENABLE = 0x00
ATIME = 0x01
CONTROL = 0x0F
CDATA = 0x14  # Clear data low byte

bus.write_byte_data(addr, COMMAND_BIT | ENABLE, 0x03)
bus.write_byte_data(addr, COMMAND_BIT | ATIME, 0xD5)
bus.write_byte_data(addr, COMMAND_BIT | CONTROL, 0x01)
time.sleep(0.7)

def read_word(reg):
    low = bus.read_byte_data(addr, COMMAND_BIT | reg)
    high = bus.read_byte_data(addr, COMMAND_BIT | (reg+1))
    return (high << 8) | low

def get_rgb():
    c = read_word(CDATA)
    r = read_word(CDATA+2)
    g = read_word(CDATA+4)
    b = read_word(CDATA+6)
    if c == 0: return (0,0,0)
    r_std = max(0, min(255, int((r/c)*255)))
    g_std = max(0, min(255, int((g/c)*255)))
    b_std = max(0, min(255, int((b/c)*255)))
    return (r_std, g_std, b_std)

def is_turn_colour(rgb):
    r,g,b = rgb
    # thresholds with tolerance
    if r < 80 and g < 80 and b > 150:   # blue
        return True
    if r > 180 and g > 100 and b < 80:  # orange
        return True
    return False

# -----------------------------
# Camera Setup
# -----------------------------
hl = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)
print("Knock:", hl.knock())

# -----------------------------
# State Machine
# -----------------------------
STRAIGHT, TURN, SEARCH = 0, 1, 2
state = STRAIGHT
turn_timer = 0
missing_frames = 0
sweep_angle, sweep_dir = 70, 1

def steering_angle(arrow, gain=0.15, tolerance=5):
    mid_x = (arrow.xHead + arrow.xTail) / 2
    offset = mid_x - 160
    angle = 90 + gain * offset
    angle = max(35, min(145, angle))
    if 90 - tolerance <= angle <= 90 + tolerance:
        angle = 90
    return angle

# -----------------------------
# Main Loop
# -----------------------------
try:
    while True:
        rgb = get_rgb()
        results = hl.requestAll()

        if state == STRAIGHT:
            if results:
                missing_frames = 0
                for r in results:
                    if r.type == "ARROW":
                        servo_angle = steering_angle(r)
                        print(f"Servo={servo_angle:.1f}°")
                        # update servo + motor here
            else:
                missing_frames += 1
                if missing_frames > 10:
                    state = SEARCH

            if is_turn_colour(rgb):
                print("Colour sensor triggered TURN")
                state = TURN
                turn_timer = time.time()

        elif state == TURN:
            # lock servo to turn direction
            print("Executing TURN...")
            # set servo hard left/right + motor speed
            if time.time() - turn_timer > 2.0:
                state = STRAIGHT

        elif state == SEARCH:
            sweep_angle += sweep_dir * 5
            if sweep_angle > 120 or sweep_angle < 60:
                sweep_dir *= -1
            print(f"Searching... Servo={sweep_angle}°")
            # servo sweep + slow motor
            if results:
                state = STRAIGHT
                missing_frames = 0

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Interrupted")
