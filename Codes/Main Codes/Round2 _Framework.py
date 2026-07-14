import Hobot.GPIO as GPIO
import time
import serial
import struct
import smbus2
import threading
from huskylib import HuskyLensLibrary

# -----------------------------
# GPIO Setup
# -----------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

IN1 = 29
IN2 = 31
LEDPin = 37
ServoPin = 32
ENA = 33
ButtonPin = 18
GPIO.setup([IN1, IN2, LEDPin, ENA, ServoPin], GPIO.OUT)
GPIO.output(LEDPin, GPIO.HIGH)
GPIO.setup(ButtonPin, GPIO.IN)

# -----------------------------
# Route array system
# -----------------------------
ROUTE_SLOTS = [0, 0, 0, 0]
ROUTE_WRITE_INDEX = 1
TURN_COUNT = 0
RECORDING_ROUTES = True
REPLAY_INDEX = 0
REPLAY_LAPS = 0
MAX_REPLAY_LAPS = 2  # total laps = 3 (1st lap recording + 2 replay laps)
REPLAY_ACTIVE = False
REPLAY_NEXT_TIME = 0.0
REPLAY_PAUSE_SECONDS = 0.35

CLOCKWISE_ROUTE_GROUPS = {
    "A": {1, 3, 5, 9, 13, 19, 25, 31},
    "B": {2, 4, 6, 8, 10, 18, 24, 30, 36},
    "C": {14, 20, 26, 32},
    "D": {15, 27, 33},
}

ANTICLOCKWISE_ROUTE_GROUPS = {
    "A": {1, 3, 5, 9, 13, 19, 25, 31},
    "B": {2, 4, 6, 8, 10, 18, 24, 30, 36},
    "C": {14, 20, 26, 32},
    "D": {15, 27, 33},
}

ROUTE_GROUP_TO_ID = {"A": 1, "B": 2, "C": 3, "D": 4}

# -----------------------------
# Movement Class
# -----------------------------
class Movement:
    # -----------------------------
    # Bit-Bashing Helpers
    # -----------------------------

    current_angle = 0
    offset = 0
    neutral_ms = 1.4
    pulse_ms = 1.4

    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power = 0

    @staticmethod
    def set_steering_angle(wheel_angle, max_angle=40, full_range=False):
        corrected = wheel_angle + Movement.offset
        Movement.current_angle = max(-max_angle, min(max_angle, corrected))

        if full_range:
            Movement.pulse_ms = 1.5 + (Movement.current_angle / max_angle) * 0.5
            Movement.pulse_ms = max(0.9, min(2.1, Movement.pulse_ms))
        else:
            Movement.pulse_ms = Movement.neutral_ms + (Movement.current_angle / max_angle) * 0.5
            Movement.pulse_ms = max(1.0, min(2.0, Movement.pulse_ms))

    @staticmethod
    def _servo_loop():
        while Movement._servo_thread_running:
            high_time = Movement.pulse_ms / 1000.0
            frame_time = 0.02
            t0 = time.time()
            GPIO.output(ServoPin, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ServoPin, GPIO.LOW)
            elapsed = time.time() - t0
            remainder = frame_time - elapsed
            if remainder > 0:
                time.sleep(remainder)

    @staticmethod
    def start_servo():
        if not Movement._servo_thread_running:
            Movement._servo_thread_running = True
            threading.Thread(target=Movement._servo_loop, daemon=True).start()

    @staticmethod
    def stop_servo():
        Movement._servo_thread_running = False
        GPIO.output(ServoPin, GPIO.LOW)

    @staticmethod
    def set_motor_forward(power=50):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_power = max(0, min(100, int(power)))

    @staticmethod
    def _motor_loop():
        freq = 200
        period = 1.0 / freq
        while Movement._motor_thread_running:
            pwr = Movement._motor_power
            if pwr > 0:
                high_time = (pwr / 100.0) * period
                low_time = period - high_time
                GPIO.output(ENA, GPIO.HIGH)
                time.sleep(high_time)
                GPIO.output(ENA, GPIO.LOW)
                if low_time > 0:
                    time.sleep(low_time)
            else:
                GPIO.output(ENA, GPIO.LOW)
                time.sleep(period)

    @staticmethod
    def start_motor():
        if not Movement._motor_thread_running:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            Movement._motor_thread_running = True
            threading.Thread(target=Movement._motor_loop, daemon=True).start()

    @staticmethod
    def stop_motor():
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)
        GPIO.output(ENA, GPIO.LOW)

    @staticmethod
    def brake():
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.LOW)

# -----------------------------
# Gyro Sensor Class
# -----------------------------
class GyroSensor:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)
        self.base_yaw = None
        self.yaw = 0.0

    def update(self):
        while self.ser.in_waiting >= 11:
            header = self.ser.read(1)
            if header != b'\x55':
                continue
            packet_type = self.ser.read(1)
            data = self.ser.read(9)
            if len(data) != 9:
                continue
            if packet_type == b'\x53':
                raw_yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180
                if self.base_yaw is None:
                    self.base_yaw = raw_yaw
                self.yaw = raw_yaw - self.base_yaw
                if self.yaw > 180:
                    self.yaw -= 360
                elif self.yaw < -180:
                    self.yaw += 360

# -----------------------------
# Color Sensor Class
# -----------------------------
class ColorSensor:
    def __init__(self, i2c_bus=0, addr=0x29):
        self.bus = smbus2.SMBus(i2c_bus)
        self.addr = addr
        self.COMMAND_BIT = 0x80
        self.CDATA = 0x14
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x00, 0x03)
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x01, 0xFF)
        self.bus.write_byte_data(self.addr, self.COMMAND_BIT | 0x0F, 0x03)
        time.sleep(0.7)

    def read_word(self, reg):
        low = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | reg)
        high = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | (reg + 1))
        return (high << 8) | low

    def get_rgb(self):
        try:
            c = self.read_word(self.CDATA)
            r = self.read_word(self.CDATA + 2)
            g = self.read_word(self.CDATA + 4)
            b = self.read_word(self.CDATA + 6)
        except OSError:
            return (0, 0, 0)

        if c == 0:
            return (r, g, b)

        r_std = int((r / c) * 255)
        g_std = int((g / c) * 255)
        b_std = int((b / c) * 255)
        return (r_std, g_std, b_std)

# -----------------------------
# Orientation helpers
# -----------------------------
def is_blue_line(r, g, b):
    return (
        (58 <= r <= 227) and
        (83 <= g <= 237) and
        (106 <= b <= 182) and
        (b > g) and
        (b > r)
    )

def is_orange_line(r, g, b):
    return (
        (150 <= r <= 255) and
        (90 <= g <= 206) and
        (50 <= b <= 132) and
        (r > g) and
        (r > b)
    )

def within_tolerance(value, target, tol=0.05):
    return abs(value - target) <= target * tol

def normalize_angle_error(target, current):
    error = target - current
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

# ---------------------------------------
# Camera Setup (two HuskyLens instances)
# ---------------------------------------
# initialize left on /dev/ttyS3 and right on /dev/ttyS1
try:
    hl_left = HuskyLensLibrary("SERIAL", "/dev/ttyS3", 9600)
    hl_right = HuskyLensLibrary("SERIAL", "/dev/ttyS1", 9600)
    # Switch to Color Recognition algorithm and rename IDs to friendly names
    try:
        hl_left.algorthim("ALGORITHM_COLOR_RECOGNITION")
    except Exception:
        pass
    try:
        hl_right.algorthim("ALGORITHM_COLOR_RECOGNITION")
    except Exception:
        pass
    try:
        hl_left.setCustomName("Green", 1)
        hl_left.setCustomName("Red", 2)
        hl_right.setCustomName("Green", 1)
        hl_right.setCustomName("Red", 2)
    except Exception:
        pass
except Exception:
    # If HuskyLens init fails, create placeholders so code doesn't crash.
    hl_left = None
    hl_right = None

# Primary camera variable: will be assigned after orientation detection
PRIMARY_CAMERA = None

def map_detected_id_to_route_group(detected_id, turn_direction):
    """Map a detected turn ID to a compact route group ID for the fixed four-slot array."""
    route_groups = CLOCKWISE_ROUTE_GROUPS if turn_direction == "clockwise" else ANTICLOCKWISE_ROUTE_GROUPS
    for group_name, ids in route_groups.items():
        if detected_id in ids:
            return ROUTE_GROUP_TO_ID[group_name]
    return 0


def demo_detected_id_for_turn(turn_count):
    """Provide a simple turn-ID sequence that exercises each route group.

    Replace this with the real turn detector output if your course provides
    an actual numeric turn ID from the hardware.
    """
    demo_ids = [1, 2, 14, 15]
    if turn_count < len(demo_ids):
        return demo_ids[turn_count]
    return 0


def record_route(route_id, turn_direction, detected_id):
    """Store the mapped route ID into the next slot of the fixed four-slot array."""
    global ROUTE_SLOTS, ROUTE_WRITE_INDEX, TURN_COUNT, RECORDING_ROUTES, REPLAY_INDEX, REPLAY_LAPS, REPLAY_ACTIVE, REPLAY_NEXT_TIME

    if not RECORDING_ROUTES:
        return

    ROUTE_SLOTS[ROUTE_WRITE_INDEX] = route_id
    print(
        f"First lap turn #{TURN_COUNT + 1}: {turn_direction} turn (ID {detected_id}) -> "
        f"stored route ID {route_id} in slot {ROUTE_WRITE_INDEX}"
    )

    TURN_COUNT += 1
    ROUTE_WRITE_INDEX = (ROUTE_WRITE_INDEX + 1) % 4

    if TURN_COUNT >= 4:
        RECORDING_ROUTES = False
        REPLAY_INDEX = 0
        REPLAY_LAPS = 0
        REPLAY_ACTIVE = False
        REPLAY_NEXT_TIME = 0.0
        print("First lap route recording complete. Replaying the stored route array for the remaining laps.")


def execute_route(route_id):
    """Execute one route from the stored array using a simple, compact motion profile."""
    if route_id == 1:
        Movement.set_steering_angle(-22)
        Movement.set_motor_forward(28)
        time.sleep(0.7)
    elif route_id == 2:
        Movement.set_steering_angle(22)
        Movement.set_motor_forward(28)
        time.sleep(0.7)
    elif route_id == 3:
        Movement.set_steering_angle(0)
        Movement.set_motor_forward(24)
        time.sleep(0.55)
    elif route_id == 4:
        Movement.set_steering_angle(0)
        Movement.set_motor_forward(22)
        time.sleep(0.45)
    else:
        Movement.set_steering_angle(0)
        Movement.set_motor_forward(20)
        time.sleep(0.3)


# -------------------------------------------------
# Colour smoothing (Used for Blue colour detection)
# -------------------------------------------------
RGB_EMA_ALPHA = 0.3
ema_r = None
ema_g = None
ema_b = None


def ema_update(r, g, b):
    global ema_r, ema_g, ema_b
    if ema_r is None:
        ema_r, ema_g, ema_b = r, g, b
    else:
        ema_r = RGB_EMA_ALPHA * r + (1 - RGB_EMA_ALPHA) * ema_r
        ema_g = RGB_EMA_ALPHA * g + (1 - RGB_EMA_ALPHA) * ema_g
        ema_b = RGB_EMA_ALPHA * b + (1 - RGB_EMA_ALPHA) * ema_b
    return int(ema_r), int(ema_g), int(ema_b)


# -----------------------------
# Main loop
# -----------------------------
try:
    gyro = GyroSensor()
    color = ColorSensor()

    print("Waiting for button press to start...")
    while GPIO.input(ButtonPin) == GPIO.LOW:
        time.sleep(0.1)

    print("Button pressed — starting route-array controller")

    Movement.set_steering_angle(0)
    Movement.start_servo()
    Movement.start_motor()

    print("Calibrating gyro...")
    for _ in range(50):
        gyro.update()
        time.sleep(0.01)
    print(f"Gyro calibrated. Base yaw: {gyro.base_yaw:.2f}°")

    orientation_colour = None
    last_color_detected = None
    line_cooldown = False
    orange_frames = 0
    blue_frames = 0
    color_read_threshold = 10
    blue_confirm_threshold = 3

    while True:
        gyro.update()
        rgb = color.get_rgb()
        raw_r, raw_g, raw_b = rgb
        r, g, b = ema_update(raw_r, raw_g, raw_b)

        # Request detections from both HuskyLens cameras each loop and print them
        if hl_left is not None:
            for res in hl_left.requestAll() or []:
                if hasattr(res, "ID"):
                    tag = "[LeftCam][ID1]" if PRIMARY_CAMERA == "LeftCam" else "[LeftCam][ID2]"
                    print(f"{tag} Valid ID{res.ID} detection at ({res.x},{res.y}) size ({res.width}x{res.height})")

        if hl_right is not None:
            for res in hl_right.requestAll() or []:
                if hasattr(res, "ID"):
                    tag = "[RightCam][ID1]" if PRIMARY_CAMERA == "RightCam" else "[RightCam][ID2]"
                    print(f"{tag} Valid ID{res.ID} detection at ({res.x},{res.y}) size ({res.width}x{res.height})")

        # Orientation detection
        if orientation_colour is None:
            if is_orange_line(raw_r, raw_g, raw_b):
                orientation_colour = "orange"
                PRIMARY_CAMERA = "RightCam"
                print("Orientation detected: orange => clockwise")
            elif is_blue_line(r, g, b):
                orientation_colour = "blue"
                PRIMARY_CAMERA = "LeftCam"
                print("Orientation detected: blue => anticlockwise")

        # Show which camera is primary (1) or secondary (2) on each HuskyLens top-right corner
        if hl_left is not None:
            hl_left.customText("1" if PRIMARY_CAMERA == "LeftCam" else "2", 300, 10)

        if hl_right is not None:
            hl_right.customText("1" if PRIMARY_CAMERA == "RightCam" else "2", 300, 10)

        if orientation_colour in {"orange", "blue"}:
            is_orientation_color = is_orange_line(raw_r, raw_g, raw_b) if orientation_colour == "orange" else is_blue_line(r, g, b)
            is_opposite_color = is_blue_line(r, g, b) if orientation_colour == "orange" else is_orange_line(raw_r, raw_g, raw_b)
        else:
            is_orientation_color = False
            is_opposite_color = False

        if not is_orientation_color and not is_opposite_color:
            last_color_detected = None

        if RECORDING_ROUTES:
            # Drive forward until a turn is detected, then record the mapped route.
            Movement.set_steering_angle(0)
            Movement.set_motor_forward(22)

            if is_orientation_color and last_color_detected != "orientation":
                turn_direction = "clockwise" if orientation_colour == "orange" else "anticlockwise"
                detected_id = demo_detected_id_for_turn(TURN_COUNT)
                route_id = map_detected_id_to_route_group(detected_id, turn_direction)
                record_route(route_id, turn_direction, detected_id)
                execute_route(route_id)
                last_color_detected = "orientation"
            else:
                if is_orange_line(raw_r, raw_g, raw_b):
                    orange_frames += 1
                    blue_frames = 0
                elif is_blue_line(r, g, b):
                    blue_frames += 1
                    orange_frames = 0
                else:
                    orange_frames = 0
                    blue_frames = 0

                if not line_cooldown and ((orientation_colour == "orange" and orange_frames >= color_read_threshold) or
                                           (orientation_colour == "blue" and blue_frames >= blue_confirm_threshold)):
                    line_cooldown = True
                    last_color_detected = "orientation"
        else:
            # After the first lap, stop checking for new IDs and replay the stored route array.
            if REPLAY_ACTIVE:
                if time.time() >= REPLAY_NEXT_TIME:
                    REPLAY_ACTIVE = False
            else:
                if REPLAY_LAPS < MAX_REPLAY_LAPS:
                    route_id = ROUTE_SLOTS[REPLAY_INDEX]
                    print(f"Replaying route ID {route_id} from slot {REPLAY_INDEX}")
                    execute_route(route_id)
                    REPLAY_INDEX = (REPLAY_INDEX + 1) % 4
                    if REPLAY_INDEX == 0:
                        REPLAY_LAPS += 1
                        print(f"Replay lap {REPLAY_LAPS}/{MAX_REPLAY_LAPS} complete")
                        if REPLAY_LAPS >= MAX_REPLAY_LAPS:
                            Movement.brake()
                            Movement.stop_servo()
                            print("Route replay complete.")
                            break
                    REPLAY_ACTIVE = True
                    REPLAY_NEXT_TIME = time.time() + REPLAY_PAUSE_SECONDS
                else:
                    Movement.brake()
                    Movement.stop_servo()
                    print("Route replay complete.")
                    break

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    Movement.stop_motor()
    Movement.stop_servo()
    GPIO.cleanup()
    print("GPIO cleanup complete.")
