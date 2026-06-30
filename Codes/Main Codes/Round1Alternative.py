import Hobot.GPIO as GPIO
import time
import serial
import struct
import smbus2
import threading
import math

# -----------------------------
# Adjustable parameters
# -----------------------------
CIRCLE_RADIUS_CM     = 200.0   # desired circle radius in cm
WHEELBASE_CM         = 12.0    # wheelbase in cm (centre to centre)
MANUAL_OVERSHOOT_DEG = 50.0    # how far to overshoot during manual turn
CIRCLE_SPEED_PERCENT = 80      # motor duty cycle during circle laps
MAX_LAPS             = 3

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
# Movement Class (bit-bash servo + motor)
# -----------------------------
class Movement:
    current_angle         = 0
    offset                = 0
    neutral_ms            = 1.4
    pulse_ms              = 1.4
    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power          = 0

    @staticmethod
    def set_steering_angle(wheel_angle, max_angle=40, full_range=False):
        corrected = wheel_angle + Movement.offset
        Movement.current_angle = max(-max_angle, min(max_angle, corrected))
        if full_range:
            Movement.pulse_ms = 1.5 + (Movement.current_angle / max_angle) * 0.5
            Movement.pulse_ms = max(SERVO_TURN_MIN_MS, min(SERVO_TURN_MAX_MS, Movement.pulse_ms))
        else:
            Movement.pulse_ms = Movement.neutral_ms + (Movement.current_angle / max_angle) * 0.5
            Movement.pulse_ms = max(1.0, min(2.0, Movement.pulse_ms))

    @staticmethod
    def _servo_loop():
        while Movement._servo_thread_running:
            high_time  = Movement.pulse_ms / 1000.0
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
        freq   = 200
        period = 1.0 / freq
        while Movement._motor_thread_running:
            pwr = Movement._motor_power
            if pwr > 0:
                high_time = (pwr / 100.0) * period
                low_time  = period - high_time
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
# Gyro Sensor Class (relative yaw + reset)
# -----------------------------
class GyroSensor:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyS7', baudrate=9600, timeout=0.01)
        self.base_yaw = None
        self._last_raw_yaw = None
        self._rel_yaw = 0.0

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
                self._last_raw_yaw = raw_yaw
                if self.base_yaw is None:
                    self.base_yaw = raw_yaw
                rel = raw_yaw - self.base_yaw
                if rel > 180:
                    rel -= 360
                elif rel < -180:
                    rel += 360
                self._rel_yaw = rel

    def get_relative_yaw(self):
        return self._rel_yaw

    def reset_base_to_current(self):
        if self._last_raw_yaw is not None:
            self.base_yaw = self._last_raw_yaw
            self._rel_yaw = 0.0

# -----------------------------
# Colour Sensor Class
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
        low  = self.bus.read_byte_data(self.addr, self.COMMAND_BIT | reg)
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
# Helpers
# -----------------------------
def normalize_angle_error(target, current):
    error = target - current
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

def is_blue_line(r, g, b):
    br = r + g + b
    if br < 70:
        return False
    rb = r / br
    gb = g / br
    bb = b / br
    return (bb > 0.34) and (bb > rb + 0.06) and (gb > 0.28) and (br < 380)

def is_orange_line(r, g, b):
    return (
        (110 <= r <= 125) and
        (85 <= g <= 112) and
        (52 <= b <= 79) and
        (r > g) and
        (r > b)
    )

# -----------------------------
# Constants (kept)
# -----------------------------
NORMAL_SPEED       = 25
TURN_SPEED         = 45
TURN_CRAWL_SPEED   = 40
TURN_MAX_ANGLE     = 65
TURN_SETTLE_FRAMES = 6
EXIT_BURST_FRAMES  = 10

SERVO_TURN_MIN_MS  = 0.9
SERVO_TURN_MAX_MS  = 2.1

RGB_EMA_ALPHA = 0.45
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
# Main
# -----------------------------
gyro  = GyroSensor()
color = ColorSensor()

orientation_colour = None
manual_turn_mode = False
manual_turn_frames = 0
manual_turn_pulse_mode = False
manual_turn_pulse_frames = 0
manual_turn_target = 0
manual_turn_steer_target = 0
manual_turn_direction = None
last_color_detected = None
color_read_threshold = 10
blue_confirm_threshold = 3

# circle mode state
circle_mode_active = False
first_manual_turn_done = False
steering_set_once = False

# lap counting (signed continuous accumulation)
continuous_angle = 0.0
prev_rel_y = None
passed_half_turn = False
lap_count = 0

# compute circle steering angle once (degrees) from geometry: theta = atan(L / R)
circle_angle = math.degrees(math.atan(WHEELBASE_CM / CIRCLE_RADIUS_CM))

print("Waiting for button press to start...")
while GPIO.input(ButtonPin) == GPIO.LOW:
    time.sleep(0.1)

print("Button pressed — starting up...")

# centre servo and start threads
Movement.set_steering_angle(0)
Movement.start_servo()
Movement.start_motor()

# Calibrate gyro and explicitly set its zero point BEFORE driving forward
print("Calibrating gyro...")
for _ in range(50):
    gyro.update()
    time.sleep(0.01)

# Ensure the current heading becomes the zero reference so the robot drives straight initially
gyro.reset_base_to_current()
print(f"Gyro calibrated and zeroed. Base yaw set. Current rel: {gyro.get_relative_yaw():.2f}°")

frame_count = 0

# main loop
while True:
    gyro.update()
    raw_r, raw_g, raw_b = color.get_rgb()
    r, g, b = ema_update(raw_r, raw_g, raw_b)
    frame_count += 1

    # Orientation detection (only until first manual turn is started)
    if orientation_colour is None:
        if is_orange_line(r, g, b):
            orientation_colour = "orange"
            print("Orientation detected: ORANGE (clockwise)")
        elif is_blue_line(r, g, b):
            orientation_colour = "blue"
            print("Orientation detected: BLUE (anticlockwise)")

    # Colour flags
    is_orientation_color = False
    is_opposite_color = False
    if orientation_colour == "orange":
        if is_orange_line(r, g, b):
            is_orientation_color = True
        elif is_blue_line(r, g, b):
            is_opposite_color = True
    elif orientation_colour == "blue":
        if is_blue_line(r, g, b):
            is_orientation_color = True
        elif is_orange_line(r, g, b):
            is_opposite_color = True

    if not is_orientation_color and not is_opposite_color:
        last_color_detected = None

    # Before first manual turn is done: drive forward and allow one manual turn
    if not first_manual_turn_done:
        Movement.set_motor_forward(NORMAL_SPEED)

        if is_orientation_color and last_color_detected != "orientation":
            last_color_detected = "orientation"
            manual_turn_mode = True
            manual_turn_frames = 0
            manual_turn_pulse_mode = False
            manual_turn_pulse_frames = 0
            manual_turn_start = gyro.get_relative_yaw()

            # Decide manual_turn_target using adjustable overshoot
            if orientation_colour == "orange":
                manual_turn_target = manual_turn_start + MANUAL_OVERSHOOT_DEG
            else:
                manual_turn_target = manual_turn_start - MANUAL_OVERSHOOT_DEG

            manual_turn_steer_target = manual_turn_target
            _init_err = normalize_angle_error(manual_turn_steer_target, gyro.get_relative_yaw())

            # Corrected direction mapping: if _init_err > 0 we need to steer left to reduce error
            manual_turn_direction = "left" if _init_err > 0 else "right"
            print(f"Starting manual turn. Target {manual_turn_target:.1f}°, direction locked: {manual_turn_direction}")

        # Manual turn handling (original behaviour)
        if manual_turn_mode:
            if is_opposite_color and last_color_detected != "opposite" and not manual_turn_pulse_mode:
                manual_turn_pulse_mode = True
                manual_turn_pulse_frames = 0
                last_color_detected = "opposite"
                print("Opposite color detected — entering pulse phase...")

            error = normalize_angle_error(manual_turn_steer_target, gyro.get_relative_yaw())
            raw_angle = -TURN_MAX_ANGLE if error > 0 else TURN_MAX_ANGLE
            if manual_turn_direction == "left":
                raw_angle = max(-TURN_MAX_ANGLE, min(0, raw_angle))
            else:
                raw_angle = max(0, min(TURN_MAX_ANGLE, raw_angle))
            Movement.set_steering_angle(raw_angle, max_angle=TURN_MAX_ANGLE, full_range=True)

            if manual_turn_pulse_mode:
                manual_turn_pulse_frames += 1
                Movement.set_motor_forward(TURN_SPEED)
                if manual_turn_pulse_frames >= 8:
                    manual_turn_mode = False
                    manual_turn_pulse_mode = False
                    manual_turn_frames = 0
                    exit_burst_frames = EXIT_BURST_FRAMES
                    print(f"Manual turn pulse complete. Current yaw: {gyro.get_relative_yaw():.2f}°")
            else:
                manual_turn_frames += 1
                if manual_turn_frames <= TURN_SETTLE_FRAMES:
                    Movement.set_motor_forward(TURN_CRAWL_SPEED)
                    print(f"TURN SETTLE frame={manual_turn_frames}/{TURN_SETTLE_FRAMES} | Yaw={gyro.get_relative_yaw():.2f}°")
                else:
                    Movement.set_motor_forward(TURN_SPEED)
                    print(f"MANUAL TURN running | Yaw={gyro.get_relative_yaw():.2f}° | Angle={raw_angle:.0f}°")

    # Transition to circle mode immediately after the first manual turn finishes
    if (not manual_turn_mode) and (not circle_mode_active) and (orientation_colour is not None) and (not first_manual_turn_done):
        Movement.set_motor_forward(0)
        time.sleep(1.0)

        # reset gyro zero to current heading (explicit recalibration at manual-turn exit)
        gyro.reset_base_to_current()

        # prepare continuous angle accumulation
        prev_rel_y = gyro.get_relative_yaw()
        continuous_angle = 0.0
        passed_half_turn = False

        # choose steering sign based on manual_turn_direction
        if manual_turn_direction == "right":
            Movement.set_steering_angle(-circle_angle, max_angle=40, full_range=False)
            print(f"Circle mode steering set once to {-circle_angle:.2f}° (clockwise).")
        else:
            Movement.set_steering_angle(circle_angle, max_angle=40, full_range=False)
            print(f"Circle mode steering set once to {circle_angle:.2f}° (anticlockwise).")

        Movement.set_motor_forward(CIRCLE_SPEED_PERCENT)
        circle_mode_active = True
        steering_set_once = True
        first_manual_turn_done = True
        print("Entered circle mode: gyro zeroed, motor set to {}%.".format(CIRCLE_SPEED_PERCENT))

    # Circle mode: accumulate signed continuous angle and count laps
    if circle_mode_active:
        gyro.update()
        cur_rel = gyro.get_relative_yaw()
        delta = (cur_rel - prev_rel_y + 180.0) % 360.0 - 180.0
        prev_rel_y = cur_rel

        continuous_angle += delta

        if not passed_half_turn and abs(continuous_angle) >= 180.0:
            passed_half_turn = True

        if passed_half_turn and abs(continuous_angle) >= 360.0:
            lap_count += 1
            print(f"Lap {lap_count} complete (continuous angle {continuous_angle:.1f}°).")
            continuous_angle -= math.copysign(360.0, continuous_angle)
            passed_half_turn = False
            time.sleep(0.25)

        if lap_count >= MAX_LAPS:
            Movement.brake()
            Movement.stop_servo()
            print("Course complete. Car stopped.")
            break

    time.sleep(0.01)

# Cleanup
Movement.brake()
Movement.stop_servo()
Movement.stop_motor()
GPIO.output(LEDPin, GPIO.LOW)
print("Cleanup complete.")
