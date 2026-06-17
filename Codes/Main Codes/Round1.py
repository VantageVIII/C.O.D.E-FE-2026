import Hobot.GPIO as GPIO
import time
import serial
import struct
import smbus2
import threading

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
# Movement Class
# -----------------------------
class Movement:
    current_angle = 0
    offset = 0          # Trim this by ±2–3 if one turn direction is consistently tighter/wider
    _servo_thread_running = False
    pulse_ms = 1.5
    neutral_ms = 1.4

    @staticmethod
    def motor_forward(power=50, freq=200):
        period = 1.0 / freq
        high_time = (power / 100.0) * period
        low_time = period - high_time
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENA, GPIO.HIGH)
        time.sleep(high_time)
        GPIO.output(ENA, GPIO.LOW)
        time.sleep(low_time)

    @staticmethod
    def set_steering_angle(wheel_angle, max_angle=40, full_range=False):
        """
        Set the steering servo angle.

        full_range=False (default / normal driving):
            Uses calibrated neutral_ms as centre.
            pulse = neutral_ms ± 0.5 ms

        full_range=True (turn mode):
            Maps across the full physical 1.0–2.0 ms servo span,
            centred at 1.5 ms regardless of neutral trim.
            At max_angle this hits exactly 1.0 ms or 2.0 ms,
            giving the maximum possible physical deflection.
        """
        corrected = wheel_angle + Movement.offset
        Movement.current_angle = max(-max_angle, min(max_angle, corrected))

        if full_range:
            # Centre at 1.5 ms so ±max_angle maps to exactly 1.0–2.0 ms
            Movement.pulse_ms = 1.5 + (Movement.current_angle / max_angle) * 0.5
        else:
            Movement.pulse_ms = Movement.neutral_ms + (Movement.current_angle / max_angle) * 0.5

        Movement.pulse_ms = max(1.0, min(2.0, Movement.pulse_ms))

    @staticmethod
    def _servo_loop():
        while Movement._servo_thread_running:
            high_time = Movement.pulse_ms / 1000.0
            frame = 0.02
            start = time.time()
            GPIO.output(ServoPin, GPIO.HIGH)
            time.sleep(high_time)
            GPIO.output(ServoPin, GPIO.LOW)
            elapsed = time.time() - start
            time.sleep(max(0, frame - elapsed))

    @staticmethod
    def start_servo():
        if not Movement._servo_thread_running:
            Movement._servo_thread_running = True
            threading.Thread(target=Movement._servo_loop, daemon=True).start()

    @staticmethod
    def stop_servo():
        Movement._servo_thread_running = False

    @staticmethod
    def brake():
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
                # Limit yaw to -180..+180
                if self.yaw > 180:
                    self.yaw -= 360
                elif self.yaw < -180:
                    self.yaw += 360

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
# Helper: tolerance check
# -----------------------------
def within_tolerance(value, target, tol=0.05):
    return abs(value - target) <= target * tol

# Helper: normalize angle error to handle 180/-180 wrapping
def normalize_angle_error(target, current):
    error = target - current
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

# -----------------------------
# Main Loop
# -----------------------------
gyro = GyroSensor()
color = ColorSensor()

rotation_array = [0]
current_index = 0
lap_count = 0
max_laps = 3
orientation_colour = None

# Manual turn mode variables
manual_turn_mode = False
manual_turn_target = 0
manual_turn_start_angle = 0
manual_turn_frames = 0
manual_turn_pulse_mode = False
manual_turn_pulse_frames = 0
last_color_detected = None
color_read_threshold = 10   # frames to wait before allowing same color to be read again
correction_mode = False
correction_target = 0
correction_frames = 0

# ------------------------------------------------------------------
# Turn constants — change only these two values to tune sharpness
# ------------------------------------------------------------------
TURN_MAX_ANGLE  = 75    # servo deflection limit during turns (degrees, ±75)
TURN_SPEED      = 55    # motor power during turns — lower = tighter radius
# ------------------------------------------------------------------

print("Waiting for button press to start...")
while GPIO.input(ButtonPin) == GPIO.LOW:
    time.sleep(0.1)

print("Button pressed, starting forward movement...")
Movement.start_servo()

# Wait for gyro to stabilize and calibrate
print("Calibrating gyro...")
for i in range(50):    # 50 update cycles to stabilize
    gyro.update()
    time.sleep(0.01)
print(f"Gyro calibrated. Base yaw: {gyro.base_yaw:.2f}°, Current yaw: {gyro.yaw:.2f}°")

frame_count = 0

while True:
    gyro.update()
    rgb = color.get_rgb()
    r, g, b = rgb
    frame_count += 1

    # Detect orientation once
    if orientation_colour is None:
        if (155 <= r <= 185) and (85 <= g <= 125) and (50 <= b <= 90):
            rotation_array = [0, -105, 15, 75]   # anticlockwise
            orientation_colour = "orange"
            print("\nCounterclockwise rotation sequence selected")
        elif (85 <= r <= 110) and (90 <= g <= 115) and (85 <= b <= 120):
            rotation_array = [0, 105, -15, -75]  # clockwise
            orientation_colour = "blue"
            print("\nClockwise rotation sequence selected")

    # Color detection logic
    is_orientation_color = False
    is_opposite_color = False

    if orientation_colour == "orange":
        if (155 <= r <= 185) and (85 <= g <= 125) and (50 <= b <= 90):
            is_orientation_color = True
        elif (85 <= r <= 110) and (90 <= g <= 115) and (85 <= b <= 120):
            is_opposite_color = True
    elif orientation_colour == "blue":
        if (85 <= r <= 110) and (90 <= g <= 115) and (85 <= b <= 120):
            is_orientation_color = True
        elif (155 <= r <= 185) and (85 <= g <= 125) and (50 <= b <= 90):
            is_opposite_color = True

    # ------------------------------------------------------------------
    # Handle manual turn mode
    # ------------------------------------------------------------------
    if manual_turn_mode:
        # Opposite colour triggers the final pulse phase
        if is_opposite_color and last_color_detected != "opposite" and not manual_turn_pulse_mode:
            manual_turn_pulse_mode = True
            manual_turn_pulse_frames = 0
            last_color_detected = "opposite"
            print(f"\nOpposite color detected, entering pulse phase...")

        error = normalize_angle_error(manual_turn_target, gyro.yaw)

        if manual_turn_pulse_mode:
            # Pulse phase: hold full-lock steering for 8 frames then exit
            manual_turn_pulse_frames += 1
            raw_angle = -TURN_MAX_ANGLE if error > 0 else TURN_MAX_ANGLE

            if manual_turn_pulse_frames >= 8:
                manual_turn_mode = False
                manual_turn_pulse_mode = False
                manual_turn_frames = 0
                current_index += 1
                print(f"\nPulse complete, advancing to index {current_index}. Current angle: {gyro.yaw:.2f}°")
        else:
            # Regular turn: lock to full deflection throughout
            manual_turn_frames += 1
            raw_angle = -TURN_MAX_ANGLE if error > 0 else TURN_MAX_ANGLE

        # full_range=True maps ±TURN_MAX_ANGLE to the full 1.0–2.0 ms servo range
        Movement.set_steering_angle(raw_angle, max_angle=TURN_MAX_ANGLE, full_range=True)
        Movement.motor_forward(TURN_SPEED)
        print(f"MANUAL TURN | Target={manual_turn_target}° | Current={gyro.yaw:.2f}° | "
              f"Error={error:.2f}° | RawAngle={raw_angle:.1f}° | RGB={rgb}")

    # ------------------------------------------------------------------
    # Correction mode
    # ------------------------------------------------------------------
    elif correction_mode:
        if is_orientation_color and last_color_detected != "orientation" and current_index == 0:
            manual_turn_mode = True
            manual_turn_frames = 0
            manual_turn_pulse_mode = False
            manual_turn_pulse_frames = 0
            correction_mode = False
            manual_turn_start_angle = gyro.yaw
            if rotation_array[current_index] >= 0:
                manual_turn_target = rotation_array[current_index] + 50
            else:
                manual_turn_target = rotation_array[current_index] - 50

            last_color_detected = "orientation"
            print(f"\nOrientation color detected during correction! Entering manual turn. "
                  f"Target: {rotation_array[current_index]}° → Manual target: {manual_turn_target:.2f}°")
        else:
            error = normalize_angle_error(correction_target, gyro.yaw)
            raw_angle = max(-55, min(55, -error))

            Movement.set_steering_angle(raw_angle)
            Movement.motor_forward(60)

            print(f"CORRECTION | Target={correction_target}° | Current={gyro.yaw:.2f}° | "
                  f"Error={error:.2f}° | RawAngle={raw_angle:.1f}° | RGB={rgb}")

            correction_frames -= 1
            if correction_frames <= 0:
                correction_mode = False
                print(f"\nCorrection complete, returning to normal mode")

    # ------------------------------------------------------------------
    # Normal mode
    # ------------------------------------------------------------------
    else:
        if is_orientation_color and last_color_detected != "orientation" and current_index == 0:
            manual_turn_mode = True
            manual_turn_frames = 0
            manual_turn_pulse_mode = False
            manual_turn_pulse_frames = 0
            manual_turn_start_angle = gyro.yaw
            if rotation_array[current_index] >= 0:
                manual_turn_target = rotation_array[current_index] + 50
            else:
                manual_turn_target = rotation_array[current_index] - 50

            last_color_detected = "orientation"
            print(f"\nOrientation color detected! Starting manual 50° turn. "
                  f"Target: {rotation_array[current_index]}° → Manual target: {manual_turn_target:.2f}°")
        else:
            # Normal straight driving with gyro correction
            target_angle = rotation_array[current_index]
            error = normalize_angle_error(target_angle, gyro.yaw)

            raw_angle = max(-60, min(60, -error))

            Movement.set_steering_angle(raw_angle)
            Movement.motor_forward(65)

            print(f"Target={target_angle}° | Rotation={gyro.yaw:.2f}° | Error={error:.2f}° | "
                  f"RawAngle={raw_angle:.1f}° | RGB={rgb} | Lap={lap_count}")

            if orientation_colour == "orange" and is_orientation_color and last_color_detected != "orange":
                current_index += 1
                last_color_detected = "orange"
                print(f"\nOrange detected, moving to next index {current_index}")
            elif orientation_colour == "blue" and is_orientation_color and last_color_detected != "blue":
                current_index += 1
                last_color_detected = "blue"
                print(f"\nBlue detected, moving to next index {current_index}")

        if current_index >= len(rotation_array):
            current_index = 0
            lap_count += 1
            last_color_detected = None
            print(f"\nLap {lap_count} complete")

        if lap_count >= max_laps:
            Movement.brake()
            Movement.stop_servo()
            print("\nCourse complete. Car stopped.")
            break

    time.sleep(0.01)