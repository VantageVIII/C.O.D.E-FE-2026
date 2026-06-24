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
    current_angle         = 0
    offset                = 0      # trim ±2–3 if one direction turns tighter/wider than the other
    neutral_ms            = 1.5   # new servo: centre = 150° = 1500 μs = 1.5 ms
    # Initialise pulse_ms to neutral so the servo doesn't snap on thread start.
    pulse_ms              = 1.5

    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power          = 0      # shared duty-cycle value, written by main thread,
                                   # read by motor thread — range 0–100

    # ── Servo bit-bang ────────────────────────────────────────────────────────
    @staticmethod
    def set_steering_angle(wheel_angle, max_angle=40, full_range=False):
        """
        Map a steering wheel_angle (±max_angle °) to a servo pulse width.

        New servo spec (from sweep code):
            Total range  : 0–300°
            Pulse range  : 500–2500 μs  (0.5–2.5 ms)
            Centre       : 150° = 1500 μs = 1.5 ms  (straight ahead)

        Formula:
            servo_deg = SERVO_CENTRE_DEG + wheel_angle
            pulse_us  = SERVO_MIN_US + (servo_deg / SERVO_RANGE_DEG)
                        * (SERVO_MAX_US - SERVO_MIN_US)

        Examples at TURN_MAX_ANGLE = 75°:
            wheel_angle  -75° → servo 75°  → 1000 μs = 1.0 ms  (full left)
            wheel_angle    0° → servo 150° → 1500 μs = 1.5 ms  (centre)
            wheel_angle  +75° → servo 225° → 2000 μs = 2.0 ms  (full right)

        full_range is kept for API compatibility but has no effect —
        the formula is the same at all ranges.
        """
        corrected = wheel_angle + Movement.offset
        Movement.current_angle = max(-max_angle, min(max_angle, corrected))

        servo_deg = SERVO_CENTRE_DEG + Movement.current_angle
        pulse_us  = SERVO_MIN_US + (servo_deg / SERVO_RANGE_DEG) * (SERVO_MAX_US - SERVO_MIN_US)
        Movement.pulse_ms = max(SERVO_PULSE_MIN_MS, min(SERVO_PULSE_MAX_MS, pulse_us / 1000.0))

    @staticmethod
    def _servo_loop():
        """
        Bit-bang a 50 Hz servo signal with accurate pulse timing.

        Problem with plain time.sleep() on Linux:
            The OS scheduler has a minimum granularity of ~1–10 ms, so
            time.sleep(0.0014) can actually sleep 5–10 ms.  This makes
            the servo pulse wildly inaccurate → servo doesn't respond.

        Fix:
            Use a busy-wait (spinning on time.monotonic()) for the HIGH
            pulse (1.0–2.1 ms) where accuracy is critical.
            Use time.sleep() for most of the LOW period (17–19 ms) where
            a few ms of imprecision doesn't matter, then busy-wait the
            last 1 ms to land on the frame boundary accurately.
        """
        FRAME = 0.02   # 50 Hz → 20 ms per frame

        while Movement._servo_thread_running:
            pulse = Movement.pulse_ms / 1000.0   # ms → seconds
            t0    = time.monotonic()

            # ── HIGH pulse — busy-wait for microsecond accuracy ──────────
            GPIO.output(ServoPin, GPIO.HIGH)
            t_pulse_end = t0 + pulse
            while time.monotonic() < t_pulse_end:
                pass
            GPIO.output(ServoPin, GPIO.LOW)

            # ── LOW remainder — sleep bulk, busy-wait the last 1 ms ──────
            t_frame_end  = t0 + FRAME
            sleep_target = t_frame_end - 0.001   # leave 1 ms for busy-wait
            remaining    = sleep_target - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            while time.monotonic() < t_frame_end:  # fine-tune to frame end
                pass

    @staticmethod
    def start_servo():
        if not Movement._servo_thread_running:
            Movement._servo_thread_running = True
            threading.Thread(target=Movement._servo_loop, daemon=True).start()

    @staticmethod
    def stop_servo():
        Movement._servo_thread_running = False
        GPIO.output(ServoPin, GPIO.LOW)

    # ── Motor bit-bang ────────────────────────────────────────────────────────
    @staticmethod
    def set_motor_forward(power=50):
        """
        Set direction to forward and update the target duty cycle.
        The motor thread reads _motor_power and handles all PWM timing —
        this call returns immediately so it never blocks the main loop.
        """
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        Movement._motor_power = max(0, min(100, int(power)))

    @staticmethod
    def _motor_loop():
        """
        Bit-bang ~200 Hz PWM on ENA in its own daemon thread.
        Completely decoupled from the main loop so timing is consistent
        regardless of gyro reads, colour reads, or print statements.
        """
        freq   = 200
        period = 1.0 / freq    # 5 ms per PWM cycle
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
                # Power is zero — keep ENA low for the full period
                GPIO.output(ENA, GPIO.LOW)
                time.sleep(period)

    @staticmethod
    def start_motor():
        if not Movement._motor_thread_running:
            # Direction defaults to forward; caller sets it before the first
            # set_motor_forward() if needed.
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
            Movement._motor_thread_running = True
            threading.Thread(target=Movement._motor_loop, daemon=True).start()

    @staticmethod
    def stop_motor():
        """Stop the motor thread and ensure ENA is left low."""
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)          # allow thread to finish its current pulse
        GPIO.output(ENA, GPIO.LOW)

    @staticmethod
    def brake():
        """Hard brake: short IN1/IN2, kill ENA, stop motor thread."""
        Movement._motor_power = 0
        Movement._motor_thread_running = False
        time.sleep(0.015)          # allow thread to finish its current pulse
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
def within_tolerance(value, target, tol=0.05):
    return abs(value - target) <= target * tol

def normalize_angle_error(target, current):
    error = target - current
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

# ------------------------------------------------------------------
# Tuning constants — only touch these to adjust behaviour
# ------------------------------------------------------------------
NORMAL_SPEED       = 40    # motor duty cycle during straight driving
CORRECTION_SPEED   = 35    # motor duty cycle during heading correction
TURN_SPEED         = 25    # motor duty cycle during the main turn phase
PULSE_SPEED        = 30    # motor duty cycle during the pulse phase (end of turn)
TURN_CRAWL_SPEED   = 10    # motor duty cycle during settle phase (barely rolling)
TURN_MAX_ANGLE     = 75    # servo angle during turns, degrees (±75)
TURN_SETTLE_FRAMES = 6     # frames the servo is held at full lock before
                           # accelerating — gives wheels time to reach endpoint

# New servo physical spec — matches the sweep code
# Change these if you swap servos again.
SERVO_CENTRE_DEG  = 150      # servo angle for straight ahead (degrees)
SERVO_RANGE_DEG   = 300      # total servo travel (degrees)
SERVO_MIN_US      = 500      # pulse width at 0°  (μs)
SERVO_MAX_US      = 2500     # pulse width at 300° (μs)
# Safety clamp — pulse is kept inside these limits even if the maths goes out
# of range.  Full spec is 0.5–2.5 ms; keeping a small margin protects the servo.
SERVO_PULSE_MIN_MS = 0.5     # = 500 μs  (0° hard stop)
SERVO_PULSE_MAX_MS = 2.5     # = 2500 μs (300° hard stop)
# ------------------------------------------------------------------

# -----------------------------
# Main
# -----------------------------
gyro  = GyroSensor()
color = ColorSensor()

rotation_array = [0]
current_index  = 0
lap_count      = 0
max_laps       = 3
orientation_colour = None

manual_turn_mode        = False
manual_turn_target      = 0
manual_turn_start_angle = 0
manual_turn_frames      = 0
manual_turn_pulse_mode  = False
manual_turn_pulse_frames = 0
manual_turn_direction   = None   # "left" or "right" — locked at turn entry
last_color_detected     = None
color_read_threshold    = 10
correction_mode         = False
correction_target       = 0
correction_frames       = 0

print("Waiting for button press to start...")
print("(Touch the capacitive sensor to begin)")

# Give the sensor time to stabilise on power-up before sampling it.
time.sleep(0.5)

# Auto-detect resting polarity: read the pin NOW (before any touch) and
# treat the OPPOSITE level as "touched".  This works whether your sensor
# outputs HIGH-at-rest or LOW-at-rest without any code changes.
resting_state = GPIO.input(ButtonPin)
active_state  = GPIO.LOW if resting_state == GPIO.HIGH else GPIO.HIGH
print(f"Sensor resting: {'HIGH' if resting_state else 'LOW'} — "
      f"will start on {'HIGH' if active_state else 'LOW'}")

# Debounce: require 8 consecutive reads of the active state (~160 ms)
# so electrical noise or a glitch can never trigger a false start.
debounce_count = 0
while debounce_count < 8:
    if GPIO.input(ButtonPin) == active_state:
        debounce_count += 1
    else:
        debounce_count = 0   # reset on any noise
    time.sleep(0.02)

print("Touch detected — starting up...")

# Centre the servo at neutral BEFORE starting its thread so the wheels
# do not twitch to one side on power-up.
Movement.set_steering_angle(0)
Movement.start_servo()

# Startup servo sweep — move left → centre → right → centre so you can
# immediately see whether the servo is physically responding.
# If the wheels don't move here, it's a wiring / power issue, not code.
print("Servo test: sweeping left → centre → right → centre...")
Movement.set_steering_angle(-30)
time.sleep(0.6)
Movement.set_steering_angle(0)
time.sleep(0.4)
Movement.set_steering_angle(30)
time.sleep(0.6)
Movement.set_steering_angle(0)
time.sleep(0.4)
print("Servo test complete — if wheels didn't move, check wiring/power.")

# Start the motor bit-bang thread (motor is idle until set_motor_forward is called)
Movement.start_motor()

print("Calibrating gyro...")
for _ in range(50):
    gyro.update()
    time.sleep(0.01)
print(f"Gyro calibrated.  Base yaw: {gyro.base_yaw:.2f}°  Current: {gyro.yaw:.2f}°")

frame_count = 0

while True:
    gyro.update()
    rgb = color.get_rgb()
    r, g, b = rgb
    frame_count += 1

    # ── Orientation detection (runs once) ────────────────────────────────────
    if orientation_colour is None:
        if (155 <= r <= 185) and (85 <= g <= 125) and (50 <= b <= 90):
            rotation_array    = [0, -105, 15, 75]   # anticlockwise
            orientation_colour = "orange"
            print("\nCounterclockwise rotation sequence selected")
        elif (85 <= r <= 110) and (90 <= g <= 115) and (85 <= b <= 120):
            rotation_array    = [0, 105, -15, -75]  # clockwise
            orientation_colour = "blue"
            print("\nClockwise rotation sequence selected")

    # ── Colour flags ─────────────────────────────────────────────────────────
    is_orientation_color = False
    is_opposite_color    = False

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

    # ── Manual turn mode ─────────────────────────────────────────────────────
    if manual_turn_mode:
        if is_opposite_color and last_color_detected != "opposite" and not manual_turn_pulse_mode:
            manual_turn_pulse_mode   = True
            manual_turn_pulse_frames = 0
            last_color_detected      = "opposite"
            print("\nOpposite color detected — entering pulse phase...")

        error = normalize_angle_error(manual_turn_target, gyro.yaw)

        # Derive servo angle from the direction locked at turn entry.
        # Never recompute from live error — error can briefly flip sign
        # mid-corner (gyro noise / slight overshoot) and would cause a
        # momentary pulse in the wrong direction.
        if manual_turn_direction == "left":
            raw_angle = -TURN_MAX_ANGLE
        elif manual_turn_direction == "right":
            raw_angle = TURN_MAX_ANGLE
        else:
            # Fallback only — should never reach here
            raw_angle = -TURN_MAX_ANGLE if error > 0 else TURN_MAX_ANGLE
        Movement.set_steering_angle(raw_angle, max_angle=TURN_MAX_ANGLE, full_range=True)

        if manual_turn_pulse_mode:
            # ── Pulse phase: full lock, drive, exit after 8 frames ──────────
            manual_turn_pulse_frames += 1
            Movement.set_motor_forward(PULSE_SPEED)

            if manual_turn_pulse_frames >= 8:
                manual_turn_mode        = False
                manual_turn_pulse_mode  = False
                manual_turn_frames      = 0
                current_index          += 1
                print(f"\nPulse complete — advancing to index {current_index}.  "
                      f"Current angle: {gyro.yaw:.2f}°")
        else:
            manual_turn_frames += 1

            if manual_turn_frames <= TURN_SETTLE_FRAMES:
                # ── Settle phase ─────────────────────────────────────────────
                # Servo is already commanding full lock (set above).
                # Crawl slowly so the wheels physically reach their endpoint
                # before the car builds speed into the corner.
                Movement.set_motor_forward(TURN_CRAWL_SPEED)
                print(f"TURN SETTLE  frame={manual_turn_frames}/{TURN_SETTLE_FRAMES} | "
                      f"Target={manual_turn_target}° | Yaw={gyro.yaw:.2f}° | "
                      f"Pulse={Movement.pulse_ms:.3f} ms | RGB={rgb}")
            else:
                # ── Full turn phase ───────────────────────────────────────────
                Movement.set_motor_forward(TURN_SPEED)
                print(f"MANUAL TURN  Target={manual_turn_target}° | Yaw={gyro.yaw:.2f}° | "
                      f"Error={error:.2f}° | Angle={raw_angle:.0f}° | "
                      f"Pulse={Movement.pulse_ms:.3f} ms | RGB={rgb}")

    # ── Correction mode ───────────────────────────────────────────────────────
    elif correction_mode:
        if is_orientation_color and last_color_detected != "orientation" and current_index == 0:
            manual_turn_mode         = True
            manual_turn_frames       = 0
            manual_turn_pulse_mode   = False
            manual_turn_pulse_frames = 0
            correction_mode          = False
            manual_turn_start_angle  = gyro.yaw
            if rotation_array[current_index] >= 0:
                manual_turn_target = rotation_array[current_index] + 50
            else:
                manual_turn_target = rotation_array[current_index] - 50

            last_color_detected = "orientation"
            print(f"\nOrientation colour during correction — entering manual turn.  "
                  f"Target: {rotation_array[current_index]}° → {manual_turn_target:.2f}°")
            # Lock the direction now so it can't flip mid-turn
            _init_err = normalize_angle_error(manual_turn_target, gyro.yaw)
            manual_turn_direction = "left" if _init_err > 0 else "right"
            print(f"Turn direction locked: {manual_turn_direction}")
        else:
            error     = normalize_angle_error(correction_target, gyro.yaw)
            raw_angle = max(-55, min(55, -error))
            Movement.set_steering_angle(raw_angle)
            Movement.set_motor_forward(CORRECTION_SPEED)
            print(f"CORRECTION  Target={correction_target}° | Yaw={gyro.yaw:.2f}° | "
                  f"Error={error:.2f}° | Angle={raw_angle:.1f}° | RGB={rgb}")

            correction_frames -= 1
            if correction_frames <= 0:
                correction_mode = False
                print("\nCorrection complete — returning to normal mode")

    # ── Normal mode ───────────────────────────────────────────────────────────
    else:
        if is_orientation_color and last_color_detected != "orientation" and current_index == 0:
            manual_turn_mode         = True
            manual_turn_frames       = 0
            manual_turn_pulse_mode   = False
            manual_turn_pulse_frames = 0
            manual_turn_start_angle  = gyro.yaw
            if rotation_array[current_index] >= 0:
                manual_turn_target = rotation_array[current_index] + 50
            else:
                manual_turn_target = rotation_array[current_index] - 50

            last_color_detected = "orientation"
            print(f"\nOrientation colour detected — starting manual 50° turn.  "
                  f"Target: {rotation_array[current_index]}° → {manual_turn_target:.2f}°")
            # Lock the direction now so it can't flip mid-turn
            _init_err = normalize_angle_error(manual_turn_target, gyro.yaw)
            manual_turn_direction = "left" if _init_err > 0 else "right"
            print(f"Turn direction locked: {manual_turn_direction}")
        else:
            # Straight driving with gyro correction
            target_angle = rotation_array[current_index]
            error        = normalize_angle_error(target_angle, gyro.yaw)
            raw_angle    = max(-60, min(60, -error))

            Movement.set_steering_angle(raw_angle)
            Movement.set_motor_forward(NORMAL_SPEED)

            print(f"Target={target_angle}° | Yaw={gyro.yaw:.2f}° | "
                  f"Error={error:.2f}° | Angle={raw_angle:.1f}° | "
                  f"RGB={rgb} | Lap={lap_count}")

            # Explicit turn-mode guard: index must not advance while a turn
            # is in progress.  Already structurally protected by the
            # if/elif/else above, but stated explicitly to prevent any
            # future refactor from accidentally breaking the lock.
            if orientation_colour == "orange" and is_orientation_color \
                    and last_color_detected != "orange" and not manual_turn_mode:
                current_index      += 1
                last_color_detected = "orange"
                print(f"\nOrange detected — moving to index {current_index}")
            elif orientation_colour == "blue" and is_orientation_color \
                    and last_color_detected != "blue" and not manual_turn_mode:
                current_index      += 1
                last_color_detected = "blue"
                print(f"\nBlue detected — moving to index {current_index}")

        if current_index >= len(rotation_array):
            current_index       = 0
            lap_count          += 1
            last_color_detected = None
            print(f"\nLap {lap_count} complete")

        if lap_count >= max_laps:
            Movement.brake()
            Movement.stop_servo()
            print("\nCourse complete.  Car stopped.")
            break

    time.sleep(0.01)