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
    neutral_ms            = 1.4
    # Initialise pulse_ms to neutral, NOT 1.5 — prevents the servo from
    # snapping right the moment the servo thread starts.
    pulse_ms              = 1.4

    _servo_thread_running = False
    _motor_thread_running = False
    _motor_power          = 0      # shared duty-cycle value, written by main thread,
                                   # read by motor thread — range 0–100

    # ── Servo bit-bang ────────────────────────────────────────────────────────
    @staticmethod
    def set_steering_angle(wheel_angle, max_angle=40, full_range=False):
        """
        full_range=False  normal driving: centres on calibrated neutral_ms (1.4 ms).
        full_range=True   turn mode:      centres at 1.5 ms so ±max_angle maps
                          across the widest safe physical range (SERVO_TURN_MIN_MS
                          to SERVO_TURN_MAX_MS).
        """
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
        """Bit-bang a 50 Hz servo signal.  Runs in its own daemon thread."""
        while Movement._servo_thread_running:
            high_time  = Movement.pulse_ms / 1000.0
            frame_time = 0.02          # 50 Hz → 20 ms frame
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
# Colour detection helpers — keeps all conditions in one place
# ------------------------------------------------------------------
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

# ------------------------------------------------------------------
# Tuning constants — only touch these to adjust behaviour
# ------------------------------------------------------------------
NORMAL_SPEED       = 15    # motor duty cycle during straight driving
CORRECTION_SPEED   = 15    # motor duty cycle during heading correction
TURN_SPEED         = 35    # motor duty cycle during the main turn phase
TURN_CRAWL_SPEED   = 35    # motor duty cycle during settle phase (barely rolling)
TURN_MAX_ANGLE     = 65    # servo angle during turns, degrees (±75)
TURN_SETTLE_FRAMES = 6     # frames the servo is held at full lock before
                           # accelerating — gives wheels time to reach endpoint
EXIT_BURST_POWER   = 70    # brief high-power pulse after exiting turn mode
EXIT_BURST_FRAMES  = 10    # number of frames the burst lasts (doubled for longer momentum)

# Extend servo range slightly beyond the standard 1.0–2.0 ms spec for
# maximum physical deflection.  If the servo grunts or buzzes at the
# extremes, change these back to 1.0 and 2.0.
SERVO_TURN_MIN_MS  = 0.9
SERVO_TURN_MAX_MS  = 2.1
# ------------------------------------------------------------------

# -----------------------------
# Colour smoothing (EMA) - added for more robust blue detection
# -----------------------------
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
manual_turn_steer_target = 0     # heading used for servo error (index 0: overshoot target)
last_color_detected     = None
color_read_threshold    = 10
# Faster confirmation threshold for blue only
blue_confirm_threshold  = 3
correction_mode         = False
correction_target       = 0
correction_frames       = 0
exit_burst_frames       = 0
post_sequence_mode      = False


def advance_rotation_index():
    global current_index, lap_count, last_color_detected
    current_index += 1
    if current_index >= len(rotation_array):
        current_index = 0
        lap_count += 1
        last_color_detected = None
        print(f"\nLap {lap_count} complete")


print("Waiting for button press to start...")
while GPIO.input(ButtonPin) == GPIO.LOW:
    time.sleep(0.1)

print("Button pressed — starting up...")

# Centre the servo at neutral BEFORE starting its thread so the wheels
# do not twitch to one side on power-up.
Movement.set_steering_angle(0)
Movement.start_servo()

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
    raw_r, raw_g, raw_b = rgb
    r, g, b = ema_update(raw_r, raw_g, raw_b)
    # use raw_* for orange checks and r,g,b (EMA) for blue checks
    orange_check_r, orange_check_g, orange_check_b = raw_r, raw_g, raw_b
    blue_check_r, blue_check_g, blue_check_b = r, g, b
    frame_count += 1

    # ── Orientation detection (runs once) ────────────────────────────────────
    if orientation_colour is None:
        if is_orange_line(orange_check_r, orange_check_g, orange_check_b):
            rotation_array    = [0, -90, 179, 90]   #clockwise
            orientation_colour = "orange"
            print("\nCounterclockwise rotation sequence selected")
        elif is_blue_line(blue_check_r, blue_check_g, blue_check_b):
            rotation_array    = [0, 90, -179, -90]  # anticlockwise
            orientation_colour = "blue"
            print("\nClockwise rotation sequence selected")

    # ── Colour flags ─────────────────────────────────────────────────────────
    is_orientation_color = False
    is_opposite_color    = False

    if orientation_colour == "orange":
        if is_orange_line(orange_check_r, orange_check_g, orange_check_b):
            is_orientation_color = True
        elif is_blue_line(blue_check_r, blue_check_g, blue_check_b):
            is_opposite_color = True
    elif orientation_colour == "blue":
        if is_blue_line(blue_check_r, blue_check_g, blue_check_b):
            is_orientation_color = True
        elif is_orange_line(orange_check_r, orange_check_g, orange_check_b):
            is_opposite_color = True

    # ── Reset colour tracking on non-colour frames ───────────────────────────
    # Prevents a single noisy reading on white from permanently locking
    # last_color_detected and blocking all future colour triggers.
    if not is_orientation_color and not is_opposite_color:
        last_color_detected = None

    # ── Manual turn mode ─────────────────────────────────────────────────────
    if manual_turn_mode:
        if is_opposite_color and last_color_detected != "opposite" and not manual_turn_pulse_mode:
            manual_turn_pulse_mode   = True
            manual_turn_pulse_frames = 0
            last_color_detected      = "opposite"
            print("\nOpposite color detected — entering pulse phase...")

        error = normalize_angle_error(manual_turn_steer_target, gyro.yaw)

        # Lock servo to full deflection on every turn frame.
        # Set this BEFORE the speed decision so the servo starts
        # moving toward the endpoint as early as possible.
        raw_angle = -TURN_MAX_ANGLE if error > 0 else TURN_MAX_ANGLE

        # Direction lock: keep the servo within the half-range that matches
        # the direction captured at turn entry.  Prevents the servo crossing
        # centre if the gyro error briefly flips sign mid-corner.
        #   "left"  → allowed range [-TURN_MAX_ANGLE,  0]
        #   "right" → allowed range [0,  TURN_MAX_ANGLE]
        if manual_turn_direction == "left":
            raw_angle = max(-TURN_MAX_ANGLE, min(0, raw_angle))
        else:  # "right"
            raw_angle = max(0, min(TURN_MAX_ANGLE, raw_angle))
        Movement.set_steering_angle(raw_angle, max_angle=TURN_MAX_ANGLE, full_range=True)

        if manual_turn_pulse_mode:
            # ── Pulse phase: full lock, drive, exit after 8 frames ──────────
            manual_turn_pulse_frames += 1
            Movement.set_motor_forward(TURN_SPEED)

            if manual_turn_pulse_frames >= 8:
                manual_turn_mode        = False
                manual_turn_pulse_mode  = False
                manual_turn_frames      = 0
                exit_burst_frames       = EXIT_BURST_FRAMES
                advance_rotation_index()
                print(f"\nPulse complete — advancing to index {current_index}.  "
                      f"Current angle: {gyro.yaw:.2f}°  "
                      f"Burst: {exit_burst_frames} frames at {EXIT_BURST_POWER}%")
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
                      f"Pulse={Movement.pulse_ms:.3f} ms | RGB={r,g,b}")
            else:
                # ── Full turn phase ───────────────────────────────────────────
                Movement.set_motor_forward(TURN_SPEED)
                print(f"MANUAL TURN  Target={manual_turn_target}° | Yaw={gyro.yaw:.2f}° | "
                      f"Error={error:.2f}° | Angle={raw_angle:.0f}° | "
                      f"Pulse={Movement.pulse_ms:.3f} ms | RGB={r,g,b}")

    # ── Correction mode ───────────────────────────────────────────────────────
    elif correction_mode:
        if is_orientation_color and last_color_detected != "orientation":
            manual_turn_mode         = True
            manual_turn_frames       = 0
            manual_turn_pulse_mode   = False
            manual_turn_pulse_frames = 0
            correction_mode          = False
            manual_turn_start_angle  = gyro.yaw
            # Overshoot 50° in the direction of the next heading in the sequence.
            if current_index + 1 < len(rotation_array):
                _next_hdg = rotation_array[current_index + 1]
            else:
                _next_hdg = rotation_array[0]
            if _next_hdg >= rotation_array[current_index]:
                manual_turn_target = rotation_array[current_index] + 50
            else:
                manual_turn_target = rotation_array[current_index] - 50

            last_color_detected = "orientation"
            print(f"\nOrientation colour during correction — entering manual turn.  "
                  f"Target: {rotation_array[current_index]}° → {manual_turn_target:.2f}° "
                  f"(next heading = {_next_hdg}°)")
            # Lock the direction now so it can't flip mid-turn
            if current_index == 0:
                manual_turn_steer_target = manual_turn_target
                _init_err = normalize_angle_error(manual_turn_target, gyro.yaw)
            else:
                manual_turn_steer_target = _next_hdg
                _init_err = normalize_angle_error(_next_hdg, gyro.yaw)
            manual_turn_direction = "left" if _init_err > 0 else "right"
            print(f"Turn direction locked: {manual_turn_direction}")
        else:
            error     = normalize_angle_error(correction_target, gyro.yaw)
            raw_angle = max(-55, min(55, -error))
            Movement.set_steering_angle(raw_angle)
            Movement.set_motor_forward(CORRECTION_SPEED)
            print(f"CORRECTION  Target={correction_target}° | Yaw={gyro.yaw:.2f}° | "
                  f"Error={error:.2f}° | Angle={raw_angle:.1f}° | RGB={r,g,b}")

            correction_frames -= 1
            if correction_frames <= 0:
                correction_mode = False
                print("\nCorrection complete — returning to normal mode")

    # ── Post-sequence mode ──────────────────────────────────────────────────
    elif post_sequence_mode:
        # Drive straight forward using heading 0°
        target_angle = 0
        error = normalize_angle_error(target_angle, gyro.yaw)
        raw_angle = max(-60, min(60, -error))
        Movement.set_steering_angle(raw_angle)
        Movement.set_motor_forward(NORMAL_SPEED)

        # Stop when the same orientation colour is detected
        if orientation_colour == "orange" and is_orange_line(orange_check_r, orange_check_g, orange_check_b):
            Movement.brake()
            Movement.stop_servo()
            post_sequence_mode = False
            print("\nFinal orange line detected — stop at heading 0°.")
            break
        elif orientation_colour == "blue" and is_blue_line(blue_check_r, blue_check_g, blue_check_b):
            Movement.brake()
            Movement.stop_servo()
            post_sequence_mode = False
            print("\nFinal blue line detected — stop at heading 0°.")
            break

    # ── Normal mode ───────────────────────────────────────────────────────────
    else:
        if is_orientation_color and last_color_detected != "orientation":
            manual_turn_mode         = True
            manual_turn_frames       = 0
            manual_turn_pulse_mode   = False
            manual_turn_pulse_frames = 0
            manual_turn_start_angle  = gyro.yaw
            # Overshoot 50° in the direction of the next heading in the sequence,
            # so the car turns toward the next gate, not always the same direction.
            if current_index + 1 < len(rotation_array):
                _next_hdg = rotation_array[current_index + 1]
            else:
                _next_hdg = rotation_array[0]
            if _next_hdg >= rotation_array[current_index]:
                manual_turn_target = rotation_array[current_index] + 50
            else:
                manual_turn_target = rotation_array[current_index] - 50

            last_color_detected = "orientation"
            print(f"\nOrientation colour detected — starting manual 50° turn.  "
                  f"Target: {rotation_array[current_index]}° → {manual_turn_target:.2f}° "
                  f"(next heading = {_next_hdg}°)")
            # Lock the direction now so it can't flip mid-turn
            if current_index == 0:
                manual_turn_steer_target = manual_turn_target
                _init_err = normalize_angle_error(manual_turn_target, gyro.yaw)
            else:
                manual_turn_steer_target = _next_hdg
                _init_err = normalize_angle_error(_next_hdg, gyro.yaw)
            manual_turn_direction = "left" if _init_err > 0 else "right"
            print(f"Turn direction locked: {manual_turn_direction}")
        else:
            # Straight driving with gyro correction
            target_angle = rotation_array[current_index]
            error        = normalize_angle_error(target_angle, gyro.yaw)
            raw_angle    = max(-60, min(60, -error))

            Movement.set_steering_angle(raw_angle)

            # ── Exit burst: brief high-power pulse after a turn ──────────────
            # Gives the car momentum to overcome steering resistance when the
            # front wheels are still at an angle from the corner exit.
            if exit_burst_frames > 0:
                Movement.set_motor_forward(EXIT_BURST_POWER)
                exit_burst_frames -= 1
                print(f"EXIT BURST  frame={EXIT_BURST_FRAMES - exit_burst_frames}/{EXIT_BURST_FRAMES} | "
                      f"Power={EXIT_BURST_POWER}% | "
                      f"Target={target_angle}° | Yaw={gyro.yaw:.2f}°")
            else:
                Movement.set_motor_forward(NORMAL_SPEED)
                print(f"Target={target_angle}° | Yaw={gyro.yaw:.2f}° | "
                      f"Error={error:.2f}° | Angle={raw_angle:.1f}° | "
                      f"RGB={r,g,b} | Lap={lap_count}")

            # ── Consecutive-frame colour counters ────────────────────────────────────
            # Count how many frames in a row match each line colour.  A colour is only
            # "confirmed" once its counter reaches its threshold.  Any frame that
            # does not match a line colour resets BOTH counters, so white or noise can
            # never accumulate toward the threshold.
            # (This block moved here to keep behaviour identical while using EMA'd RGB)
            if is_orange_line(orange_check_r, orange_check_g, orange_check_b):
                orange_frames += 1
                blue_frames    = 0
            elif is_blue_line(blue_check_r, blue_check_g, blue_check_b):
                blue_frames   += 1
                orange_frames  = 0
            else:
                orange_frames  = 0
                blue_frames    = 0

            orange_confirmed = orange_frames >= color_read_threshold
            blue_confirmed   = blue_frames   >= blue_confirm_threshold

            # ── Line cooldown reset ──────────────────────────────────────────────────
            # Once the robot is off the line (no colour confirmed), clear the cooldown
            # so the next line crossing is allowed to increment the index again.
            if not orange_confirmed and not blue_confirmed:
                line_cooldown = False

            # Use the defensive advance helper and cooldown to avoid double increments
            if orientation_colour == "orange" and is_orientation_color and last_color_detected != "orange":
                if not line_cooldown and orange_confirmed:
                    line_cooldown = True
                    last_color_detected = "orange"
                    advance_rotation_index()
                    print(f"\nOrange detected — moving to index {current_index}")
                    # Debug print (commented) — uncomment for tuning
                    # print(f"DEBUG raw={raw_r,raw_g,raw_b} ema={r,g,b} br={r+g+b} ratios={(r/(r+g+b), g/(r+g+b), b/(r+g+b))} orange_frames={orange_frames} idx={current_index}")
            elif orientation_colour == "blue" and is_orientation_color and last_color_detected != "blue":
                if not line_cooldown and blue_confirmed:
                    line_cooldown = True
                    last_color_detected = "blue"
                    advance_rotation_index()
                    print(f"\nBlue detected — moving to index {current_index}")
                    # Debug print (commented) — uncomment for tuning
                    # print(f"DBG raw={raw_r,raw_g,raw_b} ema={r,g,b} br={r+g+b} ratios={(r/(r+g+b), g/(r+g+b), b/(r+g+b))} blue_frames={blue_frames} idx={current_index}")

        if lap_count >= max_laps:
            post_sequence_mode = True
            print("\nSequence complete. Entering post-sequence forward mode...")

    time.sleep(0.01)
